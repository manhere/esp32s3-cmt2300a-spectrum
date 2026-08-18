/*
 * ook_decoder.cpp — OOK 码流采集与解码实现
 *
 * 【核心设计：判决全部放在"时长域"，不依赖 Te 量化】
 *   历史教训：旧版先估 Te、把每段量化成整数倍 k，再用 k 做码元周期校验与 0/1 判决。
 *   这条链路对 Te 误差极其敏感——实测 433 遥控 5706068(0x571154, Te 约 390us) 那次，
 *   缓冲被噪声灌满导致 Te 被估成 294us，于是 440us->k=1 而 463us->k=2（整数边界恰在
 *   441us），码元周期在 5/6 之间跳变，本来完好的帧被"周期恒定校验"当成脏帧全部丢弃。
 *   现在 0/1 判决与周期校验直接用微秒比值，天然与 Te 无关；Te 只用于
 *   (a) 毛刺合并阈值 (b) 帧间隙粗切 (c) 结果显示，这三处都容忍 ±30% 的误差。
 *
 * 解码链路：
 *   - **毛刺合并**：把 <Te/3 的窄段与前后两段并回去（切片器在包络边沿会吐窄脉冲，
 *     一个毛刺就能把 3T 劈成三段，既污染直方图又整体打乱 (高,低) 配对）；
 *   - **切帧**：以"超长段(>=8×Te0)=帧间同步间隙"切成若干完整帧，首尾被采集窗口
 *     截断的半帧一律丢弃（否则带着错误位数混进投票）；
 *   - **单帧解码（时长域）**：对齐到第一个高电平，(高,低) 成对判决；
 *       · 码元周期恒定校验：|P - P中位数| 必须 <=20%。
 *         低电平段丢失(切片器门限偏高/AGC 拉满)会让前后高段合并：丢 1T 低 -> P=7T(+75%)，
 *         丢 3T 低 -> P=5T(+25%)。容差必须 <25% 才能把后一档也拦住，否则会解出
 *         "少一位的移位假码"并骗过多帧投票。锚定中位数而非首符号，避免首符号自身抖动。
 *       · 0/1 判决：高/低 时长比 >=1.25 判 1，反之判 0，介于其间即停（无法判决）。
 *   - **各帧同步间隙一致性**（借鉴 rc-switch 的 diff(duration,timings[0])<200）：
 *     真帧的前导同步间隙彼此接近；噪声凑出来的"伪帧"间隙长度五花八门，据此剔出干净集。
 *   - **多帧一致性投票**：仅用"干净帧"（整段配对完 + 周期恒定 + 间隙一致），
 *     取最长位数组那一组的众数，永不降级到更短的截断残帧。
 *     repeat<2 时判定"未识别"，**绝不输出未复核的码值**（旧版会把单帧当结果输出，
 *     正是 315 遥控 A9D661 偶尔显示成 7BFBFF 的来历）。
 *   - **协议判型 + 由码元周期反推 Te**（借鉴 rc-switch 协议表 / 由同步间隙反推 Te）：
 *     用模板表拟合 (高,低) 形态，Te = 码元周期 / 模板周期。这比脉宽直方图准得多，
 *     因为码元周期是在已判定为干净的帧上量的，完全不受噪声段影响。
 *   - **正相/反相两种极性假设**：rc-switch 12 个协议里有 4 个是 inverted
 *     (HT6P20B/HT12E/SM5212/1ByOne)，只试正相会整类解不出。两种假设各跑一遍取优。
 *
 * 采集：ESP32 GPIO9 上挂 CHANGE 中断，micros() 记录边沿时长。
 *   前提：CMT2300A 的 OOK 解调区(0x20-0x37)必须已在 Init 加载（见 cmt2300a.cpp）。
 *   该区缺失时芯片用复位默认门限解调，DOUT 输出的是底噪包络，这里再怎么算也解不出码。
 *   载波消失后 AGC 拉满，DOUT 变随机噪声并迅速灌满缓冲，故采用"采满即试解、
 *   解不出就清空重采"的多轮策略（见 ook_decoder_run）。
 */
#include "ook_decoder.h"
#include "cmt2300a.h"

#include <Arduino.h>
#include <driver/gpio.h>
#include <string.h>

/* ============ 采集缓冲（ISR 与消费端共享） ============ */
static int              s_pin = OOK_DOUT_PIN;
static volatile uint32_t g_dur[OOK_MAX_PULSES];  /* 每段时长 us */
static volatile uint8_t  g_lvl[OOK_MAX_PULSES];  /* 该段自身的电平(0/1)，见 ISR 注释 */
static volatile uint16_t g_head = 0;             /* 已写入段数 */
static volatile uint32_t g_last = 0;             /* 上次跳变时间戳 us */
static volatile bool     g_cap  = false;         /* 采集使能 */
static volatile uint16_t g_gaps = 0;             /* 已出现的帧间隙数（>=OOK_GAP_US 的段） */

/* 解码工作副本：从 g_* 快照过来，非 volatile，可原地压缩（毛刺合并） */
static uint32_t w_dur[OOK_MAX_PULSES];
static uint8_t  w_lvl[OOK_MAX_PULSES];

/* CHANGE 中断：记录"刚刚结束的那一段"的时长与电平。
   顺带 O(1) 统计帧间隙数，供采集循环判断"已抓够几帧"（不能在主循环里扫全缓冲）。

   ★ 电平必须取反 ★
   进中断时 dur = now - g_last，描述的是**刚结束**的那一段；而此刻 gpio_get_level()
   读到的已经是跳变**之后**的新电平，即刚结束那段电平的反相。二值信号下两者恒相反，
   所以要存 lvl^1，否则整个 g_lvl[] 相对 g_dur[] 错开一段。
   历史 bug：曾直接存 lvl，导致
     a) (高,低) 被整体互换 -> 解出的码是真值的逐位取反
        （实测 315MHz 遥控真值 0xA9D661 被解成 0x56299E，正好按位求反）；
     b) "对齐到第一个高电平"实际对齐到了低电平，配对跨越比特边界，
        相邻位不同时立刻停止 -> 每帧位数 <8 被丢弃 -> frames=0。
   两个症状同源，此处一处修复即全好。 */
static void IRAM_ATTR ook_isr(void)
{
    uint32_t now = micros();
    uint8_t  lvl = (uint8_t)gpio_get_level((gpio_num_t)s_pin);
    uint32_t dur = now - g_last;
    g_last = now;
    if (g_cap && g_head < OOK_MAX_PULSES) {
        g_dur[g_head] = dur;
        g_lvl[g_head] = (uint8_t)(lvl ^ 1);      /* 存刚结束那一段自身的电平 */
        g_head++;
        if (dur >= OOK_GAP_US) g_gaps++;
    }
}

bool ook_decoder_init(int gpio_pin)
{
    s_pin = gpio_pin;
    pinMode(s_pin, INPUT);
    g_head = 0;
    g_cap  = false;
    attachInterrupt(digitalPinToInterrupt(s_pin), ook_isr, CHANGE);
    return true;
}

/* ============ 协议模板表 ============
   源自 rc-switch 的 12 个协议，按 (bit0 形态, bit1 形态) 去重，只保留"码元周期恒定"的形态。
   rc-switch 的 inverted 标志不进表：反相由外层的极性假设统一处理。
   未收录 #8/#9 (Conrad RS-200 {7,16}/{3,16})：其 0/1 周期不等(23T vs 19T)，
   与"周期恒定"这一核心防护冲突，且该型号极罕见，收录反而会放宽防护。 */
typedef struct {
    uint8_t     h0, l0;      /* bit0 = (h0 单位高, l0 单位低) */
    uint8_t     h1, l1;      /* bit1 */
    uint8_t     period;      /* 码元周期（单位数）= h0+l0 = h1+l1 */
    const char *name;
} ook_tmpl_t;

static const ook_tmpl_t TMPL[] = {
    { 1,  3, 3, 1,  4, "EV1527/PT2262" },  /* rc-switch #1(350us) / #4(380us) / #10 */
    { 1,  2, 2, 1,  3, "PWM 1:2"       },  /* rc-switch #2 #5 #6 #11 #12 */
    { 1,  6, 6, 1,  7, "HS2303-PT"     },  /* rc-switch #7 */
    { 4, 11, 9, 6, 15, "Conrad RS-200" },  /* rc-switch #3 */
};
#define TMPL_N   ((int)(sizeof(TMPL) / sizeof(TMPL[0])))

#define OOK_RATIO_NUM     5      /* 0/1 判决门限 = 5/4 = 1.25 倍 */
#define OOK_RATIO_DEN     4
#define OOK_PGUARD_PCT    20     /* 码元周期容差 %，必须 <25（见文件头注释） */
#define OOK_GAPCONS_PCT   35     /* 各帧同步间隙一致性容差 % */
#define OOK_TMPL_TOL_PCT  12     /* 模板拟合归一化误差上限 % */
#define OOK_TE_XVAL_PCT   40     /* 借鉴①：间隙反推 Te 与拟合/模板 Te 交叉校验容差 % */
#define MAX_FRAMES        32
#define MAX_BITS          64
#define MAX_SAMPLES       64

/* ============ 工具 ============ */

static void code_to_hex(uint64_t code, int bits, char *hx, int hxsz)
{
    int hn = 0;
    if (bits <= 0) { hx[0] = 0; return; }
    for (int shift = ((bits - 1) / 4) * 4; shift >= 0 && hn < hxsz - 1; shift -= 4)
        hx[hn++] = "0123456789ABCDEF"[(int)((code >> shift) & 0xF)];
    hx[hn] = 0;
}

static void code_to_bin(uint64_t code, int bits, char *bin, int binsz)
{
    int p = 0;
    for (int b = bits - 1; b >= 0 && p < binsz - 1; b--)
        bin[p++] = ((code >> b) & 1) ? '1' : '0';
    bin[p] = 0;
}

static inline uint32_t absd(uint32_t a, uint32_t b) { return (a > b) ? (a - b) : (b - a); }

/* 无符号数组插入排序取中位数（n <= MAX_BITS，规模很小） */
static uint32_t median_u32(uint32_t *v, int n)
{
    for (int a = 1; a < n; a++) {
        uint32_t x = v[a]; int b = a - 1;
        while (b >= 0 && v[b] > x) { v[b + 1] = v[b]; b--; }
        v[b + 1] = x;
    }
    return v[n / 2];
}

/* ============ 粗估 Te（只用于毛刺阈值与帧间隙阈值，容许 ±30% 误差） ============
   三个必须避开的陷阱：
   1) 分箱不能按 [0,max] 均分：帧间同步间隙有上万 us，会把箱宽拉到上百 us；
      固定 10us 箱宽、只在 0-2560us 内找峰，结果与 max 无关。
   2) 不能直接取最高峰：EV1527 每码元恰含一个 1T 和一个 3T，最高峰有一半概率落在 3T 上。
   3) 也不能取"最小的显著箱"：切片器毛刺同样成簇（几十 us）。
   方案：枚举显著箱作候选，用"全体脉宽能否整数倍拟合"打分取优。 */
#define TE_BIN_US   10
#define TE_BIN_N    256
#define TE_CAND_MAX 12

static uint32_t refine_te(uint16_t n, const uint32_t *dur, uint32_t approx)
{
    uint32_t lo = approx * 6 / 10, hi = approx * 14 / 10;
    uint32_t sum = 0; uint16_t m = 0;
    for (uint16_t i = 0; i < n; i++)
        if (dur[i] >= lo && dur[i] <= hi) { sum += dur[i]; m++; }
    return m ? (sum / m) : approx;
}

static int32_t te_score(uint16_t n, const uint32_t *dur, uint32_t te)
{
    if (te < OOK_TE_MIN_US) return -32768;
    int32_t sc = 0;
    uint32_t tol = te * 3 / 10;
    for (uint16_t i = 0; i < n; i++) {
        uint32_t d = dur[i];
        if (d < OOK_GLITCH_MIN_US) continue;
        uint32_t q = (d + te / 2) / te;
        if (q >= 8) continue;                        /* 同步间隙：与 Te 无关，中性 */
        if (q < 1) { sc -= 2; continue; }
        sc += (absd(d, q * te) <= tol && q <= 4) ? 2 : -1;
    }
    return sc;
}

static uint32_t estimate_te(uint16_t n, const uint32_t *dur)
{
    static uint16_t bins[TE_BIN_N];
    memset(bins, 0, sizeof(bins));
    uint16_t used = 0;
    for (uint16_t i = 0; i < n; i++) {
        if (dur[i] < OOK_GLITCH_MIN_US) continue;
        uint32_t b = dur[i] / TE_BIN_US;
        if (b < TE_BIN_N) { bins[b]++; used++; }
    }
    if (used < 8) return 0;

    uint16_t pc = 0;
    for (int b = 0; b < TE_BIN_N; b++) if (bins[b] > pc) pc = bins[b];
    if (pc == 0) return 0;

    uint32_t bestTe = 0; int32_t bestSc = -0x7FFFFFFF; int cand = 0;
    for (int b = 0; b < TE_BIN_N && cand < TE_CAND_MAX; b++) {
        if ((uint32_t)bins[b] * 4 < (uint32_t)pc) continue;
        uint32_t approx = (uint32_t)b * TE_BIN_US + TE_BIN_US / 2;
        if (approx < OOK_TE_MIN_US) continue;
        cand++;
        uint32_t te = refine_te(n, dur, approx);
        if (te < OOK_TE_MIN_US || te > 2000) continue;
        int32_t sc = te_score(n, dur, te);
        if (sc > bestSc) { bestSc = sc; bestTe = te; }
    }
    return bestTe;
}

/* 毛刺合并（原地压缩，返回新段数）。
   一个宽度 <thr 的窄段意味着电平翻上去又立刻翻回来，真实信号里那本是连续的一段。
   把 [前一段 + 毛刺 + 后一段] 三段合并为一段（电平沿用前一段）。 */
static uint16_t deglitch(uint16_t n, uint32_t *dur, uint8_t *lvl, uint32_t thr)
{
    uint16_t w = 0;
    for (uint16_t i = 0; i < n; i++) {
        if (dur[i] < thr) {
            if (w > 0 && i + 1 < n) {
                dur[w - 1] += dur[i] + dur[i + 1];   /* 吸收毛刺与其后同电平段 */
                i++;
                continue;
            }
            continue;                                 /* 缓冲首尾的毛刺直接丢弃 */
        }
        dur[w] = dur[i];                              /* w<=i 恒成立，原地安全 */
        lvl[w] = lvl[i];
        w++;
    }
    return w;
}

/* ============ 单帧解码（时长域） ============
   区间 [s,e)，对齐到第一个 hiLvl 段，(高,低) 成对判 0/1。
   hiLvl=1 正相；hiLvl=0 反相（HT6P20B/HT12E 这类 rc-switch inverted 协议）。
   *outClean = 整个区间都被成功配对消费完（尾部至多剩 1 段 = 下帧的同步高电平）。 */
static int decode_one_frame(uint16_t s, uint16_t e, const uint32_t *dur, const uint8_t *lvl,
                            uint8_t hiLvl, uint64_t *outCode, bool *outClean,
                            uint32_t *outSumP, uint16_t *outNp, uint16_t *outStart)
{
    *outCode = 0; *outClean = false; *outSumP = 0; *outNp = 0;
    while (s < e && lvl[s] != hiLvl) s++;
    *outStart = s;      /* 对齐后的起点：模板样本必须从这里采，否则首段非高电平时取不到样本 */

    /* 第一遍：量出本帧码元周期的中位数。
       不能锚定首符号——首符号自身的抖动会整体抬高/压低门限，
       与 20% 容差叠加后会误杀正常帧或放过 +25% 的"丢 3T 低段"合并。 */
    static uint32_t ps[MAX_BITS];
    int np = 0;
    for (uint16_t j = s; j + 1 < e && np < MAX_BITS; j += 2) {
        if (lvl[j] != hiLvl) break;
        ps[np++] = dur[j] + dur[j + 1];
    }
    if (np == 0) return 0;
    uint32_t pmed = median_u32(ps, np);
    if (pmed == 0) return 0;

    uint64_t code = 0; int bits = 0; uint32_t sumP = 0; uint16_t cnt = 0;
    uint16_t i = s;
    for (; i + 1 < e; i += 2) {
        if (lvl[i] != hiLvl) break;                   /* 相位错乱，停止 */
        uint32_t hi = dur[i], lo = dur[i + 1], P = hi + lo;
        /* ★关键防护★ 码元周期恒定校验（详见文件头） */
        if (absd(P, pmed) * 100 > pmed * OOK_PGUARD_PCT) break;
        int bit;
        /* 极性已用实物核对：315MHz 遥控真值 0xA9D661(=11130465)、
           433MHz 遥控真值 0x571154(=5706068)，均为长高+短低=1、短高+长低=0，
           与 EV1527 标准一致。切勿再翻转。 */
        if      (hi * OOK_RATIO_DEN >= lo * OOK_RATIO_NUM) bit = 1;
        else if (lo * OOK_RATIO_DEN >= hi * OOK_RATIO_NUM) bit = 0;
        else break;                                   /* 高低太接近，无法判决 */
        if (bits < 63) code = (code << 1) | (uint64_t)bit;
        bits++; sumP += P; cnt++;
    }
    *outCode = code; *outClean = (i + 1 >= e); *outSumP = sumP; *outNp = cnt;
    return bits;
}

/* ============ 协议判型 + 由码元周期反推 Te ============
   对每个模板：unit = 码元周期 / 模板周期，再算所有 (高,低) 样本对模板的偏差和。
   ★误差必须按 meanP 归一，不能按 unit 归一★：后者会系统性偏袒周期小(unit 大)的模板，
   把 {4,11}/{9,6}(周期15) 误判成 1:2(周期3)。仿真里这个 bug 让 Conrad 全被叫成 PWM 1:2。
   返回模板下标（-1 = 都不匹配，按通用 PWM 处理），*outTe 写入反推出的单位脉宽。 */
static int pick_tmpl(uint32_t meanP, const uint32_t *hiS, const uint32_t *loS, int ns,
                     uint32_t *outTe)
{
    *outTe = 0;
    if (ns <= 0 || meanP == 0) return -1;
    int best = -1; uint64_t bestTot = (uint64_t)-1; uint32_t bestUnit16 = 0;
    for (int t = 0; t < TMPL_N; t++) {
        uint32_t u16 = meanP * 16u / TMPL[t].period;          /* unit 的 16 倍定点 */
        if (u16 < (uint32_t)OOK_TE_MIN_US * 16u) continue;
        uint64_t tot = 0;
        for (int i = 0; i < ns; i++) {
            uint32_t h = hiS[i] * 16u, l = loS[i] * 16u;
            uint32_t e0 = absd(h, TMPL[t].h0 * u16) + absd(l, TMPL[t].l0 * u16);
            uint32_t e1 = absd(h, TMPL[t].h1 * u16) + absd(l, TMPL[t].l1 * u16);
            tot += (e0 < e1) ? e0 : e1;
        }
        if (tot < bestTot) { bestTot = tot; best = t; bestUnit16 = u16; }
    }
    if (best < 0) return -1;
    /* 归一化误差 = bestTot / (ns * meanP * 16) 必须 <= OOK_TMPL_TOL_PCT% */
    if (bestTot * 100ull >
        (uint64_t)ns * (uint64_t)meanP * 16ull * (uint64_t)OOK_TMPL_TOL_PCT) return -1;
    *outTe = (bestUnit16 + 8) / 16;
    return best;
}

/* ============ 单个极性假设下的完整尝试 ============ */
typedef struct {
    uint8_t  repeat, frames;
    int      tmpl;
    uint32_t bits, te;
    bool     lowConf;     /* 借鉴①：间隙反推 Te 与拟合/模板 Te 严重不符 -> 低置信 */
    uint64_t code;
    uint64_t cand_code;   /* 跨轮累加候选：即便 repeat<2 也填最佳单帧码值 */
    uint32_t cand_bits;
    int      cand_tmpl;
} try_res_t;

static bool try_polarity(uint16_t n, const uint32_t *dur, const uint8_t *lvl,
                         uint32_t gapmin, uint8_t hiLvl, uint32_t te0, try_res_t *r)
{
    static uint64_t fcode[MAX_FRAMES];
    static int      fbits[MAX_FRAMES];
    static bool     fclean[MAX_FRAMES];
    static uint32_t fgap[MAX_FRAMES], fsum[MAX_FRAMES];
    static uint16_t fnp[MAX_FRAMES], fs[MAX_FRAMES], fe[MAX_FRAMES];

    r->repeat = 0; r->frames = 0; r->bits = 0; r->code = 0; r->tmpl = -1; r->te = 0; r->lowConf = false; r->cand_code = 0; r->cand_bits = 0; r->cand_tmpl = -1;

    int nf = 0;
    for (uint16_t i = 0; i < n && nf < MAX_FRAMES; ) {
        if (dur[i] < gapmin) { i++; continue; }
        uint32_t gap = dur[i];
        uint16_t s = (uint16_t)(i + 1), e = s;
        while (e < n && dur[e] < gapmin) e++;
        if (e >= n) break;                     /* 尾部半帧：没等到下一个同步，丢弃 */
        if (e > s) {
            uint64_t c; bool cl; uint32_t sp; uint16_t npr, s0;
            int b = decode_one_frame(s, e, dur, lvl, hiLvl, &c, &cl, &sp, &npr, &s0);
            if (b >= 8) {
                fcode[nf] = c; fbits[nf] = b; fclean[nf] = cl;
                fgap[nf] = gap; fsum[nf] = sp; fnp[nf] = npr;
                fs[nf] = s0; fe[nf] = e; nf++;      /* 存对齐后的起点 */
            }
        }
        i = e;
    }
    r->frames = (uint8_t)nf;
    if (nf == 0) return false;

    /* 借鉴点②：各帧同步间隙一致性。真帧的前导间隙彼此接近；
       噪声凑出来的"伪帧"间隙长度五花八门，直接剔出干净集。 */
    static uint32_t gsort[MAX_FRAMES];
    for (int a = 0; a < nf; a++) gsort[a] = fgap[a];
    uint32_t gmed = median_u32(gsort, nf);
    if (gmed) {
        for (int a = 0; a < nf; a++)
            if (absd(fgap[a], gmed) * 100 > gmed * OOK_GAPCONS_PCT) fclean[a] = false;
    }

    /* 仅干净帧、最长位数组、取众数，永不降级到更短的截断残帧 */
    int maxBits = 0;
    for (int a = 0; a < nf; a++)
        if (fclean[a] && fbits[a] > maxBits) maxBits = fbits[a];
    if (maxBits == 0) return false;

    int bestIdx = -1, bestCnt = 0;
    for (int a = 0; a < nf; a++) {
        if (!fclean[a] || fbits[a] != maxBits) continue;
        int c = 0;
        for (int b = 0; b < nf; b++)
            if (fclean[b] && fbits[b] == maxBits && fcode[b] == fcode[a]) c++;
        if (c > bestCnt) { bestCnt = c; bestIdx = a; }
    }
    if (bestIdx < 0) return false;

    /* 借鉴点①③：用胜出帧的实际波形做模板判型，并由码元周期反推 Te */
    static uint32_t hiS[MAX_SAMPLES], loS[MAX_SAMPLES];
    int ns = 0;
    for (int a = 0; a < nf && ns < MAX_SAMPLES; a++) {
        if (!fclean[a] || fbits[a] != maxBits || fcode[a] != fcode[bestIdx]) continue;
        uint16_t lim = (uint16_t)(fs[a] + 2 * maxBits);
        for (uint16_t j = fs[a]; j < lim && j + 1 < fe[a] && ns < MAX_SAMPLES; j += 2) {
            if (lvl[j] != hiLvl) break;
            hiS[ns] = dur[j]; loS[ns] = dur[j + 1]; ns++;
        }
    }
    uint32_t meanP = fnp[bestIdx] ? (fsum[bestIdx] / fnp[bestIdx]) : 0;
    uint32_t te = 0;
    int tm = pick_tmpl(meanP, hiS, loS, ns, &te);
    if (tm < 0) te = (meanP + 2) / 4;      /* 判不出型：按最常见的 4T 码元估个 Te 供显示 */

    r->repeat = (uint8_t)bestCnt;
    r->bits   = (uint32_t)maxBits;
    r->code   = fcode[bestIdx];
    r->cand_code = fcode[bestIdx];   /* 跨轮累加候选：最佳单帧码值，repeat<2 也保留 */
    r->cand_bits = (uint32_t)maxBits;
    r->cand_tmpl = tm;
    r->tmpl   = tm;
    r->te     = te;

    /* 借鉴①：由同步间隙反推 Te（EV1527/PT2262 同步=31T，间隙最稳、最抗段丢失），
       并与拟合值 te0、模板反推 te 交叉校验；三者严重不符 -> 波形不规整，降级低置信。
       仅对 tm==0（EV1527/PT2262 族，sync 确为 31T）启用；其它协议 sync 长度未知，不动。 */
    uint32_t te_gap = (gmed && tm == 0) ? (gmed / 31) : 0;
    if (te_gap) {
        bool xok = false;
        if (te0 && absd(te_gap, te0) * 100 <= ((te_gap > te0) ? te_gap : te0) * OOK_TE_XVAL_PCT) xok = true;
        if (te  && absd(te_gap, te ) * 100 <= ((te_gap > te ) ? te_gap : te ) * OOK_TE_XVAL_PCT) xok = true;
        if (!xok) r->lowConf = true;
        else      r->te = te_gap;          /* 间隙反推更准，覆盖模板 te 供显示 */
    }
    return true;
}

/* ============ 主解码 ============
   返回 true = 解出了经多帧复核的码值（out->hex 有效）。
   返回 false 时 out->te/frames/repeat/glitch 仍已填好，供前端显示诊断信息。 */
static bool decode_signal(uint16_t n, uint32_t *dur, uint8_t *lvl, decode_result_t *out)
{
    /* 0) 先粗估 Te 定毛刺阈值 -> 合并毛刺。阈值取 Te/3 并夹在 [40,150]us。 */
    uint32_t te0 = estimate_te(n, dur);
    uint32_t thr = te0 ? (te0 / 3) : OOK_GLITCH_MIN_US;
    if (thr < OOK_GLITCH_MIN_US) thr = OOK_GLITCH_MIN_US;
    if (thr > OOK_GLITCH_MAX_US) thr = OOK_GLITCH_MAX_US;
    uint16_t n2 = deglitch(n, dur, lvl, thr);
    out->glitch = (uint16_t)(n - n2);
    n = n2;
    if (n < 8) { out->te = te0; return false; }

    te0 = estimate_te(n, dur);
    out->te = te0;

    /* 1) 帧间隙阈值：只需"远大于数据段、远小于同步间隙"。
          数据段最长约 6T、EV1527 同步 31T，取 8×Te0 有 30% 以上余量，
          所以 te0 估偏也不影响切帧（这正是把判决搬出量化域的收益）。 */
    uint32_t gapmin = te0 ? te0 * 8 : 2500;
    if (gapmin < 1800)  gapmin = 1800;
    if (gapmin > 20000) gapmin = 20000;

    /* 2) 正相 / 反相 两种极性假设各跑一遍，按 (一致帧数, 位数) 取优 */
    try_res_t ra, rb;
    bool oka = try_polarity(n, dur, lvl, gapmin, 1, te0, &ra);
    bool okb = try_polarity(n, dur, lvl, gapmin, 0, te0, &rb);

    try_res_t *best = NULL;
    if (oka) best = &ra;
    if (okb && (!best ||
                rb.repeat * 1000u + rb.bits > best->repeat * 1000u + best->bits))
        best = &rb;

    /* frames 取两种假设中切出帧数更多的一个，纯诊断用 */
    uint8_t fa = ra.frames, fb = rb.frames;
    out->frames = (fa > fb) ? fa : fb;

    if (!best) {
        snprintf(out->proto_name, sizeof(out->proto_name), "%s", "PWM-OOK(未识别)");
        return false;
    }

    out->frames = best->frames;
    out->repeat = best->repeat;
    if (best->te) out->te = best->te;
    out->cand_code = best->cand_code;   /* 跨轮累加候选：即便 repeat<2 也填，供 ook_decoder_run 计数 */
    out->cand_bits = best->cand_bits;
    out->cand_tmpl = best->cand_tmpl;

    if (best->repeat < 2 || best->lowConf) { /* 未复核 或 Te 反推冲突 -> 绝不输出码值 */
        snprintf(out->proto_name, sizeof(out->proto_name), "%s", "PWM-OOK(未识别)");
        return false;
    }

    out->bits = best->bits;
    code_to_hex(best->code, (int)best->bits, out->hex, sizeof(out->hex));
    code_to_bin(best->code, (int)best->bits, out->bin, sizeof(out->bin));
    snprintf(out->proto_name, sizeof(out->proto_name), "%s",
             (best->tmpl >= 0) ? TMPL[best->tmpl].name : "通用PWM-OOK");
    return true;
}

/* 原始脉宽诊断串（尽可能多的段，us），便于人工判读 */
static void build_raw(const uint32_t *dur, uint16_t n, char *raw, int rawsz)
{
    int p = 0;
    raw[0] = 0;
    for (uint16_t i = 0; i < n && p < rawsz - 8; i++)
        p += snprintf(raw + p, rawsz - p, "%s%lu", (i ? "," : ""), (unsigned long)dur[i]);
    raw[rawsz - 1] = 0;
}

/* ============ 对外：阻塞式采集+解码（多轮重采） ============ */
bool ook_decoder_run(float freq_mhz, uint32_t timeout_ms, decode_result_t *out)
{
    memset(out, 0, sizeof(*out));
    out->freq  = freq_mhz;
    out->proto = -2;                       /* -2 = 无数据 */

    /* 1) 锁频到目标频点并进入 RX：DIRECT 模式下 GPIO2(DOUT) 持续输出解调数据 */
    CMT2300A_GoStby();
    CMT2300A_SetFrequency((uint32_t)(freq_mhz * 1e6f));
    CMT2300A_ClearInterruptFlags();
    CMT2300A_GoRx();

    uint32_t tStart = millis();
    bool     solved = false;
    uint8_t  tries  = 0;

    /* 保底快照：所有轮次都没解出时，挑"切出帧数最多"的那轮回传，
       其原始脉宽最可能含真信号，供人工判读。 */
    static decode_result_t bestSnap;
    bool haveSnap = false;

    /* 跨轮 repeat 累加状态（局部即可：每次调用自动归零）。
       单轮 repeat<2 的"最佳单帧候选"若跨轮同 code+位宽+模板 出现 >=2 次，
       视为与单轮 repeat>=2 同等可信——两轮独立捕获都解出同一 24 位码才输出，
       噪声几乎不可能跨轮复现同一码，安全性不降级。 */
    uint64_t acc_code = 0;
    uint32_t acc_bits = 0;
    int      acc_tmpl = -1;
    uint8_t  acc_rep  = 0;
    uint32_t acc_te   = 0;

    while (millis() - tStart < timeout_ms) {
        if (tries >= 200) break;               /* 防御：tries 为 uint8_t，绝不让它回绕 */
        tries++;

        /* 2) 采集一轮。
           关键：不能像最早的版本那样"一遇到静默就停"——那个静默正是 EV1527 的 31T
           同步间隙，在那里收工只会得到从随机时刻起截取的半帧。这里一直采到抓够
           OOK_WANT_GAPS 个帧边界（即多个完整帧），交给解码端投票。 */
        g_head = 0;
        g_gaps = 0;
        g_last = micros();
        g_cap  = true;

        uint32_t t0 = millis();
        uint32_t budget = timeout_ms - (millis() - tStart);
        while (true) {
            if (millis() - t0 >= budget)      break;   /* 总窗口用尽 */
            if (g_head >= OOK_MAX_PULSES)     break;   /* 缓冲满（多半是噪声灌的） */
            if (g_gaps >= OOK_WANT_GAPS)      break;   /* 已抓够完整帧 */
            if (micros() - g_last > OOK_NOSIG_US) break; /* 长时间无边沿 = 没信号 */
            delayMicroseconds(150);
        }
        g_cap = false;

        uint16_t n = g_head;

        /* 3) 快照到工作副本后再解码：解码会原地压缩数组，不能动 ISR 缓冲 */
        for (uint16_t i = 0; i < n; i++) { w_dur[i] = g_dur[i]; w_lvl[i] = g_lvl[i]; }

        /* static：decode_result_t 约 640B，scan_task 栈只有几 KB，
           而解码全程单线程串行执行，放静态区更安全。 */
        static decode_result_t cur;
        memset(&cur, 0, sizeof(cur));
        cur.freq   = freq_mhz;
        cur.pulses = n;
        build_raw(w_dur, n, cur.raw, sizeof(cur.raw));   /* raw 用合并毛刺前的原始值 */

        if (n >= 8 && decode_signal(n, w_dur, w_lvl, &cur)) {
            cur.proto = 1;
            *out = cur;
            solved = true;
            break;
        }

        /* 跨轮 repeat 安全累加：本轮即便 repeat<2，其最佳单帧候选已写入 cur.cand_*。
           与历史同 code+位宽+模板 则计数+1；异码则重置为当前候选。累计 >=2 即输出。 */
        if (cur.cand_code && cur.cand_bits && cur.cand_tmpl >= 0) {
            if (acc_rep == 0) {
                acc_code = cur.cand_code; acc_bits = cur.cand_bits;
                acc_tmpl = cur.cand_tmpl; acc_te = cur.te; acc_rep = 1;
            } else if (acc_code == cur.cand_code && acc_bits == cur.cand_bits
                       && acc_tmpl == cur.cand_tmpl) {
                acc_rep++;
            } else {
                acc_code = cur.cand_code; acc_bits = cur.cand_bits;
                acc_tmpl = cur.cand_tmpl; acc_te = cur.te; acc_rep = 1;
            }
            if (acc_rep >= 2) {
                memset(out, 0, sizeof(*out));
                out->freq   = freq_mhz;
                out->proto  = 1;
                out->bits   = acc_bits;
                out->te     = acc_te;
                out->repeat = acc_rep;
                out->frames = acc_rep;
                out->pulses = n;
                out->ms     = millis() - tStart;
                out->tries  = tries;
                code_to_hex(acc_code, (int)acc_bits, out->hex, sizeof(out->hex));
                code_to_bin(acc_code, (int)acc_bits, out->bin, sizeof(out->bin));
                snprintf(out->proto_name, sizeof(out->proto_name), "%s",
                         (acc_tmpl >= 0) ? TMPL[acc_tmpl].name : "通用PWM-OOK");
                solved = true;
                break;
            }
        }

        /* 本轮没解出：留作保底快照（帧数多者优先，其次段数多者） */
        if (!haveSnap || cur.frames > bestSnap.frames ||
            (cur.frames == bestSnap.frames && cur.pulses > bestSnap.pulses)) {
            bestSnap = cur;
            haveSnap = true;
        }

        /* 完全没有边沿 -> 这个频点根本没信号，重采也是白等，立刻收工 */
        if (n < 8) break;

        /* 剩余时间不够再采一轮就别开新轮了，避免半截数据 */
        if (timeout_ms - (millis() - tStart) < OOK_MIN_RETRY_MS) break;
    }

    if (!solved) {
        if (haveSnap) *out = bestSnap;
        out->freq  = freq_mhz;
        out->proto = (out->pulses >= 8) ? -1 : -2;
        if (!out->proto_name[0])
            snprintf(out->proto_name, sizeof(out->proto_name), "%s",
                     (out->pulses >= 8) ? "PWM-OOK(未识别)" : "无信号");
    }
    out->tries = tries;
    out->ms    = millis() - tStart;
    return (out->pulses >= 8);
}
