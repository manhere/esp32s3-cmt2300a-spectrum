/*
 * spectrum.cpp — 扫频引擎实现（移植自 esp32s3-si4463-spectrum，底层射频换为 CMT2300A）
 *
 * 工作原理（与参考工程一致）
 *  1) 把 [start, stop] 切成 N 个 bin，用 CMT2300A 逐点跳频（GoStby->SetFrequency->GoRx），
 *     每个 bin 停留 dwellUs 后读一次 RSSI。一轮扫描 = 一帧频谱。
 *  2) 自适应噪声本底：非对称 EMA（下降快、上升慢），信号出现时本底不会被拉高。
 *  3) 最大保持(Max-Hold)：遥控器只发几毫秒，一次扫描很可能错过；用带衰减的
 *     最大保持把短促突发"抓住"，检测就在 Max-Hold 上做。
 *  4) 触发状态机：Max-Hold 相对本底超过门限 → 进入捕获窗口，窗口内跟踪最强点，
 *     突发结束后用抛物线插值细化峰值频率，再归一化到常见遥控频点。
 *  5) 快嗅探（自动解码）：mode=1 时在预设频点间高速轮转读 RSSI，实时载波命中
 *     即在该频点进 OOK 接收解码（ook_decoder_run 内部自带锁频，见下）。
 *
 * 与参考工程(Si4463)的硬件差异适配：
 *  - Si4463 有信道步进机制可一次 setBaseFreq 扫一段；CMT2300A 无，逐点调谐，
 *    但按 AN197 快速手动跳频：同段(FREQ_DIVX_CODE/FREQ_VCO_BANK 不变)内
 *    CMT2300A_TuneFast 只写 FH_OFFSET/FH_CHANNEL+AFC_OVF_TH(3 寄存器)，
 *    跨段才全量写频率区(9 寄存器)，段内扫描大幅提速。
 *  - CMT2300A 存在不可调谐空洞频段(340-379 / 510-758MHz 等)：落入空洞的 bin
 *    直接置最低值(0x00=-134dBm)，不调谐、不误读相邻频点残留。
 *  - RSSI：CMT2300A_GetRssiDBm() 直接返回 dBm(-128~0)，统一换算回与 Si4463 相同的
 *    0.5dB/LSB raw 单位（dBm = raw/2 - 134），前端 raw2dbm 逻辑完全不变。
 *  - OOK 解码：CMT2300A 已配 DATA_MODE=DIRECT，GPIO2(DOUT) 直出解调码流；本项目
 *    ook_decoder_run() 内部完成 GoStby->SetFrequency->ClearInterruptFlags->GoRx，
 *    无需 enterOokRx/exitOokRx（解码结束下一轮扫描自动重新调谐）。
 */
#include "spectrum.h"
#include "cmt2300a.h"
#include "cmt2300a_hal.h"         // CMT2300A_PIN_GPIO1 = ESP32 GPIO10（本板 TX DIN 输出，飞线接 CMT GPIO3）
#include "ook_decoder.h"           // OOK 码流采集 + 时长域解码（本项目自带）
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <stdlib.h>                // strtoul（解码结果 hex -> code）
#include <math.h>
#include "driver/rmt.h"            // RMT 老 API（IDF v4.4.7）：OOK 波形硬件生成，替代 digitalWrite bit-bang
#include "driver/gpio.h"           // gpio_reset_pin（清掉 RX 模式 INPUT_PULLUP 残留，供 RMT 夺回输出）

namespace Spectrum {

static ScanSettings  g_set;
static ScanSettings  g_pending;
static volatile bool g_pendingFlag = false;

static uint8_t  g_live[MAX_BINS];        // 本轮实测
static float    g_hold[MAX_BINS];        // 最大保持 (raw 单位, 0.5dB/LSB)
static float    g_base[MAX_BINS];        // 噪声本底 (raw)
static bool     g_baseInit = false;

static uint8_t  g_pubLive[MAX_BINS];     // 发布缓冲
static uint8_t  g_pubHold[MAX_BINS];
static uint16_t g_pubBins = 0;
static uint32_t g_pubStart = 0, g_pubStep = 0;
static uint16_t g_pubSeq = 0;
static volatile bool g_frameReady = false;

static uint16_t g_bins = 0;
static float    g_sps  = 0;
static float    g_lastPeakDbm = -140, g_lastFloorDbm = -140;
static bool     g_demo = false;

// 预设模式：每个 bin 对应的实际频率 (Hz)；由 calcBins 填充
static uint32_t g_freqs[MAX_BINS];
static bool     g_preset = false;

// 快嗅探：每个候选频点的滑动本底基线(RSSI raw)，用于"瞬时高于本底 N dB"触发判定
static float    g_sniffBase[MAX_BINS];
static bool     g_sniffBaseInit = false;

// 发布用频率表（与 g_pub* 同生命周期，受互斥量保护）
static uint32_t g_pubFreqs[MAX_BINS];
static uint16_t g_pubFreqN = 0;

// 回放发射请求（Web 命令 -> scanTask 消费）
static volatile bool   g_replayReq = false;
static uint32_t g_replayFreqHz = 0;
static uint32_t g_replayCode  = 0;
static uint32_t g_replayTe    = 0;
static uint8_t  g_replayBits  = 0;

static SemaphoreHandle_t g_mux = nullptr;
static QueueHandle_t     g_hitQ = nullptr;
static Preferences       g_prefs;

/* ========== OOK 信号 RMT 硬件生成（移植自 tuya-test，已 PulseView 验证） ==========
 * 编码模型参照 esp-idf-rcswitch-rmt 的 pulse-pair codec（每比特 = {高,低} 时长对 + unit），
 * 但基于 Arduino 2.0.x / ESP-IDF v4.4.7 老 RMT API（driver/rmt.h：rmt_config /
 * rmt_driver_install / rmt_write_items）与 rmt_item32_t。时序由 RMT 计数器硬件产生，
 * 不受中断/调度抖动影响；rmt_write_items(wait=true) 内部阻塞等发完，看门狗任务照常跑，
 * 根除「整帧关中断超阈值触发 IWDT」崩溃。与 tuya-test 的发射波形逐位一致。 */
static const rmt_channel_t OOK_RMT_CH   = RMT_CHANNEL_0;
static const gpio_num_t    OOK_RMT_GPIO = (gpio_num_t)CMT2300A_PIN_GPIO1;  /* RMT API 要 gpio_num_t 枚举 */
static bool                g_rmt_installed = false;

/* 工作缓冲：一帧 ~25 item、唤醒 ~71 item、静默可拆多 item，留足余量 */
static rmt_item32_t g_items[256];

/* ---- OOK 协议：每比特 = {高 unit 数, 低 unit 数}，unit = te(us) ---- */
typedef struct { uint8_t high; uint8_t low; } OokPulsePair;   /* 单位：te */
typedef struct {
    OokPulsePair one;       /* bit=1: 高3·te / 低1·te */
    OokPulsePair zero;      /* bit=0: 高1·te / 低3·te */
    OokPulsePair sync;      /* 帧尾同步间隔：高1·te / 低31·te（等价旧 31te 同步静默） */
    bool         inverted;  /* false => 有效电平为 HIGH */
} OokProtocol;

static const OokProtocol kOokProtocol = {
    .one  = { .high = 3, .low = 1 },
    .zero = { .high = 1, .low = 3 },
    .sync = { .high = 1, .low = 31 },
    .inverted = false,
};

/* 把一段电平(us)编码成若干 rmt_item32_t（>32767us 自动拆分）。level: 1=高 / 0=低。 */
static size_t ook_build_level(rmt_item32_t *items, size_t capacity, uint8_t level, uint32_t us)
{
    size_t n = 0;
    while (us > 0 && n < capacity) {
        uint32_t d = (us > 32767U) ? 32767U : us;
        items[n].duration0 = (uint16_t)d;
        items[n].level0    = level;
        items[n].duration1 = 0;       /* 单段保持，下一段接续同电平 -> 连续 */
        items[n].level1    = level;
        us -= d;
        n++;
    }
    return n;
}

/* 把一个 {high,low} 时长对（单位 te）编码成 1 个 rmt_item32_t。返回 false 表示时长越界。 */
static bool ook_make_symbol(OokPulsePair pair, uint16_t unit_us, bool inverted, rmt_item32_t *item)
{
    if (item == NULL) return false;
    const uint32_t first  = (uint32_t)pair.high * unit_us;
    const uint32_t second = (uint32_t)pair.low  * unit_us;
    if (first == 0 || second == 0 || first > 32767U || second > 32767U) return false;
    item->duration0 = (uint16_t)first;
    item->level0    = inverted ? 0U : 1U;    /* 高 */
    item->duration1 = (uint16_t)second;
    item->level1    = inverted ? 1U : 0U;    /* 低 */
    return true;
}

/* 把一帧（code 的 bits 个 MSB 优先位 + 末尾 sync）编码进 items，返回 item 数。 */
static bool ook_build_frame(uint32_t code, uint8_t bits, const OokProtocol *proto, uint16_t unit_us,
                            rmt_item32_t *items, size_t capacity, size_t *count)
{
    if (items == NULL || proto == NULL || count == NULL || capacity < (size_t)bits + 1U) return false;
    size_t n = 0;
    for (int b = (int)bits - 1; b >= 0; b--) {
        bool one = (code >> b) & 1U;
        if (!ook_make_symbol(one ? proto->one : proto->zero, unit_us, proto->inverted, &items[n])) return false;
        n++;
    }
    if (!ook_make_symbol(proto->sync, unit_us, proto->inverted, &items[n])) return false;   /* 帧尾同步间隔 */
    n++;
    *count = n;
    return true;
}

/* 发送 items[0..count) 并阻塞到完成（wait=true 内部等 RMT 发完，不占中断）。 */
static void ookRmtWrite(rmt_item32_t *items, size_t count, uint32_t timeout_ms)
{
    (void)timeout_ms;
    if (count == 0 || !g_rmt_installed) return;
    rmt_write_items(OOK_RMT_CH, items, (uint32_t)count, true);
}

/* (重新)建立/接管 RMT 通道并绑定 GPIO10（ESP32 TX DIN 输出 -> CMT GPIO3）。通道本身只安装一次；TxOokEnd ->
   CMT2300A_Init 会把 GPIO10 改回 INPUT+中断（原接 CMT GPIO1 的空 ISR，现 GPIO10 已改接 GPIO3，ISR 无意义但保留无害），故每次进 TX 用 rmt_set_gpio
   把引脚重新路由回 RMT 输出。空闲电平=LOW -> 不发送时 DIN=0 -> 载波被门控关断。

   注意：RX 模式把 GPIO10 配成了 INPUT_PULLUP+RISING 中断（cmt2300a.cpp:358）。ESP-IDF v5
   的 legacy rmt_set_gpio 未必能干净覆盖该残留，导致引脚仍停在输入/idle -> DIN 恒电平 ->
   恒载波（pulses=1）。故每次进 TX 先 gpio_reset_pin() 清掉残留再 rmt_set_gpio()。 */
static void ookRmtSetupForTx(void)
{
    if (!g_rmt_installed) {
        rmt_config_t cfg = RMT_DEFAULT_CONFIG_TX(OOK_RMT_GPIO, OOK_RMT_CH);
        cfg.clk_div = 80;                           /* APB 80MHz / 80 = 1MHz => 1 tick = 1 us */
        cfg.tx_config.idle_level     = RMT_IDLE_LEVEL_LOW;
        cfg.tx_config.idle_output_en = true;
        cfg.mem_block_num = 2;                       /* 每块 64 item；唤醒 ~71 item 需 >=2 块，避免溢出截断 */
        esp_err_t e1 = rmt_config(&cfg);
        esp_err_t e2 = rmt_driver_install(cfg.channel, 0, 0);
        g_rmt_installed = true;
    }
    /* 清掉引脚残留状态，确保 GPIO10 能被 RMT 干净夺回为输出 */
    gpio_reset_pin(OOK_RMT_GPIO);
    rmt_set_gpio(OOK_RMT_CH, RMT_MODE_TX, OOK_RMT_GPIO, false);
}

/* 唤醒前导：te-grid（3te 高 + 1te 低）×reps，给 RX AGC 建立时间（全 1 形态）。 */
static void ookRmtSendWakeup(const OokProtocol *proto, uint16_t unit_us, uint16_t reps)
{
    size_t n = 0;
    for (uint16_t i = 0; i < reps && n < 256; i++) {
        if (!ook_make_symbol(proto->one, unit_us, proto->inverted, &g_items[n])) break;
        n++;
    }
    ookRmtWrite(g_items, n, 1000);
}

// ---- OOK 解码（命中频率后顺带解按键编码）----
static QueueHandle_t   g_decodeQ = nullptr;       // 解码结果队列

/* ============================ CMT2300A 硬件适配层 ============================
 * 统一把 CMT2300A 驱动包装成与参考工程 radio 对象等价的语义，
 * 上层扫描/检测/嗅探逻辑与参考工程逐行一致。
 * ============================================================================ */
static bool radioPresent() { return CMT2300A_IsExist(); }

// 跳频到指定频率并发起 RX（不等待锁定）。CMT2300A 无信道步进，但支持 AN197
// 快速手动跳频：同段内 CMT2300A_TuneFast 只写 FH_OFFSET/FH_CHANNEL（2 个寄存器），
// 跨段（FREQ_DIVX_CODE / FREQ_VCO_BANK 变化）自动回落全量写频率区（9 个寄存器）。
// SetFrequency 在空洞频点会静默失败，调用方必须先过滤（IsFreqSupported）。
// 锁定与 dwell 并行：调用方 delayMicroseconds(dwell) 后须用 CMT2300A_WaitRxLocked
// 确认进入 RX 再读 RSSI（大偏移 PLL 锁定慢，未锁时读数会误用上一频点残留）。
static void cmtTuneRx(uint32_t freqHz) {
    CMT2300A_GoStby();
    CMT2300A_TuneFast(freqHz);
    CMT2300A_ClearInterruptFlags();
    CMT2300A_EnterRx();
}

// 读 RSSI 并换算为 0.5dB/LSB raw（与参考工程 readRssiRaw 语义一致，dBm=raw/2-134）
static uint8_t cmtRssiRaw() {
    int dbm = CMT2300A_GetRssiDBm();
    int raw = (int)roundf((dbm + 134.0f) * 2.0f);
    return (uint8_t)constrain(raw, 0, 255);
}

static float rawToDbm(uint8_t raw) { return raw * 0.5f - 134.0f; }

/* ============================ 工具 ============================ */
static uint8_t parseFreqList(const char* s, float* out, uint8_t maxN);  // 前向声明

static uint16_t calcBins(ScanSettings& s) {
    if (s.mode == 1) {                       // 常见频点扫频：直接跳到列表里的频点
        float tmp[64];
        uint8_t m = parseFreqList(s.presetList, tmp, 64);
        uint16_t n = 0;
        for (uint8_t k = 0; k < m && n < MAX_BINS; k++) {
            uint32_t hz = (uint32_t)(tmp[k] * 1e6f + 0.5f);
            // CMT2300A 适配：过滤不可调谐的空洞频点（如 580MHz）
            if (hz >= 10000000UL && hz <= 2000000000UL && CMT2300A_IsFreqSupported(hz))
                g_freqs[n++] = hz;
        }
        g_preset = (n > 0);
        return n;
    }
    g_preset = false;
    if (s.stopHz <= s.startHz) s.stopHz = s.startHz + s.stepHz * 10;
    if (s.stepHz < 10000) s.stepHz = 10000;
    uint32_t span = s.stopHz - s.startHz;
    uint32_t n = span / s.stepHz + 1;
    if (n > MAX_BINS) {                      // 点数超限 → 自动放大步进
        s.stepHz = (span / (MAX_BINS - 1) + 999) / 1000 * 1000;
        n = span / s.stepHz + 1;
        if (n > MAX_BINS) n = MAX_BINS;
    }
    return (uint16_t)n;
}

// 解析归一化频点列表（过滤 CMT2300A 不可调谐频点）
static uint8_t parseFreqList(const char* s, float* out, uint8_t maxN) {
    uint8_t n = 0;
    const char* p = s;
    while (*p && n < maxN) {
        while (*p && (*p == ' ' || *p == ',' || *p == ';')) p++;
        if (!*p) break;
        char* end;
        float v = strtof(p, &end);
        if (end == p) break;
        if (v > 1.0f && v < 2000.0f &&
            CMT2300A_IsFreqSupported((uint32_t)(v * 1e6f + 0.5f)))
            out[n++] = v;
        p = end;
    }
    return n;
}

/* 归一化：自适应容差 —— 不超过用户设定，也不超过与相邻频点间距的一半，
   这样 433.92 / 434.5 这种只差 0.58MHz 的点不会互相抢 */
static float normalizedFrequency(float peakMHz, const float* list, uint8_t n,
                           float userTol, float* cand, float* dist) {
    for (int i = 0; i < 3; i++) { cand[i] = 0; dist[i] = 9999; }
    float best = 0, bestD = 9999;
    for (uint8_t i = 0; i < n; i++) {
        float d = fabsf(peakMHz - list[i]);
        // 插入 Top3
        for (int k = 0; k < 3; k++) {
            if (d < dist[k]) {
                for (int j = 2; j > k; j--) { dist[j] = dist[j - 1]; cand[j] = cand[j - 1]; }
                dist[k] = d; cand[k] = list[i];
                break;
            }
        }
        // 自适应容差
        float halfGap = 9999;
        for (uint8_t j = 0; j < n; j++) {
            if (j == i) continue;
            float g = fabsf(list[i] - list[j]) * 0.5f;
            if (g < halfGap) halfGap = g;
        }
        float tol = (userTol < halfGap) ? userTol : halfGap;
        if (d <= tol && d < bestD) { bestD = d; best = list[i]; }
    }
    return best;
}

/* ============================ 扫描 ============================ */
static void sweepReal() {
    // 逐点全量调谐（CMT2300A 无信道步进）；范围/预设两种模式仅频率来源不同。
    // 空洞频点（IsFreqSupported=false）跳过调谐，直接置最低值，防止
    // SetFrequency 静默失败导致误读上一有效频点的残留信号。
    for (uint16_t i = 0; i < g_bins; i++) {
        uint32_t f = g_preset ? g_freqs[i]
                              : (g_set.startHz + (uint32_t)i * g_set.stepHz);
        if (CMT2300A_IsFreqSupported(f)) {
            cmtTuneRx(f);                        // 发起调谐+RX，不等待锁定
            delayMicroseconds(g_set.dwellUs);    // dwell 与 PLL 锁定并行
            if (CMT2300A_WaitRxLocked(1000)) {   // 兜底：大步长锁定慢，等剩余时间
                g_live[i] = cmtRssiRaw();
            } else {
                g_live[i] = 0;                   // -134 dBm，避免读残留 RSSI 误触发
            }
        } else {
            g_live[i] = 0;                   // -134 dBm
        }
        if ((i & 0x1F) == 0x1F) vTaskDelay(1);   // 喂 idle 任务/看门狗
    }
}

/* 演示模式：没接 CMT2300A 时生成模拟频谱，便于先把网页调通 */
static void sweepDemo() {
    static uint32_t burstUntil = 0;
    static float    burstMHz   = 433.92f;
    uint32_t now = millis();
    if (now > burstUntil + 4000 + (uint32_t)random(4000)) {
        float pool[64]; uint8_t m = 0;
        if (g_preset) {
            for (uint16_t k = 0; k < g_bins && m < 64; k++)
                pool[m++] = g_freqs[k] / 1e6f;
        } else {
            const float rp[] = {315.0f, 433.92f, 434.5f, 868.0f, 390.0f, 431.5f};
            for (uint8_t k = 0; k < 6; k++) pool[m++] = rp[k];
        }
        burstMHz   = pool[random(m)];
        burstUntil = now + 600 + (uint32_t)random(500);
    }
    bool active = (now < burstUntil);
    for (uint16_t i = 0; i < g_bins; i++) {
        float fMHz = g_preset ? (g_freqs[i] / 1e6f)
                              : ((g_set.startHz + (uint32_t)i * g_set.stepHz) / 1e6f);
        float dbm  = -108.0f + (float)random(100) / 100.0f * 3.0f;
        dbm += 2.0f * sinf(fMHz * 0.35f);                 // 一点起伏，像真实本底
        if (active) {
            float d = fabsf(fMHz - burstMHz);
            float bw = 0.25f;
            dbm += 55.0f * expf(-(d * d) / (2 * bw * bw));  // 主瓣
            dbm += 12.0f * expf(-(d * d) / (2 * 1.2f * 1.2f)); // 裙边
        }
        int raw = (int)roundf((dbm + 134.0f) * 2.0f);
        g_live[i] = (uint8_t)constrain(raw, 0, 255);
        if ((i & 0x3F) == 0x3F) vTaskDelay(1);
    }
    vTaskDelay(pdMS_TO_TICKS(35));
}

/* ========================= 检测状态机 ========================= */
/* 触发判据（快嗅探 sniffLoop + 范围扫频 processDetection 共用）：
   deltaDb = 信号高出本底(dB)，absDbm = 信号绝对电平(dBm)
   ① deltaDb >= thresholdDb：网页可调相对门限（"一个门限管两模式"）
   ② absDbm >= SNIFF_ABS_FLOOR_DBM：绝对地板，防极低本底下的噪声尖峰误触发。 */
static inline bool triggerOk(float deltaDb, float absDbm) {
    return (deltaDb >= g_set.thresholdDb) && (absDbm >= SNIFF_ABS_FLOOR_DBM);
}

enum DetState { DET_IDLE, DET_EVENT, DET_COOL };
static DetState g_det = DET_IDLE;
static uint32_t g_evtStart = 0, g_evtLast = 0, g_coolUntil = 0;
static float    g_evtBestDelta = 0;
static uint16_t g_evtBestBin = 0;
static float    g_evtBestPeak = 0, g_evtBestFloor = 0;

static void processDetection() {
    // --- 本底自适应（用瞬时值，非对称 EMA）---
    if (!g_baseInit) {
        for (uint16_t i = 0; i < g_bins; i++) g_base[i] = g_live[i];
        g_baseInit = true;
    } else {
        for (uint16_t i = 0; i < g_bins; i++) {
            float v = g_live[i];
            if (v < g_base[i]) g_base[i] += 0.30f * (v - g_base[i]);   // 下降快
            else               g_base[i] += 0.01f * (v - g_base[i]);   // 上升慢
        }
    }

    // --- 最大保持（带衰减）---
    float dec = g_set.decayDb * 2.0f;    // dB -> raw
    float bestDelta = -999; uint16_t bestBin = 0;
    for (uint16_t i = 0; i < g_bins; i++) {
        float h = g_hold[i] - dec;
        if ((float)g_live[i] > h) h = g_live[i];
        g_hold[i] = h;
        float d = (h - g_base[i]) * 0.5f;         // dB
        if (d > bestDelta) { bestDelta = d; bestBin = i; }
    }
    g_lastPeakDbm  = rawToDbm((uint8_t)constrain((int)g_hold[bestBin], 0, 255));
    g_lastFloorDbm = g_base[bestBin] * 0.5f - 134.0f;

    if (!g_set.armed) { g_det = DET_IDLE; return; }
    uint32_t now = millis();

    switch (g_det) {
        case DET_COOL:
            if (now > g_coolUntil) g_det = DET_IDLE;
            break;

        case DET_IDLE:
            if (triggerOk(bestDelta, g_hold[bestBin] * 0.5f - 134.0f)) {
                g_det = DET_EVENT;
                g_evtStart = g_evtLast = now;
                g_evtBestDelta = bestDelta;
                g_evtBestBin   = bestBin;
                g_evtBestPeak  = g_hold[bestBin];
                g_evtBestFloor = g_base[bestBin];
                // 自动解码由【快嗅探 sniffLoop】实时载波命中后内联解码，
                //   不在慢扫峰值触发（避免"峰值已过时发射早结束 -> 需等"的旧问题）。
            }
            break;

        case DET_EVENT: {
            if (triggerOk(bestDelta, g_hold[bestBin] * 0.5f - 134.0f)) {
                g_evtLast = now;
                if (bestDelta > g_evtBestDelta) {
                    g_evtBestDelta = bestDelta;
                    g_evtBestBin   = bestBin;
                    g_evtBestPeak  = g_hold[bestBin];
                    g_evtBestFloor = g_base[bestBin];
                }
            }
            bool ended = (now - g_evtLast > 250) || (now - g_evtStart > 2500);
            if (!ended) break;

            // ---- 峰值频率 ----
            uint16_t p = g_evtBestBin;
            float peakMHz;
            if (g_preset) {
                // 预设模式各 bin 频率离散且可能相距很远，直接用该 bin 的真实频率
                peakMHz = g_freqs[p] / 1e6f;
            } else {
                // 范围扫频：相邻频点密集，用抛物线插值细化峰值
                float delta = 0;
                if (p > 0 && p + 1 < g_bins) {
                    float y1 = g_hold[p - 1] * 0.5f, y2 = g_hold[p] * 0.5f, y3 = g_hold[p + 1] * 0.5f;
                    float den = (y1 - 2 * y2 + y3);
                    if (fabsf(den) > 1e-3f) {
                        delta = 0.5f * (y1 - y3) / den;
                        delta = constrain(delta, -1.0f, 1.0f);
                    }
                }
                peakMHz = (g_set.startHz + ((float)p + delta) * g_set.stepHz) / 1e6f;
            }

            float list[48];
            uint8_t n = parseFreqList(g_set.normalizedList, list, 48);
            Hit h{};
            h.tsMs       = now;
            h.peakMHz    = peakMHz;
            h.peakDbm    = g_evtBestPeak * 0.5f - 134.0f;
            h.floorDbm   = g_evtBestFloor * 0.5f - 134.0f;
            h.deltaDb    = g_evtBestDelta;
            uint32_t durMs = now - g_evtStart;
            h.durationMs = (uint16_t)(durMs > 65535UL ? 65535UL : durMs);
            h.normalizedMHz    = g_set.normalizedEnable
                             ? normalizedFrequency(peakMHz, list, n, g_set.normalizedTolMHz, h.cand, h.candDist)
                             : 0;              // 归一化关闭：不吸附，前端显示实测峰值
            xQueueSend(g_hitQ, &h, 0);

            g_det       = DET_COOL;
            g_coolUntil = now + 200;
            break;
        }
    }
}

/* ====================== OOK 解码态（快嗅探内联调用） ======================
 * 由 sniffLoop() 在【实时载波命中】后调用：此时该频点就在发射，故不再等待，
 *   直接进入 OOK 直驱接收 + 时长域解码 -> 发布结果 -> 恢复扫频。
 * CMT2300A 适配：ook_decoder_run() 内部已 GoStby->SetFrequency->GoRx
 *   （DATA_MODE=DIRECT，GPIO2/DOUT 直出解调码流），无需 enter/exit OOK 配置。
 */
static void runDecodeState(uint32_t freq) {
    DecodeResult r{};
    r.freqMHz = freq / 1e6f;
    r.tsMs    = millis();

    if (!radioPresent()) {                     // 演示模式：合成一个解码结果供界面预览
        vTaskDelay(pdMS_TO_TICKS(350));
        r.code = 0x1A2C3F; r.bits = 24; r.proto = 1; r.delayUs = 350; r.te = 350;
        r.dbm  = g_lastPeakDbm;
        strncpy(r.hex, "1A2C3F", sizeof(r.hex) - 1); r.hex[sizeof(r.hex) - 1] = 0;
        strncpy(r.protoName, "EV1527/PT2262", sizeof(r.protoName) - 1); r.protoName[sizeof(r.protoName) - 1] = 0;
        xQueueSend(g_decodeQ, &r, 0);
        return;
    }

    // 嗅探已确认此刻该频点有载波，直接以 OOK_DECODE_TIMEOUT_MS 窗口采集解码
    // （ook_decoder_run 内部锁频 + 多轮重采 + 多帧复核）。
    decode_result_t out;
    bool captured = ook_decoder_run(freq / 1e6f, OOK_DECODE_TIMEOUT_MS, &out);
    Serial.printf("[DEC] ook_decoder: captured=%d proto=%d bits=%u te=%uus repeat=%u frames=%u pulses=%u tries=%u\n",
                  captured, out.proto, out.bits, out.te, out.repeat, out.frames, out.pulses, out.tries);
    if (out.raw[0]) Serial.printf("[DEC] raw(us)=%s\n", out.raw);

    bool got = false;
    r.tsMs    = millis();
    r.freqMHz = freq / 1e6f;
    r.dbm     = g_lastPeakDbm;
    if (out.proto == 1 && out.bits > 0) {
        // 多帧复核通过，确为有效码；本项目解码器无 code 字段，从 hex 串解析
        r.code      = (uint32_t)strtoul(out.hex, NULL, 16);
        r.bits      = (uint8_t)out.bits;
        r.proto     = 1;
        r.delayUs   = (uint16_t)out.te;
        r.te        = out.te;
        r.repeat    = (uint8_t)out.repeat;
        r.frames    = (uint8_t)out.frames;
        strncpy(r.hex, out.hex, sizeof(r.hex) - 1); r.hex[sizeof(r.hex) - 1] = 0;
        strncpy(r.protoName, out.proto_name, sizeof(r.protoName) - 1); r.protoName[sizeof(r.protoName) - 1] = 0;
        Serial.printf("[DEC] 解出! hex=%s bin=%s proto=%s te=%uus repeat=%u\n",
                      out.hex, out.bin, out.proto_name, out.te, out.repeat);
        xQueueSend(g_decodeQ, &r, 0);
        got = true;
    } else if (out.pulses >= 8) {
        // 有数据但未通过多帧复核：仍发布，供前端显示"检测到信号但未识别"并附诊断
        r.proto   = -1;
        r.code    = 0;
        r.bits    = (uint8_t)out.bits;
        r.delayUs = (uint16_t)out.te;
        r.te      = out.te;
        r.repeat  = (uint8_t)out.repeat;
        r.frames  = (uint8_t)out.frames;
        strncpy(r.hex, out.hex, sizeof(r.hex) - 1); r.hex[sizeof(r.hex) - 1] = 0;
        strncpy(r.protoName, out.proto_name, sizeof(r.protoName) - 1); r.protoName[sizeof(r.protoName) - 1] = 0;
        xQueueSend(g_decodeQ, &r, 0);
        Serial.println(F("[DEC] 有数据但未识别（多帧复核未通过）：多为信号弱/帧不完整，见上方 raw 诊断"));
    } else {
        Serial.println(F("[DEC] 无数据: GPIO 边沿数不足 => 检查 DOUT(GPIO9) 接线"));
    }
    // 无需 exitOokRx：下一轮扫描 sweepReal 自动 GoStby->SetFrequency->GoRx 恢复
}

/* ====================== 快嗅探（单次按压即解码） ======================
 * 在预设遥控频点(g_freqs，由 calcBins 在 mode=1 下填充)间高速轮转，每个频点停留
 * SNIFF_DWELL_US 读一次 RSSI。维护每频点滑动本底基线，瞬时 RSSI 高出本底
 * thresholdDb（与范围扫频共用同一门限）且绝对电平优于 SNIFF_ABS_FLOOR_DBM
 * 即判定"此刻有遥控在场"——【立即】在该频点进 OOK 接收解码。
 * 仅在 mode=1(常见频点)下启用；范围扫频点数多、轮转慢，仍走纯频谱分析。
 */
static void sniffDemo();                  // 前向声明（定义见下方）
static void publishFrame();               // 前向声明（sniffLoop/sniffDemo 内调用，定义在后）
static void sniffLoop() {
    if (g_bins == 0 || !g_preset) { vTaskDelay(pdMS_TO_TICKS(50)); return; }
    if (!g_sniffBaseInit) {
        for (uint16_t i = 0; i < g_bins; i++) g_sniffBase[i] = 255.0f;   // 255=sentinel，下一轮首读即初始化
        g_sniffBaseInit = true;
    }
    if (g_demo) { sniffDemo(); return; }

    bool     trig = false;
    uint32_t trigFreq = 0;
    int      trigRssi = 0;
    float    trigDeltaDb = 0, trigFloorRaw = 0;   // 触发时刻的 deltaDb 与该频点滑动本底(供 hit 事件)
    for (uint16_t i = 0; i < g_bins; i++) {
        uint32_t f = g_freqs[i];
        cmtTuneRx(f);                          // 发起调谐+RX，不等待锁定
        delayMicroseconds(SNIFF_DWELL_US);     // dwell 与 PLL 锁定并行
        if (!CMT2300A_WaitRxLocked(1000)) {    // 大步长锁定慢：等剩余；仍未锁则弃读
            g_live[i] = 0; g_hold[i] = 0;
            if ((i & 0x0F) == 0x0F) vTaskDelay(1);
            continue;
        }
        uint8_t rv = cmtRssiRaw();
        g_live[i] = rv;
        // 最大保持（带衰减，语义与范围模式 processDetection 一致）。
        // 此前直接 g_hold[i]=rv（瞬时覆盖）：前端 preset 模式先画 hold 柱再画 live 柱
        // 盖住，hold==live 时顶部保持段完全不可见 -> "最大保持不显示"（2026-08-14 反馈）。
        // 嗅探帧率高（~49 帧/s），decayDb 按轮衰减，等效 ~0.5dB×49≈24dB/s，可网页调小。
        {   float dec = g_set.decayDb * 2.0f;      // dB -> raw（DEF_DECAY_DB=0.5 -> 1 raw/轮）
            float h = g_hold[i] - dec;
            if (rv > h) h = rv;
            g_hold[i] = h;
        }

        // 滑动本底基线（下降快、上升慢，抗偶发尖峰）
        float v = rv;
        if (g_sniffBase[i] > 250.0f) g_sniffBase[i] = v;
        else if (v < g_sniffBase[i]) g_sniffBase[i] += 0.30f * (v - g_sniffBase[i]);
        else                         g_sniffBase[i] += 0.01f * (v - g_sniffBase[i]);

        float deltaDb = (v - g_sniffBase[i]) * 0.5f;        // raw(0.5dB/单位) -> dB
        float absDbm  = v * 0.5f - 134.0f;
        // 未布防(armed=off)：频谱照常更新显示，但不判触发 -> 不命中、不解码、不发捕获事件
        if (g_set.armed && triggerOk(deltaDb, absDbm)) {
            trig = true; trigFreq = f; trigRssi = rv;
            trigDeltaDb = deltaDb; trigFloorRaw = g_sniffBase[i];
            break;                                          // 立即跳出，载波此刻就在场
        }
        if ((i & 0x0F) == 0x0F) vTaskDelay(1);              // 喂 idle/看门狗
    }
    publishFrame();                                          // 发布本轮各频点 RSSI（前端按 g_freqs 定位）

    if (!trig) { vTaskDelay(pdMS_TO_TICKS(1)); return; }

    // 快嗅探命中也发一条【捕获事件】：让"最近一次捕获"(hitCard) 同步更新
    float hitNormMHz = 0;                       // 归一化命中频点（供下方解码频点选择）
    {
        float list[48];
        uint8_t n = parseFreqList(g_set.normalizedList, list, 48);
        Hit h{};
        h.tsMs       = millis();
        h.peakMHz    = trigFreq / 1e6f;
        h.peakDbm    = trigRssi / 2.0f - 134.0f;
        h.floorDbm   = trigFloorRaw * 0.5f - 134.0f;
        h.deltaDb    = trigDeltaDb;
        h.durationMs = 0;                         // 快嗅探无突发持续跟踪，前端显示"持续 0 ms"
        h.normalizedMHz    = g_set.normalizedEnable
                         ? normalizedFrequency(h.peakMHz, list, n, g_set.normalizedTolMHz, h.cand, h.candDist)
                         : 0;                     // 归一化关闭：不吸附，前端显示实测峰值
        hitNormMHz   = h.normalizedMHz;
        xQueueSend(g_hitQ, &h, 0);
    }

    float trigDbm = trigRssi / 2.0f - 134.0f;
    g_lastPeakDbm = trigDbm;                                // 供 runDecodeState 填充结果电平
    Serial.printf("[SNIFF] 实时载波命中 @ %.3f MHz  RSSI=%.1fdBm -> 立即解码\n", trigFreq / 1e6f, trigDbm);
    // 归一化开关：开=锁定/解码用归一化频点（容差内匹配到常见频点则用该频点，否则用原频点）；
    //             关=锁定/解码用实测命中频点。
    uint32_t decFreq = trigFreq;
    if (g_set.normalizedEnable) {
        uint32_t s = (uint32_t)(hitNormMHz * 1e6f + 0.5f);
        if (hitNormMHz > 0 && s >= CMT_FREQ_MIN_HZ && s <= CMT_FREQ_MAX_HZ) decFreq = s;
    }
    runDecodeState(decFreq);
    vTaskDelay(pdMS_TO_TICKS(SNIFF_COOLDOWN_MS));            // 成功/尝试后短冷却，避免同一长按刷屏
}

/* 演示模式下的快嗅探：生成模拟各频点本底 + 偶发合成一次解码，便于先调通网页 */
static void sniffDemo() {
    for (uint16_t i = 0; i < g_bins; i++) {
        float noise = -108.0f + (float)random(100) / 100.0f * 3.0f + 2.0f * sinf(g_freqs[i] / 1e6f * 0.35f);
        g_live[i] = (uint8_t)constrain((int)((noise + 134.0f) * 2.0f), 0, 255);
    }
    static uint32_t last = 0;
    uint32_t now = millis();
    if (now - last > 4000 + (uint32_t)random(4000)) {
        last = now;
        DecodeResult r{};
        r.code = 0x1A2C3F; r.bits = 24; r.proto = 1; r.te = 350; r.repeat = 4; r.frames = 4;
        r.freqMHz = (g_bins > 0) ? (g_freqs[random(g_bins)] / 1e6f) : 433.92f;
        r.dbm = -55; strncpy(r.hex, "1A2C3F", sizeof(r.hex) - 1); r.hex[sizeof(r.hex) - 1] = 0;
        strncpy(r.protoName, "EV1527/PT2262", sizeof(r.protoName) - 1); r.protoName[sizeof(r.protoName) - 1] = 0;
        xQueueSend(g_decodeQ, &r, 0);
    }
    publishFrame();
}

/* ============================ 任务 ============================ */
static void publishFrame() {
    if (xSemaphoreTake(g_mux, pdMS_TO_TICKS(5)) != pdTRUE) return;
    memcpy(g_pubLive, g_live, g_bins);
    for (uint16_t i = 0; i < g_bins; i++)
        g_pubHold[i] = (uint8_t)constrain((int)g_hold[i], 0, 255);
    g_pubBins  = g_bins;
    if (g_preset) {                          // 频率表随帧发布，前端按真实频率定位
        g_pubStart = (g_bins > 0) ? g_freqs[0] : 0;
        g_pubStep  = 0;
        g_pubFreqN = (g_bins < MAX_BINS) ? g_bins : MAX_BINS;
        memcpy(g_pubFreqs, g_freqs, g_pubFreqN * sizeof(uint32_t));
    } else {
        g_pubStart = g_set.startHz;
        g_pubStep  = g_set.stepHz;
        g_pubFreqN = 0;
    }
    g_pubSeq++;
    g_frameReady = true;
    xSemaphoreGive(g_mux);
}

/* ====================== 解码记录回放发射（DIRECT OOK） ======================
 * 流程：Web 收到 {"cmd":"replay",...} -> requestReplay() 入队 -> scanTask 检测
 * g_replayReq -> doReplay() -> CMT2300A_TxOokBegin/End + OOK-RMT 硬件生成波形。
 *
 * 波形（与 tuya-test 已验证序列逐位一致）：EV1527 模板 '0'=高1低3、'1'=高3低1，
 * 帧 = bits 位 MSB-first + 31T 同步静默。波形由 RMT 外设硬件生成（时序不受中断/
 * 调度抖动影响，根除 bit-bang 关中断超阈值触发 IWDT 的崩溃）。帧前加 ~100ms te-grid
 * 唤醒前导（3te 高+1te 低），给 RX AGC 建立时间并规避快嗅探单点 RSSI 采样漏检。 */
void requestReplay(uint32_t freqHz, uint32_t code, uint32_t te, uint8_t bits) {
    g_replayFreqHz = freqHz;
    g_replayCode   = code;
    g_replayTe     = te;
    g_replayBits   = bits;
    g_replayReq    = true;
}

static void doReplay() {
    uint32_t freq = g_replayFreqHz;
    uint64_t code = g_replayCode;
    uint32_t te   = g_replayTe;
    uint32_t bits = g_replayBits;

    if (bits == 0 || bits > 32) { Serial.println(F("[TX] replay 放弃：bits 非法")); return; }
    if (te < 50 || te > 3000) te = 350;   // te 兜底（典型 EV1527 ~350us）

    Serial.printf("[TX] replay %.3fMHz code=0x%08X bits=%u te=%uus\n",
                  freq / 1e6f, (unsigned)code, bits, te);
    CMT2300A_SetTxPower(g_set.txPowerDbm);   // 应用当前功率档位（-10~20dBm）
    CMT2300A_TxOokBegin(freq);

    /* ★一票否决自检★：确认 DIN->PA 调制通路齐备（基准 = datasheet V1.8 §6.1 DIRECT Tx 序列）。
       - EN_CTL[5]=LOCKING_EN   : PLL 未锁禁止进 TX（SoftReset 会清 -> 曾恒关）
       - FIFO_CTL[7]=TX_DIN_EN  : 须=1，使能 DIN 输入驱动 PA（§6.1 step1；缺则 PA 恒载波无调制）
       - FIFO_CTL[6:5]=TX_DIN_SEL: 须=0x40=GPIO3（本板 ESP32 GPIO10 -> CMT GPIO3 飞线，§6.1 step2）
       - FIFO_CTL[1]=FIFO_MERGE_EN: ★须=0★（FIFO/Packet 模式专用；DIRECT 发射置 1 无益，§5.2/§6.1 均未涉及）
       - INT2_CTL[5]=TX_DIN_INV : 须=0（正逻辑；=1 会整帧反码）
       - IO_SEL GPIO3_SEL[5:4]  : 须=0x10=DIN/DOUT（DIN 输入） */
    uint8_t en    = CMT2300A_ReadReg(CMT2300A_CUS_EN_CTL);
    uint8_t fifo  = CMT2300A_ReadReg(CMT2300A_CUS_FIFO_CTL);
    uint8_t iosel = CMT2300A_ReadReg(CMT2300A_CUS_IO_SEL);
    uint8_t int2c = CMT2300A_ReadReg(CMT2300A_CUS_INT2_CTL);
    bool din_pin_ok    = ((iosel & 0x30u) == 0x10u);                             // GPIO3_SEL[5:4]=0b01=DIN/DOUT
    bool din_en_ok     = (fifo & CMT2300A_MASK_TX_DIN_EN);                        // bit7 使能
    bool din_sel_ok    = ((fifo & CMT2300A_MASK_TX_DIN_SEL) == CMT2300A_TX_DIN_SEL_GPIO3); // TX_DIN_SEL=GPIO3
    bool merge_off_ok  = !(fifo & CMT2300A_MASK_FIFO_MERGE_EN);                   // bit1 必须【关】
    bool din_inv_off   = !(int2c & CMT2300A_MASK_TX_DIN_INV);                     // INT2_CTL[5] 必须【关】
    bool din_route_ok  = din_en_ok && din_sel_ok && merge_off_ok;
    if (!((en & CMT2300A_MASK_LOCKING_EN) && din_route_ok && din_pin_ok && din_inv_off)) {
        Serial.println(F("[TX] 致命：DIN->PA 通路未打开（需 EN_CTL[5]=1、FIFO_CTL[7]=1、FIFO_CTL[6:5]=0x40(GPIO3)、FIFO_CTL[1]=0、INT2_CTL[5]=0、IO_SEL GPIO3=DIN）—— 中止发射"));
        CMT2300A_TxOokEnd();
        return;
    }

    Serial.flush();   // 日志发完再开 RMT（防 UART 冻结丢失）

    /* 接管 GPIO10（本板 TX DIN 输出 -> CMT GPIO3）：用 RMT 硬件生成 OOK 波形（根除 bit-bash 关中断超阈值触发 IWDT
       的崩溃）。GPIO10 现为专用 RMT 发射脚，不再挂任何中断。 */
    ookRmtSetupForTx();

    /* 唤醒前导：te-grid（3te 高 + 1te 低）×reps，约 100ms，给 RX AGC 建立时间。
       用 te-grid 而非 1ms/0.3ms：非 te 整数倍边沿会在 RX 的 Te 估计里成离群值污染直方图。 */
    uint16_t wakeReps = (uint16_t)(100000U / (4U * te));
    if (wakeReps < 1)   wakeReps = 1;
    if (wakeReps > 250) wakeReps = 250;
    ookRmtSendWakeup(&kOokProtocol, (uint16_t)te, wakeReps);

    /* 预构建一帧（bits 位 MSB 优先 + 末尾 sync），burst 内重复发送。 */
    rmt_item32_t frame_items[40];
    size_t frame_count = 0;
    if (!ook_build_frame((uint32_t)code, (uint8_t)bits, &kOokProtocol, (uint16_t)te,
                         frame_items, 40, &frame_count)) {
        Serial.println(F("[TX] 致命：构建帧符号失败"));
        CMT2300A_TxOokEnd();
        return;
    }

    /* 分组连续发帧：每组 4 帧（数据块 ≈248ms，远小于 RX 捕获窗口余量 400ms，
       无论相位落在块内何处都能采到整块 ≥2 帧）。组间插入 carrier-off 静默 120ms
       —— RX OOK 解调切片器/AGC 复位直流基线的硬性要求（<40ms 解调死透、≥120ms 正常）。
       静默期间 RMT 空闲电平=LOW -> DIN=0 -> 载波被门控关断，等价于旧 digitalWrite(LOW)。 */
    uint32_t tBurstEnd = millis() + 1900;   // 突发总时长 ~1.9s
    while (millis() < tBurstEnd) {
        for (uint8_t f = 0; f < 4; f++) {
            ookRmtWrite(frame_items, frame_count, 500);
        }
        yield();   // 组间放开，喂 WDT
        size_t sil_n = ook_build_level(g_items, 256, 0, 120000U);   // 120ms carrier-off 静默
        ookRmtWrite(g_items, sil_n, 300);
    }
    /* 结尾静默（引脚由 RMT 空闲电平保持 LOW） */
    size_t end_n = ook_build_level(g_items, 256, 0, (uint32_t)te * 31U);
    ookRmtWrite(g_items, end_n, 300);

    CMT2300A_TxOokEnd();
    Serial.println(F("[TX] replay 完成，已恢复 RX"));
}

static void scanTask(void*) {
    uint32_t tLast = millis();
    uint16_t cnt = 0;
    for (;;) {
        /* 回放发射请求优先：执行完再继续扫描（TxOokEnd 已恢复 RX，下一轮自动续扫） */
        if (g_replayReq) {
            g_replayReq = false;
            doReplay();
            continue;
        }
        if (g_pendingFlag) {
            if (xSemaphoreTake(g_mux, pdMS_TO_TICKS(20)) == pdTRUE) {
                g_set = g_pending;
                g_pendingFlag = false;
                xSemaphoreGive(g_mux);
            }
            g_bins = calcBins(g_set);
            g_baseInit = false;
            g_sniffBaseInit = false;            // 候选频点/本底可能已变，下一轮嗅探重建基线
            memset(g_hold, 0, sizeof(g_hold));
        }
        if (!g_set.running) { vTaskDelay(pdMS_TO_TICKS(60)); continue; }

        // 自动解码 + 常见频点模式：进入【快嗅探】——实时载波命中即内联解码（单次按压即可）
        if (g_set.autoDecode && g_preset) {
            sniffLoop();
        } else {
            // 纯频谱分析（或范围扫频/未开自动解码）：慢扫 + 突发检测 + 发布
            if (g_demo) sweepDemo();
            else        sweepReal();

            processDetection();
            publishFrame();
        }

        cnt++;
        uint32_t now = millis();
        if (now - tLast >= 1000) {
            g_sps  = cnt * 1000.0f / (now - tLast);
            cnt    = 0;
            tLast  = now;
        }
        vTaskDelay(1);
    }
}

/* ============================ 对外接口 ============================ */
void loadSettings() {
    // 读写方式打开：NVS 命名空间不存在时自动创建。
    g_prefs.begin("cmtsniff", false);
    constexpr uint8_t NVS_VER = 1;   // 调整 DEF_* 默认参数时递增此值，触发重新落盘。
    if (g_prefs.getUChar("nvver", 0) != NVS_VER) {
        g_prefs.putULong("start", DEF_START_HZ);
        g_prefs.putULong("stop",  DEF_STOP_HZ);
        g_prefs.putULong("step",  DEF_STEP_HZ);
        g_prefs.putUShort("dwell", DEF_DWELL_US);
        g_prefs.putFloat("thr",   DEF_THRESH_DB);
        g_prefs.putFloat("dec",   DEF_DECAY_DB);
        g_prefs.putFloat("normtol",   DEF_NORMALIZED_TOL_MHZ);
        g_prefs.putBool("normon", true);
        g_prefs.putString("normlist", DEF_NORMALIZED_LIST);
        g_prefs.putUChar("mode",  DEF_MODE);
        g_prefs.putChar("txpwr", DEF_TX_POWER_DBM);
        g_prefs.putString("preset", DEF_PRESET_LIST);
        g_prefs.putUChar("nvver", NVS_VER);
    }
    g_set.startHz     = g_prefs.getULong("start", DEF_START_HZ);
    g_set.stopHz      = g_prefs.getULong("stop",  DEF_STOP_HZ);
    g_set.stepHz      = g_prefs.getULong("step",  DEF_STEP_HZ);
    g_set.dwellUs     = g_prefs.getUShort("dwell", DEF_DWELL_US);
    g_set.thresholdDb = g_prefs.getFloat("thr",   DEF_THRESH_DB);
    g_set.decayDb     = g_prefs.getFloat("dec",   DEF_DECAY_DB);
    g_set.normalizedTolMHz  = g_prefs.getFloat("normtol",   DEF_NORMALIZED_TOL_MHZ);
    g_set.normalizedEnable  = g_prefs.getBool("normon", true);
    g_set.mode        = g_prefs.getUChar("mode",  DEF_MODE);
    g_set.txPowerDbm  = (int8_t)g_prefs.getChar("txpwr", DEF_TX_POWER_DBM);
    if (g_set.txPowerDbm < -10 || g_set.txPowerDbm > 20) g_set.txPowerDbm = DEF_TX_POWER_DBM;
    // autoDecode 不持久化：每次上电默认 DEF_AUTO_DECODE，仅本次运行有效
    String sl         = g_prefs.getString("normlist", DEF_NORMALIZED_LIST);
    strncpy(g_set.normalizedList, sl.c_str(), sizeof(g_set.normalizedList) - 1);
    g_set.normalizedList[sizeof(g_set.normalizedList) - 1] = '\0';
    String pl         = g_prefs.getString("preset", DEF_PRESET_LIST);
    strncpy(g_set.presetList, pl.c_str(), sizeof(g_set.presetList) - 1);
    g_set.presetList[sizeof(g_set.presetList) - 1] = '\0';
    g_prefs.end();
    g_set.running = true;
    g_set.armed   = true;
}

void saveSettings() {
    ScanSettings s = getSettings();          // 优先保存尚未生效的 pending 值
    g_prefs.begin("cmtsniff", false);
    g_prefs.putULong("start", s.startHz);
    g_prefs.putULong("stop",  s.stopHz);
    g_prefs.putULong("step",  s.stepHz);
    g_prefs.putUShort("dwell", s.dwellUs);
    g_prefs.putFloat("thr",   s.thresholdDb);
    g_prefs.putFloat("dec",   s.decayDb);
    g_prefs.putFloat("normtol",   s.normalizedTolMHz);
    g_prefs.putBool("normon", s.normalizedEnable);
    g_prefs.putString("normlist", s.normalizedList);
    g_prefs.putUChar("mode",  s.mode);
    g_prefs.putChar("txpwr", s.txPowerDbm);
    g_prefs.putString("preset", s.presetList);
    g_prefs.end();
    Serial.println(F("[Spectrum] 参数已保存到 NVS"));
}

/* 恢复初始配置：清空 cmtsniff 命名空间（含 nvver 版本键）后重启。
 * 重启后 loadSettings() 因 nvver 缺失会按编译期默认值重新落盘 -> 出厂状态。 */
void resetToFactory() {
    g_prefs.begin("cmtsniff", false);
    g_prefs.clear();                         // 只清本命名空间，不影响其他
    g_prefs.end();
    Serial.println(F("[Spectrum] 配置已清除，重启恢复初始配置..."));
    delay(100);
    ESP.restart();
}

void begin() {
    g_mux  = xSemaphoreCreateMutex();
    g_hitQ = xQueueCreate(8, sizeof(Hit));
    g_decodeQ = xQueueCreate(4, sizeof(DecodeResult));
    ook_decoder_init(OOK_DOUT_PIN);            // 注册 OOK 解码 CHANGE 中断（ISR 内 g_cap 默认 false，仅解码态才采集）
    loadSettings();
    g_bins = calcBins(g_set);
    g_demo = !radioPresent();
#if !ALLOW_DEMO_MODE
    if (g_demo) Serial.println(F("[Spectrum] 未检测到 CMT2300A，且演示模式已关闭"));
#else
    if (g_demo) Serial.println(F("[Spectrum] 未检测到 CMT2300A -> 进入演示模式(模拟频谱)"));
#endif
    xTaskCreatePinnedToCore(scanTask, "scan", 6144, nullptr, 3, nullptr, 1);
}

ScanSettings getSettings() {
    ScanSettings s;
    if (xSemaphoreTake(g_mux, pdMS_TO_TICKS(20)) == pdTRUE) {
        s = g_pendingFlag ? g_pending : g_set;
        xSemaphoreGive(g_mux);
    } else s = g_set;
    return s;
}

void applySettings(const ScanSettings& s) {
    if (xSemaphoreTake(g_mux, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_pending     = s;
        g_pendingFlag = true;
        xSemaphoreGive(g_mux);
    }
}

void resetBaseline() { g_baseInit = false; }
void clearMaxHold()  { memset(g_hold, 0, sizeof(g_hold)); }

uint16_t takeFrame(uint8_t* live, uint8_t* maxHold, uint16_t maxLen,
                   uint32_t& startHz, uint32_t& stepHz, uint16_t& seq) {
    if (!g_frameReady) return 0;
    uint16_t n = 0;
    if (xSemaphoreTake(g_mux, pdMS_TO_TICKS(5)) != pdTRUE) return 0;
    n = (g_pubBins < maxLen) ? g_pubBins : maxLen;
    memcpy(live, g_pubLive, n);
    memcpy(maxHold, g_pubHold, n);
    startHz = g_pubStart;
    stepHz  = g_pubStep;
    seq     = g_pubSeq;
    g_frameReady = false;
    xSemaphoreGive(g_mux);
    return n;
}

bool popHit(Hit& h) { return xQueueReceive(g_hitQ, &h, 0) == pdTRUE; }

bool popDecode(DecodeResult& r) { return xQueueReceive(g_decodeQ, &r, 0) == pdTRUE; }

bool takeFreqTable(uint32_t* outHz, uint16_t maxN, uint16_t& n) {
    n = 0;
    if (xSemaphoreTake(g_mux, pdMS_TO_TICKS(5)) != pdTRUE) return false;
    uint16_t c = (g_pubFreqN < maxN) ? g_pubFreqN : maxN;
    for (uint16_t i = 0; i < c; i++) outHz[i] = g_pubFreqs[i];
    n = c;
    xSemaphoreGive(g_mux);
    return n > 0;
}

uint16_t binCount()     { return g_bins; }
float    sweepsPerSec() { return g_sps; }
bool     radioOk()      { return radioPresent(); }
bool     demoMode()     { return g_demo; }
float    lastPeakDbm()  { return g_lastPeakDbm; }
float    lastFloorDbm() { return g_lastFloorDbm; }

}  // namespace Spectrum
