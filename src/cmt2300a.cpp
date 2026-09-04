/*
 * cmt2300a.cpp — CMT2300A驱动实现
 *
 * 针对OOK遥控器检测优化：
 * - 使用OOK解调模式（DATA_MODE=DIRECT，RSSI 才有效）
 * - GPIO2(pin3) 输出 DOUT 原始码流，已飞线至 ESP32 GPIO9 供 OOK 解码
 * - 批量SPI写入优化频率切换速度
 */
#include "cmt2300a.h"
#include "cmt2300a_hal.h"

/* ============ 常量定义 ============ */
#define XTAL_HZ     26000000UL
#define RX_IF_HZ    (XTAL_HZ / 92)  /* ~282609 Hz */

/* 快速手动跳频（AN197）FH 步进：2.5kHz/LSB（手册标称值）。
   公式 FREQ = 基础频点 + FH_STEP_HZ × FH_OFFSET × FH_CHANNEL。
   若实测跳频落点有偏差，可按实际频率分辨率修正此值。 */
#define FH_STEP_HZ  2500

/* OOK2(0x34) bit7 = AUTO_ABS_EN：自动"绝对"门限。
   RFPDK 默认关闭(0)，此时切片器用相对(峰谷跟踪)门限——无载波时会把底噪
   放大成随机跳变，DOUT 一直在乱翻。若补齐解调区后仍出现"没按遥控也满屏脉宽"，
   把这里改成 1 重新烧录即可（唯一需要改的地方）。 */
#define CMT2300A_OOK_AUTO_ABS   0

/* OOK 解调参数表（RFPDK 1.45 导出 ook.exp：433.92MHz / OOK / 2.7kbps /
   AGC On / 带宽 Auto / Demod Middle，匹配 350us 级 EV1527、PT2262 遥控）。
   对应寄存器区 0x20-0x37，与 ook.exp 的 [Data Rate Bank] 段逐字节一致。 */
static const uint8_t kOokDataRateBank[CMT2300A_DATA_RATE_BANK_SIZE] = {
    /* 0x20 */ 0x38, 0x1B, 0x80, 0xDD,
    /* 0x24 */ 0x00, 0x00, 0x00, 0x00,
    /* 0x28 */ 0x00, 0x00, 0x00, 0x29,
    /* 0x2C */ 0xC0, 0x9D, 0x25, 0x4B,
    /* 0x30 */ 0x05, 0x00, 0x50, 0x2D,
    /* 0x34 */ 0x00, 0x01, 0x05, 0x05,
};

/* TX bank（0x55-0x5F）见下方 kSh4TxBank——直接采用官方 +20dBm RFPDK 导出
   cmt2300a_ook.exp（本工程 20dBm 模块的权威配置）。 */

/* ===== sh4_rf 量产 bank（DIRECT OOK，SPI 逻辑分析仪抓取 + 真实硅片验证）=====
   逐字节取自 sh4_rf/components/sh4_rf/cmt2300a_params_433.h（涂鸦 SH4 固件反汇编）。
   这些是 CMT2300A DIRECT OOK 在 433.92MHz 的量产配置；DIRECT TX 时主机 bit-bash DIN
   直驱，符号率由主机时序决定，故「频率无关」部分（CMT/SYSTEM/DATA_RATE/BASEBAND/TX）可
   直接照搬。频率区(0x18-0x1F) 由 SetFrequency() 按目标频点重算覆盖，不抄 433 硬编码。
   参考优先级：sh4_rf 流程+bank > 寄存器手册位定义 > vendor FIFO/Packet demo。 */
/* CMT bank（0x00-0x0B）。与官方 +20dBm RFPDK 导出 cmt2300a_ook.exp 逐字节比对，
   唯一差异是 0x03(CMT4) bit0：sh4(低功率遥控反汇编)=0x1C、exp(20dBm)=0x1D。
   ★bit0 的权威语义 = RFPDK 的 "Tx power double bit"，仅当目标功率 >16dBm 时由 RFPDK
   置位★（OpenDTU params_860.h 原注释 "(and Tx power double bit not set)"：其 20dBm
   匹配网络跑 +13dBm 时 0x03=0x1C，bit0=0）——它【不是】"20dBm 模块必须常置的 PA 高功率
   档门控"（旧注释与 commit 60d4e88 的推断是误判）。此处保留 exp 值 0x1D 只决定"复位/重
   加载后默认落在 +20dBm 档"，实际档位一律由 SetTxPower() 在每次发射前重写。 */
static const uint8_t kSh4CmtBank[12] = {       /* 0x00-0x0B */
    0x00, 0x66, 0xEC, 0x1D, 0xF0, 0x80, 0x14, 0x08, 0x91, 0x02, 0x02, 0xD0
};
static const uint8_t kSh4SystemBank[12] = {    /* 0x0C-0x17 */
    0xAE, 0xE0, 0x35, 0x00, 0x00, 0xF4, 0x10, 0xE2, 0x42, 0x20, 0x00, 0x81
};
static const uint8_t kSh4DataRateBank[24] = { /* 0x20-0x37 */
    0x32, 0x18, 0x80, 0xDD, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x29, 0xC0, 0x51, 0x2A, 0x4B,
    0x05, 0x00, 0x50, 0x2D, 0x00, 0x01, 0x05, 0x05
};
static const uint8_t kSh4BasebandBank[29] = { /* 0x38-0x54，含 PKT1=0x10(DIRECT) */
    0x10, 0x08, 0x00, 0xAA, 0x02, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xD4, 0x2D, 0x00, 0x1F,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60,
    0xFF, 0x00, 0x00, 0x1F, 0x10
};
/* TX bank（0x55-0x5F）：直接采用官方 +20dBm RFPDK 导出配置 cmt2300a_ook.exp
   （2026.08.17 生成，433.92MHz / OOK / 20dBm 匹配网络 / Tx Power +20dBm）。
   除功率字外的其余字节是"发射通路"配置，照搬 exp 不再手改：
     - TX6(0x5A)[7:5]=PA_VBLPF_SEL = 0xB0(SEL=5)：20dBm 模块的 PA 偏置/阻抗须 SEL=5；
       此前曾误改成 0x90(SEL=4)，反而偏离 20dBm，现已还原为 exp 值。
     - TX1(0x55)=0x55：bit5=0(FSK，非 GFSK) / bit4=1(PA_RAMP_EN) / bit2=1(TX_DIN_SOURCE)，
       与 DIRECT OOK 调制通路兼容（与 sh4_rf 量产值一致）。
     - 0x5F 是 CUS_LBD（低电压检测）不是 TX11，同为 exp 导出值。
   CMT bank 0x08=0x91(PA 配置) 已与 exp 一致，无需再改。
   ★TX8(0x5C)/TX9(0x5D) 只是"复位后默认值"，发射前必被 SetTxPower()（文件内 static）覆盖★
   （+20dBm 档已实测定值 0x8A18，见 SetTxPower 定义处注释）。 */
static const uint8_t kSh4TxBank[11] = {       /* 0x55-0x5F，官方 +20dBm RFPDK 导出 */
    0x55, 0x9A, 0x0C, 0x00, 0x0F, 0xB0, 0x00, 0x7A, 0x17, 0x3F, 0x7F
};

/* 频段表：定义各频段的VCO分频参数 */
typedef struct {
    uint32_t lo;        /* 频段下限 */
    uint32_t hi;        /* 频段上限 */
    uint8_t  vco;       /* VCO选择 */
    uint8_t  divx;      /* 分频系数 */
    uint8_t  div;       /* 总分频比 */
} freq_band_t;

static const freq_band_t bands[] = {
    { 126330000UL, 140000000UL, 0b110, 0b011, 12 },
    { 140000000UL, 170000000UL, 0b001, 0b011, 12 },
    { 189500000UL, 210000000UL, 0b110, 0b010, 8  },
    { 210000000UL, 255000000UL, 0b001, 0b010, 8  },
    { 252670000UL, 280000000UL, 0b110, 0b101, 6  },
    { 280000000UL, 340000000UL, 0b001, 0b101, 6  },
    { 379000000UL, 420000000UL, 0b110, 0b001, 4  },
    { 420000000UL, 510000000UL, 0b001, 0b001, 4  },
    { 758000000UL, 840000000UL, 0b110, 0b000, 2  },
    { 840000000UL, 1020000000UL, 0b001, 0b000, 2 },
};

/* ============ 基础操作 ============ */

void CMT2300A_SoftReset(void)
{
    CMT2300A_WriteReg(0x7F, 0xFF);
    delay(20);
}

bool CMT2300A_GoStby(void)
{
    CMT2300A_WriteReg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_STBY);
    /* 轮询 MODE_STA 确认已进入 STBY（AN198 1.2：状态切换命令后须查询确认，
       STBY 态 SPI 仍可读写）。RX->STBY 切换通常 <100us，轮询 1~2 次即返回；
       500us 超时兜底防异常。原固定 100us 盲等：切换快的频点白等、
       切换慢的频点（相邻频点残留态）时序不足。 */
    uint32_t t0 = micros();
    while ((CMT2300A_ReadReg(CMT2300A_CUS_MODE_STA) & 0x0F) != CMT2300A_STA_STBY) {
        if ((micros() - t0) > 500) break;
        delayMicroseconds(10);
    }
    return true;
}

/* 仅发送 go_rx 命令，不等待 PLL 锁定（锁定与 dwell 并行；读 RSSI 前用
   CMT2300A_WaitRxLocked 确认）——快速扫频省去同步等待占用的时序。 */
void CMT2300A_EnterRx(void)
{
    CMT2300A_WriteReg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_RX);
}

/* 轮询 MODE_STA 直到进入 RX 态。LOCKING_EN 已使能(CUS_EN_CTL |= MASK_LOCKING_EN)：
   发 go_rx 后芯片停在 LOCKING 态(0x08)直到 PLL 锁定再进入 RX(0x05)。
   典型锁定 50-200us(AN142)，最坏 STBY->RX 约 350us(AN198)。
   AN198 Table2: MODE_STA[3:0] 0101=RX, 1000=LOCKING。
   返回是否真在 RX：大偏移(如 330 从 303 偏 27MHz) PLL 可能超时未锁，
   此时 RSSI 读到上一频点残留值会误触发，调用方须据此放弃读数。 */
bool CMT2300A_WaitRxLocked(uint32_t timeoutUs)
{
    uint32_t t0 = micros();
    while ((CMT2300A_ReadReg(CMT2300A_CUS_MODE_STA) & 0x0F) != CMT2300A_STA_RX) {
        if ((micros() - t0) > timeoutUs) break;
        delayMicroseconds(10);
    }
    return ((CMT2300A_ReadReg(CMT2300A_CUS_MODE_STA) & 0x0F) == CMT2300A_STA_RX);
}

bool CMT2300A_GoRx(void)
{
    CMT2300A_EnterRx();
    return CMT2300A_WaitRxLocked(1000);
}

bool CMT2300A_GoSleep(void)
{
    CMT2300A_WriteReg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_SLEEP);
    delay(5);
    return true;
}

bool CMT2300A_IsExist(void)
{
    uint8_t back = CMT2300A_ReadReg(CMT2300A_CUS_PKT17);
    CMT2300A_WriteReg(CMT2300A_CUS_PKT17, 0xAA);
    uint8_t dat = CMT2300A_ReadReg(CMT2300A_CUS_PKT17);
    CMT2300A_WriteReg(CMT2300A_CUS_PKT17, back);
    return (dat == 0xAA);
}

void CMT2300A_ClearInterruptFlags(void)
{
    CMT2300A_WriteReg(CMT2300A_CUS_INT_CLR1, 0x00);
    CMT2300A_WriteReg(CMT2300A_CUS_INT_CLR2,
                      CMT2300A_MASK_PREAM_OK_CLR | CMT2300A_MASK_SYNC_OK_CLR);
}

int CMT2300A_GetRssiDBm(void)
{
    return (int)CMT2300A_ReadReg(CMT2300A_CUS_RSSI_DBM) - 128;
}

/* ============ 频率设置 ============ */

/* 快速手动跳频（AN197）状态：最后一次全量重配所在的频段下标与基准频率。
   同段内跳频只需写 FH_OFFSET(0x64)/FH_CHANNEL(0x63)（+AFC_OVF_TH），
   跨段（FREQ_DIVX_CODE / FREQ_VCO_BANK 变化）才需全量写频率区。 */
static int      g_curBandIdx   = -1;
static uint32_t g_curBandBaseHz = 0;

/* AFC_OVF_TH(0x27) 的正确值由 RFPDK 按 频点/数据率/晶体PPM 计算（AN1971 跳频计算表），
   Init 已随 OOK 解调区加载。历史实现"每次跳频重算"的公式与官方不符（系数差 20~29 倍
   + 未取 min），会把正确值覆盖成错值导致 AFC 异常、OOK 解调失败（315 能解 433.92 解不
   出的现场根因），故 SetFrequency/TuneFast 不再写 FSK4，保持 RFPDK 值。 */

void CMT2300A_SetFrequency(uint32_t freq_hz)
{
    /* 查找频段 */
    freq_band_t b;
    int bi = -1;
    for (size_t i = 0; i < sizeof(bands)/sizeof(bands[0]); i++) {
        if (freq_hz >= bands[i].lo && freq_hz < bands[i].hi) {
            b = bands[i];
            bi = (int)i;
            break;
        }
    }
    if (bi < 0) return;

    /* 计算RX N/K值（AN199：小数部分 ×2^20 后四舍五入；进位保护防 k 溢出 20bit） */
    uint64_t flo_rx = freq_hz + RX_IF_HZ;
    uint64_t nk_rx  = flo_rx * b.div;
    uint32_t n_rx   = (uint32_t)(nk_rx / XTAL_HZ);
    uint32_t k_rx   = (uint32_t)((((nk_rx % XTAL_HZ) * (1UL << 20)) + XTAL_HZ / 2) / XTAL_HZ);
    if (k_rx >= (1UL << 20)) { k_rx = 0; n_rx++; }   /* 四舍五入进位到 N */

    /* 计算TX N/K值（同上） */
    uint64_t nk_tx  = (uint64_t)freq_hz * b.div;
    uint32_t n_tx   = (uint32_t)(nk_tx / XTAL_HZ);
    uint32_t k_tx   = (uint32_t)((((nk_tx % XTAL_HZ) * (1UL << 20)) + XTAL_HZ / 2) / XTAL_HZ);
    if (k_tx >= (1UL << 20)) { k_tx = 0; n_tx++; }

    uint8_t paldo = (freq_hz >= 500000000UL) ? 1 : 0;

    /* 批量写入RF寄存器（9个寄存器一次SPI传输，含 AFC_OVF_TH） */
    uint8_t addr[] = {
        CMT2300A_CUS_RF1, CMT2300A_CUS_RF2, CMT2300A_CUS_RF3, CMT2300A_CUS_RF4,
        CMT2300A_CUS_RF5, CMT2300A_CUS_RF6, CMT2300A_CUS_RF7, CMT2300A_CUS_RF8,
        CMT2300A_CUS_FSK4
    };
    uint8_t data[] = {
        (uint8_t)n_rx,
        (uint8_t)(k_rx & 0xFF),
        (uint8_t)((k_rx >> 8) & 0xFF),
        (uint8_t)((paldo << 7) | (b.divx << 4) | ((k_rx >> 16) & 0x0F)),
        (uint8_t)n_tx,
        (uint8_t)(k_tx & 0xFF),
        (uint8_t)((k_tx >> 8) & 0xFF),
        (uint8_t)((b.vco << 4) | ((k_tx >> 16) & 0x0F)),
        /* AFC_OVF_TH：保持 RFPDK 导出值，勿重算覆盖（见 SetFrequency 上方注释） */
        kOokDataRateBank[CMT2300A_CUS_FSK4 - CMT2300A_DATA_RATE_BANK_ADDR]
    };
    CMT2300A_WriteRegsMulti(addr, data, 9);

    /* 清快速手动跳频偏移（0x63/0x64），否则残留偏移会叠加到新基准频点上 */
    CMT2300A_WriteReg(CMT2300A_CUS_FREQ_CHNL, 0);
    CMT2300A_WriteReg(CMT2300A_CUS_FREQ_OFS, 0);
    g_curBandIdx   = bi;
    g_curBandBaseHz = freq_hz;
}

/* 段感知快速跳频（AN197 快速手动跳频）：
   同段（FREQ_DIVX_CODE / FREQ_VCO_BANK 均不变）且目标不落后于基准时，
   只写 FH_OFFSET(0x64) + FH_CHANNEL(0x63) 两个寄存器（AFC_OVF_TH 保持 RFPDK 值，
   不在此重算——见 SetFrequency 上方注释），
   跳频公式：FREQ = 基准 + 2.5kHz × FH_OFFSET × FH_CHANNEL。
   跨段 / 首跳 / 偏移无法分解（2.5kHz 整数因子 >255）时回落全量 SetFrequency。
   注意：调用前须已 GoStby（手册流程：go_stby -> 设 FH -> go_rx）。
   返回实际生效频率 Hz；0 = 不可调谐（空洞频点）。 */
uint32_t CMT2300A_TuneFast(uint32_t freq_hz)
{
    int bi = -1;
    for (size_t i = 0; i < sizeof(bands)/sizeof(bands[0]); i++) {
        if (freq_hz >= bands[i].lo && freq_hz < bands[i].hi) { bi = (int)i; break; }
    }
    if (bi < 0) return 0;                       /* 空洞频点：不可调谐 */

    if (bi == g_curBandIdx && freq_hz >= g_curBandBaseHz) {
        int64_t offHz = (int64_t)freq_hz - g_curBandBaseHz;
        /* FH 步进为 2.5kHz/LSB（AN197 公式，标称值；若实测有偏差改这里）。
           偏移必须是其整数倍，否则快速跳频落点不准，直接回落全量重配。 */
        if (offHz % FH_STEP_HZ == 0) {
            int64_t unit = offHz / FH_STEP_HZ;  /* FH_STEP_HZ 单位 */
            /* 分解 FH_OFFSET × FH_CHANNEL = unit（两者均 8bit，最大 255×255=65025 单位 ≈162.6MHz） */
            uint8_t off = 0, chn = 0;
            for (uint32_t o = 1; o <= 255; o++) {
                if (unit % (int64_t)o == 0) {
                    int64_t c = unit / (int64_t)o;
                    if (c <= 255) { off = (uint8_t)o; chn = (uint8_t)c; break; }
                }
            }
            if (chn != 0 || unit == 0) {        /* 分解成功（unit==0 即回基准，off/chn 写 0） */
                CMT2300A_WriteReg(CMT2300A_CUS_FREQ_OFS, off);
                CMT2300A_WriteReg(CMT2300A_CUS_FREQ_CHNL, chn);
                return freq_hz;
            }
            /* 偏移无法分解为 ≤255×≤255：回落全量（同段但频点距基准太远） */
        }
    }

    CMT2300A_SetFrequency(freq_hz);
    return freq_hz;
}

bool CMT2300A_IsFreqSupported(uint32_t freq_hz)
{
    for (size_t i = 0; i < sizeof(bands) / sizeof(bands[0]); i++) {
        if (freq_hz >= bands[i].lo && freq_hz < bands[i].hi) {
            return true;
        }
    }
    return false;
}

/* ============ 初始化 ============ */

bool CMT2300A_Init(void)
{
    CMT2300A_InitGpio();

    /* 软复位 */
    CMT2300A_SoftReset();

    /* 配置晶振和系统参数（SYS3：XTAL_STB_TIME=2480us，STBY 态常开晶振不受影响） */
    CMT2300A_WriteReg(CMT2300A_CUS_SYS2, 0x00);
    CMT2300A_WriteReg(CMT2300A_CUS_SYS3, 0x75);
    CMT2300A_WriteReg(CMT2300A_CUS_CMT10,
                      (CMT2300A_ReadReg(CMT2300A_CUS_CMT10) & ~0x07) | 0x02);

    /* 进入STBY */
    CMT2300A_GoStby();

    /* ===== 加载 OOK 解调参数区（0x20-0x37）=====
       此前 Init 从未写过这一区，芯片一直用【复位默认】的符号率/接收带宽/AGC/
       切片门限去解调 OOK，于是 GPIO2(DOUT) 输出的是没解对的噪声包络——
       表现为：频谱在 433.92 明明有 -43dBm 强载波，DOUT 却全程 8~450us 随机乱翻、
       无帧间静默，解码器只能吐原始符号。这是 OOK 解不出码的根因。

       安全性（与频谱通路无重叠，不会打回平线）：
         - 本区不含 PKT1/DATA_MODE(0x38)、IO_SEL(0x65)、INT_EN(0x68)、频率区(0x18-0x1F)；
         - 区内 0x27(AFC_OVF_TH) 由 SetFrequency 每次跳频重算覆盖，此处初值无影响；
         - 仅在 Init 执行一次，不在扫描循环里改写寄存器。
       为什么不加载 Baseband bank(0x38-0x54)：它的首字节就是 PKT1，RFPDK 默认
       DATA_MODE=Packet(0x12)，写下去 RSSI_DBM 会恒 0 → 频谱直接变平线；而且
       DIRECT 模式下收包引擎(前导/同步字/CRC/FIFO)本就旁路，那一区对解码没有意义。
       注意：参考工程 firmware/src/cmt2300a_params.h 的同名数组只有 23 字节
       （漏了 0x2A），0x2A 之后整体错位，故此处以 RFPDK 原始导出 ook.exp 为准。 */
    for (uint8_t i = 0; i < CMT2300A_DATA_RATE_BANK_SIZE; i++) {
        CMT2300A_WriteReg(CMT2300A_DATA_RATE_BANK_ADDR + i, kOokDataRateBank[i]);
    }
#if CMT2300A_OOK_AUTO_ABS
    CMT2300A_WriteReg(CMT2300A_CUS_OOK2, 0x80);   /* AUTO_ABS_EN：无载波时抑制底噪乱跳 */
#endif

    /* 配置 GPIO 功能（CUS_IO_SEL 0x65，AN192/AN143 权威定义）：
       GPIO1_SEL[1:0]=00 -> DOUT/DIN（本扫描器只读 RSSI + GPIO2 采码流，不用 GPIO1）
       GPIO2_SEL[3:2]=10 -> DOUT/DIN：CMT2300A GPIO2(pin3) 输出解调后的原始 OOK 码流，
                            已飞线至 ESP32 GPIO9，供 OOK 解码（ook_decoder）使用。
       GPIO3_SEL[5:4] 复位为 00(CLKO)：TX 时会被 TxOokBegin 改成 DIN，此处确保回到 RX 态时 GPIO3 不再是 DIN 输入。 */
    uint8_t io_sel = CMT2300A_ReadReg(CMT2300A_CUS_IO_SEL);
    io_sel &= ~0x3F;          // 清 GPIO1_SEL[1:0] + GPIO2_SEL[3:2] + GPIO3_SEL[5:4]（保留 GPIO4）
    io_sel |= 0x08;           // GPIO2_SEL[3:2]=0b10 => DOUT/DIN
    CMT2300A_WriteReg(CMT2300A_CUS_IO_SEL, io_sel);

    /* 保留历史固件的两处寄存器配置（与 GPIO1 空 ISR 无关，空 ISR 已删除）：
       - INT_EN=0x10（PREAM_OK 中断使能；当前 GPIO1 配为 DOUT/DIN，该中断无输出路径，仅维持复位前行为）
       - PKT5=0x08（SYNC_TOL/SYNC_SIZE；DIRECT 模式旁路，保留复位前行为） */
    CMT2300A_WriteReg(CMT2300A_CUS_INT_EN, 0x10);
    CMT2300A_WriteReg(CMT2300A_CUS_PKT5, 0x08);

    /* 配置为OOK模式（参照参考基线 longcat——20260808 的 PKT1 配置） */
    uint8_t pkt1 = CMT2300A_ReadReg(CMT2300A_CUS_PKT1);
    pkt1 &= ~(0x03 << 5);
    pkt1 |= (0x01 << 5);   /* RX_PREAM_SIZE[6:5]=01 */
    pkt1 &= ~(0x03 << 3);  /* RX_PREAM_SIZE[4:3]=00 */
    /* 关键：强制 DATA_MODE = DIRECT(0x00)。
       PKT1[1:0]=DATA_MODE（权威定义 CMT2300A_MASK_DATA_MODE=0x03）：
       DIRECT(0x00) 时 RSSI_DBM 才正确反映信号强度；PACKET(0x02) 会让 RSSI_DBM 恒 0 → GetRssiDBm 恒 -128（频谱平线）。
       参考基线只清了 [4:3] 位（误注"数据模式=原始"），并未强制 DIRECT，故须在此补回，否则频谱失效。 */
    pkt1 &= ~0x03;         /* DATA_MODE = DIRECT(0x00) */
    CMT2300A_WriteReg(CMT2300A_CUS_PKT1, pkt1);

    /* 配置控制区 */
    uint8_t tmp = CMT2300A_ReadReg(CMT2300A_CUS_MODE_STA);
    tmp |= CMT2300A_MASK_CFG_RETAIN;
    CMT2300A_WriteReg(CMT2300A_CUS_MODE_STA, tmp);

    tmp = CMT2300A_ReadReg(CMT2300A_CUS_EN_CTL);
    tmp |= CMT2300A_MASK_LOCKING_EN;
    CMT2300A_WriteReg(CMT2300A_CUS_EN_CTL, tmp);

    CMT2300A_ClearInterruptFlags();

    return true;
}

/* ============ 发射（DIRECT OOK 回放） ============
 * 每次发射前完整 SoftReset 重初始化（不保留 RX 状态打补丁），退出发射由 CMT2300A_Init()
 * 完整恢复 RX（含 SoftReset 重载 EEPROM 默认 + OOK 解调区 + GPIO 配置）。
 *
 * 参考优先级（用户明确）：涂鸦量产固件 start_tx()（DIRECT OOK，真实硅片验证）> 寄存器手册
 * > 厂商官方 demo / EasyCMT2300A 库（后者走 FIFO/Packet 模式，与 DIRECT 不同通路，降为最低）。
 * ★但须区分两类寄存器★：
 *   (a)「模式使能项」——EN_CTL[5]=LOCKING_EN、INT_EN=0x3D、SYS2[5] 清：与板无关，照搬涂鸦；
 *   (b)「引脚路由项」——IO_SEL、FIFO_CTL 的 TX_DIN_SEL：与板级接线绑定，**必须按本板翻译**。
 *   涂鸦 IO_SEL=0x0A 解出 GPIO1=INT2、GPIO2=DOUT/DIN，说明它的 DIN 在 GPIO2；
 *   本板 DIN 在 GPIO1（ESP32 GPIO10 -> CMT GPIO1），照抄 0x0A 会把 GPIO1 变成 INT2 输出，
 *   DIN 断开 -> OOK 输入恒 0 -> 载波被门控关断（实测：接收端连 RSSI 命中都没有）。
 *
 * 排查历程（三次实测逐步收敛）：
 *   1. 只有 FIFO_CTL[7]：TX 出载波（RSSI≈-67dBm）但 pulses=0 -> 缺 DIN 门控 EN_CTL[5]；
 *   2. 补 EN_CTL[5] 后仍 pulses=0 -> 说明还缺涂鸦的 INT_EN/SYS2 等使能项；
 *   3. 全量照抄涂鸦（含 IO_SEL=0x0A）-> 载波完全消失（RSSI 全无）= 引脚路由被改错，
 *      由此定位 (a)/(b) 两类寄存器必须分别对待。当前实现即为该结论的落地。 */

/* 功率设置实现在本文件末尾（static，不对外暴露），此处前置声明以便在 TxOokBegin 内调用。
   完整文档（功率字表来源、RFPDK 交叉验证、CMT4 bit0 语义）见其定义处。 */
static void SetTxPower(int8_t dBm);

void CMT2300A_TxOokBegin(uint32_t freq_hz, int8_t dBm)
{
    /* 1) 软复位 -> 重载 EEPROM 默认（CMT/System/Baseband 回到 OOK 出厂值） */
    CMT2300A_SoftReset();
    CMT2300A_GoStby();

    /* 2) 加载 sh4_rf 量产 bank（DIRECT OOK 完整配置，SPI 逻辑分析仪抓取、真实硅片验证）。
          频率区(0x18-0x1F) 不在此写，由 SetFrequency() 重算覆盖；其余 bank 频率无关照搬。
          参考优先级：sh4_rf bank > 寄存器手册位定义 > vendor FIFO/Packet demo。

          ★CMT2300A 是平坦寄存器映射，没有分页★（中文 Datasheet Rev1.1 p38-39 原文：
          "地址从 0x00 到 0x71……地址是连续的，操作方式无本质区别，都是使用 SPI 按照访问
          寄存器的时序进行直接读写操作"）。旧注释所称"写 bank 前必须先向页选寄存器写页值"
          是误判，那 5 条写（0x00←0x00 / 0x0C←0x0C / 0x20←0x20 / 0x38←0x38 / 0x55←0x55）
          实打实把脏值写进了 CMT1/SYS1/RF9/PKT1/TX1，只是被紧随的 bank 循环覆盖才没出事；
          一旦循环长度或顺序变动就会留下隐患，故删除。 */
    for (uint8_t i = 0; i < 12; i++) CMT2300A_WriteReg(0x00 + i, kSh4CmtBank[i]);
    for (uint8_t i = 0; i < 12; i++) CMT2300A_WriteReg(0x0C + i, kSh4SystemBank[i]);
    for (uint8_t i = 0; i < 24; i++) CMT2300A_WriteReg(0x20 + i, kSh4DataRateBank[i]);
    for (uint8_t i = 0; i < 29; i++) CMT2300A_WriteReg(0x38 + i, kSh4BasebandBank[i]);
    for (uint8_t i = 0; i < CMT2300A_CUS_TX_BANK_SIZE; i++)
        CMT2300A_WriteReg(CMT2300A_CUS_TX_BANK_ADDR + i, kSh4TxBank[i]);

    /* 3) TX bank 已随上一步 0x55-0x5F 一并写入 kSh4TxBank（与上一步合并，无需再写） */

    /* 4) 晶振稳定时间 xosc_aac = 2 */
    CMT2300A_WriteReg(CMT2300A_CUS_CMT10,
                      (CMT2300A_ReadReg(CMT2300A_CUS_CMT10) & ~0x07) | 0x02);

    /* 5) 频率（RF bank 0x18-0x1F） */
    CMT2300A_SetFrequency(freq_hz);

    /* 注：DATA_MODE=DIRECT 已由 BASEBAND bank(PKT1=0x10) 载入，无需单独写 PKT1 */

    /* ===== 以下 7~11 步：★以 CMOSTEK 官方 DIRECT 模式 StartTx 为唯一基准（datasheet V1.8 §6.1）=====
       V1.8 §6.1 Tx 序列：① TX_DIN_EN=1 使能 DIN；② TX_DIN_SEL=00→GPIO1 / 01→GPIO2 / 10→GPIO3；
       ③ go_tx，DIN 上送数据即发射。全程不涉及 FIFO_MERGE_EN（那是 FIFO/Packet 模式功能，§5.2）。
       本板 DIN 接 CMT GPIO3（ESP32 GPIO10 -> CMT GPIO3 新飞线），故 TX_DIN_SEL=0x40(GPIO3)。 */

    /* 7) IO_SEL：本板 TX 改用 CMT GPIO3 作 DIN（ESP32 GPIO10 -> CMT GPIO3，新飞线）。
       GPIO3_SEL[5:4] = 0x10 = DIN/DOUT；GPIO1 不再作 DIN -> 配 INT1(0x01) 避免争用数据功能；
       GPIO2 维持 RX 的 DOUT 配置（TX 不占用 GPIO2，无冲突，由 Init 设定）。
       掩码 = MASK_GPIO12_SEL(0x0F) | MASK_GPIO3_SEL(0x30) = 0x3F：
       清 GPIO1_SEL[1:0]+GPIO2_SEL[3:2]+GPIO3_SEL[5:4]（保留 GPIO4）。 */
    uint8_t io = CMT2300A_ReadReg(CMT2300A_CUS_IO_SEL);
    io = (io & ~(CMT2300A_MASK_GPIO12_SEL | CMT2300A_MASK_GPIO3_SEL))
         | CMT2300A_GPIO3_SEL_DIN | CMT2300A_GPIO1_SEL_INT1;
    CMT2300A_WriteReg(CMT2300A_CUS_IO_SEL, io);

    /* 7b) INT2_CTL：INT2 源 = 0x0A(TX_DONE)，并★清 bit5=TX_DIN_INV★（正逻辑 DIN）。
       掩码用 ~0x3F：清 INT2_SEL[4:0] + TX_DIN_INV[5]，保留 bit6=LFOSC_OUT_EN。
       若 EEPROM/bank 默认 bit5=1，DIN 被硅片反相 -> 整帧码型反码（即便调制通了也解不出预期值），故一并清掉。 */
    uint8_t int2 = CMT2300A_ReadReg(CMT2300A_CUS_INT2_CTL);
    int2 = (int2 & ~0x3Fu) | 0x0Au;
    CMT2300A_WriteReg(CMT2300A_CUS_INT2_CTL, int2);

    /* 8) INT_EN = 0x3D（涂鸦 step2；DIRECT 模式必设项，之前遗漏） */
    CMT2300A_WriteReg(CMT2300A_CUS_INT_EN, 0x3D);

    /* 9) SYS2(0x0D) 仅清 bit5(LFOSC_CAL2_EN)，保留 bit7/6(LFOSC_RECAL_EN/LFOSC_CAL1_EN)。
           对齐 sh4_rf 量产固件 start_tx 的 SYS2 &= ~0x20（Bank 载入值 0xE0 -> 0xC0）。
           ★关键修复★：旧版误写成 CMT2300A_WriteReg(SYS2, 0x00) 整清，把 bit7/6 也关掉，
           导致 LFOSC 失去重校准/校准 -> DIN->PA 采样时钟失准 -> 「载波开、DIN 不调制」(pulses=1)。
           注：rfpdk .exp 的 System Bank 0x0D=0xE0，sh4_rf 在其后 &=~0x20 -> 0xC0，本工程须与之完全一致。 */
    uint8_t sys2 = CMT2300A_ReadReg(CMT2300A_CUS_SYS2);
    sys2 &= ~0x20u;
    CMT2300A_WriteReg(CMT2300A_CUS_SYS2, sys2);

    /* 10) EN_CTL(0x62) bit5 = UNLOCK_STOP_EN（CMT2300A 同名 LOCKING_EN）：PLL 未锁禁止进 TX。
           这【不是】DIN 调制门控——DIN 调制使能 = FIFO_CTL[7]=TX_DIN_EN + TX1[2]=TX_DIN_SOURCE。
           未置位则切 TX 被拦在 LOCKING 态(0x08)不进 TX(0x06) -> 无载波。 */
    uint8_t en = CMT2300A_ReadReg(CMT2300A_CUS_EN_CTL);
    en |= CMT2300A_MASK_LOCKING_EN;
    CMT2300A_WriteReg(CMT2300A_CUS_EN_CTL, en);

    /* 11) ★datasheet V1.8 §6.1 定论★ FIFO_CTL(0x69)：DIRECT(OOK) 的「DIN -> PA」通路只需两件事：
           bit7  = TX_DIN_EN(0x80)      : 置 1 —— 使能 DIN 输入驱动 PA（§6.1 Tx step1）
           [6:5] = TX_DIN_SEL(0x40)     : 选 GPIO3 作 DIN 源（§6.1 Tx step2；本板 ESP32 GPIO10 -> CMT GPIO3 飞线）
           bit1  = FIFO_MERGE_EN(0x02)  : ★必须清 0★ —— 仅 FIFO/Packet 模式合并 TX/RX FIFO 用（§5.2），
                                           与 DIN->PA 调制无关；DIRECT 发射置 1 只会打乱 FIFO 组织，无益。
           最终 = 0xC0。
           ★纠正历史误判★：此前设成 0x82（多置 bit1），依据是 sh4_rf 反汇编注释把 `FIFO_CTL|=0x02`
           标成 "EnableTxDinInvert"（实为 INT2_CTL[5]）。V1.8 §6.1 实证 DIRECT TX 序列完全不碰 FIFO_MERGE_EN。 */
    uint8_t fifo = CMT2300A_ReadReg(CMT2300A_CUS_FIFO_CTL);
    fifo = (fifo & ~CMT2300A_MASK_TX_DIN_SEL & ~CMT2300A_MASK_FIFO_MERGE_EN)
           | CMT2300A_MASK_TX_DIN_EN | CMT2300A_TX_DIN_SEL_GPIO3;
    CMT2300A_WriteReg(CMT2300A_CUS_FIFO_CTL, fifo);

    /* 11c) TX1(0x55)[2]=TX_DIN_SOURCE 显式置位（与 FIFO_CTL[7] 共构 DIN 调制使能） */
    uint8_t tx1 = CMT2300A_ReadReg(CMT2300A_CUS_TX_BANK_ADDR);
    tx1 |= CMT2300A_MASK_TX_DIN_SOURCE;
    CMT2300A_WriteReg(CMT2300A_CUS_TX_BANK_ADDR, tx1);

    /* 11b) DIN(ESP32 GPIO10 -> CMT GPIO3) 由 OOK-RMT 通道驱动（见 spectrum.cpp 的 OOK-RMT 段），不再 digitalWrite 直驱。 */

    /* 12) ★应用发射功率★——必须放在全部 bank 写完之后、GoSleep 之前，两个原因：
          (a) 本函数开头 SoftReset + bank 全量重写会覆盖 0x03/0x5C/0x5D，调用方在
              TxOokBegin 之前调 SetTxPower 会被整组抹掉（历史 bug：功率档位永远不生效）；
          (b) AN142 §3.5 step10 要求"改完配置区 -> go_sleep 让配置生效"，紧随其后的
              第 13 步 GoSleep 正好满足，无需额外状态切换。 */
    SetTxPower(dBm);

    /* 13) GoSleep -> GoStby -> GoTx -> 轮询进入 TX 态（等 PLL 锁定 + PA 武装）。
           补 GoStby：状态机先把上面新写的 TX_DIN_EN / TX_DIN_SEL / IO_SEL 真正生效后，再进 TX。 */
    CMT2300A_GoSleep();
    delayMicroseconds(200);
    CMT2300A_GoStby();
    delayMicroseconds(200);
    CMT2300A_WriteReg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_TX);
    uint32_t tx_ready_t0 = micros();
    while ((CMT2300A_ReadReg(CMT2300A_CUS_MODE_STA) & 0x0F) != CMT2300A_STA_TX) {
        if ((micros() - tx_ready_t0) > 5000) break;
        delayMicroseconds(10);
    }
}

/* 设置发射功率档位（-10 ~ +20 dBm，步进 1 dB）。
   ★文件内私有（static），不对外暴露★：本函数必须在 TxOokBegin 内部、全部 bank 写完
   之后调用；由外部在 TxOokBegin 之前调用会被开头的 SoftReset + bank 全量重写抹掉
   （历史 bug：功率档位永远不生效）。设为 static 可从编译期杜绝该用法再次出现。
   如需运行时改功率，重新进 TxOokBegin(freq, dBm) 即可（内部含完整生效序列）。

   功率字表完全参考 OpenDTU（tbnobody/OpenDTU lib/CMT2300a/cmt2300wrapper.cpp
   setPALevel()，TRx Matching Network = 20 dBm），已用两份 RFPDK 导出交叉验证：
     - OpenDTU params_860 / params_900（20dBm 匹配网络，Tx Power=+13dBm）
       -> 0x5C/0x5D = 0x53/0x09，正是本表 13dBm 档的 0x5309；
     - AN142 官方示例（433.92MHz / FSK / +16dBm）-> 0x5C/0x5D = 0x72/0x0C，
       与本表 16dBm 档 0x7D0C 仅高字节差 0x0B（低字节一致）。
   即：功率字与频率基本无关，但 433MHz 下绝对 dBm 仍有约 1dB 级偏差，
   ★本表在 433.92MHz 只能当"相对刻度"用，绝对功率需实测校准（见 +20dBm 档 TODO）★。
   16 位功率字大端写入 TX8(0x5C)=高字节 / TX9(0x5D)=低字节。

   ★CMT4 bit0 的权威语义 = RFPDK 的 "Tx power double bit"，仅目标功率 >16dBm 时置位★
   （OpenDTU params_860.h 原注释即 "(and Tx power double bit not set)"，其 20dBm
   匹配网络跑 +13dBm 时 CMT4=0x1C，bit0=0）。它【不是】"20dBm 模块必须常置的 PA 高功率
   档门控"——bank 里默认 0x1D 只代表"复位后落在 +20dBm 档"，实际档位一律由本函数重写。

   ★注意：本函数只改 0x03/0x5C/0x5D，TX6(0x5A)/CMT9(0x08) 等 PA 偏置沿用 bank 的
   20dBm 标定值不随档位变化（OpenDTU setPALevel 同样如此），故低功率档（-10~0dBm）
   的实际输出可能偏离标称值，需实测确认。

   @param dBm  -10 ~ 20；越界值忽略不写
*/
static void SetTxPower(int8_t dBm)
{
    uint16_t word;
    switch (dBm) {
    case -10: word = 0x0501; break;
    case  -9: word = 0x0601; break;
    case  -8: word = 0x0701; break;
    case  -7: word = 0x0801; break;
    case  -6: word = 0x0901; break;
    case  -5: word = 0x0A01; break;
    case  -4: word = 0x0B01; break;
    case  -3: word = 0x0C01; break;
    case  -2: word = 0x0D01; break;
    case  -1: word = 0x0E01; break;
    case   0: word = 0x1002; break;
    case   1: word = 0x1302; break;
    case   2: word = 0x1602; break;
    case   3: word = 0x1902; break;
    case   4: word = 0x1C02; break;
    case   5: word = 0x1F03; break;
    case   6: word = 0x2403; break;
    case   7: word = 0x2804; break;
    case   8: word = 0x2D04; break;
    case   9: word = 0x3305; break;
    case  10: word = 0x3906; break;
    case  11: word = 0x4107; break;
    case  12: word = 0x4908; break;
    case  13: word = 0x5309; break;
    case  14: word = 0x5E0B; break;
    case  15: word = 0x6C0C; break;
    case  16: word = 0x7D0C; break;
    /* 以下档位需 CMT4 bit0 = double Tx value */
    case  17: word = 0x4A0C; break;
    case  18: word = 0x580F; break;
    case  19: word = 0x6B12; break;
    /* ★+20dBm 档定值（实测确定）★：本工程 433.92MHz / 20dBm 匹配网络下实测对比，
       OpenDTU 表（RFPDK 官方标定）0x8A18 的信号高于 RFPDK 导出 exp 的 0x7A17
       （两值低字节几乎一致、高字节差 0x10），故最终采用 0x8A18。 */
    case  20: word = 0x8A18; break;   /* 实测确定（exp 0x7A17 信号更低，已弃用） */
    default: return;                  /* 越界档位不改 */
    }
    uint8_t cmt4 = CMT2300A_ReadReg(CMT2300A_CUS_CMT4);
    if (dBm > 16) cmt4 |= 0x01; else cmt4 &= 0xFE;
    CMT2300A_WriteReg(CMT2300A_CUS_CMT4, cmt4);
    CMT2300A_WriteReg(CMT2300A_CUS_TX8, word >> 8);
    CMT2300A_WriteReg(CMT2300A_CUS_TX9, word & 0xFF);
}

void CMT2300A_TxOokEnd(void)
{
    /* 直接完整重初始化回接收态（SoftReset 会清掉 TX_DIN_EN 等 TX 配置，无需手动清） */
    CMT2300A_Init();
}
