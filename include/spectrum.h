/*
 * spectrum.h — 扫频引擎 + 突发检测 + 遥控频点归一化
 */
#pragma once
#include <Arduino.h>
#include "config.h"

struct ScanSettings {
    uint32_t startHz    = DEF_START_HZ;
    uint32_t stopHz     = DEF_STOP_HZ;
    uint32_t stepHz     = DEF_STEP_HZ;
    uint16_t dwellUs    = DEF_DWELL_US;
    float    thresholdDb= DEF_THRESH_DB;  // 触发门限(dB)：范围扫频峰值检测 + 快嗅探实时载波触发共用
    float    decayDb    = DEF_DECAY_DB;
    float    normalizedTolMHz = DEF_NORMALIZED_TOL_MHZ;
    bool     normalizedEnable = true;   // 归一化开关：命中后锁定/解码用归一化频点；关=用实测峰值频率
    bool     running    = true;
    bool     armed      = true;      // 布防：允许输出捕获事件
    uint8_t  mode       = DEF_MODE;  // 0 = 范围扫频, 1 = 常见频点扫频
    bool     autoDecode = DEF_AUTO_DECODE;  // 命中频率后顺带 OOK 解码按键编码
    int8_t   txPowerDbm = DEF_TX_POWER_DBM; // 发射功率档位（-10~20 dBm）
    char     normalizedList[256] = DEF_NORMALIZED_LIST;
    char     presetList[256] = DEF_PRESET_LIST;  // 常见频点扫频的频点列表(MHz)
};

struct Hit {
    uint32_t tsMs;
    float    peakMHz;        // 抛物线插值后的实测峰值频率
    float    normalizedMHz;  // 归一化命中频点，<=0 表示未匹配
    float    peakDbm;        // 峰值电平
    float    floorDbm;       // 该点的本底
    float    deltaDb;        // 高出本底
    uint16_t durationMs;     // 突发持续时间
    float    cand[3];        // Top3 候选频点
    float    candDist[3];    // 与实测峰值的距离 (MHz)
};

struct DecodeResult {
    uint32_t tsMs;
    uint32_t code;           // 解码出的整数码（时长域，multi-frame 复核通过）
    uint8_t  bits;           // 码长（位）
    uint8_t  proto;          // 协议号（1=已识别 -1=有数据未识别 -2=无数据）
    uint16_t delayUs;        // 一个比特的时长(µs)
    float    freqMHz;        // 锁定频率
    float    dbm;            // 该频点电平
    /* 以下为 longcat 时长域解码器新增字段 */
    uint32_t te;             // 单位脉宽 Te (µs)
    uint8_t  repeat;         // 多帧一致帧数（>=2 才输出码值）
    uint8_t  frames;         // 切出的完整帧数
    char     hex[32];        // 十六进制串（完整码值，优先于 code 字段）
    char     protoName[28];  // 协议名（"EV1527/PT2262" / "PWM-OOK(未识别)" 等）
};

namespace Spectrum {

void begin();                                   // 启动扫频任务
ScanSettings getSettings();
void         applySettings(const ScanSettings& s);  // 线程安全，下一轮扫描生效
void         saveSettings();
void         loadSettings();
void         resetToFactory();                       // 清空 NVS 配置并重启（恢复初始）

void  resetBaseline();
void  clearMaxHold();

// 取一帧数据（供 WebSocket 发送）。返回 bin 数，0 = 无新帧
uint16_t takeFrame(uint8_t* live, uint8_t* maxHold, uint16_t maxLen,
                   uint32_t& startHz, uint32_t& stepHz, uint16_t& seq);

// 取当前模式每个 bin 的实际频率 (Hz)，供 WebSocket 下发频率表（预设模式）。
// 返回 true 表示预设模式且已填充 n 个频率；outHz 长度需 >= maxN。
bool    takeFreqTable(uint32_t* outHz, uint16_t maxN, uint16_t& n);

bool  popHit(Hit& h);                            // 取出一条捕获事件
bool  popDecode(DecodeResult& r);                 // 取出一条解码结果

// 解码记录回放发射：freqHz 目标频率、code 码值、te 单比特时长(us)、bits 码长
void  requestReplay(uint32_t freqHz, uint32_t code, uint32_t te, uint8_t bits);

uint16_t binCount();
float  sweepsPerSec();
bool   radioOk();
bool   demoMode();
float  lastPeakDbm();
float  lastFloorDbm();

}  // namespace Spectrum
