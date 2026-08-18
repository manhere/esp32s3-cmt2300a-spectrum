/*
 * ook_decoder.h — OOK 遥控器码流采集与解码
 *
 * 硬件前提（用户已飞线）：
 *   CMT2300A GPIO2(pin3) -> ESP32 GPIO9（DOUT，输出解调后的原始 OOK 码流）
 *   CMT2300A 须处于 DATA_MODE=DIRECT（本项目已强制），RX 态下 GPIO2 即输出解调数据。
 *
 * 采集方式：ESP32 GPIO9 上挂 CHANGE 中断，记录每次跳变的时间戳（micros），
 * 离线还原 high/low 脉宽序列。不依赖 RMT 驱动，避免 Arduino-ESP32 2.x/3.x
 * RMT API 差异带来的编译风险（对 ~250us 单位脉冲，micros() 精度足够）。
 *
 * 解码：EV1527 / PT2262 / HT12E / HT6P20B / HS2303 等 PWM 类 OOK 固定码，
 * 判决在**时长域**完成（不依赖 Te 量化），并始终输出原始脉宽便于人工判读。
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define OOK_DOUT_PIN       9      /* ESP32 GPIO9 <- CMT2300A GPIO2(DOUT)，已飞线 */
#define OOK_MAX_PULSES     1200   /* 单轮采集容量（脉宽段数）；EV1527 一帧约 50 段 */

/* 采集必须跨越"帧间同步间隙"抓到多个完整帧，否则每次都是从帧中途随机起点截一段，
   同一个按键每次解出的码都不一样（历史 bug：旧版 OOK_SILENCE_US=12000us 会在
   EV1527 的 31T 同步低电平处直接结束采集，恰好把每帧切断在随机位置）。
   帧数要留足冗余：切片器毛刺会污染个别帧，多抓几帧才能靠投票把它们淘汰掉。 */
#define OOK_GAP_US          3000  /* >=3ms 无跳变视为帧边界（数据段最长 3T 约 1.2ms） */
/* 采到这么多帧边界即收工。原 14（≈7 帧）让"命中即解码"阻塞 scanTask 约 200ms，
   前端频谱冻结成卡顿感（2026-08-14 反馈）。EV1527 一帧 24bit≈19ms，4 帧投票
   即使有 1-2 帧被毛刺污染仍能凑齐 repeat>=2 复核，可靠性足够；解码卡顿减半。 */
#define OOK_WANT_GAPS       8     /* 采到 4 个完整帧即收工（原 14≈7 帧） */
#define OOK_NOSIG_US        100000/* 100ms 完全无跳变 = 没信号，提前退出（原 200ms） */
#define OOK_DECODE_TIMEOUT_MS 1000 /* 单次解码总窗口（ms，含多轮重采）；2000→1000：命中后更快恢复扫频 */

/* ★重采机制★ 载波消失后 CMT2300A 的 AGC 会拉满，DOUT 变成随机包络噪声，
   几百毫秒就能把缓冲灌满，把真正的遥控帧挤出窗口（实测 433 遥控 5706068 漏解：
   800 段里只有开头 18 段是真信号，其余全是噪声，Te 被带偏到 294us）。
   对策：一轮采满后立刻试解，解不出就清空重采，直到总窗口耗尽。 */
#define OOK_MIN_RETRY_MS    350   /* 剩余时间不足这么多就不再重采 */

/* 毛刺门限：CMT2300A 切片器在包络边沿会吐出几十 us 的窄脉冲，会把一个 3T 段
   劈成三段，既污染 Te 直方图又打乱 (高,低) 配对。解码前先按 Te/3 合并掉。 */
#define OOK_GLITCH_MIN_US   40    /* 毛刺阈值下限 */
#define OOK_GLITCH_MAX_US   150   /* 毛刺阈值上限（Te 很大时也不能误吞真实 1T） */
#define OOK_TE_MIN_US       50    /* Te 合理下限：低于此值一律视为噪声 */

typedef struct {
    float    freq;        /* 解码目标频点 MHz */
    uint32_t te;          /* 单位脉宽 us（由码元周期反推；0=未估出） */
    int      proto;       /* >=1: 已解出码值  -1: 有数据但未识别  -2: 无数据 */
    char     proto_name[28]; /* 协议名（"EV1527/PT2262" / "PWM-OOK(未识别)" 等） */
    uint32_t bits;        /* 解码出的比特数 */
    char     hex[32];     /* 十六进制串；未识别时为空 */
    char     bin[72];     /* 二进制位流 */
    uint32_t pulses;      /* 采集到的脉宽段数（被采纳那一轮） */
    uint32_t ms;          /* 解码耗时 ms（含全部重采轮次） */
    uint8_t  repeat;      /* 与最终结果完全一致的帧数（>=2 才输出码值） */
    uint8_t  frames;      /* 本轮共切出的完整帧数 */
    uint8_t  tries;       /* 实际采集轮数（>1 说明前几轮是噪声，已丢弃重采） */
    uint16_t glitch;      /* 解码前被合并掉的毛刺段数（越多说明信号越脏） */
    uint64_t cand_code;   /* 跨轮累加候选：本轮最佳单帧码值（repeat<2 时也填，供跨轮计数） */
    uint32_t cand_bits;   /* 候选比特数 */
    int      cand_tmpl;   /* 候选协议下标（-1=无） */
    char     raw[480];    /* 前若干脉宽(us)诊断串，便于人工判读 */
} decode_result_t;

/* 初始化：配置 GPIO9 为输入并挂 CHANGE 中断（仅注册，采集时再开启） */
bool ook_decoder_init(int gpio_pin);

/* 阻塞式采集+解码：锁频到 freq_mhz、采集 DOUT 脉宽、解码，结果写入 out。
   返回是否有有效数据（即使未识别，out 也含 raw 供诊断）。 */
bool ook_decoder_run(float freq_mhz, uint32_t timeout_ms, decode_result_t *out);
