/*
 * cmt2300a.h — CMT2300A OOK收发芯片驱动
 *
 * 针对OOK遥控器检测优化：
 * - 使用OOK解调模式（DATA_MODE=DIRECT）
 * - GPIO3(pin4) 输入（TX 时配为 DIN，接收 ESP32 GPIO10 的 OOK 比特流，作本板 OOK 发射脚）
 * - GPIO2(pin3) 已飞线至 ESP32 GPIO9，输出解调后的原始 OOK 码流，供 OOK 解码使用
 *
 * 引脚映射（3线SDIO）：
 *   CSB(pin5)   -> GPIO4    寄存器片选
 *   FCSB(pin2)  -> GPIO5    FIFO片选
 *   SCK(pin8)   -> GPIO6    时钟
 *   SDIO(pin6)  -> GPIO7    数据（双向）
 *   GPIO1(pin9) -> (无连接 NC，原 GPIO10 飞线已改接 GPIO3)
 *   GPIO2(pin3) -> GPIO9    DOUT（OOK 解码输入，飞线）
 *   GPIO3(pin4) -> GPIO10   DIN（OOK 发射输入，飞线；本板 TX 脚）
 */
#ifndef CMT2300A_H
#define CMT2300A_H

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

/* ============ 寄存器地址 ============ */
#define CMT2300A_CUS_CMT1       0x00
#define CMT2300A_CUS_CMT4       0x03   /* bit0=double Tx value（>16dBm 功率档门控，OpenDTU 证实） */
#define CMT2300A_CUS_CMT10      0x09
#define CMT2300A_CUS_SYS1       0x0C
#define CMT2300A_CUS_SYS2       0x0D
#define CMT2300A_CUS_SYS3       0x0E

#define CMT2300A_CUS_RF1        0x18
#define CMT2300A_CUS_RF2        0x19
#define CMT2300A_CUS_RF3        0x1A
#define CMT2300A_CUS_RF4        0x1B
#define CMT2300A_CUS_RF5        0x1C
#define CMT2300A_CUS_RF6        0x1D
#define CMT2300A_CUS_RF7        0x1E
#define CMT2300A_CUS_RF8        0x1F

#define CMT2300A_CUS_FSK4       0x27   /* AFC_OVF_TH */

/* Data Rate bank：0x20-0x37 共 24 字节，是 OOK 解调参数的所在区
   （RF9-12 符号率/接收带宽、FSK1-7、CDR1-4、AGC1-4、OOK1-5 切片与峰谷检测）。
   注意区内 0x27(AFC_OVF_TH) 由 SetFrequency 每次跳频重算覆盖。 */
#define CMT2300A_DATA_RATE_BANK_ADDR  0x20
#define CMT2300A_DATA_RATE_BANK_SIZE  24
#define CMT2300A_CUS_OOK1       0x33
#define CMT2300A_CUS_OOK2       0x34   /* bit7 = AUTO_ABS_EN（OOK 自动绝对门限） */
#define CMT2300A_CUS_OOK5       0x37

#define CMT2300A_CUS_PKT1       0x38   /* 数据模式、调制方式 */
#define CMT2300A_CUS_PKT5       0x3C   /* 前导码配置 */
#define CMT2300A_CUS_PKT17      0x48   /* 测试寄存器（用于在位检测） */

#define CMT2300A_CUS_MODE_CTL   0x60   /* 模式切换命令 */
#define CMT2300A_CUS_MODE_STA   0x61   /* 芯片状态 */
#define CMT2300A_CUS_EN_CTL     0x62
#define CMT2300A_CUS_FREQ_CHNL  0x63   /* 快速跳频信道 */
#define CMT2300A_CUS_FREQ_OFS   0x64   /* 快速跳频偏移 */
#define CMT2300A_CUS_IO_SEL     0x65   /* GPIO功能选择 */
#define CMT2300A_CUS_INT1_CTL   0x66   /* 中断1 控制：bit7=RF_SWT1_EN、bit6=RF_SWT2_EN（天线开关信号使能，AN192） */
#define CMT2300A_CUS_INT_EN     0x68   /* 中断使能 */
#define CMT2300A_CUS_INT_CLR1   0x6A   /* 中断清除1 */
#define CMT2300A_CUS_INT_CLR2   0x6B   /* 中断清除2 */
#define CMT2300A_CUS_INT_FLAG   0x6D   /* 中断标志 */
#define CMT2300A_CUS_RSSI_DBM   0x70   /* RSSI值 */

/* ============ 关键位定义 ============ */
#define CMT2300A_MASK_PREAM_OK_FLG     0x10
#define CMT2300A_MASK_PREAM_OK_CLR     0x10
#define CMT2300A_MASK_SYNC_OK_CLR      0x08
#define CMT2300A_MASK_CFG_RETAIN       0x10
#define CMT2300A_MASK_LOCKING_EN       0x20

/* ============ 发射（DIRECT OOK 回放）相关寄存器 / 位 ============ */
#define CMT2300A_CUS_FIFO_CTL          0x69   /* FIFO 控制：TX_DIN 使能 + 选源 */
#define CMT2300A_CUS_INT2_CTL          0x67   /* 中断2 控制：TX_DIN 极性反转 */
#define CMT2300A_CUS_TX_BANK_ADDR      0x55   /* TX 配置区首地址（0x55-0x5F 共 11 字节） */
#define CMT2300A_CUS_TX_BANK_SIZE      11
#define CMT2300A_CUS_TX8               0x5C   /* TX 功率字高字节（TX_POWER） */
#define CMT2300A_CUS_TX9               0x5D   /* TX 功率字低字节 */
#define CMT2300A_MASK_TX_DIN_EN        0x80   /* FIFO_CTL[7]=TX_DIN_EN：使能 DIN 输入驱动 PA。DIRECT(OOK) 模式必须置位（datasheet V1.8 §6.1 Tx step1）。缺则 PA 恒载波无调制。 */
#define CMT2300A_MASK_TX_DIN_SEL       0x60   /* FIFO_CTL[6:5]=TX_DIN_SEL：选哪个 GPIO 作 DIN 源。00=GPIO1 / 0x20=GPIO2 / 0x40=GPIO3
                                                （datasheet V1.8 §6.1 Tx step2）。本板 ESP32 GPIO10 -> CMT GPIO3 飞线，故须 0x40(GPIO3)。
                                                ★DIN 用哪只脚是「软件可选」的，但需对应 CMT GPIO 已飞线到 ESP32★ */
#define CMT2300A_MASK_FIFO_MERGE_EN    0x02   /* FIFO_CTL[1]=FIFO_MERGE_EN：把 TX/RX FIFO 合并成 64 字节，属【FIFO/Packet 模式】功能（datasheet V1.8 §5.2）。
                                                ★DIRECT 发射必须清 0★：置 1 与 DIN->PA 调制无关，且会打乱 FIFO 组织；V1.8 §6.1 的 DIRECT TX
                                                序列完全不涉及本位。最终 FIFO_CTL = 0xC0（bit7 + TX_DIN_SEL=0x40=GPIO3）。 */
#define CMT2300A_MASK_TX_DIN_INV       0x20   /* INT2_CTL[5]=TX_DIN_INV：DIN 极性反转。DIRECT 正逻辑须清 0（datasheet 未要求反转；置 1 整帧反码） */
#define CMT2300A_MASK_TX_DIN_SOURCE    0x04   /* TX1(0x55)[2]=TX_DIN_SOURCE：DIRECT 模式 TX 数据来自 GPIO 直通，与 FIFO_CTL[7] 共构 DIN 调制使能 */

/* IO_SEL(0x65) GPIO 功能选择（权威位定义：CMOSTEK cmt2300a_defs.h:476-492 / datasheet V1.8 pin table）
     GPIO1_SEL[1:0]: DIN/DOUT=0x00, INT1=0x01, INT2=0x02, DCLK=0x03
     GPIO2_SEL[3:2]: INT1=0x00,     INT2=0x04, DIN/DOUT=0x08, DCLK=0x0C
     GPIO3_SEL[5:4]: CLKO=0x00,     DIN/DOUT=0x10, INT2=0x20, DCLK=0x30
   注：同一档位的 "DIN" 与 "DOUT" 是同一个值（数据脚），方向由 TX/RX 态决定；
   究竟哪只脚在 TX 时充当 DIN，由 FIFO_CTL[6:5]=TX_DIN_SEL 决定。 */
#define CMT2300A_GPIO1_SEL_DIN         0x00
#define CMT2300A_GPIO2_SEL_INT2        0x04
#define CMT2300A_MASK_GPIO12_SEL       0x0F   /* GPIO1_SEL + GPIO2_SEL 合并掩码（不动 GPIO3/GPIO4） */
#define CMT2300A_GPIO3_SEL_DIN         0x10   /* GPIO3_SEL[5:4]=0b01 -> DIN/DOUT（TX 时作 DIN 源） */
#define CMT2300A_GPIO1_SEL_INT1        0x01   /* GPIO1_SEL[1:0]=0b01 -> INT1（TX 时不作数据脚，避免与 GPIO3 争用 DIN 功能） */
#define CMT2300A_MASK_GPIO3_SEL        0x30   /* GPIO3_SEL[5:4] 掩码 */
#define CMT2300A_TX_DIN_SEL_GPIO3      0x40   /* FIFO_CTL[6:5]=TX_DIN_SEL=0b10 -> GPIO3 作 DIN 源 */

/* ============ 状态切换命令 ============ */
#define CMT2300A_GO_STBY               0x02
#define CMT2300A_GO_RX                 0x08
#define CMT2300A_GO_SLEEP              0x10
#define CMT2300A_GO_TX                 0x40   /* MODE_CTL[6]：进入发射态 */

/* ============ 芯片状态 ============ */
#define CMT2300A_STA_SLEEP             0x01
#define CMT2300A_STA_STBY              0x02
#define CMT2300A_STA_RX                0x05
#define CMT2300A_STA_TX                0x06   /* MODE_STA[3:0]=0110：发射态（PLL 锁定 + PA 武装完成） */

/* ============ 驱动API ============ */

/**
 * 初始化CMT2300A
 * 配置OOK模式、前导码检测、GPIO1中断输出
 */
bool CMT2300A_Init(void);

/**
 * 芯片在位检测
 * 通过读改写测试寄存器实现
 */
bool CMT2300A_IsExist(void);

/**
 * 进入指定频率的接收模式
 * @param freq_hz 目标频率（Hz）
 */
void CMT2300A_SetFrequency(uint32_t freq_hz);

/**
 * 段感知快速跳频（AN197 快速手动跳频）
 * 同段（FREQ_DIVX_CODE / FREQ_VCO_BANK 不变）内只写 FH_OFFSET/FH_CHANNEL
 * + AFC_OVF_TH 三个寄存器；跨段/首跳/偏移不可分解时回落全量 SetFrequency。
 * 调用前须已 GoStby（扫描流程 cmtTuneRx 满足）。
 * @param freq_hz 目标频率（Hz）
 * @return 实际生效频率 Hz；0=不可调谐（空洞频点）
 */
uint32_t CMT2300A_TuneFast(uint32_t freq_hz);

/**
 * 频率是否落在 CMT2300A 可调谐频段内
 * CMT2300A 存在真实频段空洞（如 340-379MHz、510-758MHz），
 * 落入空洞时 SetFrequency 会静默失败、芯片停在上一有效频点，
 * 导致把相邻频点(如433附近434.5)的 RSSI 误读为当前空洞频点(如580)的信号。
 * 扫描前应先调用本函数过滤空洞频点。
 * @param freq_hz 目标频率（Hz）
 * @return true=可调谐 false=落入空洞(不可测)
 */
bool CMT2300A_IsFreqSupported(uint32_t freq_hz);

/**
 * 进入待机模式（轮询 MODE_STA 确认已切到 STBY，超时 500us 兜底）
 */
bool CMT2300A_GoStby(void);

/**
 * 仅发送 go_rx 命令，不等待 PLL 锁定（锁定与 dwell 并行，读 RSSI 前
 * 调用 CMT2300A_WaitRxLocked 确认）——快速扫频专用
 */
void CMT2300A_EnterRx(void);

/**
 * 轮询 MODE_STA 直到进入 RX 态（PLL 锁定完成）
 * @param timeoutUs 超时上限
 * @return true=已进入 RX；false=超时未锁(调用方应放弃本次读数，防残留 RSSI 误触发)
 */
bool CMT2300A_WaitRxLocked(uint32_t timeoutUs);

/**
 * 进入接收模式（EnterRx + WaitRxLocked(1000)，同步等待锁定）
 * @return true=芯片已确认进入 RX 态；false=1ms 内未锁定(调用方应放弃本次读数)
 */
bool CMT2300A_GoRx(void);

/**
 * 获取RSSI值
 * @return RSSI（dBm），范围-128~0
 */
int CMT2300A_GetRssiDBm(void);

/**
 * 清除中断标志
 */
void CMT2300A_ClearInterruptFlags(void);

/**
 * 进入 DIRECT OOK 发射态（解码记录回放用）
 * 流程（对齐工作参考 tuya_rf / OOKwiz：每次发射前完整 SoftReset 重初始化）：
 *   SoftReset -> GoStby -> 写 DataRate/TX bank -> xosc_aac=2 -> SetFrequency
 *   -> DATA_MODE=DIRECT -> IO_SEL=0x14 -> EN_CTL[5]=LOCKING_EN(DIN 使能门控)
 *   -> FIFO_CTL 使能 TX_DIN_EN(GPIO1) -> TX_DIN_INV=0 -> GPIO1 配输出
 *   -> GoSleep->GoStby->GoTx
 * @param freq_hz 发射频率（Hz）
 */
void CMT2300A_TxOokBegin(uint32_t freq_hz);

/**
 * 设置发射功率档位（-10 ~ +20 dBm，步进 1 dB）
 * 功率字表完全参考 OpenDTU（tbnobody/OpenDTU lib/CMT2300a/cmt2300wrapper.cpp setPALevel()，
 * TRx Matching Network = 20 dBm）：
 *   16 位功率字写入 TX8(0x5C)=高字节 / TX9(0x5D)=低字节；
 *   功率 > 16 dBm 时 CMT4 bit0 置 1（"double Tx value" 档位门控，OpenDTU 证实）。
 * @param dBm  -10 ~ 20；越界值忽略不写
 */
void CMT2300A_SetTxPower(int8_t dBm);

/**
 * 退出发射态、完整重初始化回接收（直接调用 CMT2300A_Init，无状态补丁）
 */
void CMT2300A_TxOokEnd(void);

#endif
