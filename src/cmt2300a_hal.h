/*
 * cmt2300a_hal.h — CMT2300A 3线SDIO硬件抽象层
 *
 * 3线SDIO协议：MOSI兼作MISO
 * 写寄存器时：CSB=0 → 发送地址(r/w=0) → 发送数据 → CSB=1
 * 读寄存器时：CSB=0 → SDIO输出 → 发送地址(r/w=1) → SDIO切输入 → 读取数据 → CSB=1
 *
 * 引脚映射（实测验证）：
 *   CSB(pin5)   -> GPIO4    寄存器片选
 *   FCSB(pin2)  -> GPIO5    FIFO片选
 *   SCK(pin8)   -> GPIO6    时钟
 *   MOSI(pin6)  -> GPIO7    数据（双向SDIO）
 *   MISO(pin7)  -> GPIO8    未用（3线模式）
 *   GPIO1(pin9) -> GPIO10   芯片GPIO1（前导码检测中断）
 */
#ifndef CMT2300A_HAL_H
#define CMT2300A_HAL_H

#include <Arduino.h>
#include <stdint.h>

/* 引脚定义 */
#define CMT2300A_PIN_CSB   4
#define CMT2300A_PIN_FCSB  5
#define CMT2300A_PIN_SCK   6
#define CMT2300A_PIN_SDIO  7
#define CMT2300A_PIN_GPIO1 10

void     CMT2300A_InitGpio(void);
uint8_t  CMT2300A_ReadReg(uint8_t addr);
void     CMT2300A_WriteReg(uint8_t addr, uint8_t dat);
void     CMT2300A_ReadFifo(uint8_t *buf, uint16_t len);
void     CMT2300A_WriteFifo(const uint8_t *buf, uint16_t len);
void     CMT2300A_WriteRegsMulti(const uint8_t *addr, const uint8_t *dat, uint8_t n);

#endif
