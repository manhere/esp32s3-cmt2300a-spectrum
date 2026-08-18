/*
 * cmt2300a_hal.cpp — 3线SDIO SPI实现
 */
#include "cmt2300a_hal.h"

#define csb_1()   digitalWrite(CMT2300A_PIN_CSB,  HIGH)
#define csb_0()   digitalWrite(CMT2300A_PIN_CSB,  LOW)
#define fcsb_1()  digitalWrite(CMT2300A_PIN_FCSB, HIGH)
#define fcsb_0()  digitalWrite(CMT2300A_PIN_FCSB, LOW)
#define sclk_1()  digitalWrite(CMT2300A_PIN_SCK,  HIGH)
#define sclk_0()  digitalWrite(CMT2300A_PIN_SCK,  LOW)
#define sdio_1()  digitalWrite(CMT2300A_PIN_SDIO, HIGH)
#define sdio_0()  digitalWrite(CMT2300A_PIN_SDIO, LOW)
#define sdio_in()  pinMode(CMT2300A_PIN_SDIO, INPUT)
#define sdio_out() pinMode(CMT2300A_PIN_SDIO, OUTPUT)
#define sdio_read() digitalRead(CMT2300A_PIN_SDIO)

/* SPI 位时钟延时（可调宏，单位 µs，整数）。
   CMT2300A Datasheet：SCLK 最大 5MHz（半周期 ≥100ns）。
   档位参考（实际位周期 = SPI_DELAY_US×2 + GPIO 翻转固有开销）：
     1  = ~500kHz（原速度，最稳）
     0  = 不显式延时，仅靠 digitalWrite/read 翻转固有时序，实测约 1MHz+（推荐，
          仍远低于 5MHz 上限，扫频提速主要靠此档）
   若高速档下 RSSI/解码异常（飞线线长/寄生电容导致波形劣化），逐级改回 1 即可。 */
#define SPI_DELAY_US  0
#define SPI_DELAY()   delayMicroseconds(SPI_DELAY_US)

static void spi_send(uint8_t data8)
{
    for (int i = 0; i < 8; i++) {
        sclk_0();
        if (data8 & 0x80) sdio_1(); else sdio_0();
        SPI_DELAY();
        data8 <<= 1;
        sclk_1();
        SPI_DELAY();
    }
}

static uint8_t spi_recv(void)
{
    uint8_t data8 = 0xFF;
    for (int i = 0; i < 8; i++) {
        sclk_0();
        SPI_DELAY();
        data8 <<= 1;
        sclk_1();
        if (sdio_read()) data8 |= 0x01; else data8 &= ~0x01;
        SPI_DELAY();
    }
    return data8;
}

void CMT2300A_InitGpio(void)
{
    pinMode(CMT2300A_PIN_CSB,   OUTPUT);
    pinMode(CMT2300A_PIN_FCSB,  OUTPUT);
    pinMode(CMT2300A_PIN_SCK,   OUTPUT);
    pinMode(CMT2300A_PIN_SDIO,  OUTPUT);
    pinMode(CMT2300A_PIN_GPIO1, INPUT);

    csb_1();
    sclk_0();
    sdio_1();
    fcsb_1();
    delayMicroseconds(10);
    sdio_in();
}

uint8_t CMT2300A_ReadReg(uint8_t addr)
{
    uint8_t dat;
    sdio_out();
    sdio_1();
    sclk_0();
    fcsb_1();
    csb_0();
    SPI_DELAY(); SPI_DELAY();
    spi_send(addr | 0x80);
    sdio_in();
    dat = spi_recv();
    sclk_0();
    SPI_DELAY(); SPI_DELAY();
    csb_1();
    sdio_1();
    sdio_in();
    fcsb_1();
    return dat;
}

void CMT2300A_WriteReg(uint8_t addr, uint8_t dat)
{
    sdio_out();
    sdio_1();
    sclk_0();
    fcsb_1();
    csb_0();
    SPI_DELAY(); SPI_DELAY();
    spi_send(addr & 0x7F);
    spi_send(dat);
    sclk_0();
    SPI_DELAY(); SPI_DELAY();
    csb_1();
    sdio_1();
    sdio_in();
    fcsb_1();
}

void CMT2300A_WriteRegsMulti(const uint8_t *addr, const uint8_t *dat, uint8_t n)
{
    if (n == 0) return;
    sdio_out();
    sdio_1();
    sclk_0();
    fcsb_1();
    csb_0();
    SPI_DELAY(); SPI_DELAY();
    for (uint8_t i = 0; i < n; i++) {
        spi_send(addr[i] & 0x7F);
        spi_send(dat[i]);
    }
    sclk_0();
    SPI_DELAY(); SPI_DELAY();
    csb_1();
    sdio_1();
    sdio_in();
    fcsb_1();
}

void CMT2300A_ReadFifo(uint8_t *buf, uint16_t len)
{
    fcsb_1(); csb_1(); sclk_0();
    sdio_in();
    for (uint16_t i = 0; i < len; i++) {
        fcsb_0();
        SPI_DELAY(); SPI_DELAY();
        buf[i] = spi_recv();
        sclk_0();
        delayMicroseconds(3);
        fcsb_1();
        delayMicroseconds(5);
    }
    sdio_in(); fcsb_1();
}

void CMT2300A_WriteFifo(const uint8_t *buf, uint16_t len)
{
    fcsb_1(); csb_1(); sclk_0();
    sdio_out();
    for (uint16_t i = 0; i < len; i++) {
        fcsb_0();
        SPI_DELAY(); SPI_DELAY();
        spi_send(buf[i]);
        sclk_0();
        delayMicroseconds(3);
        fcsb_1();
        delayMicroseconds(5);
    }
    sdio_in(); fcsb_1();
}
