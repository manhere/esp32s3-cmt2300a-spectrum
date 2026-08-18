/*
 * config.h — 全局硬件 / 网络 / 默认参数配置
 * ESP32-S3 N16R8 + CMT2300A 射频遥控频率捕获仪
 */
#pragma once
#include <Arduino.h>

// ======================= WiFi =======================
#define WIFI_SSID       "your-wifi"
#define WIFI_PASSWORD   "your-password"
#define WIFI_HOSTNAME   "rfmaster"    // mDNS: http://rfmaster.local
#define WIFI_TIMEOUT_MS 20000
// 连不上热点时自动开 AP，方便现场调试；SSID = AP_SSID_PREFIX + "_" + MAC后两节（如 rfmaster_ABCD）
#define AP_SSID_PREFIX  "rfmaster"
#define AP_PASSWORD     "cmt2300a"

// ============ 扫频默认参数 (可在网页上改并掉电保存) ============
#define DEF_START_HZ    420000000UL   // 起始频率 420 MHz
#define DEF_STOP_HZ     450000000UL   // 终止频率 450 MHz
#define DEF_STEP_HZ     100000UL      // 步进 100 kHz
#define DEF_DWELL_US    200           // 每个频点驻留时间 (PLL 重锁 + AGC 建立)
#define DEF_THRESH_DB   30.0f         // 触发门限：高于本底 N dB（一个门限管两模式：
                                      //   快嗅探实时载波触发 + 范围扫频峰值检测共用）
#define DEF_DECAY_DB    0.5f          // 最大保持每扫衰减 dB
#define DEF_NORMALIZED_TOL_MHZ 1.5f   // 归一化最大容差 (会按邻近频点自动收紧)

#define MAX_BINS        512           // 单次扫描最大点数 (受 WS 帧长与内存限制)

// 扫描模式：0 = 范围扫频（连续大范围），1 = 常见频点扫频（直接跳到指定频点测 RSSI）
#define DEF_MODE        1

// 自动解码：开启后进入【快嗅探】模式 —— 在预设遥控频点(g_freqs)间高速轮转读 RSSI，
//   哪个频点【此刻】有载波就立刻在该频点进 OOK 接收并解码。单次短按即可被捕获。
//   默认开：开箱即按一下遥控器即锁频解码。
#define DEF_AUTO_DECODE 1
// 实际解码窗口由 ook_decoder.h 的 OOK_DECODE_TIMEOUT_MS（1000ms）控制，见 runDecodeState。

// ============ 快嗅探（单次按压即解码）参数 ============
// 每个候选频点停留时间：PLL 重锁 + AGC 建立 + RSSI 读取。
// CMT2300A STBY→RX 典型 50-200us(AN142)，500us 驻留足够。
#define SNIFF_DWELL_US    500
// 绝对地板：瞬时 RSSI 折算电平须优于此值(更靠近 0)才触发，避免安静频点上的噪声尖峰误触发。
#define SNIFF_ABS_FLOOR_DBM  (-92.0f)
// 一次成功解码后的静默冷却(ms)：避免同一长按被同一帧反复解码刷屏
#define SNIFF_COOLDOWN_MS 400

// 常见射频遥控频点 (MHz)，网页可编辑；"常见频点扫频"模式直接跳到这些频点逐个测量。
// 注意：CMT2300A 存在不可调谐空洞(340-379 / 510-758MHz 等)，落入空洞的频点(如 580)
//   由 Spectrum::calcBins 自动过滤，实际扫描 bin 以固件下发的频率表为准。
#define DEF_PRESET_LIST  "303,306,310,315,330,350,370,390,418,430.5,431.5,432.8,433.92,434.5,580,868.35,915"

// 归一化用的常见频点 (MHz)，网页可编辑（范围扫频时把检测到的峰值归一到最近遥控频点）
#define DEF_NORMALIZED_LIST "303,306,310,315,330,350,370,390,418,430.5,431.5,432.8,433.92,434.5,580,868.35,915"

// ============ CMT2300A 频率范围 ============
// 可调谐频段由 cmt2300a.cpp 的 bands[] 决定：126.33~170 / 189.5~340 / 379~510 / 758~1020 MHz
#define CMT_FREQ_MIN_HZ 126330000UL
#define CMT_FREQ_MAX_HZ 1020000000UL

// 无硬件时是否允许"演示模式"(生成模拟频谱，用于先调通网页)
#define ALLOW_DEMO_MODE 1
