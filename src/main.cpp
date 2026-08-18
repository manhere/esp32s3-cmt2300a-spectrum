/*
 * main.cpp — CMT2300A 射频遥控频率捕获仪（网页移植自 esp32s3-si4463-spectrum）
 *
 * 启动流程：串口 -> CMT2300A初始化 -> WiFi -> 扫频引擎 -> Web服务
 *
 * WiFi模式：
 * - STA模式：连接热点dm（密码q1w2e3r4dm）
 * - AP回退：如果STA失败，自建热点 rfmaster_<MAC后两节>（密码cmt2300a）
 *
 * 网页访问：
 * - STA模式：http://<分配的IP>/   或 mDNS http://rfmaster.local/
 * - AP模式：http://192.168.4.1/
 */
#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>

#include "config.h"
#include "cmt2300a.h"
#include "spectrum.h"
#include "web_server.h"

/* 固件版本标记：烧录后看串口第一屏确认是否运行到本版 */
#define FW_VERSION "20260814-si4463ui-cmt2300a"

static void wifiConnect() {
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(WIFI_HOSTNAME);
    WiFi.setSleep(false);                 // 关省电，降低 WS 延迟
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.printf("[WiFi] 正在连接 \"%s\" ", WIFI_SSID);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_TIMEOUT_MS) {
        delay(300);
        Serial.print('.');
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[WiFi] 已连接  IP=%s  RSSI=%d dBm\n",
                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
    } else {
        Serial.println(F("[WiFi] 连接失败 -> 启动 AP 模式"));
        WiFi.mode(WIFI_AP);
        uint8_t mac[6];
        WiFi.macAddress(mac);
        char apSsid[32];
        snprintf(apSsid, sizeof(apSsid), "%s_%02X%02X", AP_SSID_PREFIX, mac[4], mac[5]);
        WiFi.softAP(apSsid, AP_PASSWORD);
        Serial.printf("[WiFi] AP: %s / %s   IP=%s\n",
                      apSsid, AP_PASSWORD, WiFi.softAPIP().toString().c_str());
    }

    if (MDNS.begin(WIFI_HOSTNAME)) {
        MDNS.addService("http", "tcp", 80);
        Serial.printf("[mDNS] http://%s.local/\n", WIFI_HOSTNAME);
    }
}

void setup() {
    Serial.begin(115200);
    delay(400);
    Serial.println();
    Serial.println(F("================================================"));
    Serial.println(F(" RF Master  (ESP32-S3 + CMT2300A)"));
    Serial.println(F("================================================"));
    Serial.printf("[SYS] CPU %d MHz  Flash %u MB  PSRAM %u KB\n",
                  getCpuFrequencyMhz(), ESP.getFlashChipSize() / 1048576,
                  ESP.getPsramSize() / 1024);
    Serial.printf("[SYS] FW %s\n", FW_VERSION);

    // 1. 射频前端（Init 内含 GPIO 初始化，须在 IsExist 之前）
    if (!CMT2300A_Init()) {
        Serial.println(F("[SYS] CMT2300A 初始化失败"));
    }
    if (!CMT2300A_IsExist()) {
        Serial.println(F("[SYS] CMT2300A 未响应：将以演示模式运行网页"));
    } else {
        Serial.println(F("[SYS] CMT2300A 已响应(SPI正常)"));
    }

    // 2. 网络
    wifiConnect();

    // 3. 扫频引擎 (跑在 core 1 的独立任务)
    Spectrum::begin();

    // 4. Web / WebSocket
    Web::begin();

    Serial.println(F("[SYS] 就绪，浏览器打开上面的地址开始扫描"));
}

void loop() {
    Web::loop();

    // WiFi 掉线自动重连
    static uint32_t tCheck = 0;
    if (millis() - tCheck > 5000) {
        tCheck = millis();
        if (WiFi.getMode() == WIFI_STA && WiFi.status() != WL_CONNECTED) {
            Serial.println(F("[WiFi] 掉线，重连中..."));
            WiFi.reconnect();
        }
    }
    delay(2);
}
