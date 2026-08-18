/*
 * web_server.cpp — ESPAsyncWebServer + WebSocket
 * 移植自 esp32s3-si4463-spectrum（WebSocket 协议与前端 index.html 完全一致，
 * 仅底层射频由 Spectrum 引擎提供，射频芯片为 CMT2300A）。
 *
 * WebSocket 协议
 *   服务端 -> 浏览器
 *     二进制帧 (频谱):
 *        [0]     0xA5  magic
 *        [1]     0x01  type = spectrum
 *        [2..3]  seq        u16 LE
 *        [4..7]  startHz    u32 LE
 *        [8..11] stepHz     u32 LE
 *        [12..13]nBins      u16 LE
 *        [14]    flags  bit0 = 附带 maxHold
 *        [15]    保留
 *        [16..]  live[nBins]  (RSSI raw, dBm = raw/2-134)
 *        [..]    hold[nBins]
 *     文本帧 (JSON): {"type":"cfg"|"stat"|"hit"|"decode", ...}
 *
 *   浏览器 -> 服务端 (JSON)
 *     {"cmd":"get"} {"cmd":"set",...} {"cmd":"run","v":bool} {"cmd":"arm","v":bool}
 *     {"cmd":"clear"} {"cmd":"baseline"} {"cmd":"save"} {"cmd":"reset"}
 */
#include "web_server.h"
#include "config.h"
#include "spectrum.h"
#include "history.h"
#include "web_index.h"

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ElegantOTA.h>

namespace Web {

static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");

static uint8_t  g_live[MAX_BINS];
static uint8_t  g_hold[MAX_BINS];
static uint8_t  g_pkt[16 + MAX_BINS * 2];
static uint32_t g_lastSend = 0;
static uint32_t g_lastStat = 0;

/* ------------------------- JSON 构造 ------------------------- */
static String buildCfgJson() {
    ScanSettings s = Spectrum::getSettings();
    JsonDocument d;
    d["type"]   = "cfg";
    d["start"]  = s.startHz / 1e6;
    d["stop"]   = s.stopHz / 1e6;
    d["step"]   = s.stepHz / 1000.0;
    d["dwell"]  = s.dwellUs;
    d["thr"]    = s.thresholdDb;
    d["decay"]  = s.decayDb;
    d["normtol"]    = s.normalizedTolMHz;
    d["normon"] = s.normalizedEnable;
    d["normlist"]   = s.normalizedList;
    d["mode"]   = s.mode;
    d["auto"]   = s.autoDecode;
    d["preset"] = s.presetList;
    d["run"]    = s.running;
    d["arm"]    = s.armed;
    // 2026-08-12：移除前端不用的 bins(频谱帧头已带)/ip(前端用 location.host)
    d["radio"]  = Spectrum::radioOk();
    d["demo"]   = Spectrum::demoMode();
    d["part"]   = "CMT2300A";
    String out;
    serializeJson(d, out);
    return out;
}

static String buildStatJson() {
    JsonDocument d;
    d["type"]  = "stat";
    d["sps"]   = roundf(Spectrum::sweepsPerSec() * 10) / 10.0f;
    d["peak"]  = roundf(Spectrum::lastPeakDbm() * 10) / 10.0f;
    d["floor"] = roundf(Spectrum::lastFloorDbm() * 10) / 10.0f;
    d["rssi"]  = WiFi.RSSI();
    d["heap"]  = ESP.getFreeHeap();
    d["up"]    = millis() / 1000;
    String out;
    serializeJson(d, out);
    return out;
}

static void sendFreqTable(AsyncWebSocketClient* client) {
    uint32_t ft[MAX_BINS]; uint16_t n = 0;
    if (!Spectrum::takeFreqTable(ft, MAX_BINS, n) || n == 0) return;
    uint8_t fp[6 + MAX_BINS * 4];
    fp[0] = 0xA5; fp[1] = 0x02;
    fp[2] = 0; fp[3] = 0;                 // seq（频率表不依赖帧序号）
    fp[4] = (uint8_t)(n & 0xFF);
    fp[5] = (uint8_t)(n >> 8);
    for (uint16_t i = 0; i < n; i++) memcpy(&fp[6 + i * 4], &ft[i], 4);
    if (client) client->binary(fp, 6 + n * 4);
    else        ws.binaryAll((const char*)fp, 6 + n * 4);
}

static String buildHitJson(const Hit& h) {
    JsonDocument d;
    d["type"]   = "hit";
    d["peak"]   = roundf(h.peakMHz * 1000) / 1000.0f;
    d["normlist"]   = h.normalizedMHz;
    d["dbm"]    = roundf(h.peakDbm * 10) / 10.0f;
    d["floor"]  = roundf(h.floorDbm * 10) / 10.0f;
    d["delta"]  = roundf(h.deltaDb * 10) / 10.0f;
    d["dur"]    = h.durationMs;
    JsonArray c = d["cand"].to<JsonArray>();
    for (int i = 0; i < 3; i++) {
        if (h.cand[i] <= 0) continue;
        JsonObject o = c.add<JsonObject>();
        o["f"] = h.cand[i];
        o["d"] = roundf(h.candDist[i] * 1000) / 1000.0f;
    }
    String out;
    serializeJson(d, out);
    return out;
}

static String buildDecodeJson(const DecodeResult& r) {
    JsonDocument d;
    d["type"]  = "decode";
    d["freq"]  = roundf(r.freqMHz * 1000) / 1000.0f;
    d["code"]  = r.code;
    d["bits"]  = r.bits;
    d["proto"] = r.proto;
    d["delay"] = r.delayUs;
    d["te"]    = r.te;          // 前端解码记录表格显示单比特时长(us)
    d["name"]  = r.protoName;   // 前端解码记录表格显示协议名(EV1527/PT2262)
    // 2026-08-12：仅发送前端用到的字段；repeat/frames/hex(前端自算)/dbm 仍不发送
    String out;
    serializeJson(d, out);
    return out;
}
static void handleCmd(AsyncWebSocketClient* client, const String& msg) {
    JsonDocument d;
    if (deserializeJson(d, msg)) return;
    const char* cmd = d["cmd"] | "";

    if (!strcmp(cmd, "get")) {
        client->text(buildCfgJson());
        return;
    }
    if (!strcmp(cmd, "set")) {
        ScanSettings s = Spectrum::getSettings();
        if (d["start"].is<float>()) s.startHz = (uint32_t)(d["start"].as<double>() * 1e6);
        if (d["stop"].is<float>())  s.stopHz  = (uint32_t)(d["stop"].as<double>() * 1e6);
        if (d["step"].is<float>())  s.stepHz  = (uint32_t)(d["step"].as<double>() * 1000.0);
        if (d["dwell"].is<float>()) s.dwellUs = (uint16_t)constrain(d["dwell"].as<int>(), 60, 5000);
        if (d["thr"].is<float>())   s.thresholdDb = constrain(d["thr"].as<float>(), 2.0f, 60.0f);
        if (d["decay"].is<float>()) s.decayDb = constrain(d["decay"].as<float>(), 0.0f, 20.0f);
        if (d["normtol"].is<float>())   s.normalizedTolMHz = constrain(d["normtol"].as<float>(), 0.05f, 10.0f);
        if (d["normon"].is<bool>()) s.normalizedEnable = d["normon"].as<bool>();
        if (d["normlist"].is<const char*>()) {
            strncpy(s.normalizedList, d["normlist"].as<const char*>(), sizeof(s.normalizedList) - 1);
            s.normalizedList[sizeof(s.normalizedList) - 1] = '\0';
        }
        if (d["mode"].is<int>()) s.mode = (uint8_t)constrain(d["mode"].as<int>(), 0, 1);
        if (d["auto"].is<bool>()) s.autoDecode = d["auto"].as<bool>();
        if (d["preset"].is<const char*>()) {
            strncpy(s.presetList, d["preset"].as<const char*>(), sizeof(s.presetList) - 1);
            s.presetList[sizeof(s.presetList) - 1] = '\0';
        }
        // 合法性钳位：CMT2300A 可调谐范围 126.33~1020MHz（空洞频点由 Spectrum 内部过滤）
        s.startHz = constrain(s.startHz, CMT_FREQ_MIN_HZ, CMT_FREQ_MAX_HZ);
        s.stopHz  = constrain(s.stopHz, s.startHz + 200000UL, CMT_FREQ_MAX_HZ);
        Spectrum::applySettings(s);
        ws.textAll(buildCfgJson());
        sendFreqTable(nullptr);          // 切换/编辑后主动下发频率表
        return;
    }
    if (!strcmp(cmd, "run") || !strcmp(cmd, "arm")) {
        ScanSettings s = Spectrum::getSettings();
        bool v = d["v"] | true;
        if (cmd[0] == 'r') s.running = v; else s.armed = v;
        Spectrum::applySettings(s);
        ws.textAll(buildCfgJson());
        return;
    }
    if (!strcmp(cmd, "clear"))    { Spectrum::clearMaxHold();  return; }
    if (!strcmp(cmd, "baseline")) { Spectrum::resetBaseline(); Spectrum::clearMaxHold(); return; }
    if (!strcmp(cmd, "replay")) {
        // 注意：不能用 d["freq"] | 0 —— 字面量 0 是 int，operator| 会按 is<int>() 检查，
        // 而 JSON 里的 433.92 是 Float 标签，is<int>() 返回 false，于是永远回退成 0。
        // 正确做法是用 .as<float>() 直接按浮点解析（缺字段时返回 0.0，闸门仍能拦住陈旧记录）。
        float   f    = d["freq"].as<float>();
        uint32_t code = (uint32_t)(d["code"] | 0);
        uint32_t te   = d["te"]   | 0;
        uint8_t  bits = d["bits"] | 0;
        // 频率合法性闸门：freq<=0 或过大 → 多半是点击了陈旧记录(早期版本 freq 字段缺失/为0)。
        // 直接拒绝，避免 SetFrequency(0) 发出垃圾频点(≈430.5MHz 伪命中)。请刷新页面再试。
        if (f <= 0.0f || f > 1000.0f) {
            Serial.printf("[WS] replay 拒绝: freq=%.3fMHz 非法(记录可能过期/缺失频率) -> 请硬刷新页面(F5)后重试\n", f);
            client->text("{\"type\":\"replay\",\"ok\":0,\"err\":\"bad_freq\"}");
            return;
        }
        Spectrum::requestReplay((uint32_t)(f * 1e6f + 0.5f), code, te, bits);
        Serial.printf("[WS] replay cmd freq=%.3fMHz code=0x%08X bits=%u te=%uus\n",
                      f, code, bits, te);
        client->text("{\"type\":\"replay\",\"ok\":1}");
        return;
    }
    if (!strcmp(cmd, "save"))     { Spectrum::saveSettings();  client->text("{\"type\":\"saved\"}"); return; }
    /* ===== 解码历史（LittleFS） ===== */
    if (!strcmp(cmd, "hlist"))  { client->text(History::toListJson()); return; }
    if (!strcmp(cmd, "hclear")) {
        History::clear();
        client->text("{\"type\":\"hclear\",\"ok\":1}");
        return;
    }
    if (!strcmp(cmd, "hdel")) {
        uint32_t id = d["id"] | (uint32_t)0;
        bool ok = History::remove(id);
        char buf[48]; snprintf(buf, sizeof(buf), "{\"type\":\"hdel\",\"ok\":%d}", ok ? 1 : 0);
        client->text(buf);
        return;
    }
    if (!strcmp(cmd, "hupd")) {
        uint32_t id   = d["id"]   | (uint32_t)0;
        const char* nm = d["name"] | "";
        bool ok = History::updateName(id, nm);
        char buf[48]; snprintf(buf, sizeof(buf), "{\"type\":\"hupd\",\"ok\":%d}", ok ? 1 : 0);
        client->text(buf);
        return;
    }
    if (!strcmp(cmd, "hadd")) {              // 前端"解码记录"列表手工保存一条到历史
        HistRec h{};
        h.freq  = d["freq"] | 0.0f;
        h.code  = d["code"].as<uint64_t>();
        h.bits  = d["bits"] | (uint8_t)0;
        h.te    = d["te"]   | (uint16_t)0;
        h.proto = d["proto"]| (uint8_t)0;
        strncpy(h.name, d["name"] | "", sizeof(h.name) - 1);
        h.name[sizeof(h.name) - 1] = '\0';
        bool added = History::add(h);        /* 判重：重复返回 false */
        char buf[64];
        snprintf(buf, sizeof(buf), "{\"type\":\"hadd\",\"ok\":%d,\"dup\":%d}",
                 added ? 1 : 0, added ? 0 : 1);
        client->text(buf);
        return;
    }
    if (!strcmp(cmd, "reset")) {   // 恢复初始配置：清空 NVS 配置 + 重启
        Serial.println(F("[WS] 收到重置指令：清除配置并重启..."));
        client->text("{\"type\":\"resetting\"}");
        delay(300);                                  // 给回复一点发送时间
        Spectrum::resetToFactory();                  // 内部清 NVS 后 ESP.restart()，不返回
        return;
    }
}

static void onWsEvent(AsyncWebSocket* s, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            Serial.printf("[WS] client #%u 连接 %s\n", client->id(),
                          client->remoteIP().toString().c_str());
            client->text(buildCfgJson());
            client->text(buildStatJson());
            sendFreqTable(client);
            break;
        case WS_EVT_DISCONNECT:
            Serial.printf("[WS] client #%u 断开\n", client->id());
            break;
        case WS_EVT_DATA: {
            AwsFrameInfo* info = (AwsFrameInfo*)arg;
            if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
                String msg;
                msg.reserve(len + 1);
                for (size_t i = 0; i < len; i++) msg += (char)data[i];
                handleCmd(client, msg);
            }
            break;
        }
        default: break;
    }
}

/* ------------------------- 对外接口 ------------------------- */
void begin() {
    History::begin();                        // 挂载 LittleFS + 加载解码历史
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        AsyncWebServerResponse* res =
            req->beginResponse(200, "text/html", INDEX_HTML_GZ, INDEX_HTML_GZ_LEN);
        res->addHeader("Content-Encoding", "gzip");
        res->addHeader("Cache-Control", "no-cache");
        req->send(res);
    });

    /* 解码历史导出：GET /api/history -> history.json 下载 */
    server.on("/api/history", HTTP_GET, [](AsyncWebServerRequest* req) {
        String j = History::toExportJson();
        AsyncWebServerResponse* res = req->beginResponse(200, "application/json", j);
        res->addHeader("Content-Disposition", "attachment; filename=history.json");
        req->send(res);
    });

    server.onNotFound([](AsyncWebServerRequest* req) {
        req->redirect("/");
    });
	
	ElegantOTA.begin(&server);

    server.begin();
    Serial.printf("[HTTP] http://%s/   ws://%s/ws\n",
                  WiFi.localIP().toString().c_str(), WiFi.localIP().toString().c_str());
}

void loop() {
    ws.cleanupClients();
    if (ws.count() == 0) return;

    uint32_t now = millis();

    // ---- 频谱帧：最快 ~33fps（30ms 节流），并且要等待发送队列空闲 ----
    if (now - g_lastSend >= 30 && ws.availableForWriteAll()) {
        uint32_t startHz = 0, stepHz = 0;
        uint16_t seq = 0;
        uint16_t n = Spectrum::takeFrame(g_live, g_hold, MAX_BINS, startHz, stepHz, seq);
        if (n) {
            // 预设模式：先把频率表(0x02)发到前面，保证前端在收到频谱帧前已拿到频率表
            uint32_t ft[MAX_BINS]; uint16_t fn = 0;
            bool preset = Spectrum::takeFreqTable(ft, MAX_BINS, fn);
            if (preset) {
                uint8_t fp[6 + MAX_BINS * 4];
                fp[0] = 0xA5; fp[1] = 0x02;
                fp[2] = (uint8_t)(seq & 0xFF); fp[3] = (uint8_t)(seq >> 8);
                fp[4] = (uint8_t)(fn & 0xFF); fp[5] = (uint8_t)(fn >> 8);
                for (uint16_t i = 0; i < fn; i++) memcpy(&fp[6 + i * 4], &ft[i], 4);
                ws.binaryAll((const char*)fp, 6 + fn * 4);
            }

            g_pkt[0] = 0xA5;
            g_pkt[1] = 0x01;
            g_pkt[2] = (uint8_t)(seq & 0xFF);
            g_pkt[3] = (uint8_t)(seq >> 8);
            memcpy(&g_pkt[4], &startHz, 4);
            memcpy(&g_pkt[8], &stepHz, 4);
            g_pkt[12] = (uint8_t)(n & 0xFF);
            g_pkt[13] = (uint8_t)(n >> 8);
            g_pkt[14] = 0x01 | (preset ? 0x02 : 0x00);   // bit0=带maxHold, bit1=预设模式
            g_pkt[15] = 0;
            memcpy(&g_pkt[16], g_live, n);
            memcpy(&g_pkt[16 + n], g_hold, n);
            ws.binaryAll((const char*)g_pkt, 16 + n * 2);
            g_lastSend = now;
        }
    }

    // ---- 捕获事件 ----
    Hit h;
    while (Spectrum::popHit(h)) {
        String j = buildHitJson(h);
        ws.textAll(j);
    }

    // ---- 解码事件（自动解码开启时）----
    DecodeResult dr;
    while (Spectrum::popDecode(dr)) {
        ws.textAll(buildDecodeJson(dr));
        /* 历史保存由前端"解码记录"列表的手工"保存"按钮触发（cmd=hadd），不自动落盘 */
    }

    // ---- 状态心跳 ----
    if (now - g_lastStat >= 1000) {
        g_lastStat = now;
        ws.textAll(buildStatJson());
    }
	
	ElegantOTA.loop();
}

}  // namespace Web
