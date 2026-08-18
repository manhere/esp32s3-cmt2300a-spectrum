/*
 * history.cpp —— 解码历史持久化（LittleFS）
 * 见 history.h 说明。用 ArduinoJson 序列化单行 JSON Lines。
 */
#include "history.h"
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "esp_littlefs.h"      /* esp_littlefs_format：显式格式化兜底 */

static HistRec g_rec[HIST_MAX];
static int     g_n = 0;
static uint32_t g_nextId = 1;
static bool    g_fsOk = false;

static void saveAll() {
    if (!g_fsOk) return;
    File f = LittleFS.open(HIST_FILE, "w");
    if (!f) return;
    for (int i = g_n - 1; i >= 0; i--) {       /* 数组最新在前 -> 文件按时间正序 */
        JsonDocument doc;
        doc["id"] = g_rec[i].id;
        doc["freq"] = g_rec[i].freq;
        doc["code"] = g_rec[i].code;
        doc["bits"] = g_rec[i].bits;
        doc["te"]   = g_rec[i].te;
        doc["proto"]= g_rec[i].proto;
        doc["name"] = g_rec[i].name;
        serializeJson(doc, f);
        f.print('\n');
    }
    f.close();
}

namespace History {

bool begin() {
    /* 挂载 LittleFS（core 3.x 签名：begin(formatOnFail, basePath, maxOpenFiles, partitionLabel)，
       partitionLabel 默认 "spiffs" = default_16MB 的数据分区）。
       formatOnFail=true：esp_littlefs mount 失败时自动格式化；
       若仍失败再显式 esp_littlefs_format 兜底（首次使用/旧数据分区）。 */
    g_fsOk = LittleFS.begin(true);
    if (!g_fsOk) {
        Serial.println(F("[hist] 挂载失败，尝试格式化 spiffs 分区..."));
        esp_err_t e = esp_littlefs_format("spiffs");
        g_fsOk = (e == ESP_OK) && LittleFS.begin(true);
    }
    if (!g_fsOk) { Serial.println(F("[hist] LittleFS 挂载失败，历史仅内存保存")); return false; }
    File f = LittleFS.open(HIST_FILE, "r");
    if (!f) return true;                        /* 无历史文件 = 首次使用 */
    g_n = 0; g_nextId = 1;
    while (f.available() && g_n < HIST_MAX) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.isEmpty()) continue;
        JsonDocument doc;
        if (deserializeJson(doc, line)) continue;
        HistRec& r = g_rec[g_n];
        r.id    = doc["id"] | (uint32_t)0;
        r.freq  = doc["freq"] | 0.0f;
        r.code  = doc["code"] | (uint32_t)0;
        r.bits  = doc["bits"] | (uint8_t)0;
        r.te    = doc["te"]   | (uint16_t)0;
        r.proto = doc["proto"]| (uint8_t)0;
        strncpy(r.name, doc["name"] | "", sizeof(r.name) - 1);
        r.name[sizeof(r.name) - 1] = '\0';
        if (r.id >= g_nextId) g_nextId = r.id + 1;
        g_n++;
    }
    f.close();
    Serial.printf("[hist] 已加载 %d 条历史\n", g_n);
    return true;
}

/* 判重：同 code+bits 且 |freq差|<=0.05MHz 且 |te差|<=20us 视为同一条遥控 */
static bool isDup(const HistRec& r) {
    for (int i = 0; i < g_n; i++) {
        const HistRec& e = g_rec[i];
        if (e.code == r.code && e.bits == r.bits &&
            fabsf(e.freq - r.freq) <= 0.05f &&
            abs((int)e.te - (int)r.te) <= 20)
            return true;
    }
    return false;
}

bool add(const HistRec& r) {
    if (isDup(r)) return false;                 /* 判重：不重复保存 */
    if (g_n >= HIST_MAX) g_n = HIST_MAX - 1;    /* 满则缩一格，末尾最旧将被挤出 */
    memmove(&g_rec[1], &g_rec[0], g_n * sizeof(HistRec));   /* 右移，新记录插头部 */
    g_rec[0] = r;
    g_rec[0].id = g_nextId++;
    g_n++;
    saveAll();
    return true;
}

int count() { return g_n; }

bool get(uint32_t id, HistRec& r) {
    for (int i = 0; i < g_n; i++)
        if (g_rec[i].id == id) { r = g_rec[i]; return true; }
    return false;
}

bool remove(uint32_t id) {
    for (int i = 0; i < g_n; i++) {
        if (g_rec[i].id == id) {
            memmove(&g_rec[i], &g_rec[i + 1], (g_n - i - 1) * sizeof(HistRec));
            g_n--;
            saveAll();
            return true;
        }
    }
    return false;
}

bool updateName(uint32_t id, const char* name) {
    for (int i = 0; i < g_n; i++) {
        if (g_rec[i].id == id) {
            strncpy(g_rec[i].name, name, sizeof(g_rec[i].name) - 1);
            g_rec[i].name[sizeof(g_rec[i].name) - 1] = '\0';
            saveAll();
            return true;
        }
    }
    return false;
}

void clear() {
    g_n = 0;
    saveAll();
}

String toListJson() {
    JsonDocument d;
    d["type"] = "hlist";
    d["n"] = g_n;
    JsonArray list = d["list"].to<JsonArray>();
    for (int i = 0; i < g_n; i++) {             /* 最新在前 */
        JsonObject o = list.add<JsonObject>();
        o["id"]    = g_rec[i].id;
        o["freq"]  = g_rec[i].freq;
        o["code"]  = g_rec[i].code;
        o["bits"]  = g_rec[i].bits;
        o["te"]    = g_rec[i].te;
        o["proto"] = g_rec[i].proto;
        o["name"]  = g_rec[i].name;
    }
    String out;
    serializeJson(d, out);
    return out;
}

String toExportJson() {
    JsonDocument d;
    d["type"] = "hexport";
    d["n"] = g_n;
    JsonArray list = d["list"].to<JsonArray>();
    for (int i = g_n - 1; i >= 0; i--) {        /* 时间正序 */
        JsonObject o = list.add<JsonObject>();
        o["id"]    = g_rec[i].id;
        o["freq"]  = g_rec[i].freq;
        o["code"]  = g_rec[i].code;
        o["bits"]  = g_rec[i].bits;
        o["te"]    = g_rec[i].te;
        o["proto"] = g_rec[i].proto;
        o["name"]  = g_rec[i].name;
    }
    String out;
    serializeJson(d, out);
    return out;
}

}  // namespace History
