/*
 * history.h —— 解码历史持久化（LittleFS）
 *
 * 解码成功的记录（频率/code/bits/te/协议/名称）追加保存到 LittleFS，
 * 重启不丢失，支持 列表/导出/清空/删除/改名，记录可重放。
 *
 * 存储：内存数组（最新在前，HIST_MAX 条环形）+ 每次变更整体重写
 *       /history.jsonl（JSON Lines，单行一条）。解码频率低（按按键），
 *       几百条重写几十 KB 毫秒级，无擦写寿命压力。
 */
#pragma once
#include <Arduino.h>

#define HIST_MAX   500
#define HIST_FILE  "/history.jsonl"

struct HistRec {
    uint32_t id;          /* 自增 id（重启后从文件恢复最大值+1） */
    float    freq;        /* MHz */
    uint32_t code;
    uint8_t  bits;
    uint16_t te;          /* us */
    uint8_t  proto;       /* 协议号（1=已识别） */
    char     name[24];    /* 协议名/用户编辑名 */
};

namespace History {

/* 挂载 LittleFS + 加载历史。返回 false 表示文件系统不可用（仅内存记录）。 */
bool begin();

/* 追加一条（数组头部插入 + 重写文件；超出 HIST_MAX 丢弃最旧）。
   判重：同 code+bits 且 |freq差|<=0.05MHz 且 |te差|<=20us 视为同一条，返回 false 不保存。 */
bool add(const HistRec& r);

int  count();                                  /* 当前条数 */
bool get(uint32_t id, HistRec& r);
bool remove(uint32_t id);                      /* 删除一条 */
bool updateName(uint32_t id, const char* name);/* 编辑名称 */
void clear();                                  /* 清空 */

/* 列表 JSON：{"type":"hlist","n":N,"list":[{id,freq,code,bits,te,proto,name},...]} 最新在前 */
String toListJson();
/* 导出 JSON：{"type":"hexport","n":N,"list":[...]} 原序（文件序） */
String toExportJson();

}  // namespace History
