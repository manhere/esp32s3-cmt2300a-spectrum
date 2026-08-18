#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把 data/index.html 压缩成 gzip 并生成 include/web_index.h (PROGMEM 字节数组)。
修改网页后运行：  python tools/gen_web_header.py
"""
import gzip
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "data", "index.html")
DST = os.path.join(ROOT, "include", "web_index.h")


def main():
    if not os.path.exists(SRC):
        print("找不到", SRC)
        return 1
    raw = open(SRC, "rb").read()
    gz = gzip.compress(raw, 9, mtime=0)   # mtime=0：固定 gzip 头时间戳，输出确定性（避免每次生成产生 git 噪声）

    lines = []
    for i in range(0, len(gz), 16):
        chunk = gz[i:i + 16]
        lines.append("    " + ", ".join("0x%02X" % b for b in chunk) + ",")

    out = f"""/*
 * web_index.h —— 由 tools/gen_web_header.py 自动生成，请勿手动编辑
 * 源文件: data/index.html   原始 {len(raw)} 字节 -> gzip {len(gz)} 字节
 */
#pragma once
#include <Arduino.h>

const uint8_t INDEX_HTML_GZ[] PROGMEM = {{
{chr(10).join(lines)}
}};
const size_t INDEX_HTML_GZ_LEN = sizeof(INDEX_HTML_GZ);
"""
    os.makedirs(os.path.dirname(DST), exist_ok=True)
    open(DST, "w", encoding="utf-8", newline="\n").write(out)
    print("已生成 %s  (%d -> %d 字节, 压缩率 %.1f%%)" %
          (os.path.relpath(DST, ROOT), len(raw), len(gz), 100.0 * len(gz) / len(raw)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
