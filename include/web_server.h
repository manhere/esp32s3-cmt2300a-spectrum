#pragma once
#include <Arduino.h>

namespace Web {
void begin();
void loop();          // 在 loop() 中调用：推送频谱帧 / 事件 / 状态
}
