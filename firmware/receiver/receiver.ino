// ===================================================================
// playerthree · 接收端固件 (receiver)  v2.1
// 板卡: Arduino Pro Micro / Leonardo (ATmega32U4) —— 必须用 32U4!
// 只有 32U4 原生具备 USB HID(鼠标) 能力: 插入电脑即被识别, 免驱动
// 用 UNO 编译本代码会报 "Mouse was not declared" —— 属正常, 别用 UNO
//
// 职能: 收头端 8 字节帧 → 校验 → 位移直通 HID 鼠标（第二鼠标设备）
// 头戴端按键（开关键/灵敏度键）已在头端生效，本端只透传状态供调试
//
// 双模式（与头端对应）:
//   USE_SIM=1 串口打桩: 头端模拟模式串口输出 → 本板串口输入, 联调整条链路
//   USE_SIM=0 nRF24 真机模式
// ===================================================================

#define USE_SIM 1

#if USE_SIM == 0
  #include <SPI.h>
  #include <RF24.h>
  const uint64_t PIPE = 0xE8E8F0F0E1LL;   // 与头端一致
  const uint8_t PIN_CE = 9;
  const uint8_t PIN_CSN = 10;             // 32U4 的 SS
  RF24 radio(PIN_CE, PIN_CSN);
#endif

#include <Mouse.h>   // 32U4 内建的 USB HID 鼠标库

#define FRAME_HEAD 0xAA
#define MASK_BTN_L (1 << 0)   // bit0 = 开关键状态（头控启用中=1）
#define MASK_BTN_R (1 << 1)   // bit1 = 灵敏度挡位 bit0
#define MASK_BTN_H (1 << 2)   // bit2 = 灵敏度挡位 bit1

void setup() {
  Serial.begin(115200);
  Mouse.begin();

#if USE_SIM == 1
  Serial.println(F("[SIM] receiver 串口打桩模式: 等待 8 字节帧..."));
#else
  radio.begin();
  radio.setChannel(76);
  radio.setDataRate(RF24_2MBPS);
  radio.setAutoAck(false);
  radio.setPayloadSize(8);
  radio.openReadingPipe(1, PIPE);
  radio.startListening();
#endif
}

void loop() {
  uint8_t f[8];
  bool got = false;

#if USE_SIM == 1
  // 串口累积到 8 字节取一帧（打桩联调用）
  if (Serial.available() >= 8) {
    Serial.readBytes(f, 8);
    got = true;
  }
#else
  if (radio.available()) {
    radio.read(f, 8);
    got = true;
  }
#endif

  if (!got) return;

  // ---- 帧校验 ----
  if (f[0] != FRAME_HEAD) return;            // 帧头
  uint8_t sum = 0;
  for (int i = 0; i < 7; i++) sum += f[i];
  if (sum != f[7]) return;                   // sum8 校验失败弃帧

  // ---- 解析 ----
  int16_t dx = (int16_t)(f[2] | ((uint16_t)f[3] << 8));
  int16_t dy = (int16_t)(f[4] | ((uint16_t)f[5] << 8));
  uint8_t mask = f[1];

  // ---- 输出 HID（位移直通；纹丝不动时不发包, 省 USB 流量）----
  if (dx != 0 || dy != 0) {
    Mouse.move(dx, dy);
  }

  // ---- 状态透传（调试用，可删）----
  static uint8_t lastMask = 0xFF;
  if (mask != lastMask) {
    lastMask = mask;
    Serial.print(F("[RX] mask="));
    Serial.print(mask);
    Serial.print(F("  headOn="));
    Serial.print((mask & MASK_BTN_L) ? 1 : 0);
    Serial.print(F("  sensLevel="));
    Serial.println((mask & MASK_BTN_H) ? 2 : ((mask & MASK_BTN_R) ? 1 : 0));
  }
}