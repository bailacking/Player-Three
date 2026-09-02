// ===================================================================
// playerthree · 头戴端固件 (head_unit)  v2.2
// 板卡: Arduino Pro Micro (ATmega32U4 5V/16MHz) —— 自带 USB 直插烧录
//       （台面验证阶段可直接用 UNO R3，D 引脚通用）
// 职责: 陀螺仪姿态 → 角度差积分（角速度式）→ 8字节帧 → nRF24 无线发送
//
// 双数据源模式（发板子前先用模拟模式把链路跑通）:
//   #define USE_SIM 1 → 模拟模式: 无任何硬件也能编译运行, 串口看数据流
//   #define USE_SIM 0 → 真机模式: 需安装 MPU6050(i2cdevlib) 与 RF24 库
//
// 模拟模式用法: 烧入后开串口监视器 115200
//   看到 dx/dy 随"转头波形"平滑变化即映射逻辑正常
//   输入 o=开关键(头控开/关)  s=灵敏度挡位循环   h=手动校准
//
// v2.2 变更（2026-09-02 W1 优化）:
//   - 校准系统落地: 静止 1s 自动归零 / 长按灵敏度键≥0.5s 手动校准 (SIM 用 h / 上电 2s 稳定期
//   - 滤波补全: 死区 + EMA 低通 + 静止归零(去零偏漂移)
//   - 修开关键极性 bug: 原代码按下=发送(反了), 改为 按下=暂停/关闭
//   - 1000Hz: 默认 PERIOD=1ms, 跑不动自动退 500Hz (32U4 HID 原生 1ms, 无需改描述符)
// ===================================================================

#define USE_SIM 1

#if USE_SIM == 0
  #include <SPI.h>
  #include <RF24.h>
  #include "I2Cdev.h"
  #include "MPU6050_6Axis_MotionApps20.h"
#endif

// ---------- 引脚（真机模式生效，32U4 的 I2C= D2(SDA)/D3(SCL)，按键避开）----------
#define PIN_CE     9   // nRF24 CE
#define PIN_CSN    8   // nRF24 CSN
#define PIN_BTN_L  5   // 开关键（D5 ⇔ GND：按下=头控暂停；松手=启用）
#define PIN_BTN_R  6   // 灵敏度键（D6 ⇔ GND：短按=挡位+1；长按≥0.5s=手动校准）
#define PIN_BTN_H  4   // 空（功能并入灵敏度键长按）

// ---------- 手感参数（调优时只改这里）----------
#define DEADZONE_ANG 0.05f  // 死区(角度差°): |每帧转角| < 此值不输出, 防微颤
#define GAIN_A       15.0f  // 增益A: 角度差→像素 整体速度
#define GAIN_B       1.0f   // 增益b: 非线性陡峭度(越大越"甩")
#define CLAMP_PX     40     // 单帧最大位移(px), 防猛甩头飞屏
#define EMA_ALPHA    0.4f   // 平滑系数 0~1, 越小越平滑(也越钝); 不想要就设 1.0
#define PERIOD_MS_FAST 1    // 1000Hz（32U4 HID 原生 1ms，无需改描述符）
#define PERIOD_MS_SAFE 2    // 500Hz 兜底（实测跟不上 1ms 自动退）
#define STILL_EPS     0.01f // 静止判据(°) : 每帧角度差 < 0.01°≈5°/s 视为静止
#define STILL_MS      1000  // 静止多久自动归零校准(ms)
#define HOLD_MS       500   // 长按校准阈值(ms)
#define BOOT_HOLD_MS  2000  // 上电稳定期: 前 2s 只读不输出(等 DMP/nRF/USB 就绪)

// ---------- 帧协议（与 protocol/protocol.md v1.1 一致）----------
#define FRAME_HEAD   0xAA
#define MASK_BTN_L   (1 << 0)   // bit0 = 开关键状态（头控启用中=1）
#define MASK_BTN_R   (1 << 1)   // bit1 = 灵敏度挡位 bit0
#define MASK_BTN_H   (1 << 2)   // bit2 = 灵敏度挡位 bit1（挡位 0-2）

#if USE_SIM == 0
  const uint64_t PIPE = 0xE8E8F0F0E1LL;   // 与接收端一致即可
  RF24 radio(PIN_CE, PIN_CSN);
  MPU6050 mpu;
  uint8_t devStatus;
  uint8_t fifoBuffer[64];
  Quaternion q;
  VectorFloat gravity;
  float euler[3];        // [0]=yaw [1]=pitch [2]=roll (弧度)
#endif

// ---------- 全局状态 ----------
float yaw = 0, pitch = 0;
bool headOn = true;                       // 头控启用中(开关键: 按下=暂停)
uint8_t sensLevel = 1;                    // 灵敏度挡位 0-2
float lastYaw = 0, lastPitch = 0;         // 角度差积分的上一帧角度
unsigned long lastTick = 0;
int16_t lastDx = 0, lastDy = 0;
int perfPeriodMs = PERIOD_MS_FAST;        // 实际节拍(1=1000Hz, 2=500Hz)
unsigned long bootAt = 0;
unsigned long stillMs = 0;
#if USE_SIM == 0
  bool lastBtnR = true;                   // 灵敏度键边沿检测
  bool calibrating = false;               // 长按中(已触发一次校准)
  unsigned long btnDownAt = 0;
#endif

void setup() {
  Serial.begin(115200);
  delay(2000);   // 32U4 USB-CDC 初始化需 ~1-2s，等就绪再打印（2026-09-02 W1 排障）

#if USE_SIM == 1
  Serial.println(F("[SIM] head_unit 模拟模式启动（无硬件）"));
  Serial.println(F("[SIM] 串口输入: o=开关键 s=灵敏度挡位 h=手动校准"));
#else
  // ---------- 真机初始化 ----------
  pinMode(PIN_BTN_L, INPUT_PULLUP);
  pinMode(PIN_BTN_R, INPUT_PULLUP);
  pinMode(PIN_BTN_H, INPUT_PULLUP);

  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  // 出厂偏移校准: 模块到手后, 按教程把下面三个数改成实测值(当前为占位)
  // mpu.setXGyroOffset(220); mpu.setYGyroOffset(76); mpu.setZGyroOffset(-85);
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);   // DMP 内置融合, 主控只读结果
  }

  radio.begin();
  radio.setChannel(76);
  radio.setDataRate(RF24_2MBPS);  // 延迟优先: 最短空口时间
  radio.setAutoAck(false);        // 丢包即弃, 下一帧 2ms 后就来
  radio.setPayloadSize(8);
  radio.openWritingPipe(PIPE);
#endif

  bootAt = millis();
}

void loop() {
  // ---- 节拍（默认 1000Hz，跟不上自动退 500Hz）----
  unsigned long now = millis();
  unsigned long elapsed = now - lastTick;
  if (elapsed < (unsigned long)perfPeriodMs) return;
  if (perfPeriodMs == PERIOD_MS_FAST && elapsed > 3) {   // 1ms 档实际间隙 >3ms → 退档
    perfPeriodMs = PERIOD_MS_SAFE;
#if USE_SIM == 1
    Serial.println(F("[PERF] 1000Hz 跟不上, 自动退 500Hz"));
#endif
  }
  lastTick = now;

  // 1) 数据源: 原始姿态角(°) + 按键
  readInputs();

#if USE_SIM == 1
  // 2) 模拟按键: 串口命令（仅模拟模式）
  while (Serial.available()) {
    char c = Serial.read();
    if      (c == 'o') headOn = !headOn;                 // 开关键
    else if (c == 's') sensLevel = (sensLevel + 1) % 3;  // 挡位循环
    else if (c == 'h') calibrateNow();                   // 手动校准
  }
#endif

  // 3) 角速度积分: 每帧"角度差" → 位移（头停=差为 0=准星停）
  float dyaw = yaw - lastYaw;
  float dpitch = pitch - lastPitch;
  lastYaw = yaw;
  lastPitch = pitch;

  // 4) 静止检测 → 满 STILL_MS 自动归零（去零偏漂移，防"准星自己爬"）
  if (fabsf(dyaw) < STILL_EPS && fabsf(dpitch) < STILL_EPS) {
    stillMs += perfPeriodMs;
    if (stillMs >= STILL_MS) {
      calibrateNow();            // 归零基准，输出恢复 0
      stillMs = 0;
    }
  } else {
    stillMs = 0;
  }

  // 5) 位移映射 + EMA 平滑
  int16_t dx = mapAngle(dyaw);
  int16_t dy = mapAngle(dpitch);
  dx = (int16_t)(EMA_ALPHA * dx + (1.0f - EMA_ALPHA) * lastDx);
  dy = (int16_t)(EMA_ALPHA * dy + (1.0f - EMA_ALPHA) * lastDy);
  lastDx = dx;
  lastDy = dy;

  // 6) 组 8 字节帧
  uint8_t frame[8];
  frame[0] = FRAME_HEAD;
  frame[1] = (headOn ? MASK_BTN_L : 0)
           | ((sensLevel & 0x01) ? MASK_BTN_R : 0)
           | ((sensLevel & 0x02) ? MASK_BTN_H : 0);
  frame[2] = (uint8_t)(dx & 0xFF);
  frame[3] = (uint8_t)((dx >> 8) & 0xFF);
  frame[4] = (uint8_t)(dy & 0xFF);
  frame[5] = (uint8_t)((dy >> 8) & 0xFF);
  frame[6] = 0;                       // 保留（V2 滚轮/附加键）
  uint8_t sum = 0;
  for (int i = 0; i < 7; i++) sum += frame[i];
  frame[7] = sum;                     // sum8 校验

  // 7) 输出（开关键按下=暂停；上电稳定期内不发送）
#if USE_SIM == 1
  Serial.print(F("on="));    Serial.print(headOn ? 1 : 0);
  Serial.print(F(" lv="));   Serial.print(sensLevel);
  Serial.print(F("  dx="));  Serial.print(dx);
  Serial.print(F(" dy="));   Serial.println(dy);
#else
  if (now - bootAt < BOOT_HOLD_MS) return;   // 上电稳定期: 只读不输出
  if (headOn) {                              // 开关键: 按下=暂停, 松手=启用
    radio.write(&frame, 8);
  }
#endif
}

// 手动校准/自动归零: 把当前角度设为新基准 → 输出立即归 0
void calibrateNow() {
  lastYaw = yaw;
  lastPitch = pitch;
  lastDx = 0; lastDy = 0;
  stillMs = 0;
#if USE_SIM == 1
  Serial.println(F("[CAL] 基准已归零"));
#endif
}

// ---------- 数据源层: 读原始姿态角 + 按键 ----------
void readInputs() {
#if USE_SIM == 1
  // 模拟: 固定"转头波形"代替真实头动, 输出应规则平滑
  static float t = 0;
  t += (float)perfPeriodMs / 1000.0f;
  yaw   = 30.0f * sin(t * 2.0f);
  pitch = 18.0f * sin(t * 1.3f + 1.0f);
#else
  // 真机: DMP FIFO → 欧拉角（弧度→度）；DMP 有包才更新，无包沿用旧值
  while (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetEuler(euler, &q);
    yaw   = euler[0] * 180.0f / PI;
    pitch = euler[1] * 180.0f / PI;
  }

  // 开关键: 按下（低电平）→ 头控暂停；松手 → 启用
  headOn = digitalRead(PIN_BTN_L);

  // 灵敏度键: 短按=挡位+1（循环 0-2）；长按≥HOLD_MS=手动校准
  bool btnRPin = !digitalRead(PIN_BTN_R);
  if (btnRPin) {
    if (!lastBtnR) { btnDownAt = millis(); calibrating = false; }  // 按下沿
    if (!calibrating && millis() - btnDownAt >= HOLD_MS) {          // 长按触发一次
      calibrating = true;
      calibrateNow();
      Serial.println(F("[KEY] 手动校准完成"));
    }
  } else {
    if (lastBtnR && !calibrating) {                                 // 松开且没长按过 → 短按
      sensLevel = (sensLevel + 1) % 3;
      Serial.print(F("[KEY] sensLevel="));
      Serial.println(sensLevel);
    }
    calibrating = false;
  }
  lastBtnR = btnRPin;
#endif
}

// ---------- 角度差→位移: 死区 → 指数增益×挡位 → clamp ----------
// 手感方向反了? 把返回值取负即可(dx/dy 同理)
float sensMult() {
  if (sensLevel == 0) return 1.0f;
  if (sensLevel == 1) return 1.5f;
  return 2.2f;
}

int16_t mapAngle(float deg) {
  float a = fabsf(deg);
  if (a < DEADZONE_ANG) return 0;                       // 死区（防微颤）
  float gain = GAIN_A * (expf(GAIN_B * a) - 1.0f) * sensMult();  // 指数增益
  if (deg < 0) gain = -gain;
  int16_t out = (int16_t)gain;
  if (out >  CLAMP_PX) out =  CLAMP_PX;                 // 防飞屏
  if (out < -CLAMP_PX) out = -CLAMP_PX;
  return out;
}