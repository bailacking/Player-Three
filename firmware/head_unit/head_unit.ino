// ===================================================================
// playerthree · 头戴端固件 (head_unit)  v2.1
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
//   输入 o=开关键(头控开/关)  s=灵敏度挡位循环   h=校准(预留)
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
#define PIN_BTN_L  5   // 开关键（D5 ⇔ GND：按下=头控关闭）
#define PIN_BTN_R  6   // 灵敏度键（D6 ⇔ GND：按下沿=挡位+1 循环 0-2）
#define PIN_BTN_H  4   // 预留（V2：手动校准）

// ---------- 手感参数（调优时只改这里）----------
#define DEADZONE_ANG 0.05f  // 死区(角度差°): |每帧转角| < 此值不输出, 防微颤
#define GAIN_A       15.0f  // 增益A: 角度差→像素 整体速度
#define GAIN_B       1.0f   // 增益b: 非线性陡峭度(越大越"甩")
#define CLAMP_PX     40     // 单帧最大位移(px), 防猛甩头飞屏
#define EMA_ALPHA    0.4f   // 平滑系数 0~1, 越小越平滑(也越钝); 不想要就设 1.0
#define PERIOD_MS    2      // 上报周期 = 500Hz（1ms=1000Hz 属极限优化项）

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
bool btnL = false, btnH = false;          // btnL=开关键状态（按下=关闭）
uint8_t sensLevel = 1;                    // 灵敏度挡位 0-2
float lastYaw = 0, lastPitch = 0;         // 角度差积分的上一帧角度
unsigned long lastTick = 0;
int16_t lastDx = 0, lastDy = 0;
#if USE_SIM == 0
  bool lastBtnR = true;                   // 灵敏度键边沿检测
#endif

void setup() {
  Serial.begin(115200);

#if USE_SIM == 1
  Serial.println(F("[SIM] head_unit 模拟模式启动（无硬件）"));
  Serial.println(F("[SIM] 串口输入: o=开关键 s=灵敏度挡位 h=校准(预留)"));
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
}

void loop() {
  // ---- 节拍（默认 500Hz）----
  unsigned long now = millis();
  if (now - lastTick < PERIOD_MS) return;
  lastTick = now;

  // 1) 数据源: 原始姿态角(°) + 按键
  readInputs();

#if USE_SIM == 1
  // 2) 模拟按键: 串口命令（仅模拟模式）
  while (Serial.available()) {
    char c = Serial.read();
    if      (c == 'o') btnL = !btnL;                       // 开关键
    else if (c == 's') sensLevel = (sensLevel + 1) % 3;    // 挡位循环
    else if (c == 'h') btnH = !btnH;
  }
#endif

  // 3) 角速度积分: 每帧"角度差" → 位移（头停=差为 0=准星停）
  float dyaw = yaw - lastYaw;
  float dpitch = pitch - lastPitch;
  lastYaw = yaw;
  lastPitch = pitch;
  int16_t dx = mapAngle(dyaw);
  int16_t dy = mapAngle(dpitch);

  // 4) EMA 平滑
  dx = (int16_t)(EMA_ALPHA * dx + (1.0f - EMA_ALPHA) * lastDx);
  dy = (int16_t)(EMA_ALPHA * dy + (1.0f - EMA_ALPHA) * lastDy);
  lastDx = dx;
  lastDy = dy;

  // 5) 组 8 字节帧
  uint8_t frame[8];
  frame[0] = FRAME_HEAD;
  frame[1] = (btnL ? MASK_BTN_L : 0)
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

  // 6) 输出（开关键按下=头控关闭时停止发送, 省电+零输入）
#if USE_SIM == 1
  Serial.print(F("on="));    Serial.print(btnL ? 1 : 0);
  Serial.print(F(" lv="));   Serial.print(sensLevel);
  Serial.print(F("  dx="));  Serial.print(dx);
  Serial.print(F(" dy="));   Serial.println(dy);
#else
  if (btnL) {                          // 开关键未按下 → 头控启用中
    radio.write(&frame, 8);
  }
#endif
}

// ---------- 数据源层: 读原始姿态角 + 按键 ----------
void readInputs() {
#if USE_SIM == 1
  // 模拟: 固定"转头波形"代替真实头动, 输出应规则平滑
  static float t = 0;
  t += (float)PERIOD_MS / 1000.0f;
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

  // 开关键：按下（低电平）→ 头控关闭
  btnL = !digitalRead(PIN_BTN_L);

  // 灵敏度键：按下沿 → 挡位 +1（循环 0-2）
  bool btnRPin = !digitalRead(PIN_BTN_R);
  if (btnRPin && !lastBtnR) {
    sensLevel = (sensLevel + 1) % 3;
    Serial.print(F("[KEY] sensLevel="));
    Serial.println(sensLevel);
  }
  lastBtnR = btnRPin;
  btnH = false;
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