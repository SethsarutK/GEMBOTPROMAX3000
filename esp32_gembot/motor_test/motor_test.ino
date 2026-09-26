// =====================================================================
//  motor_test.ino — ทดสอบขับล้ออย่างเดียว (ไม่มี WiFi / servo)
//  ต้องใช้ esp32 core 3.x (ledcAttach)
//  ลำดับ: หน้า 2s -> หยุด 1s -> ถอย 2s -> หยุด -> เลี้ยวซ้าย 1s -> หยุด -> เลี้ยวขวา 1s -> หยุด -> วน
//  ยกล้อลอยจากพื้นก่อนทดสอบครั้งแรก!
// =====================================================================

// ---------- TODO แก้ให้ตรงบอร์ดจริง ----------
#define MOTOR_MODE  1        // 1 = IN1/IN2 (2 ขา PWM ต่อมอเตอร์)   2 = DIR + PWM
#define L_PIN_A     16       // mode1: IN1 | mode2: DIR
#define L_PIN_B     17       // mode1: IN2 | mode2: PWM
#define R_PIN_A     18
#define R_PIN_B     19
#define L_INVERT    false    // เปลี่ยนเป็น true ถ้าล้อซ้ายหมุนถอยตอนสั่งเดินหน้า
#define R_INVERT    false
#define SPEED       150      // 0..255 เริ่มต่ำ ๆ ก่อน

void motorWrite(int pinA, int pinB, int v, bool invert) {   // v = -255..255
  if (invert) v = -v;
  int pwm = abs(v);
#if MOTOR_MODE == 1
  if (v > 0)      { ledcWrite(pinA, pwm); ledcWrite(pinB, 0); }
  else if (v < 0) { ledcWrite(pinA, 0);   ledcWrite(pinB, pwm); }
  else            { ledcWrite(pinA, 0);   ledcWrite(pinB, 0); }
#else
  ledcWrite(pinA, v >= 0 ? 255 : 0);
  ledcWrite(pinB, pwm);
#endif
}

void drive(int l, int r, const char *name, int ms) {
  Serial.println(name);
  motorWrite(L_PIN_A, L_PIN_B, l, L_INVERT);
  motorWrite(R_PIN_A, R_PIN_B, r, R_INVERT);
  delay(ms);
  motorWrite(L_PIN_A, L_PIN_B, 0, L_INVERT);
  motorWrite(R_PIN_A, R_PIN_B, 0, R_INVERT);
  delay(1000);
}

void setup() {
  Serial.begin(115200);
  ledcAttach(L_PIN_A, 20000, 8);
  ledcAttach(L_PIN_B, 20000, 8);
  ledcAttach(R_PIN_A, 20000, 8);
  ledcAttach(R_PIN_B, 20000, 8);
  delay(2000);                       // มีเวลาวางหุ่น
}

void loop() {
  drive( SPEED,  SPEED, "FORWARD", 2000);
  drive(-SPEED, -SPEED, "BACKWARD", 2000);
  drive(-SPEED,  SPEED, "TURN LEFT", 1000);
  drive( SPEED, -SPEED, "TURN RIGHT", 1000);
}
