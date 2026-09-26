// =====================================================================
//  GEMBOT ESP32 firmware  (Arduino IDE, board: ESP32 Dev Module)
//  v3.2: ปากหนีบ = servo ตัวเดียว (เฟืองคุมสองข้าง) — คีบแล้วลากไปโซน ไม่ยก
//  ขับ servo ด้วย LEDC ตรง ๆ ไม่ใช้ library ESP32Servo
//
//  ไม่มีกระบะ ไม่มี servo ยกแขน — ปากหนีบระดับพื้น
//  PICK  = เปิด -> หนีบ (ค้างไว้ ถือหินไปทั้งอย่างนั้น)
//  DUMP  = เปิดปล่อยหิน (ค้างเปิดไว้ พร้อมหยิบเม็ดถัดไป)
//
//  โปรโตคอล UDP (text, จบด้วย \n)
//   PC -> ESP32
//     M,<vL>,<vR>,<seq>       ความเร็วล้อ -100..100
//     PICK,<seq>               ลำดับหยิบ: เปิด -> หนีบค้าง
//     DUMP,<seq>               ลำดับปล่อย: เปิด
//     S,<grip>,<seq>           สั่งมุม servo หลัก (ตอนจูน)
//     A,<assist>,<seq>         สั่งมุม servo ตัวช่วย GPIO 19 อย่างเดียว (ตอนจูนให้ตรงกับตัวหลัก)
//     STOP                     หยุดทุกอย่าง
//     PING                     ตอบ PONG
//   ESP32 -> PC (ทุก STATUS_MS)
//     ST,<seq_last>,<busy>,<vbat>,<uptime_ms>
// =====================================================================
#include <WiFi.h>
#include <WiFiUdp.h>
#include "config.h"

WiFiUDP udp;
IPAddress pcIP;
uint16_t  pcPort = 0;
bool      havePC = false;

// ---------------- Servo ผ่าน LEDC (50 Hz, 16 bit) ----------------
// มุม 0..180 -> พัลส์ 500..2400 us
struct LedcServo {
  int pin;
  void attach(int p) { pin = p; ledcAttach(pin, 50, 16); }
  void write(int deg) {
    deg = constrain(deg, 0, 180);
    uint32_t us = 500 + (uint32_t)deg * (2400 - 500) / 180;
    uint32_t duty = (uint32_t)((uint64_t)us * 65535 / 20000);   // คาบ 20 ms
    ledcWrite(pin, duty);
  }
};
LedcServo sGrip, sAssist;

// เปิด/ปิดปาก: สั่งตัวหลัก และตัวช่วย (ถ้าเปิดใช้) พร้อมกัน
void gripTo(bool open) {
  sGrip.write(open ? GRIP_OPEN : GRIP_CLOSE);
#if GRIP_ASSIST
  sAssist.write(open ? ASSIST_OPEN : ASSIST_CLOSE);
#endif
}

unsigned long lastCmdMs    = 0;
unsigned long lastStatusMs = 0;
long          lastSeq      = 0;
int           curL = 0, curR = 0;

// ---------------- Motor ----------------
void motorSetup() {
  ledcAttach(L_PIN_A, PWM_FREQ, PWM_RES_BITS);
  ledcAttach(L_PIN_B, PWM_FREQ, PWM_RES_BITS);
  ledcAttach(R_PIN_A, PWM_FREQ, PWM_RES_BITS);
  ledcAttach(R_PIN_B, PWM_FREQ, PWM_RES_BITS);
}

// v: -100..100  -> PWM ที่ขา
void motorWrite(int pinA, int pinB, int v, float trim, bool invert) {
  if (invert) v = -v;
  int mag = abs(v);
  int pwm = 0;
  if (mag > 0) pwm = map(mag, 1, 100, MOTOR_MIN_PWM, 255);
  pwm = constrain((int)(pwm * trim), 0, 255);
#if MOTOR_MODE == 1
  if (v > 0)      { ledcWrite(pinA, pwm); ledcWrite(pinB, 0); }
  else if (v < 0) { ledcWrite(pinA, 0);   ledcWrite(pinB, pwm); }
  else            { ledcWrite(pinA, 0);   ledcWrite(pinB, 0); }
#else
  ledcWrite(pinA, v >= 0 ? 255 : 0);   // DIR
  ledcWrite(pinB, pwm);                // PWM
#endif
}

void setMotors(int vL, int vR) {
  curL = constrain(vL, -100, 100);
  curR = constrain(vR, -100, 100);
  motorWrite(L_PIN_A, L_PIN_B, curL, L_TRIM, L_INVERT);
  motorWrite(R_PIN_A, R_PIN_B, curR, R_TRIM, R_INVERT);
}

// ---------------- Servo sequence (non-blocking) ----------------
// PICK จบด้วยปากหนีบ "ปิดค้าง" (ถือหินไว้) / DUMP = เปิดปล่อยที่โซน
enum SeqState { IDLE, PICK1, PICK2, DUMP1 };
SeqState seq = IDLE;
unsigned long seqT = 0;

bool busy() { return seq != IDLE; }

// รับเป็น int (ไม่ใช่ SeqState) เพราะ Arduino IDE สร้าง prototype ไว้ก่อน enum จะถูกประกาศ
void seqStart(int s) { seq = (SeqState)s; seqT = millis(); }

void seqStep(int next, int wait) {
  if (millis() - seqT >= (unsigned long)wait) { seq = (SeqState)next; seqT = millis(); }
}

void seqUpdate() {
  switch (seq) {
    case IDLE: break;
    // ---- PICK: เปิด -> หนีบค้าง ----
    case PICK1: gripTo(true);  seqStep(PICK2, T_GRIP); break;
    case PICK2: gripTo(false); seqStep(IDLE,  T_GRIP); break;
    // ---- DUMP: เปิดปล่อย (ค้างเปิดไว้) ----
    case DUMP1: gripTo(true);  seqStep(IDLE,  T_GRIP); break;
  }
}

void servoSetup() {
  sGrip.attach(SERVO_GRIP_PIN);
#if GRIP_ASSIST
  sAssist.attach(SERVO_ASSIST_PIN);
#endif
  gripTo(true);   // พร้อมหยิบเม็ดแรก
}

// ---------------- Battery ----------------
float readVbat() {
  if (VBAT_PIN < 0) return -1.0f;
  int raw = analogRead(VBAT_PIN);
  return raw / 4095.0f * 3.3f * VBAT_DIVIDER;
}

// ---------------- Command parser ----------------
void handlePacket(char *buf) {
  // ตัด \r\n
  for (char *p = buf; *p; ++p) if (*p == '\r' || *p == '\n') { *p = 0; break; }

  char *tok[6]; int n = 0;
  char *save;
  for (char *t = strtok_r(buf, ",", &save); t && n < 6; t = strtok_r(NULL, ",", &save)) tok[n++] = t;
  if (n == 0) return;

  if (!strcmp(tok[0], "M") && n >= 3) {
    // ขณะทำ sequence ให้ล้อหยุดนิ่ง กันหุ่นขยับตอนหยิบ
    if (!busy()) setMotors(atoi(tok[1]), atoi(tok[2]));
    if (n >= 4) lastSeq = atol(tok[3]);
    lastCmdMs = millis();
  }
  else if (!strcmp(tok[0], "STOP")) {
    setMotors(0, 0);
    seq = IDLE;
    lastCmdMs = millis();
  }
  else if (!strcmp(tok[0], "PICK")) {
    setMotors(0, 0);
    if (!busy()) seqStart(PICK1);
    if (n >= 2) lastSeq = atol(tok[1]);
    lastCmdMs = millis();
  }
  else if (!strcmp(tok[0], "DUMP")) {
    setMotors(0, 0);
    if (!busy()) seqStart(DUMP1);
    if (n >= 2) lastSeq = atol(tok[1]);
    lastCmdMs = millis();
  }
  else if (!strcmp(tok[0], "S") && n >= 2) {
    seq = IDLE;
    sGrip.write(constrain(atoi(tok[1]), 0, 180));
    if (n >= 3) lastSeq = atol(tok[2]);
    lastCmdMs = millis();
  }
  else if (!strcmp(tok[0], "A") && n >= 2) {
    // จูนตัวช่วย: attach เมื่อถูกสั่งครั้งแรก (ถ้า GRIP_ASSIST ปิดอยู่ก็ยังจูนได้)
    static bool assistReady = false;
    if (!assistReady) { sAssist.attach(SERVO_ASSIST_PIN); assistReady = true; }
    seq = IDLE;
    sAssist.write(constrain(atoi(tok[1]), 0, 180));
    if (n >= 3) lastSeq = atol(tok[2]);
    lastCmdMs = millis();
  }
  else if (!strcmp(tok[0], "PING")) {
    udp.beginPacket(pcIP, pcPort); udp.print("PONG\n"); udp.endPacket();
  }
}

void sendStatus() {
  if (!havePC) return;
  char out[64];
  snprintf(out, sizeof(out), "ST,%ld,%d,%.2f,%lu\n", lastSeq, busy() ? 1 : 0, readVbat(), millis());
  udp.beginPacket(pcIP, pcPort); udp.print(out); udp.endPacket();
}

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(115200);
  motorSetup();
  setMotors(0, 0);
  servoSetup();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(200);
  udp.begin(UDP_PORT);
  Serial.printf("GEMBOT AP up: %s  IP=%s  UDP=%d\n", AP_SSID, WiFi.softAPIP().toString().c_str(), UDP_PORT);
}

void loop() {
  // 1) รับ UDP
  int len = udp.parsePacket();
  if (len > 0) {
    char buf[96];
    int r = udp.read(buf, sizeof(buf) - 1);
    buf[r > 0 ? r : 0] = 0;
    pcIP = udp.remoteIP(); pcPort = udp.remotePort(); havePC = true;
    handlePacket(buf);
  }

  // 2) servo sequence
  seqUpdate();

  // 3) watchdog
  if ((curL != 0 || curR != 0) && millis() - lastCmdMs > WATCHDOG_MS) {
    setMotors(0, 0);
    Serial.println("WATCHDOG: link lost -> motors stop");
  }

  // 4) status
  if (millis() - lastStatusMs >= STATUS_MS) { lastStatusMs = millis(); sendStatus(); }
}
