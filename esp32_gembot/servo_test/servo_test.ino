// servo_test.ino — ทดสอบ servo ตัวเดียว ไม่มีอย่างอื่นเลย (esp32 core 3.x)
// กวาด 0 -> 180 -> 0 ทุก 2 วินาที  ถ้าไม่ขยับ = ขา/ไฟ ไม่ใช่โค้ด
#define SERVO_PIN 19          // TODO ลอง 19 แล้ว 32 แล้วขาที่พิมพ์ข้างช่องเสียบ

void servoWrite(int pin, int deg) {
  uint32_t us = 500 + (uint32_t)deg * (2400 - 500) / 180;
  ledcWrite(pin, (uint32_t)((uint64_t)us * 65535 / 20000));
}
void setup() {
  Serial.begin(115200);
  ledcAttach(SERVO_PIN, 50, 16);
}
void loop() {
  Serial.println("0");   servoWrite(SERVO_PIN, 0);   delay(2000);
  Serial.println("90");  servoWrite(SERVO_PIN, 90);  delay(2000);
  Serial.println("180"); servoWrite(SERVO_PIN, 180); delay(2000);
}
