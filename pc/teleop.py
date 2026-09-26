"""teleop.py — ขับหุ่นด้วยคีย์บอร์ด + จูน servo  (pip install opencv-python)
v3.2: ปากหนีบ servo ตัวเดียว (เฟืองคุมสองข้าง) คีบแล้วลาก ไม่ยก

รัน:  python teleop.py
คีย์:
  W/S      เดินหน้า/ถอย        A/D  หมุนซ้าย/ขวา     ปล่อยคีย์ = หยุด
  +/-      เพิ่ม/ลดความเร็ว
  P        PICK sequence (หนีบค้างไว้)    O    DUMP sequence (เปิดปล่อย)
  1/2      grip  -5/+5   (servo หลัก GPIO 32)
  5/6      assist -5/+5  (servo ตัวช่วย GPIO 19 — จูนให้ปากอยู่ตำแหน่งเดียวกับตัวหลัก)
  R        รีเซ็ต servo เป็นค่าเริ่ม
  SPACE    STOP ฉุกเฉิน         Q    ออก
(กดค้างต้องให้หน้าต่างสีดำเป็น focus)
"""
import time
import cv2
import numpy as np
from link import RobotLink

SPEED = 40
grip = 90                          # ค่าเริ่ม ให้ตรงกับ GRIP_OPEN ใน config.h
assist = 90

link = RobotLink()
cv2.namedWindow("GEMBOT teleop")

last_key_t = 0.0
vl = vr = 0
servo_t = 0.0          # เวลาที่กดปุ่ม servo ล่าสุด (ไว้ส่งซ้ำกัน packet หาย)
servo_resend_t = 0.0
print(__doc__)

while True:
    img = np.zeros((220, 460, 3), np.uint8)
    lines = [
        f"link: {'OK' if link.alive else 'LOST'}   busy: {link.busy}   vbat: {link.vbat:.2f}",
        f"speed: {SPEED}   L={vl} R={vr}",
        f"servo  grip={grip}   assist={assist}",
        "W/S/A/D drive  P pick  O dump  1/2 grip 5/6 assist  SPACE stop  Q quit",
    ]
    for i, t in enumerate(lines):
        cv2.putText(img, t, (10, 35 + i * 40), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    cv2.imshow("GEMBOT teleop", img)

    k = cv2.waitKey(50) & 0xFF
    now = time.time()
    nvl = nvr = 0

    if k == ord('w'):   nvl, nvr = SPEED, SPEED
    elif k == ord('s'): nvl, nvr = -SPEED, -SPEED
    elif k == ord('a'): nvl, nvr = -SPEED, SPEED
    elif k == ord('d'): nvl, nvr = SPEED, -SPEED
    elif k in (ord('+'), ord('=')): SPEED = min(100, SPEED + 5)
    elif k == ord('-'): SPEED = max(10, SPEED - 5)
    elif k == ord('p'): link.pick()
    elif k == ord('o'): link.dump()
    elif k == ord('1'): grip -= 5;  link.servo(grip); servo_t = now
    elif k == ord('2'): grip += 5;  link.servo(grip); servo_t = now
    elif k == ord('5'): assist -= 5; link.assist(assist)
    elif k == ord('6'): assist += 5; link.assist(assist)
    elif k == ord('r'): grip = 90; link.servo(grip); servo_t = now
    elif k == ord(' '): link.stop()
    elif k == ord('q'): break

    if k in (ord('w'), ord('s'), ord('a'), ord('d')):
        last_key_t = now

    # กดค้าง: OpenCV ส่งคีย์ซ้ำ ๆ ถ้าไม่ได้คีย์ใน 0.15 s ถือว่าปล่อย
    if now - last_key_t < 0.15 and k in (ord('w'), ord('s'), ord('a'), ord('d')):
        vl, vr = nvl, nvr
    elif now - last_key_t >= 0.15:
        vl = vr = 0
    link.drive(vl, vr)          # ส่งตลอด 20 Hz เพื่อเลี้ยง watchdog
    link.tick()                 # ส่ง PICK/DUMP ซ้ำถ้า packet หาย

    # ส่งค่า servo ซ้ำทุก 0.25 วิ เป็นเวลา 2 วิหลังกดปุ่ม กัน UDP packet หาย
    # (ไม่ส่งตลอดเวลา เพราะคำสั่ง S จะไปยกเลิก sequence PICK/DUMP)
    if servo_t and now - servo_t < 2.0 and now - servo_resend_t > 0.25:
        link.servo(grip); servo_resend_t = now

link.close()
cv2.destroyAllWindows()
