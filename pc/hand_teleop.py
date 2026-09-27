"""hand_teleop.py — ขับหุ่นด้วยท่ามือ (MediaPipe) + คีย์บอร์ดสำรอง
ต่อยอดจาก teleop.py v3.2 (ใช้ RobotLink ตัวเดิม) + hand_tracking.py

ติดตั้ง:
  pip install opencv-python mediapipe
  โมเดล: gesture_recognizer.task วางไว้โฟลเดอร์เดียวกัน (ดาวน์โหลดจาก
  https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task)

รัน:  python hand_teleop.py [--cam 0]      (กล้องเว็บแคมโน้ตบุ๊ก = 0, กล้องสนาม = 1)

ท่ามือ (ต้องค้างท่า HOLD_S วินาที ถึงจะสั่ง):
  Thumb_Up     เดินหน้า          Thumb_Down   ถอย
  Pointing_Up  หมุนซ้าย          Victory      หมุนขวา
  Closed_Fist  PICK (ครั้งเดียวต่อการกำ, cooldown)
  Open_Palm    DUMP (ครั้งเดียวต่อการแบ, cooldown)
  None / ไม่เห็นมือ / ท่าอื่น  = หยุดทันที

คีย์ (หน้าต่างกล้องต้องเป็น focus):
  H        สลับโหมด HAND <-> KEY
  SPACE    STOP ฉุกเฉิน (โหมดมือ: ล็อกไว้จนกว่าจะเอามือออก/ทำท่า None)
  +/-      เพิ่ม/ลดความเร็ว
  Q        ออก
  เฉพาะโหมด KEY (เหมือน teleop.py):
  W/S/A/D  ขับ    P pick   O dump
  1/2 grip -5/+5   5/6 assist -5/+5   R รีเซ็ต servo
"""
import argparse
import os
import time
import cv2
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision
from link import RobotLink
import ui                     # v3.9: ตัวช่วยวาดจอ (ไม่มีค่าปรับจูน)

# ---------------- config ----------------
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gesture_recognizer.task")
CAM_W, CAM_H = 640, 480           # ต่ำกว่าเดิม (1280x720) ให้ loop เร็ว ~20-30 Hz

SPEED = 40
MIN_SCORE = 0.6                   # ความมั่นใจขั้นต่ำของท่า
HOLD_S = 0.3                      # ต้องค้างท่านานเท่านี้ก่อนสั่ง
ACTION_COOLDOWN_S = 2.0           # กัน PICK/DUMP ซ้ำ
KEY_HOLD_S = 0.15                 # โหมดคีย์: ไม่ได้คีย์เกินนี้ = ปล่อย
GRIP_OPEN_DEFAULT = 75            # ให้ตรงกับ GRIP_OPEN ใน config.h

GESTURE_MAP = {
    "Thumb_Up": "FWD",
    "Thumb_Down": "BACK",
    "Pointing_Up": "LEFT",
    "Victory": "RIGHT",
    "Closed_Fist": "PICK",
    "Open_Palm": "DUMP",
}   # นอกนั้น (None, ILoveYou, ...) = STOP

HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
]
FINGER_TIP_IDS = (4, 8, 12, 16, 20)


def draw_landmarks(img, hand_landmarks, w, h):
    pts = [(int(lm.x * w), int(lm.y * h)) for lm in hand_landmarks]
    for a, b in HAND_CONNECTIONS:
        cv2.line(img, pts[a], pts[b], (255, 255, 255), 2)
    for p in pts:
        cv2.circle(img, p, 3, (0, 0, 255), -1)
    for i in FINGER_TIP_IDS:
        cv2.circle(img, pts[i], 6, (0, 255, 0), -1)


def draw_overlay(img, lines):
    cv2.rectangle(img, (0, 0), (img.shape[1], 28 * len(lines) + 10), (0, 0, 0), -1)
    for i, t in enumerate(lines):
        cv2.putText(img, t, (10, 25 + i * 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 1, cv2.LINE_AA)


def cmd_to_wheels(cmd, speed):
    return {
        "FWD": (speed, speed),
        "BACK": (-speed, -speed),
        "LEFT": (-speed, speed),
        "RIGHT": (speed, -speed),
    }.get(cmd, (0, 0))


def main():
    global SPEED
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", type=int, default=0, help="index กล้องที่ส่องมือ (เว็บแคม = 0)")
    args = ap.parse_args()

    if not os.path.exists(MODEL_PATH):
        raise SystemExit(f"[ERROR] ไม่พบ {MODEL_PATH}\n"
                         "ดาวน์โหลดจาก https://storage.googleapis.com/mediapipe-models/gesture_recognizer/"
                         "gesture_recognizer/float16/1/gesture_recognizer.task แล้ววางในโฟลเดอร์ pc\\")

    camera = cv2.VideoCapture(args.cam, cv2.CAP_DSHOW) if os.name == "nt" else cv2.VideoCapture(args.cam)
    if not camera.isOpened():
        raise RuntimeError(f"เปิดกล้อง {args.cam} ไม่ได้")
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)

    options = vision.GestureRecognizerOptions(
        base_options=mp_python.BaseOptions(model_asset_path=MODEL_PATH),
        running_mode=vision.RunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    link = RobotLink()
    print(__doc__)

    mode = "HAND"
    vl = vr = 0
    grip = GRIP_OPEN_DEFAULT
    assist = 90

    # hand state
    cand, cand_t = "STOP", 0.0     # ท่าที่กำลังนับเวลาค้าง
    active = "STOP"                # ท่าที่ยืนยันแล้ว
    last_action_t = -ACTION_COOLDOWN_S
    estop = False
    raw_label, raw_score = "-", 0.0
    last_event = ""

    # key state
    last_key_t = 0.0
    servo_t = 0.0
    servo_resend_t = 0.0

    start = time.time()
    last_ts = -1

    try:
        with vision.GestureRecognizer.create_from_options(options) as recognizer:
            while True:
                ok, img = camera.read()
                if not ok:
                    print("อ่านกล้องไม่ได้ — หยุดหุ่นแล้วออก")
                    break
                img = cv2.flip(img, 1)
                h, w, _ = img.shape
                now = time.time()

                # ---------- gesture ----------
                cmd = "STOP"
                raw_label, raw_score = "-", 0.0
                if mode == "HAND":
                    ts = int((now - start) * 1000)
                    if ts <= last_ts:
                        ts = last_ts + 1          # timestamp ต้องเพิ่มขึ้นเสมอ
                    last_ts = ts
                    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    result = recognizer.recognize_for_video(
                        mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), ts)

                    if result.hand_landmarks:
                        draw_landmarks(img, result.hand_landmarks[0], w, h)
                        if result.gestures and result.gestures[0]:
                            top = result.gestures[0][0]
                            raw_label, raw_score = top.category_name, top.score
                            if top.score >= MIN_SCORE:
                                cmd = GESTURE_MAP.get(top.category_name, "STOP")

                    # debounce
                    if cmd != cand:
                        cand, cand_t = cmd, now
                    if cmd == "STOP":
                        active = "STOP"                    # หยุดทันที ไม่ต้องรอ
                        estop = False                      # ปลดล็อก e-stop เมื่อเอามือออก/ท่า None
                    elif now - cand_t >= HOLD_S and cand != active:
                        active = cand                      # edge: เพิ่งเข้าท่านี้
                        if active in ("PICK", "DUMP") and not estop:
                            if now - last_action_t >= ACTION_COOLDOWN_S:
                                (link.pick if active == "PICK" else link.dump)()
                                last_action_t = now
                                last_event = f"{active} sent"
                            else:
                                last_event = f"{active} cooldown"

                    if estop:
                        vl = vr = 0
                    else:
                        vl, vr = cmd_to_wheels(active, SPEED)

                # ---------- overlay ----------
                hold_pct = min(1.0, (now - cand_t) / HOLD_S) if cand != "STOP" else 0.0
                rows = [
                    ("โหมด", "บังคับด้วยมือ" if mode == "HAND" else "บังคับด้วยคีย์บอร์ด", "ok"),
                    ("WiFi ถึงหุ่น", "ต่ออยู่" if link.alive else "ขาด!", "ok" if link.alive else "bad"),
                    ("หุ่นกำลังหนีบ/ปล่อย", "ใช่" if link.busy else "ว่าง", "warn" if link.busy else "dim"),
                    ("ความเร็ว", f"{SPEED} / 100"),
                    ("ล้อ ซ้าย / ขวา", f"{vl}  /  {vr}", "ok" if (vl or vr) else "dim"),
                ]
                if mode == "HAND":
                    rows += [
                        None,
                        ("ท่าที่กล้องเห็น", f"{raw_label}  ({raw_score:.2f})", "dim"),
                        ("แปลเป็นคำสั่ง", ui.GESTURE_TH.get(cmd, cmd)),
                        ("ค้างท่าแล้ว", f"{int(hold_pct * 100)} %  (ครบ 100 = สั่งงาน)",
                         "ok" if hold_pct >= 1 else "warn"),
                        ("กำลังสั่ง", ui.GESTURE_TH.get(active, active), "ok" if active != "STOP" else "dim"),
                    ]
                    if last_event:
                        rows.append(("ล่าสุด", last_event, "warn"))
                else:
                    rows += [None, ("มุมปากหนีบ", f"{grip}°"), ("มุม servo ตัวช่วย", f"{assist}°", "dim")]
                if estop:
                    rows.append(("หยุดฉุกเฉิน", "ค้างอยู่ — เอามือออกจากกล้องเพื่อปลด", "bad"))
                ui.panel(img, 10, 10, rows, title="HAND TELEOP — บังคับหุ่นด้วยท่ามือ")
                if mode == "HAND":
                    ui.keybar(img, [("นิ้วโป้งขึ้น", "เดินหน้า"), ("นิ้วโป้งลง", "ถอย"),
                                    ("ชี้ขึ้น", "หมุนซ้าย"), ("ชู 2 นิ้ว", "หมุนขวา"),
                                    ("กำมือ", "หนีบ"), ("แบมือ", "ปล่อย")],
                              y=img.shape[0] - 2 * (16 + 20))
                ui.keybar(img, [("H", "สลับโหมดมือ/คีย์บอร์ด"), ("SPACE", "หยุดฉุกเฉิน"),
                                ("+/-", "ความเร็ว"), ("Q", "ออก")])
                cv2.imshow("GEMBOT hand teleop", img)

                # ---------- keyboard ----------
                k = cv2.waitKey(1) & 0xFF
                if k == ord('q'):
                    break
                elif k == ord(' '):
                    link.stop()
                    vl = vr = 0
                    active = cand = "STOP"
                    if mode == "HAND":
                        estop = True
                    last_event = "E-STOP"
                elif k == ord('h'):
                    mode = "KEY" if mode == "HAND" else "HAND"
                    vl = vr = 0
                    active = cand = "STOP"
                    estop = False
                    last_event = ""
                    link.stop()
                elif k in (ord('+'), ord('=')):
                    SPEED = min(100, SPEED + 5)
                elif k == ord('-'):
                    SPEED = max(10, SPEED - 5)

                if mode == "KEY":
                    nvl = nvr = 0
                    if k == ord('w'):   nvl, nvr = SPEED, SPEED
                    elif k == ord('s'): nvl, nvr = -SPEED, -SPEED
                    elif k == ord('a'): nvl, nvr = -SPEED, SPEED
                    elif k == ord('d'): nvl, nvr = SPEED, -SPEED
                    elif k == ord('p'): link.pick()
                    elif k == ord('o'): link.dump()
                    elif k == ord('1'): grip -= 5; link.servo(grip); servo_t = now
                    elif k == ord('2'): grip += 5; link.servo(grip); servo_t = now
                    elif k == ord('5'): assist -= 5; link.assist(assist)
                    elif k == ord('6'): assist += 5; link.assist(assist)
                    elif k == ord('r'): grip = GRIP_OPEN_DEFAULT; link.servo(grip); servo_t = now

                    if k in (ord('w'), ord('s'), ord('a'), ord('d')):
                        last_key_t = now
                        vl, vr = nvl, nvr
                    elif now - last_key_t >= KEY_HOLD_S:
                        vl = vr = 0

                    # ส่ง servo ซ้ำ 2 วิหลังกด กัน UDP หาย (เหมือน teleop.py)
                    if servo_t and now - servo_t < 2.0 and now - servo_resend_t > 0.25:
                        link.servo(grip)
                        servo_resend_t = now

                link.drive(vl, vr)   # ส่งทุก loop เลี้ยง watchdog
                link.tick()          # ส่ง PICK/DUMP ซ้ำถ้า packet หาย
    finally:
        try:
            link.stop()
        finally:
            link.close()
            camera.release()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
