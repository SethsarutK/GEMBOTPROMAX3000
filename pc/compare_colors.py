"""
compare_colors.py
==================
เครื่องมือช่วยแยก NEON_CYAN vs DEEP_SKY_BLUE (หรือคู่สีอื่นที่ใกล้กัน)

หลักการ: วางเม็ดทั้งสองสีที่สงสัยว่าใกล้กันไว้ "ข้างกัน" ใต้กล้องสนาม
แสงเดียวกัน แล้วคลิกที่เม็ดแต่ละสีทีละจุด โปรแกรมจะพิมพ์ค่า H,S,V
เฉลี่ยตรงจุดนั้นออกมาเทียบกันให้เห็นชัด ๆ ว่าตัวแปรไหน (H, S, หรือ V)
ที่แยกสองสีนี้ออกจากกันได้จริง — ไม่ต้องเดา

วิธีใช้:
    python compare_colors.py --source 1
    คลิกที่เม็ดสีแรก (เช่น Neon Cyan) -> พิมพ์ค่าออกมา
    คลิกที่เม็ดสีสอง (เช่น Deep Sky Blue) -> พิมพ์ค่าออกมา
    สลับคลิกไปมาได้เรื่อย ๆ จนกว่าจะเห็น pattern
    กด c = เคลียร์ประวัติ, q = ออก
"""

import cv2
import numpy as np
import argparse
from collections import deque

SAMPLE_RADIUS = 6      # รัศมี (pixel) รอบจุดคลิก เอาไปเฉลี่ยค่า กันจุดเดียว fluke
HISTORY_LEN = 10        # เก็บผลคลิกล่าสุดกี่ครั้งไว้โชว์บนจอ

history = deque(maxlen=HISTORY_LEN)
current_frame_hsv = None


def on_mouse(event, x, y, flags, param):
    global current_frame_hsv
    if event != cv2.EVENT_LBUTTONDOWN or current_frame_hsv is None:
        return
    h, w = current_frame_hsv.shape[:2]
    x0, x1 = max(0, x - SAMPLE_RADIUS), min(w, x + SAMPLE_RADIUS)
    y0, y1 = max(0, y - SAMPLE_RADIUS), min(h, y + SAMPLE_RADIUS)
    patch = current_frame_hsv[y0:y1, x0:x1].reshape(-1, 3)
    if len(patch) == 0:
        return
    med = np.median(patch, axis=0)
    entry = (x, y, int(med[0]), int(med[1]), int(med[2]))
    history.append(entry)
    print(f"click @ ({x:4d},{y:4d})  ->  H={entry[2]:3d}  S={entry[3]:3d}  V={entry[4]:3d}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="1")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    args = ap.parse_args()

    global current_frame_hsv

    cap = cv2.VideoCapture(int(args.source), cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        print(f"[ERROR] เปิดกล้อง index {args.source} ไม่ได้")
        return

    win = "Compare Colors (click 2+ points, q=quit, c=clear)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, on_mouse)

    print("คลิกที่เม็ดสีที่สงสัยว่าใกล้กันทีละจุด ดูค่า H/S/V ที่ terminal\n")

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        current_frame_hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        disp = frame.copy()
        for i, (x, y, h_, s_, v_) in enumerate(history):
            col = (0, 255, 0) if i == len(history) - 1 else (0, 180, 255)
            cv2.circle(disp, (x, y), SAMPLE_RADIUS, col, 2)
            cv2.putText(disp, f"{i+1}", (x + 8, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)

        y0 = 24
        cv2.putText(disp, "click 2 gems of the same-looking color to compare",
                    (12, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
        cv2.putText(disp, "click 2 gems of the same-looking color to compare",
                    (12, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        y0 += 26
        for i, (x, y, h_, s_, v_) in enumerate(history):
            line = f"{i+1}: H={h_:3d} S={s_:3d} V={v_:3d}"
            cv2.putText(disp, line, (12, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 0), 3)
            cv2.putText(disp, line, (12, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1)
            y0 += 20

        cv2.imshow(win, disp)
        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break
        if k == ord("c"):
            history.clear()
            print("[cleared]")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
