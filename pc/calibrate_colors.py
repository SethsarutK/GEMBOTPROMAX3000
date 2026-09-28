"""
Module 1: HSV Color Calibration Tool
=====================================
เครื่องมือปรับค่า HSV ทีละสี เพื่อให้กล้อง "เห็นสีเดียว" ตามที่ต้องการ

วิธีใช้:
    python calibrate_colors.py --source 0          # กล้อง USB (index 0)
    python calibrate_colors.py --source field.png  # ทดสอบจากรูปนิ่ง

ปุ่มควบคุม:
    1-6   เลือกสีที่จะปรับ (ตามลำดับใน COLOR_CLASSES)
    s     บันทึกค่าทั้งหมดลง color_profiles.json
    l     โหลดค่าเดิมกลับมา
    m     สลับมุมมอง: mask ขาวดำ / ภาพที่ถูก mask แล้ว
    q     ออก

หลักการ:
    - HSV แยก "สี (Hue)" ออกจาก "ความสว่าง (Value)" ได้ดีกว่า RGB
      เวลาแสงเปลี่ยน Hue จะเปลี่ยนน้อยกว่า -> ทนแสงได้ดีกว่า
    - Hue ใน OpenCV มีค่า 0-179 (ไม่ใช่ 0-360)
    - สีแดงอยู่คร่อมรอยต่อ 179->0 จึงรองรับกรณี h_min > h_max
      (หมายถึง "เอาช่วงที่วนรอบ" เช่น 170-179 บวกกับ 0-10)
"""

import cv2
import numpy as np
import json
import argparse
import os

# ชื่อ class ต้องตรงกันทั้งระบบ (ฝั่งโซน / ฝั่ง gem / ฝั่ง HuskyLens)
COLOR_CLASSES = [
    "IRIDESCENT_VIOLET",
    "NEON_CYAN",
    "DEEP_CRIMSON",
    "MARIGOLD_ACCENT",
    "DEEP_SKY_BLUE",
    "LIME_GREEN",
]

PROFILE_PATH = "color_profiles.json"

# ค่าเริ่มต้นแบบหยาบ ๆ ไว้ให้เริ่มปรับ ไม่ใช่ค่าที่ใช้จริงได้เลย
DEFAULT_PROFILES = {
    "IRIDESCENT_VIOLET": {"h_min": 125, "h_max": 155, "s_min": 60, "s_max": 255, "v_min": 40,  "v_max": 255},
    "NEON_CYAN":         {"h_min": 85,  "h_max": 100, "s_min": 80, "s_max": 255, "v_min": 80,  "v_max": 255},
    "DEEP_CRIMSON":      {"h_min": 170, "h_max": 8,   "s_min": 90, "s_max": 255, "v_min": 60,  "v_max": 255},
    "MARIGOLD_ACCENT":   {"h_min": 10,  "h_max": 25,  "s_min": 90, "s_max": 255, "v_min": 80,  "v_max": 255},
    "DEEP_SKY_BLUE":     {"h_min": 100, "h_max": 115, "s_min": 70, "s_max": 255, "v_min": 80,  "v_max": 255},
    "LIME_GREEN":        {"h_min": 35,  "h_max": 75,  "s_min": 70, "s_max": 255, "v_min": 70,  "v_max": 255},
}

WINDOW_CTRL = "HSV Controls"
WINDOW_VIEW = "Calibration"


def build_mask(hsv_img, prof):
    """
    สร้าง mask ขาวดำจากค่า HSV profile
    รองรับกรณี hue วนรอบ (h_min > h_max) สำหรับสีแดง
    """
    h_min, h_max = prof["h_min"], prof["h_max"]
    s_lo, s_hi = prof["s_min"], prof["s_max"]
    v_lo, v_hi = prof["v_min"], prof["v_max"]

    if h_min <= h_max:
        lower = np.array([h_min, s_lo, v_lo], dtype=np.uint8)
        upper = np.array([h_max, s_hi, v_hi], dtype=np.uint8)
        mask = cv2.inRange(hsv_img, lower, upper)
    else:
        # กรณีวนรอบ: รวมสองช่วงเข้าด้วยกัน
        lower1 = np.array([h_min, s_lo, v_lo], dtype=np.uint8)
        upper1 = np.array([179,   s_hi, v_hi], dtype=np.uint8)
        lower2 = np.array([0,     s_lo, v_lo], dtype=np.uint8)
        upper2 = np.array([h_max, s_hi, v_hi], dtype=np.uint8)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv_img, lower1, upper1),
            cv2.inRange(hsv_img, lower2, upper2),
        )
    return mask


def clean_mask(mask, kernel_size=5):
    """
    ลบจุดรบกวนเล็ก ๆ (noise) และอุดรูในก้อนสี
    OPEN  = ลบจุดขาวเล็ก ๆ ที่ไม่ใช่วัตถุจริง
    CLOSE = อุดรูดำเล็ก ๆ ในก้อนสี ให้เป็นก้อนเดียวต่อเนื่อง
    """
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask


def load_profiles(path=PROFILE_PATH):
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        # เติมสีที่ยังไม่มีด้วยค่า default
        for name in COLOR_CLASSES:
            if name not in data:
                data[name] = dict(DEFAULT_PROFILES[name])
        print(f"[OK] loaded profiles from {path}")
        return data
    print("[INFO] no saved profile found, using defaults")
    return {k: dict(v) for k, v in DEFAULT_PROFILES.items()}


def save_profiles(profiles, path=PROFILE_PATH):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(profiles, f, indent=2, ensure_ascii=False)
    print(f"[OK] saved profiles -> {path}")


def _noop(_):
    pass


def create_trackbars(prof):
    """
    สร้าง trackbar ติดกับ "หน้าต่างเดียวกับวิดีโอ" (WINDOW_VIEW) โดยตรง
    ไม่แยกเป็นหน้าต่าง HSV Controls ต่างหากเหมือนก่อนหน้านี้

    เหตุผล: ถ้าแยกหน้าต่าง พอมือไปลาก trackbar โฟกัสคีย์บอร์ดจะค้างอยู่
    หน้าต่างนั้น กดปุ่ม 1-6/s/q ที่ตั้งใจส่งให้หน้าต่างวิดีโอจะไม่ทำงาน
    รวมเป็นหน้าต่างเดียวแก้ปัญหานี้ที่ต้นเหตุ ไม่ต้องคอยสลับโฟกัสเอง
    """
    cv2.createTrackbar("H min", WINDOW_VIEW, prof["h_min"], 179, _noop)
    cv2.createTrackbar("H max", WINDOW_VIEW, prof["h_max"], 179, _noop)
    cv2.createTrackbar("S min", WINDOW_VIEW, prof["s_min"], 255, _noop)
    cv2.createTrackbar("S max", WINDOW_VIEW, prof["s_max"], 255, _noop)
    cv2.createTrackbar("V min", WINDOW_VIEW, prof["v_min"], 255, _noop)
    cv2.createTrackbar("V max", WINDOW_VIEW, prof["v_max"], 255, _noop)


def sync_trackbars_to_profile(prof):
    """เขียนค่าจาก profile ลง trackbar (ใช้ตอนสลับสี)"""
    cv2.setTrackbarPos("H min", WINDOW_VIEW, prof["h_min"])
    cv2.setTrackbarPos("H max", WINDOW_VIEW, prof["h_max"])
    cv2.setTrackbarPos("S min", WINDOW_VIEW, prof["s_min"])
    cv2.setTrackbarPos("S max", WINDOW_VIEW, prof["s_max"])
    cv2.setTrackbarPos("V min", WINDOW_VIEW, prof["v_min"])
    cv2.setTrackbarPos("V max", WINDOW_VIEW, prof["v_max"])


def read_trackbars():
    return {
        "h_min": cv2.getTrackbarPos("H min", WINDOW_VIEW),
        "h_max": cv2.getTrackbarPos("H max", WINDOW_VIEW),
        "s_min": cv2.getTrackbarPos("S min", WINDOW_VIEW),
        "s_max": cv2.getTrackbarPos("S max", WINDOW_VIEW),
        "v_min": cv2.getTrackbarPos("V min", WINDOW_VIEW),
        "v_max": cv2.getTrackbarPos("V max", WINDOW_VIEW),
    }


def analyze_blobs(mask, min_area=80):
    """
    นับจำนวนก้อนที่เจอ และรายงานขนาด
    ใช้ดูว่าค่าที่ปรับอยู่ตอนนี้จับได้กี่ชิ้น ขนาดเท่าไหร่
    -> เอาไปตั้ง threshold ของ gem vs zone ในโมดูลถัดไป
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    areas = [cv2.contourArea(c) for c in contours]
    areas = [a for a in areas if a >= min_area]
    areas.sort(reverse=True)
    return contours, areas


def draw_hud(canvas, color_name, idx, prof, areas, view_mode):
    lines = [
        f"[{idx+1}/{len(COLOR_CLASSES)}]  {color_name}",
        f"H {prof['h_min']}-{prof['h_max']}   S {prof['s_min']}-{prof['s_max']}   V {prof['v_min']}-{prof['v_max']}",
        f"blobs found: {len(areas)}",
    ]
    if areas:
        top = "  ".join(f"{int(a)}" for a in areas[:6])
        lines.append(f"top areas: {top}")
    lines.append(f"view: {view_mode}   [1-6]=color [s]=save [l]=load [m]=view [q]=quit")

    y = 24
    for i, text in enumerate(lines):
        scale = 0.62 if i == 0 else 0.5
        thick = 2 if i == 0 else 1
        cv2.putText(canvas, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
        cv2.putText(canvas, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, (255, 255, 255), thick, cv2.LINE_AA)
        y += 24 if i == 0 else 20
    return canvas


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0",
                    help="camera index (เช่น 0) หรือ path ของรูป/วิดีโอ")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--min-area", type=int, default=80,
                    help="พื้นที่ขั้นต่ำ (pixel) ที่จะนับว่าเป็นก้อน ไม่ใช่ noise")
    args = ap.parse_args()

    # --- เตรียมแหล่งภาพ ---
    still_image = None
    cap = None
    if args.source.isdigit():
        # v4.4: ต้องเปิดแบบเดียวกับ auto_main (MSMF + MJPG) ไม่งั้นสีที่ calibrate ต่างจากตอนแข่ง
        #       (DirectShow ให้ YUY2 4 fps และโทนสีต่างจาก MJPG เล็กน้อย)
        cap = None
        if os.name == "nt":
            cap = cv2.VideoCapture(int(args.source), cv2.CAP_MSMF)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
            cap.set(cv2.CAP_PROP_FPS, 30)
            if not (cap.isOpened() and cap.read()[0]):
                cap.release(); cap = None
        if cap is None:
            cap = cv2.VideoCapture(int(args.source), cv2.CAP_DSHOW) if os.name == "nt" \
                else cv2.VideoCapture(int(args.source))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        if not cap.isOpened():
            print(f"[ERROR] เปิดกล้อง index {args.source} ไม่ได้")
            return
        # ล็อก exposure/WB ให้เหมือนตอนแข่งจริง ไม่งั้นค่าที่ calibrate จะใช้ไม่ได้
        for v in (0.25, 1):
            try:
                cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, v)
            except Exception:
                pass
        try:
            cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        except Exception:
            pass
    else:
        if not os.path.exists(args.source):
            print(f"[ERROR] ไม่พบไฟล์: {args.source}")
            return
        ext = os.path.splitext(args.source)[1].lower()
        if ext in (".png", ".jpg", ".jpeg", ".bmp"):
            still_image = cv2.imread(args.source)
            if still_image is None:
                print("[ERROR] อ่านรูปไม่ได้")
                return
        else:
            cap = cv2.VideoCapture(args.source)

    profiles = load_profiles()
    idx = 0
    view_mode = "masked"          # "masked" หรือ "mask"

    cv2.namedWindow(WINDOW_VIEW, cv2.WINDOW_NORMAL)
    create_trackbars(profiles[COLOR_CLASSES[idx]])

    print("\n=== HSV Calibration ===")
    print("ปรับทีละสี ให้เหลือเฉพาะก้อนสีนั้นสีเดียวบนจอ (สีอื่นต้องดำหมด)")
    print("กด s เพื่อบันทึก, q เพื่อออก\n")

    while True:
        if still_image is not None:
            frame = still_image.copy()
        else:
            ok, frame = cap.read()
            if not ok:
                # ถ้าเป็นไฟล์วิดีโอที่จบแล้ว ให้วนกลับต้นไฟล์
                if args.source.isdigit():
                    print("[WARN] อ่านเฟรมจากกล้องไม่ได้")
                    break
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue

        color_name = COLOR_CLASSES[idx]
        # อ่านค่าปัจจุบันจาก trackbar แล้วเก็บกลับเข้า profile ทันที
        profiles[color_name] = read_trackbars()
        prof = profiles[color_name]

        # ลด noise จากพื้นผิวก่อนแปลงสี ช่วยให้ mask นิ่งขึ้นมาก
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = build_mask(hsv, prof)
        mask = clean_mask(mask)
        contours, areas = analyze_blobs(mask, args.min_area)

        if view_mode == "mask":
            canvas = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        else:
            canvas = cv2.bitwise_and(frame, frame, mask=mask)

        # วาดกรอบรอบก้อนที่ผ่าน min_area
        for c in contours:
            if cv2.contourArea(c) < args.min_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(canvas, (x, y), (x + w, y + h), (0, 255, 0), 2)

        canvas = draw_hud(canvas, color_name, idx, prof, areas, view_mode)
        cv2.imshow(WINDOW_VIEW, canvas)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            save_profiles(profiles)
        elif key == ord("l"):
            profiles = load_profiles()
            sync_trackbars_to_profile(profiles[COLOR_CLASSES[idx]])
        elif key == ord("m"):
            view_mode = "mask" if view_mode == "masked" else "masked"
        elif ord("1") <= key <= ord("6"):
            new_idx = key - ord("1")
            if new_idx < len(COLOR_CLASSES):
                idx = new_idx
                sync_trackbars_to_profile(profiles[COLOR_CLASSES[idx]])
                print(f"[SWITCH] -> {COLOR_CLASSES[idx]}")

    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()
    print("\nจบการ calibrate — อย่าลืมกด s บันทึกก่อนออกนะครับ")


if __name__ == "__main__":
    main()