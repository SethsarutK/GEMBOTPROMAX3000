"""
Module 2: Field Vision
======================
รับภาพจากกล้องสนาม แล้วตอบ 3 คำถามหลัก:
    1. โซนสีแต่ละสีอยู่ตรงไหน  (detect_zones)
    2. ตอนนี้มี gem อะไร อยู่ตรงไหนบ้าง  (detect_gems)
    3. หุ่นอยู่ตรงไหน หันหน้าทางไหน  (detect_robot)

ออกแบบตามที่คุยกันไว้:
    - ตำแหน่งโซนไม่ fix -> ต้อง detect เอาเอง ตอนเริ่มระบบ
    - detect โซนครั้งเดียวตอน calibrate แล้วจำไว้ (เพราะโซนอาจโดนบัง
      ระหว่างแข่ง เช่น ชาม/ตัวหุ่นบังอยู่ ถ้า detect ใหม่ทุกเฟรมจะหาย)
    - gem ตรวจทุกเฟรม เพราะมันหายไปเรื่อย ๆ เมื่อถูกเก็บ

วิธีใช้:
    python field_vision.py --source 0 --setup     # โหมดตั้งค่าสนามครั้งแรก
    python field_vision.py --source 0             # โหมดดูผลตรวจจับ real-time
"""

import cv2
import numpy as np
import json
import os
import argparse

from calibrate_colors import (
    COLOR_CLASSES,
    build_mask,
    clean_mask,
    load_profiles,
)

FIELD_MAP_PATH = "field_map.json"

# --- ขนาดสนามจริงตามกติกา (cm) ---
FIELD_W_CM = 210.0
FIELD_H_CM = 120.0

# --- เกณฑ์แยก zone กับ gem (หน่วย: pixel area) ---
# ค่าเหล่านี้ขึ้นกับความละเอียดกล้องและความสูงกล้อง
# ให้ดูเลข "top areas" จาก calibrate_colors.py แล้วมาปรับตรงนี้
DEFAULT_THRESHOLDS = {
    "zone_min_area": 1500,     # โซนสีเป็นวงกลมใหญ่
    "zone_min_circularity": 0.65,
    "gem_min_area": 40,        # gem เป็นก้อนเล็ก
    "gem_max_area": 1200,
}

# สีที่ใช้วาด overlay (BGR) เอาไว้ดูด้วยตาว่าโปรแกรมเห็นตรงกับเราไหม
DRAW_BGR = {
    "IRIDESCENT_VIOLET": (150, 40, 130),
    "NEON_CYAN":         (200, 200, 40),
    "DEEP_CRIMSON":      (50, 50, 200),
    "MARIGOLD_ACCENT":   (40, 140, 230),
    "DEEP_SKY_BLUE":     (220, 160, 60),
    "LIME_GREEN":        (60, 200, 90),
}


# ==================================================================
#  ส่วนที่ 1: Homography (แปลง pixel -> cm)
# ==================================================================

class FieldCalibration:
    """
    แปลงพิกัดในภาพ (pixel) เป็นพิกัดจริงบนสนาม (cm)

    ทำไมต้องแปลง: กล้องมองจากมุมสูงแต่ไม่ได้ตั้งฉากเป๊ะ ภาพจะบิดเป็น
    สี่เหลี่ยมคางหมู ระยะ 10 pixel ตรงขอบภาพ != 10 pixel ตรงกลางภาพ
    Homography คือเมทริกซ์ที่แก้ความบิดนี้ให้กลายเป็นมุมมองจากบนตรง ๆ
    """

    def __init__(self, H=None):
        self.H = H

    @classmethod
    def from_corners(cls, pixel_corners):
        """
        pixel_corners: 4 จุดมุมสนามในภาพ เรียงตามเข็มนาฬิกา
                       เริ่มจากมุมซ้ายบน -> ขวาบน -> ขวาล่าง -> ซ้ายล่าง
        """
        src = np.array(pixel_corners, dtype=np.float32)
        dst = np.array([
            [0.0, 0.0],
            [FIELD_W_CM, 0.0],
            [FIELD_W_CM, FIELD_H_CM],
            [0.0, FIELD_H_CM],
        ], dtype=np.float32)
        H, _ = cv2.findHomography(src, dst)
        return cls(H)

    def to_field(self, pt):
        """pixel (x, y) -> field (x_cm, y_cm)"""
        if self.H is None:
            return None
        p = np.array([[[float(pt[0]), float(pt[1])]]], dtype=np.float32)
        out = cv2.perspectiveTransform(p, self.H)
        return (float(out[0][0][0]), float(out[0][0][1]))

    def to_pixel(self, pt_cm):
        """field (x_cm, y_cm) -> pixel (x, y)  (ใช้ตอนวาด overlay)"""
        if self.H is None:
            return None
        Hinv = np.linalg.inv(self.H)
        p = np.array([[[float(pt_cm[0]), float(pt_cm[1])]]], dtype=np.float32)
        out = cv2.perspectiveTransform(p, Hinv)
        return (int(out[0][0][0]), int(out[0][0][1]))


def pick_corners_interactive(frame):
    """ให้ผู้ใช้คลิก 4 มุมสนาม ตามลำดับ ซ้ายบน -> ขวาบน -> ขวาล่าง -> ซ้ายล่าง"""
    pts = []
    win = "Click 4 field corners (TL -> TR -> BR -> BL), press ENTER when done"

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(pts) < 4:
            pts.append((x, y))
            print(f"  corner {len(pts)}: ({x}, {y})")

    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, on_mouse)

    labels = ["TL", "TR", "BR", "BL"]
    while True:
        disp = frame.copy()
        for i, p in enumerate(pts):
            cv2.circle(disp, p, 7, (0, 255, 0), -1)
            cv2.putText(disp, labels[i], (p[0] + 10, p[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if len(pts) == 4:
            cv2.polylines(disp, [np.array(pts)], True, (0, 255, 255), 2)
        cv2.putText(disp, f"clicked {len(pts)}/4   [ENTER]=ok  [r]=reset  [q]=cancel",
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
        cv2.putText(disp, f"clicked {len(pts)}/4   [ENTER]=ok  [r]=reset  [q]=cancel",
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.imshow(win, disp)

        k = cv2.waitKey(20) & 0xFF
        if k == 13 and len(pts) == 4:
            break
        if k == ord("r"):
            pts = []
        if k == ord("q"):
            cv2.destroyWindow(win)
            return None
    cv2.destroyWindow(win)
    return pts


def pick_points_interactive(frame, n, title):
    """คลิก n จุดบนภาพ แล้ว ENTER (ใช้ตอนโซนหาไม่เจอ) — คืน list หรือ None"""
    pts = []

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(pts) < n:
            pts.append((x, y))

    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(title, on_mouse)
    while True:
        disp = frame.copy()
        for p in pts:
            cv2.circle(disp, p, 7, (0, 255, 0), -1)
        cv2.putText(disp, f"{title}  {len(pts)}/{n}  [ENTER]=ok [r]=reset [q]=skip",
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
        cv2.putText(disp, f"{title}  {len(pts)}/{n}  [ENTER]=ok [r]=reset [q]=skip",
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.imshow(title, disp)
        k = cv2.waitKey(20) & 0xFF
        if k == 13 and len(pts) == n:
            break
        if k == ord("r"):
            pts = []
        if k == ord("q"):
            cv2.destroyWindow(title)
            return None
    cv2.destroyWindow(title)
    return pts


# ==================================================================
#  ส่วนที่ 2: ตรวจจับโซนสี และ gem
# ==================================================================

def _circularity(contour):
    """
    ความกลม: 1.0 = วงกลมสมบูรณ์, ต่ำกว่านั้น = รูปร่างเบี้ยว
    ใช้แยก "โซนสีที่เป็นวงกลม" ออกจาก "กอง gem ที่เป็นก้อนเบี้ยว"
    """
    area = cv2.contourArea(contour)
    peri = cv2.arcLength(contour, True)
    if peri == 0:
        return 0.0
    return 4.0 * np.pi * area / (peri * peri)


def detect_zones(frame, profiles, thresholds=None):
    """
    หาโซนสีทั้ง 6 — สำหรับแต่ละ class หา blob ที่ใหญ่ที่สุดและกลมที่สุด

    return: dict { class_name: {"px": (x,y), "area": a, "radius_px": r} }
    """
    th = {**DEFAULT_THRESHOLDS, **(thresholds or {})}
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)

    zones = {}
    for name in COLOR_CLASSES:
        if name not in profiles:
            continue
        mask = clean_mask(build_mask(hsv, profiles[name]), kernel_size=7)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best, best_area = None, 0
        for c in contours:
            area = cv2.contourArea(c)
            if area < th["zone_min_area"]:
                continue
            if _circularity(c) < th["zone_min_circularity"]:
                continue
            if area > best_area:
                best, best_area = c, area

        if best is not None:
            M = cv2.moments(best)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            (_, _), radius = cv2.minEnclosingCircle(best)
            zones[name] = {"px": (cx, cy), "area": float(best_area),
                           "radius_px": float(radius)}
    return zones



# --------------------------------------------------------------------
#  v3.6: หาโซนด้วย "รูปร่าง" (วงกลมขนาดที่รู้จาก homography) แล้วจำแนกสีจาก hue กลางวง
#  ไม่พึ่ง color_profiles ของหิน (ผ้าโซนกับหินสีคนละเฉด) -> ทาง B ทำงานได้จริง
# --------------------------------------------------------------------
ZONE_REF_HUE = {           # hue อ้างอิง (OpenCV 0-179) ของผ้าโซนแต่ละสี  TODO ปรับถ้าจำแนกผิด
    "MARIGOLD_ACCENT": 14, "LIME_GREEN": 68, "NEON_CYAN": 95,
    "DEEP_SKY_BLUE": 99, "IRIDESCENT_VIOLET": 157, "DEEP_CRIMSON": 178,   # วัดจากภาพสนามจริง 25 ก.ย.
}


def _hue_dist(a, b):
    d = abs(a - b) % 180
    return min(d, 180 - d)


def detect_zones_auto(frame, calib, corners_px, zone_radius_cm=10.0, debug=False):
    """
    หาวงโซน 6 วงจากรูปร่าง: blob สีอิ่ม (S สูง) ในสนาม ที่กลมและมีรัศมี ~ zone_radius_cm
    แล้วจำแนกสีจาก hue มัธยฐานกลางวง (ใกล้ ZONE_REF_HUE ตัวไหน)
    return: (zones dict เหมือน detect_zones, รายการ candidate ทั้งหมดไว้ debug)
    """
    h, w = frame.shape[:2]
    # รัศมีวงเป็น px (คำนวณกลางสนาม)
    c0 = calib.to_pixel((105, 60)); c1 = calib.to_pixel((105 + zone_radius_cm, 60))
    r_px = float(np.hypot(c1[0] - c0[0], c1[1] - c0[1]))
    area_ref = np.pi * r_px * r_px

    # mask เฉพาะในสนาม
    field_mask = np.zeros((h, w), np.uint8)
    cv2.fillPoly(field_mask, [np.array(corners_px, np.int32)], 255)

    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (7, 7), 0), cv2.COLOR_BGR2HSV)
    S, V = hsv[:, :, 1], hsv[:, :, 2]
    # ผ้าโซนสีอิ่มกว่าพื้นไม้ชัดเจน; ตัดพื้นไม้ (S ต่ำ) และเงาดำ (V ต่ำ)
    sat = cv2.inRange(S, 90, 255) & cv2.inRange(V, 50, 255) & field_mask
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    sat = cv2.morphologyEx(sat, cv2.MORPH_CLOSE, k)      # อุดรูจากหินที่วางในวง
    sat = cv2.morphologyEx(sat, cv2.MORPH_OPEN, k)       # ลบหินเดี่ยว ๆ / เส้นบาง

    contours, _ = cv2.findContours(sat, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cands = []
    for c in contours:
        area = cv2.contourArea(c)
        if not (0.45 * area_ref <= area <= 1.8 * area_ref):
            continue
        (cx, cy), rad = cv2.minEnclosingCircle(c)
        fill = area / (np.pi * rad * rad)                 # วงกลมเต็ม ~1, กองหินเบี้ยว < 0.6
        if fill < 0.6 or _circularity(c) < 0.55:
            continue
        # hue มัธยฐานภายในวง (หดเข้า 60% กันขอบขาว/หินที่ขอบ)
        m = np.zeros((h, w), np.uint8)
        cv2.circle(m, (int(cx), int(cy)), int(rad * 0.6), 255, -1)
        pix = hsv[m == 255]
        if len(pix) == 0:
            continue
        hue = float(np.median(pix[:, 0])); sat_med = float(np.median(pix[:, 1]))
        cands.append({"px": (int(cx), int(cy)), "radius_px": float(rad), "area": float(area),
                      "hue": hue, "sat": sat_med})

    # จำแนก: จับคู่ candidate กับ class แบบ greedy ตามระยะ hue ที่สั้นสุด (แต่ละ class ได้วงเดียว)
    # คะแนน = ระยะ hue + โทษถ้าสีจาง (ตัวหุ่น/เงามี sat ต่ำกว่าผ้าโซน)
    pairs = sorted(((_hue_dist(cd["hue"], ref) + max(0.0, 130 - cd["sat"]) * 0.1, i, name)
                    for i, cd in enumerate(cands) for name, ref in ZONE_REF_HUE.items()),
                   key=lambda t: t[0])
    zones, used_c, used_n = {}, set(), set()
    for d, i, name in pairs:
        if i in used_c or name in used_n or d > 25:
            continue
        cd = cands[i]
        zones[name] = {"px": cd["px"], "radius_px": cd["radius_px"], "area": cd["area"]}
        used_c.add(i); used_n.add(name)
    # ฟ้า 2 เฉด hue ใกล้กันมาก -> ตัดสินด้วยความอิ่มสี: DEEP_SKY_BLUE เข้ม/อิ่มกว่า NEON_CYAN เสมอ
    if "NEON_CYAN" in zones and "DEEP_SKY_BLUE" in zones:
        sc = next(cd["sat"] for cd in cands if cd["px"] == zones["NEON_CYAN"]["px"])
        ss = next(cd["sat"] for cd in cands if cd["px"] == zones["DEEP_SKY_BLUE"]["px"])
        if sc > ss:
            zones["NEON_CYAN"], zones["DEEP_SKY_BLUE"] = zones["DEEP_SKY_BLUE"], zones["NEON_CYAN"]
    if debug:
        for cd in cands:
            print(f"   candidate px={cd['px']} r={cd['radius_px']:.0f} hue={cd['hue']:.0f} sat={cd['sat']:.0f}")
    return zones, cands


def split_touching_blobs(mask, roi_bbox, single_gem_area_px):
    """
    แยกเม็ดสีเดียวกันที่ติด/ซ้อนกันแน่น ออกเป็นเม็ดเดี่ยว ๆ

    ทำไมต้องมีฟังก์ชันนี้:
        contour ปกติเห็นกลุ่มเม็ดสีเดียวกันที่ชิดกันเป็น "ก้อนเดียวก้อนใหญ่"
        ถ้าปล่อยไว้เฉย ๆ หุ่นจะเข้าใจผิดว่ามี gem แค่ 1 เม็ด (พลาดเม็ดอื่นในกอง)
        และพิกัดที่คำนวณได้จะเป็น "จุดกึ่งกลางของกลุ่ม" ไม่ใช่จุดของเม็ดไหนเม็ดหนึ่ง
        จริง ๆ -> แขนจะเอื้อมไปหยิบตรงรอยต่อระหว่างเม็ด จับพลาดสูง

    หลักการ (watershed แบบย่อ):
        1. distanceTransform: ให้ pixel กลางเม็ดมีค่าสูง ขอบเม็ดมีค่าต่ำ
           (เหมือนสร้าง "เนินเขา" ตรงกลางแต่ละเม็ด)
        2. threshold เอาเฉพาะยอดเนิน = จุดที่มั่นใจว่าเป็นใจกลางเม็ดแต่ละเม็ด
        3. ใช้จุดยอดเนินเป็น marker ให้ watershed ไล่ "เติมน้ำ" แบ่งพื้นที่
           ตามยอดเนินที่ใกล้ที่สุด -> ได้ขอบเขตแยกแต่ละเม็ดออกจากกัน

    return: list ของ (cx, cy, area) เป็นพิกัดใน mask ที่ครอปมา (บวก offset เอง)
    """
    dist = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    if dist.max() <= 0:
        return []

    # 0.45 = ต้องสูงกว่า 45% ของยอดเนินสูงสุดในภาพนี้ถึงจะถือว่าเป็น "ใจกลางเม็ด"
    # ค่านี้ปรับได้: ต่ำไป -> แยกเม็ดถี่เกิน (noise กลายเป็นเม็ดปลอม)
    #              สูงไป -> เม็ดที่ติดกันแน่นมากจะไม่ถูกแยก
    _, sure_fg = cv2.threshold(dist, 0.45 * dist.max(), 255, 0)
    sure_fg = np.uint8(sure_fg)

    n_labels, markers = cv2.connectedComponents(sure_fg)
    if n_labels <= 1:
        return []
    markers = markers + 1
    unknown = cv2.subtract(mask, sure_fg)
    markers[unknown == 255] = 0

    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    cv2.watershed(mask_bgr, markers)

    ox, oy = roi_bbox[0], roi_bbox[1]
    results = []
    for label in range(2, n_labels + 1):
        ys, xs = np.where(markers == label)
        if len(xs) == 0:
            continue
        area = len(xs)
        # ก้อนเล็กเกินไปหลัง split มักเป็นเศษ noise ไม่ใช่เม็ดจริง
        if area < single_gem_area_px * 0.35:
            continue
        results.append((int(np.mean(xs)) + ox, int(np.mean(ys)) + oy, float(area)))
    return results


def _which_zone(cx, cy, exclude_zones):
    if not exclude_zones:
        return None
    for zname, z in exclude_zones.items():
        d = np.hypot(cx - z["px"][0], cy - z["px"][1])
        if d <= z["radius_px"]:
            return zname
    return None


def detect_gems(frame, profiles, thresholds=None, exclude_zones=None,
                 single_gem_area_px=None, robot_pose=None, robot_radius_px=None):
    """
    หา gem ทุกก้อนที่มองเห็น รวมถึงแยกเม็ดสีเดียวกันที่ติดกันแน่น

    robot_pose / robot_radius_px:
        ตัด blob ที่อยู่ในรัศมีรอบตัวหุ่นทิ้ง — ล้อสีเหลืองและชิ้น 3D สีฟ้า
        บนตัวหุ่นจะถูก HSV จับเป็น gem ทุกเฟรม ถ้าไม่ตัด หุ่นจะวนไล่จับตัวเอง
        ถ้าไม่ระบุ robot_radius_px จะใช้ 1.8 x ขนาด ArUco (TODO วัดจริง:
        ระยะจากกลาง tag ถึงจุดไกลสุดของหุ่น เป็น pixel)

    single_gem_area_px: พื้นที่ (pixel) โดยประมาณของเม็ดเดี่ยว ๆ 1 เม็ด
        ใช้ตัดสินใจว่า contour ที่เจอ "ใหญ่ผิดปกติ" (น่าจะเป็นหลายเม็ดติดกัน)
        หรือไม่ — วัดได้จากการดู "top areas" ตอนรัน calibrate_colors.py
        กับเม็ดที่วางแยกเดี่ยว ๆ ไม่ติดกัน ถ้าไม่ระบุ จะใช้
        thresholds["gem_max_area"] ตั้งต้นแทน (หยาบกว่า)

    exclude_zones: gem ที่อยู่ในรัศมีโซนแล้ว ถือว่า "วางเสร็จแล้ว"

    return: list ของ dict {"class", "px", "area", "in_zone", "correct"}
    """
    th = {**DEFAULT_THRESHOLDS, **(thresholds or {})}
    ref_area = single_gem_area_px or th["gem_max_area"]
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)

    # วงเว้นรอบตัวหุ่น
    rob_px, rob_r = None, 0.0
    if robot_pose is not None:
        rob_px = robot_pose["px"]
        rob_r = robot_radius_px or 1.8 * robot_pose.get("size_px", 60.0)

    def near_robot(x, y):
        return rob_px is not None and np.hypot(x - rob_px[0], y - rob_px[1]) <= rob_r

    gems = []
    for name in COLOR_CLASSES:
        if name not in profiles:
            continue
        mask = clean_mask(build_mask(hsv, profiles[name]), kernel_size=3)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            area = cv2.contourArea(c)
            if area < th["gem_min_area"]:
                continue
            bx, by, bw, bh = cv2.boundingRect(c)
            if near_robot(bx + bw / 2, by + bh / 2):
                continue        # เป็นชิ้นส่วนของหุ่นเอง

            x, y, w, h = cv2.boundingRect(c)
            pad = 3
            x0, y0 = max(x - pad, 0), max(y - pad, 0)
            x1 = min(x + w + pad, mask.shape[1])
            y1 = min(y + h + pad, mask.shape[0])
            local_mask = np.zeros_like(mask)
            cv2.drawContours(local_mask, [c], -1, 255, -1)
            local_crop = local_mask[y0:y1, x0:x1]

            if area > ref_area * 1.6:
                # ก้อนใหญ่ผิดปกติ -> น่าจะมีหลายเม็ดติดกัน ลอง split
                pieces = split_touching_blobs(local_crop, (x0, y0), ref_area)
                if len(pieces) >= 2:
                    for cx, cy, a in pieces:
                        zname = _which_zone(cx, cy, exclude_zones)
                        gems.append({"class": name, "px": (cx, cy), "area": a,
                                     "in_zone": zname,
                                     "correct": (zname == name) if zname else None,
                                     "split": True})
                    continue  # ใช้ผลจาก split แล้ว ไม่ต้องนับก้อนรวมซ้ำ
                # split ไม่สำเร็จ (เช่นภาพเบลอ) -> ตกไปนับเป็นก้อนเดียวแบบเดิมด้านล่าง

            if area > th["gem_max_area"] * 3:
                # ใหญ่เกินจริง ๆ ไม่น่าใช่ gem (อาจเป็นเงา/วัตถุอื่นสีใกล้เคียง)
                continue

            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            zname = _which_zone(cx, cy, exclude_zones)
            gems.append({"class": name, "px": (cx, cy), "area": float(area),
                         "in_zone": zname,
                         "correct": (zname == name) if zname else None,
                         "split": False})
    return gems


# ==================================================================
#  ส่วนที่ 3: ตรวจจับหุ่นด้วย ArUco
# ==================================================================

class RobotTracker:
    """
    หาตำแหน่งและทิศทางหุ่นจาก ArUco tag ที่ติดบนหลังคาหุ่น

    ArUco tag = ลายขาวดำคล้าย QR code แบบง่าย OpenCV อ่านได้เร็วมาก
    และบอกได้ทั้ง "อยู่ตรงไหน" และ "หมุนไปกี่องศา" จาก tag ใบเดียว
    """

    def __init__(self, marker_id=0, dict_name=cv2.aruco.DICT_4X4_50, calib=None,
                 cam_height_cm=0.0, tag_height_cm=0.0):
        self.marker_id = marker_id
        self.calib = calib          # FieldCalibration: ถ้ามี จะคืนมุม/ตำแหน่งใน cm ด้วย
        # v3.7 parallax: tag อยู่สูงจากพื้น -> homography (ที่ถูกเฉพาะบนพื้น) จะวางมันเลยจริง
        # แก้โดยหดระยะจากจุดใต้กล้อง (nadir ~ กลางภาพ) ด้วยสัดส่วน (H-h)/H
        self.parallax_k = (cam_height_cm - tag_height_cm) / cam_height_cm if cam_height_cm > 0 else 1.0
        self.nadir_cm = None        # คำนวณครั้งแรกที่รู้ขนาดเฟรม
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_name)
        # รองรับทั้ง OpenCV เวอร์ชันใหม่และเก่า
        if hasattr(cv2.aruco, "ArucoDetector"):
            params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, params)
            self._new_api = True
        else:
            self.params = cv2.aruco.DetectorParameters_create()
            self._new_api = False

        self.last_pose = None
        self.lost_frames = 0

    def detect(self, frame):
        """
        return: dict {"px": (x,y), "angle_deg": float, "corners": ndarray}
                หรือ None ถ้าหาไม่เจอในเฟรมนี้
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self._new_api:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.params)

        if ids is None or len(ids) == 0:
            self.lost_frames += 1
            return None

        for i, mid in enumerate(ids.flatten()):
            if mid != self.marker_id:
                continue
            c = corners[i][0]                      # 4 มุม เรียงตามเข็มนาฬิกา
            cx = float(np.mean(c[:, 0]))
            cy = float(np.mean(c[:, 1]))

            # ทิศหน้าหุ่น = เวกเตอร์จากกลางขอบล่าง ไปกลางขอบบนของ tag
            top_mid = (c[0] + c[1]) / 2.0
            bot_mid = (c[2] + c[3]) / 2.0
            vx, vy = top_mid[0] - bot_mid[0], top_mid[1] - bot_mid[1]
            angle_px = float(np.degrees(np.arctan2(vy, vx)))
            # ขนาด tag ใน pixel (ใช้ตัด blob รอบตัวหุ่นออกจาก gem)
            size_px = float(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)]))

            pose = {"px": (cx, cy), "angle_deg": angle_px, "corners": c,
                    "size_px": size_px}

            # --- มุมและตำแหน่งใน field frame (cm) ---
            # สำคัญ: กล้องเบี้ยว มุมใน pixel ไม่เท่ามุมจริง ต้องแปลงจุดผ่าน H ก่อน
            if self.calib is not None and self.calib.H is not None:
                if self.nadir_cm is None:
                    h_, w_ = frame.shape[:2]
                    self.nadir_cm = self.calib.to_field((w_ / 2.0, h_ / 2.0))
                nx, ny, k = self.nadir_cm[0], self.nadir_cm[1], self.parallax_k
                fix = lambda p: (nx + (p[0] - nx) * k, ny + (p[1] - ny) * k)
                cm_c = fix(self.calib.to_field((cx, cy)))
                cm_t = fix(self.calib.to_field(tuple(top_mid)))
                cm_b = fix(self.calib.to_field(tuple(bot_mid)))
                pose["cm"] = cm_c
                pose["angle_cm_deg"] = float(np.degrees(
                    np.arctan2(cm_t[1] - cm_b[1], cm_t[0] - cm_b[0])))

            self.lost_frames = 0
            self.last_pose = pose
            return self.last_pose

        self.lost_frames += 1
        return None

    def get_pose_or_last(self, max_lost=15):
        """
        ถ้าหาไม่เจอชั่วคราว (โดนบัง/เบลอ) ให้ใช้ตำแหน่งล่าสุดต่อได้ไม่เกิน
        max_lost เฟรม เกินกว่านั้นถือว่าหายจริง -> ต้องหยุดหุ่นเพื่อความปลอดภัย
        """
        if self.lost_frames == 0:
            return self.last_pose, "OK"
        if self.lost_frames <= max_lost and self.last_pose is not None:
            return self.last_pose, "STALE"
        return None, "LOST"


# ==================================================================
#  ส่วนที่ 4: บันทึก / โหลดแผนที่สนาม
# ==================================================================

def save_field_map(zones, corners, calib, path=FIELD_MAP_PATH):
    data = {
        "corners_px": corners,
        "homography": calib.H.tolist() if calib.H is not None else None,
        "zones": {},
    }
    for name, z in zones.items():
        entry = {"px": list(z["px"]), "radius_px": z["radius_px"]}
        if calib.H is not None:
            entry["cm"] = list(calib.to_field(z["px"]))
        data["zones"][name] = entry

    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    print(f"[OK] saved field map -> {path}")


def load_field_map(path=FIELD_MAP_PATH):
    if not os.path.exists(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    H = np.array(data["homography"], dtype=np.float64) if data.get("homography") else None
    zones = {}
    for name, z in data["zones"].items():
        zones[name] = {"px": tuple(z["px"]), "radius_px": z["radius_px"],
                       "area": 0.0}
    return {"zones": zones, "calib": FieldCalibration(H),
            "corners_px": data.get("corners_px")}


# ==================================================================
#  ส่วนที่ 5: วาด overlay สำหรับ debug
# ==================================================================

def draw_overlay(frame, zones, gems, robot_pose, calib=None, status_lines=None):
    out = frame.copy()

    for name, z in zones.items():
        col = DRAW_BGR.get(name, (255, 255, 255))
        cv2.circle(out, tuple(map(int, z["px"])), int(z["radius_px"]), col, 2)
        cv2.putText(out, name.replace("_", " "),
                    (int(z["px"][0]) - 45, int(z["px"][1]) - int(z["radius_px"]) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA)

    for g in gems:
        col = DRAW_BGR.get(g["class"], (255, 255, 255))
        if g.get("in_zone") is not None:
            # วางแล้ว: ถูกสี = วงเขียว, ผิดสี = กากบาทแดง
            if g.get("correct"):
                cv2.circle(out, g["px"], 5, (0, 255, 0), 1)
            else:
                cv2.drawMarker(out, g["px"], (0, 0, 255), cv2.MARKER_TILTED_CROSS, 12, 2)
        else:
            cv2.circle(out, g["px"], 4, col, -1)

    if robot_pose is not None:
        px = tuple(map(int, robot_pose["px"]))
        cv2.circle(out, px, 10, (0, 255, 255), 2)
        a = np.radians(robot_pose["angle_deg"])
        tip = (int(px[0] + 45 * np.cos(a)), int(px[1] + 45 * np.sin(a)))
        cv2.arrowedLine(out, px, tip, (0, 255, 255), 2, tipLength=0.3)

    y = 24
    for line in (status_lines or []):
        cv2.putText(out, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                    (255, 255, 255), 1, cv2.LINE_AA)
        y += 22
    return out


# ==================================================================
#  main
# ==================================================================

def lock_camera(cap, exposure=None, wb=None):
    """
    ปิด auto exposure / auto white balance
    ถ้าปล่อย auto ไว้ พอหุ่น (สีดำ/ขาว) วิ่งผ่าน กล้องจะปรับความสว่างใหม่
    ค่า HSV ที่ calibrate ไว้จะเลื่อนทั้งภาพ -> สีเพี้ยนกลางการแข่ง

    exposure: ค่า exposure ที่ต้องการ (Windows/DirectShow มักเป็นเลขลบ เช่น -6)
              None = แค่ปิด auto แล้วคงค่าปัจจุบัน
    TODO ทดสอบกับกล้องสนามจริง: บางกล้องไม่รับค่าพวกนี้ ต้องตั้งในโปรแกรมของกล้องแทน
    """
    # v5.0 bugfix: ของเดิม set 0.25 แล้วตามด้วย 1 เสมอ = จบที่ 1 (บน MSMF คือ "auto") -> ไม่เคยล็อกจริง
    # manual: MSMF = 0, DirectShow/V4L2 = 0.25 -> ลองทีละค่า หยุดที่ตัวแรกที่ set แล้วอ่านกลับตรง
    for v in (0, 0.25):
        try:
            if cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, v) and abs(cap.get(cv2.CAP_PROP_AUTO_EXPOSURE) - v) < 0.01:
                break
        except Exception:
            pass
    if exposure is not None:
        cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
    try:
        cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        if wb is not None:
            cap.set(cv2.CAP_PROP_WB_TEMPERATURE, wb)
    except Exception:
        pass
    print(f"[CAM] exposure={cap.get(cv2.CAP_PROP_EXPOSURE)}  "
          f"auto_exp={cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)}  "
          f"auto_wb={cap.get(cv2.CAP_PROP_AUTO_WB)}")


def open_source(source, width=1280, height=720, exposure=None):
    if str(source).isdigit():
        cap = None
        if os.name == "nt":
            # v4.4 (28 ก.ย. วัดจริง cam_probe.py): DirectShow ที่ 1280x720 ติด YUY2 = 4 fps เท่านั้น
            # MSMF + MJPG ได้ 30 fps และยังตั้ง exposure ได้ -> ใช้ MSMF ก่อน ถ้าเปิดไม่ติดค่อยถอยไป DSHOW แบบเดิม
            cap = cv2.VideoCapture(int(source), cv2.CAP_MSMF)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, 30)
            if not (cap.isOpened() and cap.read()[0]):
                cap.release(); cap = None
                print("[CAM] MSMF เปิดไม่ได้ -> ใช้ DirectShow (อาจได้แค่ 4 fps ที่ 720p)")
        if cap is None:
            cap = cv2.VideoCapture(int(source), cv2.CAP_DSHOW) if os.name == "nt" \
                else cv2.VideoCapture(int(source))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not cap.isOpened():
            # กล้องไม่มี / ถูกโปรแกรมอื่นจับอยู่ -> คืน cap ที่ปิดอยู่ให้ผู้เรียกเช็ค isOpened() เอง (ห้าม getBackendName)
            print(f"[CAM] เปิดกล้อง {source} ไม่ได้ (ไม่มี หรือโปรแกรมอื่นใช้อยู่)")
            return cap, None
        lock_camera(cap, exposure)
        try:
            be = cap.getBackendName()
        except cv2.error:
            be = "?"
        print(f"[CAM] {cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f}  backend={be}")
        return cap, None
    ext = os.path.splitext(str(source))[1].lower()
    if ext in (".png", ".jpg", ".jpeg", ".bmp"):
        return None, cv2.imread(str(source))
    return cv2.VideoCapture(str(source)), None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0")
    ap.add_argument("--setup", action="store_true",
                    help="โหมดตั้งค่าสนาม: คลิกมุมสนาม + detect โซน + บันทึก")
    ap.add_argument("--marker-id", type=int, default=0)
    ap.add_argument("--dict", default="DICT_4X4_50", help="ArUco dictionary ให้ตรงกับ tag ที่พิมพ์")
    ap.add_argument("--manual-zones", action="store_true",
                    help="setup: ไม่ใช้ HSV หาวง ให้คลิกกลางวงเองทั้ง 6 (ใช้เมื่อวงที่หาได้ไม่ตรง)")
    ap.add_argument("--zone-radius-cm", type=float, default=None,
                    help="setup: รัศมีวงจริง (cm) ใช้กับวงที่คลิกมือ ถ้าไม่ใส่จะประมาณจากวงที่ HSV หาได้")
    args = ap.parse_args()

    profiles = load_profiles()
    print_profile_report(profiles)
    cap, still = open_source(args.source)

    def grab():
        if still is not None:
            return still.copy()
        ok, f = cap.read()
        return f if ok else None

    frame = grab()
    if frame is None:
        print("[ERROR] อ่านภาพจากแหล่งที่ระบุไม่ได้")
        return

    # ---------- โหมด setup ----------
    if args.setup:
        print("\n=== FIELD SETUP ===")
        print("ขั้นที่ 1: คลิก 4 มุมสนาม")
        corners = pick_corners_interactive(frame)
        if corners is None:
            print("ยกเลิก setup")
            return
        calib = FieldCalibration.from_corners(corners)

        print("ขั้นที่ 2: ตรวจจับโซนสี...")
        if args.manual_zones:
            zones = {}
        else:
            zr = args.zone_radius_cm or 10.0
            zones, cands = detect_zones_auto(frame, calib, corners, zr, debug=True)
            print(f"  หาวงด้วยรูปร่าง: candidate {len(cands)} วง -> จำแนกได้ {len(zones)} สี")
        missing = []
        for name in COLOR_CLASSES:
            if name in zones:
                cm = calib.to_field(zones[name]["px"])
                print(f"  [OK]   {name:20s} px={zones[name]['px']}  "
                      f"cm=({cm[0]:.1f}, {cm[1]:.1f})")
            else:
                print(f"  [MISS] {name:20s} <-- หาไม่เจอ จะให้คลิกมือ")
                missing.append(name)

        # ขั้นที่ 3: โซนที่ HSV หาไม่เจอ ให้คลิกกลางวงเอง (พื้นไม้สีส้มอาจกลืนบางสี)
        if missing:
            # รัศมีอ้างอิงจากโซนที่เจอแล้ว ถ้าไม่เจอเลยใช้ค่าคร่าว ๆ
            found_r = [z["radius_px"] for z in zones.values()]
            if args.zone_radius_cm:
                # แปลง cm -> px ที่กลางสนาม
                c0 = calib.to_pixel((105, 60)); c1 = calib.to_pixel((105 + args.zone_radius_cm, 60))
                ref_r = float(np.hypot(c1[0] - c0[0], c1[1] - c0[1]))
            else:
                ref_r = float(np.median(found_r)) if found_r else 60.0
            for name in missing:
                pts = pick_points_interactive(frame, 1, f"Click CENTER of {name} zone")
                if pts:
                    zones[name] = {"px": pts[0], "radius_px": ref_r, "area": 0.0}
                    print(f"  [MANUAL] {name:20s} px={pts[0]}")

        save_field_map(zones, corners, calib)

        preview = draw_overlay(frame, zones, [], None, calib,
                               ["SETUP RESULT — กดปุ่มใดก็ได้เพื่อปิด"])
        cv2.imshow("Setup result", preview)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return

    # ---------- โหมดทำงานปกติ ----------
    fmap = load_field_map()
    if fmap is None:
        print("[ERROR] ยังไม่มี field_map.json — รัน --setup ก่อนครับ")
        return
    zones = fmap["zones"]
    calib = fmap["calib"]
    tracker = RobotTracker(marker_id=args.marker_id, dict_name=getattr(cv2.aruco, args.dict), calib=calib)

    print("กด q เพื่อออก\n")
    while True:
        frame = grab()
        if frame is None:
            break

        tracker.detect(frame)
        pose, pose_status = tracker.get_pose_or_last()
        # ตรวจหุ่นก่อน แล้วค่อยหา gem โดยตัดตัวหุ่นออก
        gems = detect_gems(frame, profiles, exclude_zones=zones, robot_pose=pose)
        gems, n_ambig = resolve_ambiguous(gems, profiles=profiles)

        loose = [g for g in gems if g["in_zone"] is None]
        placed_ok = [g for g in gems if g.get("correct") is True]
        placed_bad = [g for g in gems if g.get("correct") is False]

        lines = [
            f"loose gems: {len(loose)}   placed OK: {len(placed_ok)}   "
            f"placed WRONG: {len(placed_bad)}   ambiguous(2 colors)={n_ambig}",
            f"robot: {pose_status}" + (
                f"  angle={pose['angle_deg']:.0f}deg" if pose else ""),
        ]
        if pose and "cm" in pose:
            lines.append(f"robot field pos: ({pose['cm'][0]:.1f}, {pose['cm'][1]:.1f}) cm"
                         f"   heading(field)={pose['angle_cm_deg']:.0f}deg")

        cv2.imshow("Field Vision", draw_overlay(frame, zones, gems, pose, calib, lines))
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            break

    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()




# ==================================================================
#  ส่วนที่ 6: ตรวจสอบ profile ทับกัน + ตัดหินที่ถูกจับเป็น 2 สี
# ==================================================================

def _hue_ranges(p):
    if p["h_min"] <= p["h_max"]:
        return [(p["h_min"], p["h_max"])]
    return [(p["h_min"], 179), (0, p["h_max"])]


def profile_overlaps(profiles):
    """
    คืน list ของ (สีA, สีB, คำอธิบาย) ที่ช่วง H/S/V ทับกันทั้งสามแกน
    = มี pixel ที่ตรงทั้งสองสี -> หินก้อนเดียวจะถูกนับสองสี
    """
    out = []
    names = [n for n in COLOR_CLASSES if n in profiles]
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            pa, pb = profiles[a], profiles[b]
            h_ov = None
            for (a0, a1) in _hue_ranges(pa):
                for (b0, b1) in _hue_ranges(pb):
                    lo, hi = max(a0, b0), min(a1, b1)
                    if lo <= hi:
                        h_ov = (lo, hi)
            if h_ov is None:
                continue
            s_lo, s_hi = max(pa["s_min"], pb["s_min"]), min(pa["s_max"], pb["s_max"])
            v_lo, v_hi = max(pa["v_min"], pb["v_min"]), min(pa["v_max"], pb["v_max"])
            if s_lo <= s_hi and v_lo <= v_hi:
                out.append((a, b, f"H {h_ov[0]}-{h_ov[1]}  S {s_lo}-{s_hi}  V {v_lo}-{v_hi}"))
    return out


def resolve_ambiguous(gems, min_dist_px=15, profiles=None):
    """
    v5.0 (external review): จุดเดียวโดนจับเป็น 2 สี (มาสก์ทับกัน, ศูนย์ห่าง < 8 px)
    -> ให้สีที่โปรไฟล์ "จำเพาะกว่า" (s_min+v_min สูงกว่า เช่น NEON_CYAN ชนะ DEEP_SKY_BLUE
       ตรงพิกเซลสว่าง) ชนะ แทนการทิ้งทั้งคู่ ซึ่งทำให้หินฟ้าสว่างหายทั้งสี
    ของเดิมยังตัด "หินต่างสีที่วางชิดกัน" (8-15 px) ทิ้งด้วย -> เลิกตัด เก็บทั้งคู่
    ถ้าไม่ส่ง profiles มา (เรียกแบบเก่า) จุดทับกันจะถูกทิ้งทั้งคู่เหมือนเดิม
    คืน (list ที่สะอาดแล้ว, จำนวนที่ตัด)
    """
    SAME_BLOB_PX = 8
    def spec(cls):
        p = (profiles or {}).get(cls, {})
        return p.get("s_min", 0) + p.get("v_min", 0)
    bad = set()
    for i in range(len(gems)):
        for j in range(i + 1, len(gems)):
            if gems[i]["class"] == gems[j]["class"]:
                continue
            dx = gems[i]["px"][0] - gems[j]["px"][0]
            dy = gems[i]["px"][1] - gems[j]["px"][1]
            if dx * dx + dy * dy < SAME_BLOB_PX * SAME_BLOB_PX:
                if profiles:
                    bad.add(i if spec(gems[i]["class"]) < spec(gems[j]["class"]) else j)
                else:
                    bad.add(i); bad.add(j)
    clean = [g for k, g in enumerate(gems) if k not in bad]
    return clean, len(bad)


def print_profile_report(profiles):
    ov = profile_overlaps(profiles)
    if not ov:
        print("[PROFILES] OK ไม่มีช่วงสีทับกัน")
        return
    print("[PROFILES] !!! ช่วงสีทับกัน หินในช่วงนี้จะถูกตัดเป็น 'ไม่แน่ใจ' และไม่ถูกหยิบ:")
    for a, b, d in ov:
        print(f"   {a} <-> {b}: {d}")
    print("   แก้ด้วย calibrate_colors.py ให้ช่วงไม่ทับกัน (ดู README)")


if __name__ == "__main__":
    main()
