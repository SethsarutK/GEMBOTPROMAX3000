"""gembot_app.py — หน้าต่างเดียว รวมทุกขั้นตอนเตรียมแข่ง (เป้าหมาย: พร้อมใน < 3 นาที)

    python gembot_app.py            (จำกล้องตัวล่าสุดไว้ใน app_state.json)

ขั้นตอน (แถบบนจอ)  1 กล้อง  2 ครอบสนาม  3 วง 6 สี  4 สีหิน  5 หุ่น  6 พร้อม
ปุ่มร่วมทุกขั้น     Enter = ถัดไป   Backspace = ย้อน   R = ทำขั้นนี้ใหม่   F1-F7 = กระโดดไปขั้น (F7 = จูนวง/ระยะปาก)   Q = ออก
เมาส์              ใช้แค่คลิกจุด (มุมสนาม / กลางวง / หิน)

ไฟล์ที่ผลิต:  field_map.json (ขั้น 2-3)   color_profiles.json (ขั้น 4)   app_state.json (กล้อง)   grip_calib.json (ขั้น 7 จูนวง)
โปรแกรมเดิม (field_vision / calibrate_colors / test_follow / auto_main) ยังใช้แยกได้เหมือนเดิม
แอปนี้ไม่แตะ planner / nav / auto_config — ตอนกด SPACE ในขั้น 6 จะเปิด auto_main.py ให้ (แยกโปรเซส)
ทดสอบโดยไม่มีกล้อง:  python gembot_app.py field.png   (ใช้รูปนิ่งแทนกล้อง)
"""
import json, math, os, subprocess, sys, time

import cv2
import numpy as np

import auto_config as C
import nav
import ui
from ui import rrect, card, button, chip, stepper
from calibrate_colors import (COLOR_CLASSES, build_mask, clean_mask, load_profiles, save_profiles,
                              DEFAULT_PROFILES)
from field_vision import (FieldCalibration, detect_zones_auto, detect_gems, resolve_ambiguous,
                          save_field_map, load_field_map, open_source, RobotTracker, DRAW_BGR,
                          FIELD_W_CM, FIELD_H_CM)

HERE = os.path.dirname(os.path.abspath(__file__))
STATE_PATH = os.path.join(HERE, "app_state.json")
WIN = "GEMBOT"
STEPS = ["กล้อง", "ครอบสนาม", "วง 6 สี", "สีหิน", "หุ่น", "พร้อม", "จูนวง"]
NSTEP = len(STEPS)
CALIB_PATH = os.path.join(HERE, "grip_calib.json")
# ขั้น 7: ค่าที่จูนได้ (ชื่อใน auto_config, ขั้นปรับ, คำอธิบาย)
TUNE = [
    ("GRIP_REACH_CM",       0.5, "เพลา -> ก้นปาก (วงแดง)"),
    ("HEADING_OFFSET_DEG",  1.0, "มุมชดเชยหัว (ลูกศร)"),
    ("MARKER_TO_AXLE_CM",   0.5, "กลาง tag -> เพลา (+หน้า)"),
    ("JAW_TIP_CM",          0.5, "เพลา -> ปลายก้าม"),
    ("JAW_OPEN_HALF_CM",    0.5, "ครึ่งความกว้างก้ามอ้า"),
    ("ROBOT_BODY_R_CM",     0.5, "รัศมีตัวรถ (วงใหญ่)"),
    ("ROBOT_BODY_FRONT_CM", 0.5, "เพลา -> ขอบหน้าฐาน"),
    ("ROBOT_REAR_CM",       0.5, "เพลา -> ท้ายรถ"),
    ("ARM_R_CM",            0.5, "รัศมีโคนแขน (วงเล็ก)"),
    ("ROBOT_FRONT_CM",      0.5, "เพลา -> จุดหน้าสุด (กันชนกำแพง)"),
    ("TAG_HEIGHT_CM",       0.5, "ความสูง tag (แก้ parallax)"),
]
KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT = 0x260000, 0x280000, 0x250000, 0x270000
COLOR_TH = {"IRIDESCENT_VIOLET": "ม่วง", "NEON_CYAN": "ฟ้าอ่อน", "DEEP_CRIMSON": "แดง",
            "MARIGOLD_ACCENT": "ส้ม", "DEEP_SKY_BLUE": "น้ำเงิน", "LIME_GREEN": "เขียว"}
KEY_ENTER, KEY_BACK, KEY_ESC = 13, 8, 27
KEY_FIX = 0x7f0000                 # ปุ่มเสมือน: ไปขั้นแรกที่ยังไม่ผ่าน
KEY_F = {0x700000 + i * 0x10000: i for i in range(NSTEP)}   # F1..F7 จาก cv2.waitKeyEx บน Windows


def load_state():
    try:
        with open(STATE_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


def save_state(st):
    try:
        with open(STATE_PATH, "w", encoding="utf-8") as f:
            json.dump(st, f, indent=2, ensure_ascii=False)
    except Exception:
        pass


# ==================================================================
#  สีจากการคลิกหิน: เก็บพิกเซล HSV รอบจุดที่คลิก แล้วสร้างช่วง HSV เอง
# ==================================================================
def sample_hsv(hsv, pt, r=6):
    h, w = hsv.shape[:2]
    x, y = int(pt[0]), int(pt[1])
    m = np.zeros((h, w), np.uint8)
    cv2.circle(m, (x, y), r, 255, -1)
    return hsv[m == 255]


def profile_from_samples(samples):
    """samples: Nx3 (H,S,V) -> profile dict แบบเดียวกับ color_profiles.json
    hue ใช้สถิติเชิงวงกลม (แดงคร่อม 179->0) ช่วง = median +- max(6, 2.5*MAD) ไม่เกิน +-22
    S/V ใช้ percentile 10 ลบเผื่อ (แสงตกเงา) ขั้นต่ำ 30"""
    hh = samples[:, 0].astype(np.float64) * (2 * np.pi / 180.0)
    ref = math.atan2(np.mean(np.sin(hh)), np.mean(np.cos(hh))) * 180.0 / (2 * np.pi)
    ref %= 180.0
    dev = ((samples[:, 0].astype(np.float64) - ref + 90.0) % 180.0) - 90.0
    mad = float(np.median(np.abs(dev - np.median(dev))))
    spread = max(6.0, min(22.0, 2.5 * mad + 3.0))
    s10 = float(np.percentile(samples[:, 1], 10)); v10 = float(np.percentile(samples[:, 2], 10))
    return {"h_min": int((ref - spread) % 180), "h_max": int((ref + spread) % 180),
            "s_min": int(max(30, min(200, s10 - 35))), "s_max": 255,
            "v_min": int(max(30, min(200, v10 - 45))), "v_max": 255}


# ==================================================================
class App:
    def __init__(self, still_path=None):
        self.state = load_state()
        self.cam = int(self.state.get("cam", 1))
        self.cap = None
        self.still = cv2.imread(still_path) if still_path else None
        if still_path and self.still is None:
            raise SystemExit(f"เปิดรูป {still_path} ไม่ได้")
        self.frame = None
        self.step = 0
        self.msg = ""                     # ข้อความบรรทัดล่างของกล่องคำสั่ง
        self.mouse = (0, 0)
        self.mouse_f = None
        self.pending_key = None
        self.buttons = []
        self.clicks = []                  # คลิกที่ยังไม่ได้ใช้ (x, y)
        # ข้อมูลสนาม
        fm = load_field_map()
        self.corners = fm["corners_px"] if fm else None
        self.calib = fm["calib"] if fm else None
        self.zones = fm["zones"] if fm else {}
        self.zone_cands = []
        # สี
        self.profiles = load_profiles()
        self.samples = {c: [] for c in COLOR_CLASSES}   # HSV ที่คลิกเก็บ (ต่อสี)
        self.sel_color = 0
        self.show_mask = False
        self.gems = []
        self.gem_t = 0.0
        # หุ่น
        self.tracker = None
        self.link = None
        self.pose = None; self.pstat = "LOST"
        self.goal = None                  # (x, y) cm ตอนสั่งวิ่งทดสอบ
        self.arrived = False
        self.auto_proc = None
        self.tick = 0
        # ขั้น 7 จูนวง
        self.tune_sel = 0
        self.tune_base = {k: float(getattr(C, k)) for k, _, _ in TUNE}   # ค่าตอนเปิดแอป (ไว้กด R)
        self.tune_dirty = False
        self.meas = []                    # ตัวอย่างวัดอัตโนมัติ (fwd, lat) cm
        self.measuring = False
        self.have_all = self.corners is not None and len(self.zones) == 6 and os.path.exists("color_profiles.json")

    # ---------- กล้อง ----------
    def open_cam(self, idx):
        if self.still is not None:                      # โหมดรูปนิ่ง (ทดสอบ)
            return True
        if self.cap is not None:
            self.cap.release(); self.cap = None
            time.sleep(0.4)                              # DirectShow ปล่อยกล้องช้า
        ok = False
        for attempt in range(3):                          # กล้อง USB บางตัวเปิดครั้งแรกไม่ติด
            self.cap, _ = open_source(str(idx))
            ok = self.cap is not None and self.cap.isOpened() and self.cap.read()[0]
            if ok:
                break
            if self.cap is not None:
                self.cap.release(); self.cap = None
            time.sleep(0.6)
        if not ok:
            self.msg = (f"เปิดกล้อง {idx} ไม่ได้ — ถ้ามี auto_main / field_vision / calibrate เปิดอยู่ ให้ปิดก่อน "
                        f"(กล้องหนึ่งตัวใช้ได้ทีละโปรแกรม)")
        if ok:
            self.cam = idx
            self.state["cam"] = idx; save_state(self.state)
        return ok

    def grab(self):
        if self.still is not None:
            self.frame = self.still.copy(); return True
        if self.cap is None:
            return False
        ok, f = self.cap.read()
        if ok and f is not None:
            self.frame = f
        return ok

    # ---------- เมาส์ ----------
    def on_mouse(self, event, x, y, flags, param):
        self.mouse = (x, y)
        self.mouse_f = self.to_frame(x, y)
        if event == cv2.EVENT_LBUTTONDOWN:
            key = self.click_button(x, y)
            if key is not None:
                self.pending_key = key
            elif self.mouse_f is not None:
                self.clicks.append(self.mouse_f)

    # ---------- เปลี่ยนขั้น ----------
    def goto(self, s):
        s = max(0, min(NSTEP - 1, s))
        if s == 2 and self.calib is not None and not self.zones:
            self.auto_zones()
        if s in (4, 5, 6) and self.tracker is None and self.calib is not None:
            self.tracker = RobotTracker(marker_id=C.ARUCO_ID, dict_name=getattr(cv2.aruco, C.ARUCO_DICT),
                                        calib=self.calib, cam_height_cm=C.CAM_HEIGHT_CM,
                                        tag_height_cm=C.TAG_HEIGHT_CM)
        if s in (4, 5) and self.link is None:
            try:
                from link import RobotLink
                self.link = RobotLink()
            except Exception as e:
                self.msg = f"เปิด WiFi link ไม่ได้: {e}"
        if self.link is not None and s not in (4, 5):
            try: self.link.stop()
            except Exception: pass
        self.goal = None
        self.clicks.clear()
        self.step = s
        self.msg = ""

    # ---------- ขั้น ② ครอบสนาม ----------
    def set_corners(self, pts):
        self.corners = [list(map(int, p)) for p in pts]
        self.calib = FieldCalibration.from_corners(self.corners)
        self.tracker = None                                   # ให้สร้างใหม่ด้วย calib ใหม่
        self.zones = {}

    def zone_r_px(self):
        if self.calib is None:
            return 40.0
        a = self.calib.to_pixel((105, 60)); b = self.calib.to_pixel((115, 60))
        return float(np.hypot(a[0] - b[0], a[1] - b[1]))

    # ---------- ขั้น ③ วง ----------
    def auto_zones(self):
        if self.frame is None or self.calib is None or self.corners is None:
            return
        zones, cands = detect_zones_auto(self.frame, self.calib, self.corners, zone_radius_cm=10.0)
        self.zones = zones; self.zone_cands = cands
        self.msg = f"หาวงอัตโนมัติได้ {len(zones)}/6"

    def save_map(self):
        if self.calib is None or self.corners is None:
            return
        save_field_map(self.zones, self.corners, self.calib)

    # ---------- ขั้น ④ สี ----------
    def add_sample(self, pt):
        hsv = cv2.cvtColor(cv2.GaussianBlur(self.frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
        px = sample_hsv(hsv, pt)
        if len(px) == 0:
            return
        name = COLOR_CLASSES[self.sel_color]
        self.samples[name].append(px)
        allpx = np.concatenate(self.samples[name])
        self.profiles[name] = profile_from_samples(allpx)
        p = self.profiles[name]
        self.msg = (f"{COLOR_TH[name]}: เก็บ {len(self.samples[name])} จุด -> "
                    f"H {p['h_min']}-{p['h_max']}  S>={p['s_min']}  V>={p['v_min']}")

    def update_gems(self, force=False):
        """ตรวจหินทุก ~0.3 วิ (ไม่ต้องทุกเฟรม) นับต่อสีไว้แสดง"""
        if self.frame is None or (not force and time.time() - self.gem_t < 0.3):
            return
        self.gem_t = time.time()
        try:
            g = detect_gems(self.frame, self.profiles, exclude_zones=None)
            g, _ = resolve_ambiguous(g, profiles=self.profiles)
        except Exception:
            g = []
        self.gems = g

    # ---------- ขั้น ⑤ หุ่น ----------
    def update_robot(self):
        if self.tracker is None or self.frame is None:
            return
        self.tracker.detect(self.frame)
        self.pose, self.pstat = self.tracker.get_pose_or_last()
        if self.link is None:
            return
        vl = vr = 0
        if self.goal is not None and self.pose is not None and "cm" in self.pose and self.pstat == "OK":
            ax, ay, th = nav.axle_pose(self.pose)
            vl, vr, done = nav.go_to(ax, ay, th, *self.goal)
            if done:
                vl = vr = 0
                if not self.arrived:
                    self.msg = f"ถึงแล้ว (ห่างเป้า {nav.dist(ax, ay, *self.goal):.1f} cm)"
                self.arrived = True
        elif self.goal is not None:
            self.msg = "ไม่เห็นแท็กหุ่น -> หยุด"
        try:
            self.link.drive(vl, vr)
        except Exception:
            pass

    # ---------- ขั้น 7 จูนวง ----------
    def tune_set(self, name, val):
        setattr(C, name, round(float(val), 2))
        self.tune_dirty = True
        if name == "TAG_HEIGHT_CM" and self.tracker is not None and C.CAM_HEIGHT_CM > 0:
            self.tracker.parallax_k = (C.CAM_HEIGHT_CM - C.TAG_HEIGHT_CM) / C.CAM_HEIGHT_CM

    def tune_adjust(self, d):
        name, step, _ = TUNE[self.tune_sel]
        self.tune_set(name, getattr(C, name) + d * step)
        self.msg = f"{name} = {getattr(C, name):g}"

    def tune_reset(self):
        for k, v in self.tune_base.items():
            self.tune_set(k, v)
        self.tune_dirty = False; self.meas = []
        self.msg = "กลับเป็นค่าตอนเปิดแอป"

    def tune_save(self):
        data = {k: getattr(C, k) for k, _, _ in TUNE}
        with open(CALIB_PATH, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        self.tune_base = dict(data); self.tune_dirty = False
        self.msg = "บันทึก grip_calib.json แล้ว (auto_main จะใช้ค่านี้)"

    def jaw_sample(self):
        """หินที่อยู่ใกล้จุดปากที่สุด (ภายใน 12 cm) -> (fwd, lat) เทียบเพลา หรือ None"""
        if self.pose is None or "cm" not in self.pose or self.pstat != "OK":
            return None
        ax, ay, th = nav.axle_pose(self.pose)
        gp = nav.gripper_point(ax, ay, th)
        best, bd = None, 12.0
        for g in self.gems:
            cm = g["cm"] if "cm" in g else self.calib.to_field(g["px"])
            d = nav.dist(*cm, *gp)
            if d < bd:
                best, bd = cm, d
        if best is None:
            return None
        r = math.radians(th); dx, dy = best[0] - ax, best[1] - ay
        return (dx * math.cos(r) + dy * math.sin(r), -dx * math.sin(r) + dy * math.cos(r))

    def tune_measure_tick(self):
        """เก็บตัวอย่างต่อเฟรมตอนกด A จนครบ 30 แล้วคำนวณ GRIP_REACH / HEADING_OFFSET"""
        s = self.jaw_sample()
        if s is None:
            self.msg = f"วัด... ไม่เห็นหินที่ปาก ({len(self.meas)}/30) วางหิน 1 ก้อนที่ก้นปากหนีบ"
            return
        self.meas.append(s)
        self.msg = f"วัด... {len(self.meas)}/30"
        if len(self.meas) < 30:
            return
        fw = float(np.median([m[0] for m in self.meas])); la = float(np.median([m[1] for m in self.meas]))
        self.meas = []
        reach = math.hypot(fw, la)
        off = math.degrees(math.atan2(la, fw))
        self.tune_set("GRIP_REACH_CM", reach)
        self.tune_set("HEADING_OFFSET_DEG", nav.norm_deg(C.HEADING_OFFSET_DEG + off))
        self.measuring = False
        self.msg = (f"วัดได้: GRIP_REACH {reach:.1f} cm, หัวเบี้ยว {off:+.1f}° -> "
                    f"HEADING_OFFSET {C.HEADING_OFFSET_DEG:g}  (ดูวงแดงตรงหินไหม แล้วกด S)")

    def draw_tune(self, out):
        """วาดแคปซูลตัวรถ/แขน + ก้าม + วงแดง ด้วยค่าที่กำลังจูน"""
        if self.pose is None or self.calib is None or "cm" not in self.pose:
            return
        ax, ay, th = nav.axle_pose(self.pose)
        P = self.calib.to_pixel
        def rpx(pt, r):
            a = P(pt); b = P((pt[0] + r, pt[1])); return max(1, int(math.hypot(a[0] - b[0], a[1] - b[1])))
        sel = TUNE[self.tune_sel][0]
        col_body = (0, 220, 255) if sel in ("ROBOT_BODY_R_CM", "ROBOT_BODY_FRONT_CM", "ROBOT_REAR_CM") else (90, 90, 90)
        col_arm = (0, 220, 255) if sel == "ARM_R_CM" else (90, 90, 90)
        for (p0, p1, r), col in zip(nav.robot_capsules(self.pose), (col_body, col_arm)):
            a, b, rr = P(p0), P(p1), rpx(p1, r)
            cv2.circle(out, a, rr, col, 2, cv2.LINE_AA); cv2.circle(out, b, rr, col, 2, cv2.LINE_AA)
            cv2.line(out, a, b, col, 1, cv2.LINE_AA)
        # ก้าม: จากขอบหน้าฐานถึง JAW_TIP กว้าง +-JAW_OPEN_HALF
        half = getattr(C, "JAW_OPEN_HALF_CM", 7.0); tip = getattr(C, "JAW_TIP_CM", 20.0)
        rt = math.radians(th); lx, ly = -math.sin(rt), math.cos(rt)
        f0 = nav.point_ahead(ax, ay, th, C.ROBOT_BODY_FRONT_CM); f1 = nav.point_ahead(ax, ay, th, tip)
        quad = np.array([P((f0[0] + lx * half, f0[1] + ly * half)), P((f1[0] + lx * half, f1[1] + ly * half)),
                         P((f1[0] - lx * half, f1[1] - ly * half)), P((f0[0] - lx * half, f0[1] - ly * half))], np.int32)
        col_jaw = (0, 220, 255) if sel in ("JAW_TIP_CM", "JAW_OPEN_HALF_CM") else (200, 120, 0)
        cv2.polylines(out, [quad], True, col_jaw, 2, cv2.LINE_AA)
        # จุดหน้าสุด
        fp = P(nav.point_ahead(ax, ay, th, C.ROBOT_FRONT_CM))
        cv2.drawMarker(out, fp, (0, 220, 255) if sel == "ROBOT_FRONT_CM" else (200, 200, 200), cv2.MARKER_TILTED_CROSS, 14, 2)
        # เพลา + ลูกศร + วงแดง (ปาก)
        a = P((ax, ay))
        cv2.drawMarker(out, a, (255, 255, 255), cv2.MARKER_CROSS, 16, 2)
        cv2.arrowedLine(out, a, P(nav.point_ahead(ax, ay, th, 20)), (0, 255, 0), 3, tipLength=0.3, line_type=cv2.LINE_AA)
        gp = P(nav.gripper_point(ax, ay, th))
        cv2.circle(out, gp, rpx(nav.gripper_point(ax, ay, th), getattr(C, "JAW_CHECK_R_CM", 4.5)), (0, 0, 255), 1, cv2.LINE_AA)
        cv2.circle(out, gp, 7, (0, 0, 255), -1, cv2.LINE_AA)
        # หินที่ใช้วัด
        s = self.jaw_sample()
        if s is not None:
            ax2, ay2, th2 = ax, ay, th
            r = math.radians(th2)
            pt = (ax2 + s[0] * math.cos(r) - s[1] * math.sin(r), ay2 + s[0] * math.sin(r) + s[1] * math.cos(r))
            cv2.circle(out, P(pt), 11, (255, 0, 255), 2, cv2.LINE_AA)

    # ---------- checklist ----------
    def checklist(self):
        wifi = bool(self.link is not None and self.link.alive)
        seen = self.pstat == "OK" and self.pose is not None and "cm" in self.pose
        ncol = sum(1 for c in COLOR_CLASSES if any(g["class"] == c for g in self.gems))
        return [
            ("กล้อง", self.still is not None or (self.cap is not None and self.cap.isOpened()), f"index {self.cam}"),
            ("ครอบสนาม", self.calib is not None, "field_map.json"),
            ("วง 6 สี", len(self.zones) == 6, f"{len(self.zones)}/6"),
            ("สีหิน", os.path.exists("color_profiles.json"), f"ตอนนี้เห็น {ncol}/6 สี, {len(self.gems)} ก้อน"),
            ("กล้องเห็นหุ่น", seen, "ArUco id %d" % C.ARUCO_ID),
            ("WiFi ถึงหุ่น", wifi, "GEMBOT 192.168.4.1"),
        ]

    # ---------- เปิด auto_main แยกโปรเซส ----------
    def launch_auto(self):
        if self.link is not None:
            try: self.link.stop(); self.link.close()
            except Exception: pass
            self.link = None
        if self.cap is not None:
            self.cap.release(); self.cap = None
        cv2.destroyWindow(WIN)
        cmd = [sys.executable, os.path.join(HERE, "auto_main.py"), "--source", str(self.cam)]
        print("[APP] launching:", " ".join(cmd))
        env = dict(os.environ, PYTHONIOENCODING="utf-8")
        subprocess.call(cmd, cwd=HERE, env=env)
        print("[APP] auto_main exited -> back to app")
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN, self.CW, self.CH)
        cv2.setMouseCallback(WIN, self.on_mouse)
        self.open_cam(self.cam)
        self.tracker = None
        self.goto(5)

    # ==================================================================
    #  วาดจอ (v4.3 layout: กล้องซ้ายไม่มีอะไรบัง / การ์ดขวา / ปุ่มกดเมาส์ได้)
    # ==================================================================
    CW, CH = 1440, 810                 # canvas
    TOP = 64                           # แถบ stepper
    SIDE = 400                         # แถบขวา
    BOT = 44                           # แถบคีย์ล่าง

    def cam_rect(self):
        """พื้นที่วางภาพกล้อง (x, y, w, h) รักษาสัดส่วนเฟรม"""
        aw, ah = self.CW - self.SIDE - 24, self.CH - self.TOP - self.BOT - 16
        fh, fw = (self.frame.shape[:2] if self.frame is not None else (720, 1280))
        s = min(aw / fw, ah / fh)
        w, h = int(fw * s), int(fh * s)
        return 12 + (aw - w) // 2, self.TOP + 8 + (ah - h) // 2, w, h

    def to_frame(self, x, y):
        """พิกัดเมาส์บน canvas -> พิกัดในเฟรมกล้อง (None ถ้าอยู่นอกภาพ)"""
        cx, cy, cw, ch = self.cam_rect()
        if not (cx <= x < cx + cw and cy <= y < cy + ch) or self.frame is None:
            return None
        fh, fw = self.frame.shape[:2]
        return (int((x - cx) * fw / cw), int((y - cy) * fh / ch))

    def step_done(self, i):
        return [self.still is not None or (self.cap is not None and self.cap.isOpened()),
                self.calib is not None, len(self.zones) == 6,
                os.path.exists("color_profiles.json"), self.pstat == "OK", False,
                os.path.exists(CALIB_PATH) and not self.tune_dirty][i]

    def draw_loupe(self, out, x, y, size=180):
        """แว่นขยาย 4x ตรงเมาส์ (ไว้คลิกมุมสนามให้แม่น)"""
        if self.frame is None or self.mouse_f is None:
            rrect(out, x, y, size, size, ui.COL["panel2"], r=8)
            ui._blit_text(out, [(x + 14, y + size // 2 - 10, "เลื่อนเมาส์ไปบนภาพ", ui.COL["dim"], 15, False)])
            return
        mx, my = self.mouse_f
        H, W = self.frame.shape[:2]
        r = size // 8
        x0, y0 = min(max(0, mx - r), W - 2 * r), min(max(0, my - r), H - 2 * r)
        crop = self.frame[y0:y0 + 2 * r, x0:x0 + 2 * r]
        big = cv2.resize(crop, (size, size), interpolation=cv2.INTER_NEAREST)
        cx, cy = (mx - x0) * 4, (my - y0) * 4
        cv2.line(big, (cx, 0), (cx, size), (0, 255, 255), 1)
        cv2.line(big, (0, cy), (size, cy), (0, 255, 255), 1)
        out[y:y + size, x:x + size] = big
        rrect(out, x, y, size, size, (255, 255, 255), r=6, thickness=1)

    def draw_field(self, out):
        if self.corners:
            pts = np.array(self.corners, np.int32)
            cv2.polylines(out, [pts], True, (0, 255, 255), 2)
            for i, p in enumerate(self.corners):
                cv2.circle(out, tuple(p), 7, (0, 255, 255), -1)
                cv2.circle(out, tuple(p), 7, (0, 0, 0), 1)

    def draw_zones(self, out):
        for name, z in self.zones.items():
            col = DRAW_BGR.get(name, (255, 255, 255))
            cx, cy = int(z["px"][0]), int(z["px"][1])
            cv2.circle(out, (cx, cy), int(z["radius_px"]), col, 3, cv2.LINE_AA)
            label = f"{COLOR_CLASSES.index(name) + 1} {COLOR_TH.get(name, name)}"
            w = ui._width(label, 20, True)
            ui._shade(out, cx - w // 2 - 8, cy - 14, w + 16, 30, alpha=0.65)
            ui._blit_text(out, [(cx - w // 2, cy - 12, label, (255, 255, 255), 20, True)])

    def draw_gems(self, out):
        for g in self.gems:
            col = DRAW_BGR.get(g["class"], (255, 255, 255))
            c = tuple(map(int, g["px"]))
            cv2.circle(out, c, 13, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.circle(out, c, 13, col, 1, cv2.LINE_AA)

    def draw_robot(self, out):
        if self.pose is None or self.calib is None or "cm" not in self.pose:
            return
        ax, ay, th = nav.axle_pose(self.pose)
        p0 = self.calib.to_pixel((ax, ay)); p1 = self.calib.to_pixel(nav.point_ahead(ax, ay, th, 20))
        gp = self.calib.to_pixel(nav.gripper_point(ax, ay, th))
        cv2.arrowedLine(out, p0, p1, (0, 255, 0), 4, tipLength=0.3, line_type=cv2.LINE_AA)
        cv2.circle(out, gp, 7, (0, 0, 255), -1, cv2.LINE_AA)
        if self.goal is not None:
            cv2.circle(out, self.calib.to_pixel(self.goal), 12, (255, 0, 255), 2, cv2.LINE_AA)

    def draw_frame_overlays(self, f):
        s = self.step
        if s == 3 and self.show_mask:
            hsv = cv2.cvtColor(cv2.GaussianBlur(f, (5, 5), 0), cv2.COLOR_BGR2HSV)
            m = clean_mask(build_mask(hsv, self.profiles[COLOR_CLASSES[self.sel_color]]), kernel_size=3)
            f = cv2.addWeighted(f, 0.35, cv2.cvtColor(m, cv2.COLOR_GRAY2BGR), 0.65, 0)
        if s >= 1:
            self.draw_field(f)
            if s == 1:
                for i, p in enumerate(self.clicks[:4]):
                    cv2.circle(f, p, 8, (0, 200, 255), -1)
                    cv2.circle(f, p, 8, (0, 0, 0), 1)
                if len(self.clicks) >= 2:
                    cv2.polylines(f, [np.array(self.clicks[:4], np.int32)], len(self.clicks) >= 4, (0, 200, 255), 1)
        if s >= 2:
            self.draw_zones(f)
        if s in (3, 5, 6):
            self.draw_gems(f)
        if s in (4, 5):
            self.draw_robot(f)
        if s == 6:
            self.draw_tune(f)
        return f

    # ---------- แถบขวา ----------
    def side_content(self, out, x, y, w):
        """เขียนเนื้อหาของขั้นในการ์ด คืน (y ถัดไป, [ปุ่ม])  ปุ่ม = (label, key_hint, action_key, kind)"""
        s = self.step
        rows = []
        btns = []
        if s == 0:
            rows = [("เลือกกล้องที่เห็นสนามเต็ม", None, "text"),
                    ("กด 0-4 เพื่อสลับกล้อง", None, "dim"), None,
                    ("ตอนนี้", f"index {self.cam}" + ("" if self.grab_ok else "  เปิดไม่ได้!"),
                     "ok" if self.grab_ok else "bad")]
            btns = [("ใช้กล้องนี้", "Enter", KEY_ENTER, "primary" if self.grab_ok else "disabled")]
        elif s == 1:
            n = len(self.clicks)
            names = ["ซ้ายบน", "ขวาบน", "ขวาล่าง", "ซ้ายล่าง"]
            rows = [("คลิกมุมสนาม 4 มุม ตามลำดับ", None, "text"),
                    ("ซ้ายบน → ขวาบน → ขวาล่าง → ซ้ายล่าง", None, "dim"), None,
                    ("คลิกถัดไป", names[n] if n < 4 else "ครบ 4 มุมแล้ว", "key" if n < 4 else "ok"),
                    ("ค่าเดิม", "มี — กด Enter ใช้เลยได้" if self.corners and n == 0 else "-",
                     "ok" if (self.corners and n == 0) else "dim")]
            btns = [("ยืนยันกรอบ", "Enter", KEY_ENTER, "primary" if (n >= 4 or self.corners) else "disabled"),
                    ("ล้าง คลิกใหม่", "R", ord('r'), "normal")]
        elif s == 2:
            rows = [("โปรแกรมหาวงเอง — เช็คป้ายชื่อบนวงว่าถูกสี", None, "text"),
                    ("วงไหนหาย/ผิด: กดเลขสี แล้วคลิกกลางวง", None, "dim"), None,
                    ("วงที่ได้", f"{len(self.zones)} / 6", "ok" if len(self.zones) == 6 else "warn")]
            btns = [("บันทึกวง ไปขั้นสี", "Enter", KEY_ENTER, "primary"),
                    ("หาวงใหม่", "R", ord('r'), "normal")]
        elif s == 3:
            rows = [("กดเลขสี แล้วคลิกหินสีนั้น 2-3 ก้อน", None, "text"),
                    ("คลิกคนละก้อน คนละมุมสนาม จะทนแสงกว่า", None, "dim")]
            btns = [("บันทึกสี ไปขั้นหุ่น", "Enter", KEY_ENTER, "primary"),
                    ("ดู mask สีที่เลือก", "M", ord('m'), "normal"),
                    ("ล้างสีที่เลือก", "R", ord('r'), "normal")]
        elif s == 4:
            seen = self.pstat == "OK" and self.pose is not None and "cm" in self.pose
            pos = (f"({self.pose['cm'][0]:.0f}, {self.pose['cm'][1]:.0f}) cm  หัน {nav.axle_pose(self.pose)[2]:.0f}°"
                   if seen else "-")
            wifi = bool(self.link and self.link.alive)
            rows = [("วางหุ่นในสนาม: ลูกศรเขียว = หน้าหุ่น, จุดแดง = ปาก", None, "text"), None,
                    ("กล้องเห็นแท็ก", "เห็น" if seen else "ไม่เห็น", "ok" if seen else "bad"),
                    ("ตำแหน่ง", pos, "dim"),
                    ("WiFi ถึงหุ่น", "ต่ออยู่" if wifi else "ขาด (ต่อ WiFi GEMBOT)", "ok" if wifi else "bad"), None,
                    ("ทดสอบวิ่ง: กด 1-6 วิ่งไปวงสีนั้น", None, "dim")]
            btns = [("ไปหน้าสรุป", "Enter", KEY_ENTER, "primary"),
                    ("จูนวง / ระยะปาก", "F7", 0x700000 + 6 * 0x10000, "normal"),
                    ("หยุดหุ่น", "SPACE", ord(' '), "danger")]
        elif s == 6:
            seen = self.pstat == "OK" and self.pose is not None and "cm" in self.pose
            rows = [("ปุ่มขึ้น/ลง เลือกค่า, ซ้าย/ขวา ปรับ (วงขยับทันที)", None, "text"),
                    ("กล้องเห็นแท็ก", "เห็น" if seen else "ไม่เห็น", "ok" if seen else "bad"), None]
            for i, (name, step, desc) in enumerate(TUNE):
                v = getattr(C, name)
                changed = abs(v - self.tune_base[name]) > 1e-6
                mark = "> " if i == self.tune_sel else "   "
                rows.append((f"{mark}{desc}", f"{v:g}" + (" *" if changed else ""),
                             "key" if i == self.tune_sel else ("warn" if changed else "dim")))
            rows += [None, ("A = วัดระยะปาก+หัวเบี้ยวเอง (วางหินที่ก้นปาก)", None, "dim"),
                     ("[ ] = ปรับทีละ 0.1   * = ยังไม่บันทึก", None, "dim"),
                     ("ลบไฟล์ grip_calib.json = กลับค่าใน auto_config", None, "dim")]
            btns = [("บันทึกค่า (grip_calib.json)", "S", ord('s'), "primary" if self.tune_dirty else "normal"),
                    ("วัดอัตโนมัติจากหินที่ปาก", "A", ord('a'), "normal"),
                    ("กลับค่าเดิม", "R", ord('r'), "normal"),
                    ("ไปหน้าสรุป", "Enter", KEY_ENTER, "normal")]
        else:
            ok_all = True
            self.check_icons = []
            for name, ok, note in self.checklist():
                rows.append((name, note, "ok" if ok else "bad"))
                self.check_icons.append(ok)
                ok_all = ok_all and ok
            rows += [None, ("เขียวครบ = พร้อม  /  แดง = กด F1-F5 ไปแก้ขั้นนั้น", None, "dim")]
            btns = [("เริ่มโหมด AUTO", "SPACE", ord(' '), "primary" if ok_all else "normal"),
                    ("ไปแก้ขั้นที่ยังไม่ผ่าน", "F1-F5", KEY_FIX, "normal" if not ok_all else "disabled")]
        if self.msg:
            rows += [None, (self.msg, None, "warn")]

        # เนื้อหา
        label_w = max([ui._width(str(r[0]), 16) for r in rows if isinstance(r, tuple) and r[1] is not None] + [0])
        cy = y
        items = []
        for r in rows:
            if r is None:
                cy += 12; continue
            if r[1] is None:
                col = ui.COL.get(r[2] if len(r) > 2 else "text", ui.COL["text"])
                items.append((x, cy, str(r[0]), col, 16, False)); cy += 24
            else:
                col = ui.COL.get(r[2] if len(r) > 2 else "text", ui.COL["text"])
                items.append((x, cy, str(r[0]), ui.COL["dim"], 16, False))
                items.append((x + label_w + 14, cy, str(r[1]), col, 16, False)); cy += 24
        if s == 5:
            icons = getattr(self, "check_icons", [])
            for i, it in enumerate([it for it in items if it[3] == ui.COL["dim"]][:len(icons)]):
                cv2.circle(out, (it[0] + label_w + 14 + 8, it[1] + 10), 7,
                           ui.COL["ok"] if icons[i] else ui.COL["bad"], -1, cv2.LINE_AA)
            # เลื่อนข้อความค่าให้พ้นไอคอน
            items = [(ix + (24 if (ix != x and s == 5) else 0), iy, t, c, sz, b) for ix, iy, t, c, sz, b in items]
        ui._blit_text(out, items)
        cy += 8
        # ชิปสี (ขั้น 3 และ 4)
        if s in (2, 3):
            for i, c in enumerate(COLOR_CLASSES):
                if s == 3:
                    n = sum(1 for g in self.gems if g["class"] == c); k = len(self.samples[c])
                    sub = f"เห็น {n}" + (f" · คลิก {k}" if k else ""); st = "ok" if n else ("warn" if k else "dim")
                else:
                    have = c in self.zones
                    sub = "มีวง" if have else "ไม่มี"; st = "ok" if have else "bad"
                chip(out, x, cy, w, 32, DRAW_BGR[c], f"{i + 1}  {COLOR_TH[c]}", sub, selected=(i == self.sel_color), state=st)
                self.buttons.append(((x, cy, w, 32), ord('1') + i))
                cy += 38
            cy += 6
        return cy, btns

    def render(self):
        out = np.full((self.CH, self.CW, 3), ui.COL["canvas"], np.uint8)
        # --- ภาพกล้อง ---
        cx, cy, cw, ch = self.cam_rect()
        if self.frame is not None:
            f = self.draw_frame_overlays(self.frame.copy())
            out[cy:cy + ch, cx:cx + cw] = cv2.resize(f, (cw, ch), interpolation=cv2.INTER_AREA)
        rrect(out, cx - 1, cy - 1, cw + 2, ch + 2, ui.COL["line"], r=6, thickness=1)
        # --- stepper ---
        done = [self.step_done(i) for i in range(NSTEP)]
        stepper(out, 24, 8, self.CW - self.SIDE - 60, STEPS, self.step, done)
        ui._blit_text(out, [(self.CW - self.SIDE + 12, 22, f"GEMBOT  ·  กล้อง {self.cam}", ui.COL["dim"], 15, False)])
        # --- การ์ดขวา ---
        sx, sy, sw = self.CW - self.SIDE + 12, self.TOP + 8, self.SIDE - 24
        sh = self.CH - self.TOP - self.BOT - 16
        title = ["1) กล้อง", "2) ครอบสนาม", "3) วง 6 สี", "4) สีหิน — คลิกหิน", "5) หุ่น", "6) พร้อมแข่ง",
                 "7) จูนวง / ระยะปาก"][self.step]
        if self.step == 5:
            title = "6) พร้อมแข่ง" if all(ok for _, ok, _ in self.checklist()) else "6) ยังไม่พร้อม"
        y0 = card(out, sx, sy, sw, sh, title=title)
        self.buttons = []
        y1, btns = self.side_content(out, sx + 16, y0, sw - 32)
        if self.step == 1:
            self.draw_loupe(out, sx + (sw - 180) // 2, max(y1, sy + sh - 320), 180)
        # ปุ่ม (เรียงจากล่างขึ้น ปุ่มหลักล่างสุด)
        by = sy + sh - 16
        for label, hint, key, kind in btns:
            bh = 46 if kind == "primary" else 38
            by -= bh
            button(out, sx + 16, by, sw - 32, bh, label, kind=kind, key=hint)
            self.buttons.append(((sx + 16, by, sw - 32, bh), key))
            by -= 8
        # --- แถบล่าง ---
        keys = [("Enter", "ถัดไป"), ("Backspace", "ย้อน"), ("R", "ทำใหม่"), ("F1-F7", "ไปขั้นที่"), ("Q", "ออก")]
        ui.keybar(out, keys, y=self.CH - self.BOT, size=15)
        return out

    def click_button(self, x, y):
        for (bx, by, bw, bh), key in getattr(self, "buttons", []):
            if bx <= x < bx + bw and by <= y < by + bh:
                return key
        return None

    # ==================================================================
    #  คีย์
    # ==================================================================
    def handle_key(self, k):
        if k in (ord('q'), ord('Q'), KEY_ESC):
            return False
        if k in KEY_F:
            self.goto(KEY_F[k]); return True
        if k == KEY_FIX:
            for i, (_, ok, _) in enumerate(self.checklist()):
                if not ok:
                    self.goto(min(i, 4)); break        # WiFi (ข้อ 6) อยู่ในขั้นหุ่น
            return True
        if k == KEY_BACK:
            self.goto(self.step - 1); return True
        s = self.step
        if s == 0:
            if k in (ord('1'), ord('2'), ord('3'), ord('4'), ord('0')):
                idx = int(chr(k))
                self.msg = f"เปิดกล้อง {idx} ..."
                if not self.open_cam(idx):
                    self.msg = f"เปิดกล้อง {idx} ไม่ได้"
            elif k == KEY_ENTER and self.grab_ok:
                self.goto(1)
        elif s == 1:
            if k in (ord('r'), ord('R')):
                self.clicks.clear()
            elif k == KEY_ENTER:
                if len(self.clicks) >= 4:
                    self.set_corners(self.clicks[:4]); self.clicks.clear()
                    self.auto_zones(); self.goto(2)
                elif self.corners:
                    if not self.zones:
                        self.auto_zones()
                    self.goto(2)
                else:
                    self.msg = "ยังคลิกไม่ครบ 4 มุม"
        elif s == 2:
            if ord('1') <= k <= ord('6'):
                self.sel_color = k - ord('1'); self.msg = f"คลิกกลางวง {COLOR_TH[COLOR_CLASSES[self.sel_color]]}"
            elif k in (ord('r'), ord('R')):
                self.auto_zones()
            elif k == KEY_ENTER:
                self.save_map()
                if len(self.zones) < 6:
                    self.msg = f"บันทึกแล้ว (วง {len(self.zones)}/6 — สีที่ไม่มีวงจะไม่ถูกเก็บ)"
                self.goto(3)
        elif s == 3:
            if ord('1') <= k <= ord('6'):
                self.sel_color = k - ord('1'); self.msg = f"คลิกหิน {COLOR_TH[COLOR_CLASSES[self.sel_color]]} 2-3 ก้อน"
            elif k in (ord('m'), ord('M')):
                self.show_mask = not self.show_mask
            elif k in (ord('r'), ord('R')):
                name = COLOR_CLASSES[self.sel_color]
                self.samples[name] = []; self.profiles[name] = dict(load_profiles().get(name, DEFAULT_PROFILES[name]))
                self.msg = f"ล้าง {COLOR_TH[name]} กลับเป็นค่าที่บันทึกไว้"
            elif k == KEY_ENTER:
                save_profiles(self.profiles); self.msg = "บันทึก color_profiles.json แล้ว"
                self.goto(4)
        elif s == 4:
            if ord('1') <= k <= ord('6'):
                name = COLOR_CLASSES[k - ord('1')]
                if name in self.zones and self.calib is not None:
                    self.goal = self.calib.to_field(self.zones[name]["px"]); self.arrived = False
                    self.msg = f"วิ่งไปวง {COLOR_TH[name]}"
                else:
                    self.msg = f"ไม่มีวง {COLOR_TH[name]} ในแผนที่"
            elif k == ord(' '):
                self.goal = None
                if self.link: self.link.stop()
                self.msg = "หยุด"
            elif k == KEY_ENTER:
                self.goal = None
                if self.link: self.link.stop()
                self.goto(5)
        elif s == 5:
            if k == ord(' '):
                self.launch_auto()
            elif k == KEY_ENTER:
                pass
        elif s == 6:
            if k == KEY_UP:
                self.tune_sel = (self.tune_sel - 1) % len(TUNE)
            elif k == KEY_DOWN:
                self.tune_sel = (self.tune_sel + 1) % len(TUNE)
            elif k == KEY_LEFT:
                self.tune_adjust(-1)
            elif k == KEY_RIGHT:
                self.tune_adjust(+1)
            elif k in (ord('['), ord(']')):                       # ปรับละเอียด 0.1
                name = TUNE[self.tune_sel][0]
                self.tune_set(name, getattr(C, name) + (0.1 if k == ord(']') else -0.1))
                self.msg = f"{name} = {getattr(C, name):g}"
            elif k in (ord('a'), ord('A')):
                self.meas = []; self.measuring = True
                self.msg = "วัด... วางหิน 1 ก้อนที่ก้นปากหนีบ ให้หุ่นนิ่ง"
            elif k in (ord('s'), ord('S')):
                self.tune_save()
            elif k in (ord('r'), ord('R')):
                self.tune_reset()
            elif k == KEY_ENTER:
                if self.tune_dirty:
                    self.msg = "ยังไม่บันทึก — กด S ก่อน หรือ R ทิ้งค่า"
                else:
                    self.goto(5)
        return True

    # ==================================================================
    def run(self):
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN, self.CW, self.CH)
        cv2.setMouseCallback(WIN, self.on_mouse)
        self.grab_ok = self.open_cam(self.cam) and self.grab()
        if not self.grab_ok:
            for idx in (1, 0, 2):
                if self.open_cam(idx) and self.grab():
                    self.grab_ok = True; break
        # ทางลัด: มีทุกไฟล์แล้ว -> ไปหน้าสรุปเลย (กระโดดกลับไปแก้ขั้นไหนก็ได้ด้วย F1-F5)
        self.goto(5 if (self.have_all and self.grab_ok) else 0)
        print(__doc__)
        while True:
            self.grab_ok = self.grab()
            if self.step in (3, 5, 6):
                self.update_gems()
            if self.step in (4, 5, 6):
                self.update_robot()
            if self.step == 6 and self.measuring:
                self.tune_measure_tick()
            # ใช้คลิกค้าง
            if self.clicks and self.step == 2:
                pt = self.clicks.pop(0)
                name = COLOR_CLASSES[self.sel_color]
                self.zones[name] = {"px": (int(pt[0]), int(pt[1])), "radius_px": self.zone_r_px(), "area": 0.0}
                self.msg = f"ตั้งวง {COLOR_TH[name]} ที่ {pt}"
            elif self.clicks and self.step == 3:
                self.add_sample(self.clicks.pop(0)); self.update_gems(force=True)
            elif self.clicks and self.step != 1:
                self.clicks.clear()
            cv2.imshow(WIN, self.render())
            k = cv2.waitKeyEx(1)
            if self.pending_key is not None:
                k, self.pending_key = self.pending_key, None
            if k == -1:
                continue
            kk = k & 0xFF if k < 0x100000 else k
            if not self.handle_key(kk):
                break
        if self.link is not None:
            try: self.link.stop(); self.link.close()
            except Exception: pass
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    os.chdir(HERE)                       # field_map.json / color_profiles.json อยู่ข้างไฟล์นี้เสมอ
    arg = sys.argv[1] if len(sys.argv) > 1 else None
    if arg is not None and arg.isdigit():
        st = load_state(); st["cam"] = int(arg); save_state(st); arg = None
    App(still_path=arg).run()
