"""gembot_app.py — หน้าต่างเดียว รวมทุกขั้นตอนเตรียมแข่ง (เป้าหมาย: พร้อมใน < 3 นาที)

    python gembot_app.py            (จำกล้องตัวล่าสุดไว้ใน app_state.json)

ขั้นตอน (แถบบนจอ)  1 กล้อง  2 ครอบสนาม  3 วง 6 สี  4 สีหิน  5 หุ่น  6 พร้อม
ปุ่มร่วมทุกขั้น     Enter = ถัดไป   Backspace = ย้อน   R = ทำขั้นนี้ใหม่   F1-F6 = กระโดดไปขั้น   Q = ออก
เมาส์              ใช้แค่คลิกจุด (มุมสนาม / กลางวง / หิน)

ไฟล์ที่ผลิต:  field_map.json (ขั้น 2-3)   color_profiles.json (ขั้น 4)   app_state.json (กล้อง)
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
from calibrate_colors import (COLOR_CLASSES, build_mask, clean_mask, load_profiles, save_profiles,
                              DEFAULT_PROFILES)
from field_vision import (FieldCalibration, detect_zones_auto, detect_gems, resolve_ambiguous,
                          save_field_map, load_field_map, open_source, RobotTracker, DRAW_BGR,
                          FIELD_W_CM, FIELD_H_CM)

HERE = os.path.dirname(os.path.abspath(__file__))
STATE_PATH = os.path.join(HERE, "app_state.json")
WIN = "GEMBOT"
STEPS = ["กล้อง", "ครอบสนาม", "วง 6 สี", "สีหิน", "หุ่น", "พร้อม"]
COLOR_TH = {"IRIDESCENT_VIOLET": "ม่วง", "NEON_CYAN": "ฟ้าอ่อน", "DEEP_CRIMSON": "แดง",
            "MARIGOLD_ACCENT": "ส้ม", "DEEP_SKY_BLUE": "น้ำเงิน", "LIME_GREEN": "เขียว"}
KEY_ENTER, KEY_BACK, KEY_ESC = 13, 8, 27
KEY_F = {0x700000 + i * 0x10000: i for i in range(6)}   # F1..F6 จาก cv2.waitKeyEx บน Windows


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
        self.have_all = self.corners is not None and len(self.zones) == 6 and os.path.exists("color_profiles.json")

    # ---------- กล้อง ----------
    def open_cam(self, idx):
        if self.still is not None:                      # โหมดรูปนิ่ง (ทดสอบ)
            return True
        if self.cap is not None:
            self.cap.release()
        self.cap, _ = open_source(str(idx))
        ok = self.cap is not None and self.cap.isOpened()
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
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicks.append((x, y))

    # ---------- เปลี่ยนขั้น ----------
    def goto(self, s):
        s = max(0, min(5, s))
        if s == 2 and self.calib is not None and not self.zones:
            self.auto_zones()
        if s in (4, 5) and self.tracker is None and self.calib is not None:
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
            g, _ = resolve_ambiguous(g)
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
        cv2.setMouseCallback(WIN, self.on_mouse)
        self.open_cam(self.cam)
        self.tracker = None
        self.goto(5)

    # ==================================================================
    #  วาดจอ
    # ==================================================================
    def step_done(self, i):
        return [self.cap is not None and self.cap.isOpened(), self.calib is not None, len(self.zones) == 6,
                os.path.exists("color_profiles.json"), self.pstat == "OK", False][i]

    def draw_stepbar(self, out):
        W = out.shape[1]
        x, y, h = 12, 8, 34
        ui._shade(out, 0, 0, W, h + 16, alpha=0.8)
        for i, name in enumerate(STEPS):
            label = f"{i + 1}. {name}"
            col = "key" if i == self.step else ("ok" if self.step_done(i) else "dim")
            ui._blit_text(out, [(x, y, label, ui.COL[col], 18, i == self.step)])
            w = ui._width(label, 18, True) + 26
            if i == self.step:
                cv2.rectangle(out, (x - 6, y - 4), (x + w - 20, y + 26), ui.COL["key"], 1)
            x += w
        ui._blit_text(out, [(W - 250, y, f"กล้อง {self.cam}   ขั้นที่ {self.step + 1}/6", ui.COL["dim"], 16, False)])

    def draw_loupe(self, out):
        """แว่นขยาย 4x ตรงเมาส์ (ไว้คลิกมุมสนามให้แม่น)"""
        if self.frame is None:
            return
        mx, my = self.mouse
        H, W = self.frame.shape[:2]
        r = 24
        x0, y0 = max(0, mx - r), max(0, my - r)
        crop = self.frame[y0:y0 + 2 * r, x0:x0 + 2 * r]
        if crop.size == 0:
            return
        big = cv2.resize(crop, (2 * r * 4, 2 * r * 4), interpolation=cv2.INTER_NEAREST)
        bh, bw = big.shape[:2]
        cv2.line(big, (bw // 2, 0), (bw // 2, bh), (0, 255, 255), 1)
        cv2.line(big, (0, bh // 2), (bw, bh // 2), (0, 255, 255), 1)
        px, py = W - bw - 16, 60
        out[py:py + bh, px:px + bw] = big
        cv2.rectangle(out, (px, py), (px + bw, py + bh), (255, 255, 255), 1)

    def draw_field(self, out):
        if self.corners:
            pts = np.array(self.corners, np.int32)
            cv2.polylines(out, [pts], True, (0, 255, 255), 2)
            for i, p in enumerate(self.corners):
                cv2.circle(out, tuple(p), 6, (0, 255, 255), -1)
                cv2.putText(out, str(i + 1), (p[0] + 8, p[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    def draw_zones(self, out):
        for name, z in self.zones.items():
            col = DRAW_BGR.get(name, (255, 255, 255))
            cx, cy = int(z["px"][0]), int(z["px"][1])
            cv2.circle(out, (cx, cy), int(z["radius_px"]), col, 2)
            label = f"{COLOR_CLASSES.index(name) + 1} {COLOR_TH.get(name, name)}"
            w = ui._width(label, 18, True)
            ui._shade(out, cx - w // 2 - 6, cy - 12, w + 12, 26, alpha=0.6)
            ui._blit_text(out, [(cx - w // 2, cy - 10, label, (255, 255, 255), 18, True)])

    def draw_gems(self, out):
        for g in self.gems:
            col = DRAW_BGR.get(g["class"], (255, 255, 255))
            cv2.circle(out, tuple(map(int, g["px"])), 9, col, 2)

    def draw_robot(self, out):
        if self.pose is None or self.calib is None or "cm" not in self.pose:
            return
        ax, ay, th = nav.axle_pose(self.pose)
        p0 = self.calib.to_pixel((ax, ay)); p1 = self.calib.to_pixel(nav.point_ahead(ax, ay, th, 20))
        gp = self.calib.to_pixel(nav.gripper_point(ax, ay, th))
        cv2.arrowedLine(out, p0, p1, (0, 255, 0), 3, tipLength=0.3)
        cv2.circle(out, gp, 6, (0, 0, 255), -1)
        if self.goal is not None:
            cv2.circle(out, self.calib.to_pixel(self.goal), 10, (255, 0, 255), 2)

    def color_rows(self):
        rows = []
        for i, c in enumerate(COLOR_CLASSES):
            n = sum(1 for g in self.gems if g["class"] == c)
            k = len(self.samples[c])
            mark = "> " if i == self.sel_color else "   "
            rows.append((f"{mark}{i + 1} {COLOR_TH[c]}", f"เห็น {n} ก้อน" + (f"  (คลิกแล้ว {k})" if k else ""),
                         "ok" if n else ("warn" if k else "dim")))
        return rows

    def panel_for_step(self, out):
        s = self.step
        if s == 0:
            rows = [("กด 1-4 สลับกล้อง จนเห็นภาพสนามทั้งสนาม", None, "text"),
                    ("ตอนนี้", f"กล้อง index {self.cam}" + ("" if self.grab_ok else "  (เปิดไม่ได้!)"),
                     "ok" if self.grab_ok else "bad"), None,
                    ("Enter = ใช้กล้องนี้", None, "dim")]
            title = "1) กล้อง"
        elif s == 1:
            n = len(self.clicks)
            names = ["ซ้ายบน", "ขวาบน", "ขวาล่าง", "ซ้ายล่าง"]
            rows = [(f"คลิกมุมสนาม 4 มุม ตามลำดับ (ดูแว่นขยายมุมขวาบน)", None, "text"),
                    ("คลิกถัดไป", names[n] if n < 4 else "ครบแล้ว", "key" if n < 4 else "ok"),
                    ("ค่าเดิม", "มี — กด Enter ใช้เลยได้" if self.corners and n == 0 else "-", "ok" if self.corners else "dim"),
                    None, ("R = ล้างแล้วคลิกใหม่", None, "dim")]
            title = "2) ครอบสนาม"
        elif s == 2:
            rows = [("โปรแกรมหาวงเอง — เช็คป้ายชื่อบนวงว่าถูกสี", None, "text"),
                    ("วงที่ได้", f"{len(self.zones)}/6", "ok" if len(self.zones) == 6 else "warn"),
                    ("วงไหนหาย/ผิด", "กดเลขสี แล้วคลิกกลางวงนั้น", None),
                    (f"สีที่เลือก", f"{self.sel_color + 1} {COLOR_TH[COLOR_CLASSES[self.sel_color]]}", "key"),
                    None, ("R = หาใหม่   Enter = บันทึกไปขั้นสี", None, "dim")]
            title = "3) วง 6 สี"
        elif s == 3:
            rows = [("กดเลขสี แล้วคลิกหินสีนั้น 2-3 ก้อน (คนละก้อน/คนละมุมสนาม)", None, "text")] + self.color_rows() + \
                   [None, ("M = ดู mask สีที่เลือก   R = ล้างสีที่เลือก   Enter = บันทึก", None, "dim")]
            title = "4) สีหิน (คลิกหิน)"
        elif s == 4:
            seen = self.pstat == "OK" and self.pose is not None and "cm" in self.pose
            pos = f"({self.pose['cm'][0]:.0f}, {self.pose['cm'][1]:.0f}) หัน {nav.axle_pose(self.pose)[2]:.0f}°" if seen else "-"
            rows = [("วางหุ่นในสนาม ลูกศรเขียวต้องชี้ทางหน้าหุ่น จุดแดง = ปาก", None, "text"),
                    ("กล้องเห็นแท็ก", "เห็น" if seen else "ไม่เห็น", "ok" if seen else "bad"),
                    ("ตำแหน่ง", pos, "dim"),
                    ("WiFi ถึงหุ่น", "ต่ออยู่" if (self.link and self.link.alive) else "ขาด",
                     "ok" if (self.link and self.link.alive) else "bad"),
                    ("ทดสอบวิ่ง", "กด 1-6 วิ่งไปวงสีนั้น / SPACE หยุด", "key"),
                    None, ("Enter = ไปหน้าสรุป", None, "dim")]
            title = "5) หุ่น"
        else:
            rows = []
            ok_all = True
            for name, ok, note in self.checklist():
                rows.append((name, ("OK  " if ok else "X   ") + note, "ok" if ok else "bad"))
                ok_all = ok_all and ok
            rows += [None, ("SPACE = เริ่มโหมด AUTO (เปิด auto_main.py)", None, "key" if ok_all else "warn"),
                     ("F1-F5 = กระโดดไปแก้ขั้นที่ยังแดง", None, "dim")]
            title = "พร้อมแข่ง" if ok_all else "ยังไม่พร้อม — ดูรายการสีแดง"
        if self.msg:
            rows += [None, (self.msg, None, "warn")]
        ui.panel(out, 12, 60, rows, title=title, size=17, line_h=25)

    def render(self):
        if self.frame is None:
            out = np.zeros((720, 1280, 3), np.uint8)
        else:
            out = self.frame.copy()
        s = self.step
        if s == 3 and self.show_mask:
            hsv = cv2.cvtColor(cv2.GaussianBlur(self.frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
            m = clean_mask(build_mask(hsv, self.profiles[COLOR_CLASSES[self.sel_color]]), kernel_size=3)
            out = cv2.addWeighted(out, 0.35, cv2.cvtColor(m, cv2.COLOR_GRAY2BGR), 0.65, 0)
        if s >= 1:
            self.draw_field(out)
            if s == 1:
                for i, p in enumerate(self.clicks[:4]):
                    cv2.circle(out, p, 7, (0, 200, 255), -1)
                    cv2.putText(out, str(i + 1), (p[0] + 8, p[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
                if len(self.clicks) >= 2:
                    cv2.polylines(out, [np.array(self.clicks[:4], np.int32)], len(self.clicks) >= 4, (0, 200, 255), 1)
                self.draw_loupe(out)
        if s >= 2:
            self.draw_zones(out)
        if s in (3, 5):
            self.draw_gems(out)
        if s in (4, 5):
            self.draw_robot(out)
        self.draw_stepbar(out)
        self.panel_for_step(out)
        keys = [("Enter", "ถัดไป"), ("Backspace", "ย้อน"), ("R", "ทำใหม่"), ("F1-F6", "ไปขั้น"), ("Q", "ออก")]
        if s == 5:
            keys = [("SPACE", "เริ่ม AUTO")] + keys
        ui.keybar(out, keys)
        return out

    # ==================================================================
    #  คีย์
    # ==================================================================
    def handle_key(self, k):
        if k in (ord('q'), ord('Q'), KEY_ESC):
            return False
        if k in KEY_F:
            self.goto(KEY_F[k]); return True
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
        return True

    # ==================================================================
    def run(self):
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN, 1280, 720)
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
            if self.step in (3, 5):
                self.update_gems()
            if self.step in (4, 5):
                self.update_robot()
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
