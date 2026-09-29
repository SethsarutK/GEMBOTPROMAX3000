"""planner.py — state machine ของรอบ auto

เรียก planner.step(pose, pose_status, gems, now) ทุกเฟรม (20 Hz)
มันจะสั่ง link.drive / link.pick / link.dump เอง และคืนข้อความสถานะไว้แสดงบนจอ

หนึ่งเที่ยว = หินสีเดียว (v2: ไม่มีกระบะแล้ว คีบค้างในปากหนีบไปปล่อยที่โซนโดยตรง)
"""
import math, time
import auto_config as C
import nav


# ---------------------------------------------------------------
class GemTracker:
    """
    กรองหินให้ "นิ่ง" ก่อนเชื่อ: ตำแหน่ง+สีเดิมต้องเห็นติดกัน >= GEM_STABLE_FRAMES
    คืน list ของ dict {"cm","class","area","n"} เฉพาะที่นิ่งแล้ว
    """
    MATCH_CM = 6.0        # v4.0: 3 -> 6 หินโดนก้ามเขี่ยเล็กน้อยยังตามได้

    def __init__(self):
        self.tracks = []          # [{"cm","class","area","n","miss"}]

    def update(self, gems_cm):
        # v4.1: จับคู่ "คู่ที่ใกล้ที่สุดก่อน" ทั้งตาราง (เดิมไล่ทีละ track ตามลำดับสร้าง
        #       -> track เก่าที่หินถูกหยิบไปแล้ว แย่งจุดของเพื่อนบ้านสีเดียวกันใน 6 cm
        #       ทำให้ "หินยังอยู่" ทั้งที่หยิบได้แล้ว และ track เพื่อนบ้านกลายเป็น miss)
        pairs = []
        for ti, t in enumerate(self.tracks):
            for gi, g in enumerate(gems_cm):
                if g["class"] != t["class"]:
                    continue
                d = nav.dist(*t["cm"], *g["cm"])
                if d < self.MATCH_CM:
                    pairs.append((d, ti, gi))
        pairs.sort()
        used = [False] * len(gems_cm)
        matched = [False] * len(self.tracks)
        for d, ti, gi in pairs:
            if used[gi] or matched[ti]:
                continue
            used[gi] = matched[ti] = True
            t, g = self.tracks[ti], gems_cm[gi]
            t["cm"] = g["cm"]; t["area"] = g["area"]
            t["n"] += 1; t["miss"] = 0
        for ti, t in enumerate(self.tracks):
            if not matched[ti]:
                t["miss"] += 1
        for i, g in enumerate(gems_cm):
            if not used[i]:
                self.tracks.append({"cm": g["cm"], "class": g["class"],
                                    "area": g["area"], "n": 1, "miss": 0})
        self.tracks = [t for t in self.tracks if t["miss"] <= 3]

    def stable(self):
        return [t for t in self.tracks if t["n"] >= C.GEM_STABLE_FRAMES]

    def still_there(self, cm, cls, tol=4.0):
        for t in self.tracks:
            if t["class"] == cls and t["miss"] == 0 and nav.dist(*t["cm"], *cm) < tol:
                return True
        return False


# ---------------------------------------------------------------
class Planner:
    def __init__(self, link, zones_cm, zone_radius_cm, log=print):
        """
        zones_cm: {class: (x, y)}   zone_radius_cm: รัศมีวง (cm)
        """
        self.link = link
        self.zones = zones_cm
        self.zr = zone_radius_cm
        self.log = log
        self.tracker = GemTracker()

        self.state = "IDLE"
        self.t_state = time.time()
        self.t_start = None
        self.color = None
        self.target = None          # dict ของหินเป้าหมาย
        self.approach = None        # (x, y)
        self.retry = 0
        self.wiggle = 0             # v4.1: จำนวนครั้งที่ถอยตั้งหลักตอน CREEP (ต่อก้อน)
        self.bin_count = 0
        self.zone_before = 0
        self.delivered = 0
        self.lost_since = None
        self.blacklist = []         # หินที่หยิบไม่ได้ (cm, class)
        self.pick_pos = None
        self.pick_heading = 0.0     # v4.2
        self.turn_dir = 0           # v4.2: ข้างที่หมุนหลังหนีบ (+1/-1), 0 = ยังไม่เลือก
        self.pick_nearby = 0        # v4.1: จำนวนหินสีเป้ารอบจุดหนีบ ตอนสั่ง PICK
        self.contested = 0          # v4.1: ก้อนเป้าปัจจุบันโดน "ก้ามมีก้อนอื่นขวาง" มากี่ครั้ง
        self.contested_at = None
        self.creep_settle = None    # v4.1g: เวลาที่เริ่ม "นิ่งดูซ้ำ" ก่อนหนีบ
        self.zone_filled = set()    # v4.1h: สีของวงที่เราปล่อยหินไปแล้ว (ห้ามขับทับ/หมุนในวงพวกนี้)
        self.pre_pt = None          # v4.1j: จุดก่อนถึง approach (ให้มาถึงแบบหันหาเป้าแล้ว)
        # v4.5
        self.raw_gems = None        # blob หินทุกก้อน "ก่อนตัดตัวหุ่น" (auto_main/sim ใส่ให้) ไว้ดูว่าในปากมีอะไร; None = ไม่มีข้อมูล
        self._pose_hist = []        # (t, ax, ay, th) ไว้ตรวจว่าหุ่นติด (สั่งล้อแล้วไม่ขยับ)
        self.jaw_seen = {}          # นับสีที่เห็นในปากช่วง VERIFY_PICK
        self.jaw_lost_t = None      # เวลาแรกที่ไม่เห็นหินในปากตอนลากไปวง
        self.stall_return = "CHOOSE"
        self.creep_t0_target = None # ตำแหน่งหินเป้าตอนเริ่มคืบ (ไว้จับว่าถูกไถ)
        self._last_cmd = (0, 0)
        self.path = []              # waypoint ที่เหลือ (cm) ตอน GO_APPROACH / GO_ZONE_AP
        self.path_t = 0.0

    # ---------- helpers ----------
    def _go(self, s):
        self.log(f"[{self.elapsed():5.1f}s] {self.state} -> {s}")
        self.state = s
        self.t_state = time.time()
        self.creep_settle = None
        self.blind_t = None
        self._pose_hist = []                       # v4.5: เริ่มนับ stall ใหม่ทุกครั้งที่เปลี่ยน state
        if s == "PICK_BACKOFF":
            self.jaw_seen = {}
        if s in ("GO_ZONE", "GO_ZONE_AP"):
            self.jaw_lost_t = None
        if s == "CREEP" and self.target is not None:
            self.creep_t0_target = tuple(self.target["cm"])

    # ---------- v4.5: หินในปาก / หุ่นติด ----------
    def _jaw_class(self, ax, ay, th):
        """สีของ blob ที่อยู่ตรงจุดปาก (จาก raw_gems ที่ยังไม่ตัดตัวหุ่น) หรือ None ถ้าปากว่าง"""
        gp = nav.gripper_point(ax, ay, th)
        r = getattr(C, "JAW_CHECK_R_CM", 4.5)
        best, bd = None, r
        for g in (self.raw_gems or []):
            d = nav.dist(*g["cm"], *gp)
            if d < bd:
                best, bd = g["class"], d
        return best

    def _stalled(self, ax, ay, th):
        """สั่งล้อ (ไม่ใช่ 0) แล้วตำแหน่ง/ทิศไม่เปลี่ยนนานเกิน STALL_S -> True"""
        now = time.time()
        vl, vr = self._last_cmd
        if vl == 0 and vr == 0:
            self._pose_hist = []
            return False
        self._pose_hist.append((now, ax, ay, th))
        self._pose_hist = [h for h in self._pose_hist if now - h[0] <= getattr(C, "STALL_S", 0.7) + 0.2]
        old = self._pose_hist[0]
        if now - old[0] < getattr(C, "STALL_S", 0.7):
            return False
        moved = nav.dist(old[1], old[2], ax, ay) > 2.0 or abs(nav.norm_deg(th - old[3])) > 8.0
        return not moved

    def elapsed(self):
        return 0.0 if self.t_start is None else time.time() - self.t_start

    def time_left(self):
        return C.RUN_SECONDS - self.elapsed()

    def in_state(self):
        return time.time() - self.t_state

    def _drive(self, vl, vr):
        self._last_cmd = (vl, vr)
        self.link.drive(vl, vr)

    def start(self):
        self.t_start = time.time()
        self.delivered = 0
        self.bin_count = 0
        self.scatter_done = False
        self.scatter_rounds = 0    # v5.6 [req1]: จำนวนรอบที่พุ่งชนกองไปแล้ว
        self.scatter_t0 = None     # เวลาเริ่มพุ่งรอบแรก (คุมงบเวลารวม)
        self.jaw_open = False      # v5.3: จำว่าปากอ้าค้างอยู่ไหม (อ้าไว้ก่อนเข้าหาหินทุกครั้ง)
        self.blind_t = None        # v5.4: เวลาเริ่มเดินตามเป้าที่หายจากกล้อง (blind tracking)
        self.work_queue = None     # v5.8 [req3]: คิวสีที่จะไล่ทำทีละสี (None = ยังไม่ได้วางแผน)
        self.work_color = None     # สีที่กำลังโฟกัสอยู่ (None+คิวหมด = โหมดเก็บตกทุกสี)
        self.delivered_by = {}     # ส่งสำเร็จแล้วกี่ก้อนต่อสี
        self.scatter_pile = None
        self._go("CHOOSE")

    # ---------- v4.6: เปิดเกมพุ่งชนกองให้กระจาย ----------
    def _scatter_wanted(self, gems):
        """v5.6 [req1] Dispersion Loop: พุ่งซ้ำได้เรื่อย ๆ จนกองโล่ง (density < SCATTER_MIN_GEMS)
        หยุดเมื่อ: ครบ SCATTER_MAX_ROUNDS รอบ หรือใช้เวลารวมเกิน T_SCATTER_BUDGET"""
        if not getattr(C, "SCATTER_ENABLED", False):
            return False
        if self.scatter_rounds >= getattr(C, "SCATTER_MAX_ROUNDS", 3):
            return False
        if self.scatter_t0 is not None and time.time() - self.scatter_t0 > getattr(C, "T_SCATTER_BUDGET", 25.0):
            return False
        loose = [g for g in gems if self._in_zone(g["cm"]) is None]
        pile = self._pile_center(loose)
        if pile is None:
            return False
        # รอบแรกดูทั้งกอง (รัศมี 20) รอบถัดไปดูเฉพาะแกนกลาง (12) — กระจายแล้วก็เลิกพุ่ง
        r = 20.0 if self.scatter_rounds == 0 else 12.0
        n = sum(1 for g in loose if nav.dist(*g["cm"], *pile) <= r)
        if n < getattr(C, "SCATTER_MIN_GEMS", 12):
            return False
        self.scatter_pile = pile
        return True

    def stop(self):
        self.link.stop()
        self._go("IDLE")

    def _pile_center(self, gems):
        if not gems:
            return None
        xs = [g["cm"][0] for g in gems]; ys = [g["cm"][1] for g in gems]
        return (sum(xs) / len(xs), sum(ys) / len(ys))

    def _neighbors(self, g, gems):
        return sum(1 for o in gems if o is not g and nav.dist(*g["cm"], *o["cm"]) < C.NEIGHBOR_CM)

    def _in_zone(self, cm):
        for name, z in self.zones.items():
            if nav.dist(*cm, *z) <= self.zr:
                return name
        return None

    def _blacklisted(self, g):
        return any(g["class"] == c and nav.dist(*g["cm"], *cm) < 4 for cm, c in self.blacklist)

    def _count_in_zone(self, gems, color):
        return sum(1 for g in gems if g["class"] == color and self._in_zone(g["cm"]) == color)

    # ---------- v4.0: ตรวจว่าหินอยู่ "ในโซนก้าม" แล้วหรือยัง ----------
    def _in_jaw_zone(self, ax, ay, th, gem_cm):
        """หินอยู่ตามแนวหน้าหุ่นระหว่าง (GRIP_REACH-5) ถึง (GRIP_REACH+2) และเยื้องข้าง <= 3 cm"""
        r = math.radians(th)
        dx, dy = gem_cm[0] - ax, gem_cm[1] - ay
        fwd = dx * math.cos(r) + dy * math.sin(r)          # ระยะตามแนวหน้า
        lat = -dx * math.sin(r) + dy * math.cos(r)         # เยื้องซ้าย/ขวา
        return (C.GRIP_REACH_CM - 5.0) <= fwd <= (C.GRIP_REACH_CM + 2.0) and abs(lat) <= 3.0

    def holding(self):
        """v4.1: มีหินอยู่ในปาก (หรือกำลังหนีบ/ตรวจผล) -> ให้ vision ตัดหินตรงปากออก
        เดิมใช้ bin_count>0 อย่างเดียว ทำให้ตอน VERIFY_PICK กล้องยังเห็นหินที่หนีบติดแล้ว
        และตีความว่า 'หินยังอยู่/ถูกดัน' ทุกครั้ง"""
        return self.bin_count > 0 or self.state in ("PICK", "PICK_BACKOFF", "VERIFY_PICK")

    def _rebind_target(self):
        """v4.1: tracker ลบ track ที่หายเกิน 3 เฟรมทิ้ง แต่ self.target ยังชี้ dict เก่า (miss ค้าง 4, ตำแหน่งแช่แข็ง)
        -> ถ้า target ไม่ใช่ track ที่ยังมีชีวิต ให้หา track สีเดียวกันที่ใกล้ตำแหน่งเดิม (<= 8 cm) มาแทน"""
        if self.target is None:
            return
        tracks = self.tracker.tracks
        if any(t is self.target for t in tracks):
            return
        best, bd = None, 8.0
        for t in tracks:
            if t["class"] != self.target["class"]:
                continue
            d = nav.dist(*t["cm"], *self.target["cm"])
            if d < bd:
                best, bd = t, d
        if best is not None:
            self.target = best

    def _front_blockers(self, ax, ay, th, target, gems):
        """v4.1: หินก้อนอื่น (สีใดก็ได้) ที่อยู่ 'ในแถบก้าม' หน้าหุ่นก่อนถึง/ข้างเป้า
        -> ก้ามจะกวาดก้อนนั้นเข้ามาแทน (sim: หนีบผิดสี 15 ครั้ง/รอบในกองแน่น)"""
        tf, tl = self._rel_to_robot(ax, ay, th, target["cm"])
        out = []
        for g in gems:
            if g is target or (g["class"] == target["class"] and nav.dist(*g["cm"], *target["cm"]) < 1.0):
                continue
            f, l = self._rel_to_robot(ax, ay, th, g["cm"])
            if C.ROBOT_BODY_FRONT_CM < f < tf + 1.0 and abs(l) < 4.0:
                out.append(g)
        return out

    def _sweep_count(self, ax, ay, th, target, gems):
        """v4.4: จำนวนหินก้อนอื่นที่ 'ปลายก้ามที่อ้าอยู่' จะกวาดผ่านตอนคืบจาก (ax,ay) ไปหา target
        แถบกว้าง +-JAW_OPEN_HALF_CM ยาวถึง JAW_TIP_CM + ระยะคืบ (ไม่นับที่อยู่ในช่องหนีบ |lat|<4 ซึ่ง _front_blockers ดูแล้ว)"""
        half = getattr(C, "JAW_OPEN_HALF_CM", 7.0)
        tip = getattr(C, "JAW_TIP_CM", 20.0)
        tf, _ = self._rel_to_robot(ax, ay, th, target["cm"])
        reach = tf - C.GRIP_REACH_CM + tip              # ปลายก้ามไปถึงไหนตอนหินเข้าที่นั่ง
        n = 0
        for g in gems:
            if g is target:
                continue
            f, l = self._rel_to_robot(ax, ay, th, g["cm"])
            if C.ROBOT_BODY_FRONT_CM < f < reach and 4.0 <= abs(l) < half + 2.5:
                n += 1
        return n

    def _pre_point(self, ap, target_cm, back_cm=10.0):
        """v4.1j: จุดก่อนถึงจุดตั้งต้น ถอยจาก ap ไปทางตรงข้ามเป้า back_cm
        -> หุ่นมาถึง ap โดยหันหน้าหาเป้าอยู่แล้ว ALIGN แทบไม่ต้องหมุน (ก้ามอ้าไม่กวาดหิน/วงรอบ ๆ)"""
        dx, dy = target_cm[0] - ap[0], target_cm[1] - ap[1]
        n = math.hypot(dx, dy) or 1.0
        p = (ap[0] - dx / n * back_cm, ap[1] - dy / n * back_cm)
        return (min(max(p[0], 12), 210 - 12), min(max(p[1], 12), 120 - 12))

    def _count_same_near(self, cm, cls, within):
        """นับ track สีเดียวกันที่ยังเห็นอยู่ (miss=0) ในรัศมี within รอบจุด cm"""
        return sum(1 for t in self.tracker.tracks
                   if t["class"] == cls and t["miss"] == 0 and nav.dist(*t["cm"], *cm) <= within)

    def _rel_to_robot(self, ax, ay, th, cm):
        r = math.radians(th)
        dx, dy = cm[0] - ax, cm[1] - ay
        return dx * math.cos(r) + dy * math.sin(r), -dx * math.sin(r) + dy * math.cos(r)

    def _nearest_same(self, cm, cls, within, exclude_cm=None):
        """หาหิน (track ที่ยังเห็นอยู่) สีเดียวกัน ใกล้จุด cm ที่สุดภายในรัศมี within"""
        best, bd = None, within
        for t in self.tracker.tracks:
            if t["class"] != cls or t["miss"] > 0:
                continue
            if exclude_cm is not None and nav.dist(*t["cm"], *exclude_cm) < 4.0:
                continue
            d = nav.dist(*t["cm"], *cm)
            if d < bd:
                best, bd = t, d
        return best

    # ---------- หลบหิน (v3.8) ----------
    def _filled_zones(self, gems, exempt=None):
        """v4.1h: วงที่มีหินอยู่แล้ว (เห็นจากกล้อง หรือเราเคยปล่อยไว้) -> ห้ามขับทับ/หมุนในวง
        v5.0: exempt = ชื่อวงที่ยกเว้น (ตอนตั้งใจเข้าไปหยิบหินผิดสีออกจากวงนั้น)"""
        names = set(self.zone_filled)
        for g in gems:
            z = self._in_zone(g["cm"])
            if z:
                names.add(z)
        names.discard(exempt)
        return [self.zones[n] for n in names if n in self.zones]

    def _pickable_wrong_zone(self, g, gems):
        """v5.0 (external review): หินที่ตกใน "วงผิดสี" หยิบได้ ถ้าวงนั้นยังไม่มีหินสีถูกของมันเอง
        (ถ้ามี = เข้าไปแล้วก้ามเสี่ยงกวาดแต้มที่ได้แล้วออก -> ปล่อยทิ้งไว้เหมือนเดิม)"""
        zn = self._in_zone(g["cm"])
        if zn is None or zn == g["class"]:
            return False
        return not any(o["class"] == zn and self._in_zone(o["cm"]) == zn for o in gems)

    def _keep_out_of_zones(self, pt, gems, exempt=None):
        """v4.1h: ถ้าจุด (approach) อยู่ในวงที่มีหินแล้ว ให้เลื่อนออกไปนอกวง + รัศมีตัวหุ่น"""
        x, y = pt
        for zx, zy in self._filled_zones(gems, exempt):
            # v4.1j: ตอนหมุนตัว (ALIGN) ปลายก้ามที่อ้ากว้างกวาดเป็นวงรัศมี ~GRIP_REACH+7 รอบเพลา
            #        ถ้าเพลาห่างวงแค่ zr+BODY_R ก้ามจะกวาดหินในวงออก (sim: สาเหตุอันดับ 1 ของหินหลุดวง)
            need = self.zr + max(C.ROBOT_BODY_R_CM, C.GRIP_REACH_CM + 2.0)
            d = nav.dist(x, y, zx, zy)
            if d < need:
                ux, uy = ((x - zx) / d, (y - zy) / d) if d > 0.1 else (1.0, 0.0)
                x, y = zx + ux * need, zy + uy * need
        return (min(max(x, 12), 210 - 12), min(max(y, 12), 120 - 12))

    def _obstacles(self, gems, exclude=None):
        """ตำแหน่งหินทุกก้อนที่ต้องหลบ (ยกเว้นก้อนเป้าหมาย) + วงที่มีหินแล้ว (v4.1h)"""
        out = []
        for g in gems:
            if exclude is not None and g["class"] == exclude["class"] \
                    and nav.dist(*g["cm"], *exclude["cm"]) < 4.0:
                continue
            out.append(g["cm"])
        exempt = self._in_zone(exclude["cm"]) if exclude is not None else None
        for zx, zy in self._filled_zones(gems, exempt):
            out.append((zx, zy))
            for k in range(8):                       # จุดรอบขอบวง ให้ A* เลี่ยงทั้งวง
                a = k * math.pi / 4
                out.append((zx + self.zr * math.cos(a), zy + self.zr * math.sin(a)))
        return out

    def _go_via(self, ax, ay, th, goal, gems, exclude=None):
        """
        เหมือน nav.go_to แต่เดินตาม waypoint ที่หลบหิน (คำนวณใหม่ทุก PATH_REPLAN_S)
        return (vl, vr, done)
        """
        if not C.PATH_AVOID:
            return nav.go_to(ax, ay, th, *goal)
        if time.time() - self.path_t > C.PATH_REPLAN_S:
            p = nav.plan_path(ax, ay, goal[0], goal[1], self._obstacles(gems, exclude))
            new = [goal] if p is None else (p if p else [goal])
            # v4.1: hysteresis — เส้นทางใหม่ต้องสั้นกว่าเส้นเดิมที่เหลือชัดเจน (>20%) ถึงจะเปลี่ยน
            #       (เดิม A* สลับอ้อมกองซ้าย/ขวาทุก 1 วิ -> หุ่นหมุนกลับไปกลับมาอยู่กับที่จนหมด 25 วิ)
            def plen(path):
                pts = [(ax, ay)] + list(path)
                return sum(nav.dist(*a, *b) for a, b in zip(pts, pts[1:]))
            if self.path and len(self.path) > 1 and plen(new) >= 0.8 * plen(self.path):
                pass                                    # เดินเส้นเดิมต่อ
            else:
                self.path = new
            self.path_t = time.time()
        # ตัด waypoint กลางทางที่ถึงแล้ว (ใช้ tol หลวม) จุดสุดท้ายใช้ go_to ปกติ
        while len(self.path) > 1 and nav.dist(ax, ay, *self.path[0]) <= C.PATH_WP_TOL_CM:
            self.path.pop(0)
        if len(self.path) > 1:
            vl, vr, _ = nav.go_to(ax, ay, th, *self.path[0])
            return vl, vr, False
        return nav.go_to(ax, ay, th, *goal)

    def _path_reset(self):
        self.path, self.path_t = [], 0.0

    # ---------- v5.8 [req3] ทำทีละสีให้จบ ~80% แล้วค่อยสีถัดไป ----------
    def _work_remaining(self, color, gems):
        return sum(1 for g in gems if g["class"] == color
                   and self._in_zone(g["cm"]) != color and not self._blacklisted(g))

    def _update_work_color(self, gems):
        if not getattr(C, "COLOR_FOCUS", True):
            return
        if len(gems) < 6:
            return          # v5.8b: กล้อง/tracker ยังนับหินไม่ครบ (เช่น เพิ่งล้าง track หลัง scatter) อย่าเพิ่งวางแผน/หมุนคิว
        if self.work_queue is None:                       # วางแผนครั้งแรก (หลัง scatter จบ)
            cnt = {c: 0 for c in self.zones}
            for g in gems:
                if g["class"] in cnt and self._in_zone(g["cm"]) != g["class"]:
                    cnt[g["class"]] += 1
            order = [c for c in C.COLOR_PRIORITY if c in cnt] +                     sorted([c for c in cnt if c not in C.COLOR_PRIORITY], key=lambda c: -cnt[c])
            self.work_queue = [c for c in order if c not in C.SKIP_COLORS]
            self.work_color = self.work_queue.pop(0) if self.work_queue else None
            self.log(f"colour plan: {self.work_color} first, queue={self.work_queue}")
        c = self.work_color
        if c is None:
            return
        rem = self._work_remaining(c, gems)
        done = self.delivered_by.get(c, 0)
        ratio = done / (done + rem) if (done + rem) > 0 else 1.0
        if rem == 0 or ratio >= getattr(C, "COLOR_DONE_RATIO", 0.8):
            if rem > 0:
                self.work_queue.append(c)                 # เหลือเศษ เก็บตกท้ายคิว
            self.work_color = self.work_queue.pop(0) if self.work_queue else None
            self.log(f"colour {c} done {done}/{done+rem} -> next: {self.work_color}")

    # ---------- เลือกหิน ----------
    def _choose(self, ax, ay, gems):
        loose = [g for g in gems if (self._in_zone(g["cm"]) is None or self._pickable_wrong_zone(g, gems))
                 and g["class"] in self.zones
                 and g["class"] not in C.SKIP_COLORS
                 and not self._blacklisted(g)]
        if C.BIG_GEM_ONLY:
            loose = [g for g in loose if g["area"] >= C.BIG_GEM_AREA_PX]
        if not loose:
            return None
        pile = self._pile_center(loose)
        # v4.1: กองแน่นจนทุกก้อนมีเพื่อนบ้านเกิน MAX_NEIGHBORS -> เดิมคืน None แล้วหุ่นยืนนิ่งจนหมดเวลา
        #       ตอนนี้ผ่อนเกณฑ์เป็น "ก้อนที่เพื่อนบ้านน้อยที่สุด" (ขอบกอง) แทน
        want = self.color or self.work_color          # v5.8: ถือของ = สีนั้น, ไม่ถือ = สีที่โฟกัส
        pool = [g for g in loose if not want or g["class"] == want]
        nb = {id(g): self._neighbors(g, loose) for g in pool}
        max_nb = C.MAX_NEIGHBORS
        if pool and min(nb.values()) > max_nb:
            max_nb = min(nb.values())
        cands = []
        for g in pool:                        # v5.8b: pool กรองสี (self.color/work_color) ให้แล้ว
            if nb[id(g)] > max_nb:
                continue
            zx, zy = self.zones[g["class"]]
            cost = nav.dist(ax, ay, *g["cm"]) + 0.7 * nav.dist(*g["cm"], zx, zy)
            # v4.5: ก้อนเดี่ยว ๆ ที่กระจายอยู่ต้องถูกเลือกก่อนกองเสมอ / ก้อนในกองแน่น (เพื่อนบ้าน >= 3) แพงขึ้นมาก
            #       (ของจริง: พุ่งเข้ากลางกองแล้วมอเตอร์ดันไม่ไหว ค้าง)
            if nb[id(g)] == 0:
                cost -= 25
            elif nb[id(g)] >= 3:
                cost += 40 * (nb[id(g)] - 2)
            # "ปอกกองจากขอบนอกเข้าใน": หินที่ไกลจากใจกลางกอง (=ขอบ) ได้ cost ลดลง
            # ทำให้ถูกเลือกก่อนหินที่อยู่ลึกเข้าไปกลางกอง แม้จะอยู่ใกล้หุ่นกว่าก็ตาม
            if pile:
                cost -= C.PILE_EDGE_BIAS * nav.dist(*g["cm"], *pile)
            if C.COLOR_PRIORITY and g["class"] in C.COLOR_PRIORITY:
                cost -= 40 * (len(C.COLOR_PRIORITY) - C.COLOR_PRIORITY.index(g["class"]))
            if g["area"] >= C.BIG_GEM_AREA_PX:
                cost -= 10                    # ก้อนใหญ่จับง่ายกว่า
            # v4.9: เลิกคิดเรื่องก้อนขวาง (ทีมขอ 29 ก.ย.) — เดินตรงเข้าไปหนีบเลย
            ap_g, _ = self._approach_point(ax, ay, g, pile, loose)
            # v4.1j: ระยะที่ต้องเดินจริงคือ "ถึงจุดตั้งต้น" ไม่ใช่ถึงหิน (จุดตั้งต้นอาจอยู่คนละฝั่งกอง = เดินอ้อม 60+ cm)
            cost += nav.dist(ax, ay, *ap_g) - nav.dist(ax, ay, *g["cm"])
            cands.append((cost, g, ap_g))
        if not cands:
            return None
        cands.sort(key=lambda c: c[0])
        return cands[0][1], self._keep_out_of_zones(cands[0][2], gems,
                                                    exempt=self._in_zone(cands[0][1]["cm"]))

    def _approach_point(self, ax, ay, g, pile, gems=None):
        """จุด approach: ถอยจากหินออก "ด้านนอกกอง" ระยะ reach + standoff
        v4.1: ลองหมุนทิศเข้าหา 0, ±30, ±60, ±90° รอบทิศนอกกอง เลือกทิศแรกที่ไม่มีก้อนอื่นขวางแถบก้าม
              คืน (จุด, ยังมีก้อนขวางไหม)"""
        if pile and nav.dist(*g["cm"], *pile) > 1.0:
            dx, dy = g["cm"][0] - pile[0], g["cm"][1] - pile[1]
        else:
            dx, dy = ax - g["cm"][0], ay - g["cm"][1]
        base = math.atan2(dy, dx)
        back = C.GRIP_REACH_CM + C.PILE_STANDOFF_CM
        first = None
        zones_keep = self._filled_zones(gems, exempt=self._in_zone(g["cm"])) if gems is not None else []
        need = self.zr + max(C.ROBOT_BODY_R_CM, C.GRIP_REACH_CM + 2.0)
        best = None
        for off in (0, 30, -30, 60, -60, 90, -90, 120, -120):
            a = base + math.radians(off)
            ap = (g["cm"][0] + math.cos(a) * back, g["cm"][1] + math.sin(a) * back)
            ap = (min(max(ap[0], 12), 210 - 12), min(max(ap[1], 12), 120 - 12))   # อย่าออกนอกสนาม
            if first is None:
                first = ap
            if gems is None:
                return ap, False
            # v4.1j: จุดยืน/แนวหมุนต้องไม่ล้ำวงที่มีหินแล้ว (ก้ามอ้ากวาดหินในวงตอน ALIGN) -> ลองมุมอื่นแทนการดันจุดออก
            if any(nav.dist(*ap, zx, zy) < need for zx, zy in zones_keep):
                continue
            # v4.9: ไม่เช็คก้อนขวาง/ก้ามกวาดแล้ว — เลือกมุมที่เดินถึงเร็วสุด (ปกติ = แนวตรงจากหุ่น)
            score = nav.dist(ax, ay, *ap) + abs(off) / 30.0 * 8.0
            if best is None or score < best[0]:
                best = (score, ap)
        if best is not None:
            return best[1], False
        return first, True

    def _approach_for(self, ax, ay, g):
        """จุดตั้งต้นหน้าหิน g: ถอยจากหินมาทางหุ่น reach + standoff (ใช้ตอนเล็งก้อนที่ถูกดันใหม่)"""
        dx, dy = ax - g["cm"][0], ay - g["cm"][1]
        n = math.hypot(dx, dy) or 1.0
        back = C.GRIP_REACH_CM + C.PILE_STANDOFF_CM
        ap = (g["cm"][0] + dx / n * back, g["cm"][1] + dy / n * back)
        return (min(max(ap[0], 12), 210 - 12), min(max(ap[1], 12), 120 - 12))

    def _zone_approach(self, color, gems, ax=None, ay=None):
        """
        v2: ไม่มีกระบะแล้ว ปากหนีบถือหินค้างมาตลอดทาง -> ต้อง "หันหน้าเข้าหาวงโดยตรง"
        แล้วเดินหน้าเข้าไปปล่อย
        v3.8: ทิศเข้าวง = จาก "ตำแหน่งหุ่นตอนนี้" ไปวง (เดิมใช้ กอง->วง ซึ่งทำให้จุดยืนไปตกกลางกอง
              เมื่อวงอยู่ใกล้กอง) และถ้าจุดยืนยังทับกองอยู่ ให้ตัด standoff ออก
        """
        zx, zy = self.zones[color]
        loose = [g for g in gems if self._in_zone(g["cm"]) is None]
        pile = self._pile_center(loose) or (105, 60)
        pile_r = max([nav.dist(*g["cm"], *pile) for g in loose], default=0.0)
        pile_r = min(pile_r, 30.0)
        ox, oy = (ax, ay) if ax is not None else pile
        dx, dy = zx - ox, zy - oy               # ทิศจากหุ่นไปหาวง (หน้าหุ่นจะชี้ทางนี้)
        base = math.atan2(dy, dx)
        back = self.zr + C.ZONE_STANDOFF_CM + C.GRIP_REACH_CM
        # v4.1j: จุดยืนทับกอง -> เดิมตัด standoff ยืนชิดวง (ก้ามอ้ากวาดหินในวงตอน ZONE_ALIGN)
        #        ตอนนี้หมุนทิศเข้าวง ±30/60/90° หาจุดยืนที่พ้นกองแทน โดยคง standoff เต็ม
        chosen = None
        for off in (0, 30, -30, 60, -60, 90, -90):
            a = base + math.radians(off)
            cand = (zx - math.cos(a) * back, zy - math.sin(a) * back)
            inside = not (12 <= cand[0] <= 210 - 12 and 12 <= cand[1] <= 120 - 12)
            if inside or nav.dist(*cand, *pile) < pile_r + C.ROBOT_BODY_R_CM:
                continue
            chosen = (cand, a)
            break
        if chosen is None:
            a = base
            back = self.zr + C.GRIP_REACH_CM      # ไม่มีทิศไหนพ้นกองเลย -> ยืนชิดขอบวง (เหมือนเดิม)
            chosen = ((zx - math.cos(a) * back, zy - math.sin(a) * back), a)
        ap, a = chosen
        dx, dy = math.cos(a), math.sin(a)
        ap = (min(max(ap[0], 12), 210 - 12), min(max(ap[1], 12), 120 - 12))
        # v4.1h: จุดยืนหน้าวงต้องไม่ไปตกในวงสีอื่นที่มีหินแล้ว
        others = [g for g in gems if self._in_zone(g["cm"]) not in (None, color)]
        saved, self.zone_filled = self.zone_filled, self.zone_filled - {color}
        ap = self._keep_out_of_zones(ap, others)
        self.zone_filled = saved
        place_pt = (zx - dx * self.zr * (1 - C.DUMP_DEPTH_FRAC),
                   zy - dy * self.zr * (1 - C.DUMP_DEPTH_FRAC))
        # v4.1g: จุดปล่อยทับหินที่วางไว้แล้ว -> หินเก่าถูกเขี่ยหลุดวง (sim เห็น 1-2 ครั้ง/รอบ)
        #        เลื่อนจุดปล่อยไปข้าง ๆ (ตั้งฉากทิศเข้า) หาจุดที่ห่างหินในวงทุกก้อน >= 5.5 cm และยังลึกในวง
        in_zone = [g["cm"] for g in gems if self._in_zone(g["cm"]) == color]
        if in_zone:
            px, py = -dy, dx                         # ตั้งฉากกับทิศเข้าวง
            # v4.2 หมายเหตุ: ลองยอมเลื่อนข้าง 5 cm (zr-2.5) แล้ว sim แย่ลง (121 -> 110) จึงคงเงื่อนไข zr-3 ไว้
            #       (ในทางปฏิบัติ = ปล่อยที่จุดกลางแนวเข้าเสมอ)
            for off in (0, 5, -5, 8, -8):
                cand = (place_pt[0] + px * off, place_pt[1] + py * off)
                if nav.dist(*cand, zx, zy) > self.zr - 3.0:
                    continue
                if all(nav.dist(*cand, *g) >= 5.5 for g in in_zone):
                    place_pt = cand
                    break
        return ap, place_pt, math.degrees(math.atan2(dy, dx))

    # ---------- main step ----------
    def step(self, pose, pose_status, gems_cm, now=None):
        """
        pose: dict จาก RobotTracker (ต้องมี "cm","angle_cm_deg") หรือ None
        gems_cm: list [{"cm":(x,y),"class":..,"area":..}]
        return: ข้อความสถานะ
        """
        if gems_cm is not None:          # None = เฟรมนี้ไม่ได้ตรวจหินใหม่ ใช้ track เดิม
            self.tracker.update(gems_cm)
        gems = self.tracker.stable()
        if hasattr(self.link, "tick"):
            self.link.tick()                 # ส่ง PICK/DUMP ซ้ำถ้ายังไม่ได้ ack

        if self.state == "IDLE":
            return "IDLE (press SPACE to start)"

        # ---- หมดเวลา ----
        if self.time_left() <= C.SAFETY_STOP_BEFORE:
            self.link.stop()
            self._go("DONE")
        if self.state == "DONE":
            self.link.stop()
            return f"DONE delivered={self.delivered}"

        # ---- ไม่เห็นหุ่น ----
        if pose is None or pose_status == "LOST" or "cm" not in pose:
            if self.lost_since is None:
                self.lost_since = time.time()
            lost = time.time() - self.lost_since
            if lost > C.POSE_LOST_SEARCH_S:
                self._drive(C.TURN_SIGN * C.V_TURN_MIN, -C.TURN_SIGN * C.V_TURN_MIN)  # หมุนช้า ๆ ให้กล้องเห็น
                return "POSE LOST -> searching (slow turn)"
            if lost > C.POSE_LOST_STOP_S:
                self._drive(0, 0)
            return f"POSE LOST {lost:.1f}s"
        self.lost_since = None

        ax, ay, th = nav.axle_pose(pose)
        st = self.state
        if st in ("GO_APPROACH", "ALIGN", "CREEP", "CREEP_BACK", "BACKOFF"):
            self._rebind_target()            # v4.1: target ต้องเป็น track ที่ยังมีชีวิต

        # ---- v4.5: หุ่นติด (สั่งล้อแล้วไม่ขยับ) -> ถอยแรงแล้วเลือกใหม่ ----
        if st in ("GO_APPROACH", "GO_ZONE_AP", "CREEP", "REVERSE_IN", "BACKOFF", "CREEP_BACK", "BACKOFF_SKIP", "LEAVE") \
                and self.in_state() > 0.8 and self._stalled(ax, ay, th):
            self.log(f"STALL in {st} -> back off hard")
            if st in ("GO_APPROACH", "CREEP", "CREEP_BACK", "BACKOFF") and self.target is not None:
                self.blacklist.append((tuple(self.target["cm"]), self.target["class"]))
            self.stall_return = "GO_ZONE" if self.bin_count > 0 else "CHOOSE"
            self._go("STALL_BACK")
            return "STALL"

        # ---- v4.5: ลากหินไปวงอยู่ แต่กล้องไม่เห็นหินในปากต่อเนื่อง = หินหลุด -> กลับไปหยิบใหม่ ----
        if st in ("GO_ZONE_AP", "ZONE_ALIGN") and self.bin_count > 0 and self.raw_gems is not None:
            if self._jaw_class(ax, ay, th) is None:
                if self.jaw_lost_t is None:
                    self.jaw_lost_t = time.time()
                elif time.time() - self.jaw_lost_t > getattr(C, "JAW_LOST_S", 1.0):
                    self.log("gem dropped on the way (jaw empty) -> re-choose")
                    self.bin_count = 0; self.color = None
                    self._drive(0, 0)
                    self._go("CHOOSE")
                    return "gem lost"
            else:
                self.jaw_lost_t = None

        # ---- รอ sequence ของหุ่น (PICK/DUMP) ----
        if st in ("PICK", "DUMP"):
            wait = C.T_PICK_WAIT if st == "PICK" else C.T_DUMP_WAIT
            if (self.in_state() > 0.5 and not self.link.busy) or self.in_state() > wait:
                if self.in_state() > wait: self.log("sequence timeout")
                self._go("PICK_BACKOFF" if st == "PICK" else "VERIFY_DUMP")
            return f"{st} waiting robot busy={self.link.busy}"

        # =========================================================
        # ---- v4.6: รอบแรกพุ่งผ่ากลางกองด้วยความเร็วสูงให้หินกระจาย แล้วค่อยเริ่มเก็บ ----
        if st == "CHOOSE" and self.bin_count == 0 and self._scatter_wanted(gems):
            self.scatter_rounds += 1
            if self.scatter_t0 is None:
                self.scatter_t0 = time.time()
            self.log(f"SCATTER round {self.scatter_rounds}")
            self.log(f"SCATTER: pile at ({self.scatter_pile[0]:.0f},{self.scatter_pile[1]:.0f}) -> charge!")
            if not self.jaw_open and not self.link.busy:
                self.link.dump(); self.jaw_open = True   # v5.3: อ้าก้ามช่วยปัดหินตอนพุ่ง/หมุน
            self._go("SCATTER_AIM")
            st = "SCATTER_AIM"

        if st == "SCATTER_AIM":
            want = nav.heading_to(ax, ay, *self.scatter_pile)
            vl, vr, done = nav.turn_to_pulsed(ax, ay, th, want, self.in_state())
            if done or self.in_state() > 6.0:
                self._drive(0, 0)
                self._go("SCATTER_RUN")
                return "SCATTER aim ok -> run"
            self._drive(vl, vr)
            return f"SCATTER aim err={nav.norm_deg(want - th):.0f}"

        if st == "SCATTER_RUN":
            v = getattr(C, "V_SCATTER", 75)
            fwd, lat = self._rel_to_robot(ax, ay, th, self.scatter_pile)
            over = getattr(C, "SCATTER_OVER_CM", 12)
            # เดินตรงเข้ากอง แก้ทิศเล็กน้อยระหว่างทาง (ก่อนถึงกอง) ไม่ให้เบี้ยว
            corr = 0.0
            if fwd > 8:
                err = nav.norm_deg(nav.heading_to(ax, ay, *self.scatter_pile) - th)
                corr = max(-15.0, min(15.0, C.TURN_SIGN * err * C.K_TURN * 0.6))
            vl, vr = nav.steer(v, corr)
            passed = fwd < -over
            wall = not (8 <= ax <= nav.FIELD_W_CM - 8 and 8 <= ay <= nav.FIELD_H_CM - 8)
            stalled = self.in_state() > 0.8 and self._stalled(ax, ay, th)
            timeout = self.in_state() > getattr(C, "T_SCATTER_MAX", 4.0)
            if passed or wall or stalled or timeout:
                why = "passed" if passed else "wall" if wall else "stall" if stalled else "timeout"
                self.log(f"SCATTER run end ({why}) fwd={fwd:.0f}")
                self._drive(0, 0)
                # v4.9b: ถึงกองแล้วหมุนตัวเร็ว ๆ ปัดหินให้กระจายก่อนถอย (ทีมขอ 29 ก.ย.)
                self._go("SCATTER_SPIN" if getattr(C, "T_SCATTER_SPIN", 0) > 0 else "SCATTER_BACK")
                return f"SCATTER done ({why})"
            self._drive(vl, vr)
            return f"SCATTER charge fwd={fwd:.0f} lat={lat:.0f}"

        if st == "SCATTER_SPIN":
            v = getattr(C, "V_SCATTER", 75)
            self._drive(C.TURN_SIGN * v, -C.TURN_SIGN * v)   # หมุนอยู่กับที่แรง ๆ ให้ก้าม/ตัวปัดหินออก
            if self.in_state() > getattr(C, "T_SCATTER_SPIN", 1.5):
                self._drive(0, 0)
                self._go("SCATTER_BACK")
                return "SCATTER spin done"
            return "SCATTER spinning"

        if st == "SCATTER_BACK":
            v = getattr(C, "V_STALL_BACK", 45)
            self._drive(-v, -v)                             # ถอยออกจากกองสั้น ๆ ให้กล้องเห็นหินรอบตัวชัด
            if self.in_state() > 1.2:
                self._drive(0, 0)
                self.tracker = GemTracker()                 # ล้าง track เก่า หินย้ายที่หมดแล้ว
                self._go("SCATTER_SCAN")
                return "SCATTER back done -> scan"
            return "SCATTER backing"

        if st == "SCATTER_SCAN":
            # v5.6: นิ่ง ๆ ให้กล้องเห็นหินชุดใหม่ครบก่อน แล้ว CHOOSE จะตัดสินว่ากองยังแน่น (พุ่งอีกรอบ) หรือเริ่มเก็บ
            self._drive(0, 0)
            if self.in_state() > getattr(C, "T_SCATTER_SCAN", 0.6):
                self._go("CHOOSE")
                return "SCATTER scan done"
            return "SCATTER scanning"

        if st == "CHOOSE":
            self._drive(0, 0)
            self._update_work_color(gems)          # v5.8 [req3]
            res = self._choose(ax, ay, gems)
            if res is None:
                if self.bin_count > 0:
                    self._go("GO_ZONE")           # ไม่มีสีเดิมเหลือ ไปส่งของที่มี
                    return "no more of this color -> deliver"
                if self.color:
                    self.color = None             # เปิดให้เลือกสีอื่น
                    return "no gem of chosen color, resetting color"
                return f"no pickable gem (stable={len(gems)})"
            self.target, self.approach = res
            self.color = self.target["class"]
            self.retry = 0
            self.wiggle = 0
            # v4.1: นับ "ก้ามโดนขวาง" ต่อก้อน (ก้อนเดิม = ตำแหน่งเดิมภายใน 4 cm)
            if self.contested_at is None or nav.dist(*self.contested_at, *self.target["cm"]) > 4.0:
                self.contested = 0
            self.contested_at = tuple(self.target["cm"])
            # v5.7 [req4] Gripper FSM: CLOSED_EMPTY -(เลือกเป้า)-> OPEN ค้าง -(PICK ที่หิน)-> CLOSED_HOLD
            #             -(DUMP ที่วง)-> OPEN ค้าง ; jaw_open + ack ของ link กันส่งซ้ำ,
            #             ห้ามยิงคำสั่งใหม่ทับตอนคำสั่งเก่ายังรอ ack (link.busy)
            if not self.jaw_open and not self.link.busy:
                self.link.dump()          # อ้าปากทิ้งไว้ตั้งแต่ตอนนี้ เดินเข้าไปชนหินแล้วค่อยหนีบ
                self.jaw_open = True
            if getattr(C, "SIMPLE_APPROACH", False):
                # v5.2: ไม่หาจุด/ไม่มีจุดก่อนถึง — จุดตั้งต้น = หน้าหินฝั่งที่หุ่นยืนอยู่เท่านั้น
                self.approach = self._approach_for(ax, ay, self.target)
                self.pre_pt = None
            else:
                self.pre_pt = self._pre_point(self.approach, self.target["cm"])   # v4.1j
            self._path_reset()
            self._go("GO_APPROACH")
            return f"target {self.color} at {tuple(round(v) for v in self.target['cm'])}"

        if st == "GO_APPROACH":
            # v4.1j: ไปจุดก่อนถึง (pre_pt) ก่อน แล้วค่อยเดินตรงเข้า approach -> ถึงแล้วหันหาเป้าอยู่แล้ว
            if self.pre_pt is not None:
                if nav.dist(ax, ay, *self.pre_pt) <= C.PATH_WP_TOL_CM:
                    self.pre_pt = None
                    self._path_reset()
                else:
                    vl, vr, _ = self._go_via(ax, ay, th, self.pre_pt, gems, exclude=self.target)
                    self._drive(vl, vr)
                    if self.in_state() > C.T_STATE_TIMEOUT:
                        # v5.0: แบนก้อนนี้ด้วย ไม่งั้น CHOOSE เลือกก้อนเดิม -> วนอีก 25 วิ (external review)
                        self.log("approach timeout -> blacklist + re-choose")
                        if self.target is not None:
                            self.blacklist.append((tuple(self.target["cm"]), self.target["class"]))
                        self._go("CHOOSE")
                    return f"GO_APPROACH(pre) d={nav.dist(ax, ay, *self.pre_pt):.0f}cm"
            vl, vr, done = self._go_via(ax, ay, th, self.approach, gems, exclude=self.target)
            # v5.5 [req2]: ล็อคเป้าใกล้แล้วชะลอ ไม่พุ่งใส่จนหินถลำลึก/หลุดสายตากล้อง
            if self.target is not None and \
                    nav.dist(ax, ay, *self.target["cm"]) < getattr(C, "APPROACH_SLOW_CM", 35.0):
                cap = getattr(C, "V_APPROACH_NEAR", 30)
                m = max(abs(vl), abs(vr))
                if m > cap:
                    vl, vr = nav.floor_wheels(vl * cap / m, vr * cap / m)
            self._drive(vl, vr)
            if done:
                self._go("ALIGN")
            elif self.in_state() > C.T_STATE_TIMEOUT:
                self.log("approach timeout -> blacklist + re-choose")
                if self.target is not None:
                    self.blacklist.append((tuple(self.target["cm"]), self.target["class"]))
                self._go("CHOOSE")
            return f"GO_APPROACH d={nav.dist(ax, ay, *self.approach):.0f}cm"

        if st == "ALIGN":
            want = nav.heading_to(ax, ay, *self.target["cm"])
            vl, vr, done = nav.turn_to_pulsed(ax, ay, th, want, self.in_state())   # v4.1: หมุนเป็นจังหวะ กันเลยมุม
            self._drive(vl, vr)
            if done:
                self._go("CREEP")
            elif self.in_state() > 6:
                self._go("CREEP")
            return f"ALIGN err={nav.norm_deg(want - th):.0f}"

        if st == "CREEP":
            # หินอาจขยับ (ถูกชน) — target เป็น track เดิม ตำแหน่งอัปเดตเองตราบที่กล้องยังตามได้
            vl, vr, done = nav.creep_to(ax, ay, th, *self.target["cm"], C.GRIP_REACH_CM, C.GRAB_TOL_CM)
            # v4.0: หินเข้ามาอยู่ในโซนก้ามแล้ว = พอ ไม่ต้องคืบต่อ (เดิมคืบต่อจนดันหินเด้งออก)
            in_jaw = self._in_jaw_zone(ax, ay, th, self.target["cm"])
            # v4.1: "track หาย" นับเฉพาะตอนตำแหน่งล่าสุดอยู่ใกล้ปากแล้ว (ไม่งั้นสีกะพริบตอนยังไกล = หนีบอากาศ)
            fwd, lat = self._rel_to_robot(ax, ay, th, self.target["cm"])
            near = fwd <= C.GRIP_REACH_CM + 4.0 and abs(lat) <= 5.0
            lost = self.target.get("miss", 0) >= 2 and near   # กล้องไม่เห็นหินแล้ว (มักเพราะเข้าไปอยู่ในก้าม)
            # v4.1i: ตั้งแต่ v4.1b หินในก้ามยัง "มองเห็น" ได้ (ไม่ถูกตัดแล้ว) -> track หายตอนคืบ = หินถูกดันไปไกล/สีกะพริบ
            #        ไม่ใช่ "เข้าก้ามแล้ว" -> ห้ามหนีบอากาศ: ถ้า track ตายและหาก้อนแทนไม่ได้ ให้ถอยแล้วเลือกใหม่
            if self.target.get("miss", 0) >= 3 and not any(t is self.target for t in self.tracker.tracks):
                # v5.1: หายตอนอยู่ใกล้ปาก = โดนแขน/ก้ามบัง -> ปล่อยให้เงื่อนไข lost ข้างล่างพาไปหนีบเลย
                # v5.4: หายตอนไกล -> ยังไม่ถอยทันที เดินต่อไปตำแหน่งล่าสุดอีก T_BLIND ก่อน (เผื่อกล้อง/สีกะพริบ)
                if not near:
                    if self.blind_t is None:
                        self.blind_t = time.time()
                        self.log("target lost (far) -> blind-follow last position")
                    if time.time() - self.blind_t > getattr(C, "T_BLIND", 1.0):
                        self.log("target still lost after blind window -> re-choose (no blacklist)")
                        self._drive(0, 0)
                        self._go("BACKOFF_SKIP")
                        return "CREEP target lost"
            else:
                self.blind_t = None
            # v4.5: หินเป้าถูกดันเลื่อนไปจากจุดเริ่มคืบมาก = เรากำลังไถหินเข้ากอง (ของจริง: ดันจนมอเตอร์ค้าง)
            force_pick = False
            if self.creep_t0_target is not None and \
                    nav.dist(*self.target["cm"], *self.creep_t0_target) > getattr(C, "PUSH_ABORT_CM", 6.0):
                # v5.4 (ของจริง 29 ก.ย.: "ดันหินเสร็จแล้วเปลี่ยนก้อน"): ดันได้ = หินแตะก้ามอยู่แล้ว
                if abs(lat) <= getattr(C, "JAW_OPEN_HALF_CM", 7.0) and \
                        C.ROBOT_BODY_FRONT_CM < fwd <= C.GRIP_REACH_CM + 3.0:
                    self.log("pushing the gem inside the jaws -> grab NOW")
                    force_pick = True
                elif self.wiggle < 2:
                    self.wiggle += 1
                    self.creep_t0_target = tuple(self.target["cm"])   # นับระยะดันใหม่หลังตั้งหลัก
                    self.log(f"pushing but off-jaw (lat={lat:.1f}) -> realign {self.wiggle}")
                    self._drive(0, 0)
                    self._go("CREEP_BACK")
                    return "CREEP push realign"
                else:
                    self.log("pushed away twice -> abort creep, blacklist")
                    self.blacklist.append((tuple(self.target["cm"]), self.target["class"]))
                    self._drive(0, 0)
                    self._go("BACKOFF_SKIP")
                    return "CREEP pushing"
            # v4.1: ใกล้แล้วแต่หินเยื้องข้างเกินก้ามจะกิน -> คืบต่อไปก็แค่ดันหิน ถอยตั้งหลักแล้วหันใหม่ (ไม่เกิน 2 ครั้ง/ก้อน)
            too_deep = C.ROBOT_BODY_FRONT_CM < fwd < C.GRIP_REACH_CM - getattr(C, "DEPTH_MAX_CM", 2.5)
            if (not lost and fwd < C.GRIP_REACH_CM + 6.0
                    and (abs(lat) > 2.8 or too_deep) and self.wiggle < 2):
                self.wiggle += 1
                self.log(f"creep misaligned lat={lat:.1f} deep={too_deep} -> realign {self.wiggle}")
                self._drive(0, 0)
                self._go("CREEP_BACK")
                return "CREEP realign"
            if in_jaw or lost:
                vl = vr = 0
            # v4.1g: "ถึงแล้ว" ต้องนิ่งดูซ้ำ 0.3 วิ (กล้องหน่วง ~0.15 วิ + สั่น) ถ้ายืนยันค่อยหนีบ ไม่งั้นคืบต่อ
            #        (sim: หนีบพลาดเพราะตำแหน่งจริงห่าง 3.1-3.3 ทั้งที่ประเมินว่าถึง)
            if force_pick:
                done = True
                self.creep_settle = time.time() - 1.0     # หินแตะก้ามอยู่แล้ว ไม่ต้องรอนิ่ง
            if (done or in_jaw) and not lost:
                if self.creep_settle is None:
                    self.creep_settle = time.time()
                if time.time() - self.creep_settle < 0.3:
                    self._drive(0, 0)
                    return "CREEP settle"
            else:
                self.creep_settle = None
            self._drive(vl, vr)
            if done or in_jaw or lost or self.in_state() > C.T_CREEP_TIMEOUT:
                why = "reach" if done else ("in jaw" if in_jaw else ("track lost" if lost else "timeout"))
                self._drive(0, 0)
                self.creep_settle = None
                # v4.9: เอาระบบจับหินขวางออกทั้งหมด (ทีมขอ 29 ก.ย. — ของจริงมันเห็นก้อนเป้าเป็นก้อนขวางเอง
                #       แล้ววนเลือกใหม่ไม่หนีบสักที) -> ถึงระยะแล้วหนีบเลย
                self.log(f"creep -> pick ({why})")
                self.pick_pos = tuple(self.target["cm"])   # จำตำแหน่งหินก่อนหนีบ ไว้เช็คหลังถอย
                self.pick_heading = th                     # v4.2: ทิศตอนหนีบ ไว้วัดว่าหมุนพ้น 40° แล้ว
                self.turn_dir = 0                          # v4.2: ให้ PICK_BACKOFF เลือกข้างหมุนใหม่
                # v4.1: จำว่ารอบ ๆ จุดหนีบมีหินสีนี้กี่ก้อน (ก้ามอาจคว้า "ก้อนข้าง ๆ สีเดียวกัน" แทนก้อนเป้า)
                self.pick_nearby = self._count_same_near(self.pick_pos, self.target["class"], 8.0)
                self.link.pick()
                self.jaw_open = False     # v5.3: หนีบแล้วปากปิดค้าง
                self._go("PICK")
            gp = nav.gripper_point(ax, ay, th)
            return f"CREEP gap={nav.dist(*gp, *self.target['cm']):.1f}cm"

        if st == "STALL_BACK":
            # v4.5: ถอยแรงกว่าปกติให้หลุดจากกอง แล้วกลับไปเลือกใหม่ (ถือหินอยู่ -> ไปวงต่อ)
            v = getattr(C, "V_STALL_BACK", 45)
            self._drive(-v, -v)
            if self.in_state() > getattr(C, "T_STALL_BACK", 0.6):
                self._drive(0, 0)
                self._path_reset()
                self._go(self.stall_return)
            return "STALL_BACK"

        if st == "BACKOFF_SKIP":
            # v4.1: ถอยออกจากกอง (ก้ามมีก้อนอื่นขวาง) แล้วเลือกเป้าใหม่ ไม่หนีบ
            self._drive(-C.V_CREEP, -C.V_CREEP)
            if self.in_state() > 1.0:
                self._drive(0, 0)
                self._go("CHOOSE")
            return "BACKOFF_SKIP"

        if st == "CREEP_BACK":
            # v4.1: ถอยสั้น ๆ ให้หินกลับมาอยู่หน้าปากไกลพอ แล้วกลับไป ALIGN หันใหม่
            self._drive(-C.V_CREEP, -C.V_CREEP)
            if self.in_state() > 0.8:
                self._drive(0, 0)
                self._go("ALIGN")
            return "CREEP_BACK"

        if st == "PICK_BACKOFF":
            # v4.2: ของจริง (28 ก.ย.) คีบไม่แน่น ถอยแล้วพื้นดึงหินหลุดจากก้าม
            #       -> ไม่ถอยแล้ว "หมุนอยู่กับที่" ไปทางวงปลายทางแทน อย่างน้อย 40° (ปากเลื่อน ~9 cm
            #          จุดที่หินเคยอยู่จึงพ้นวงที่ถูกซ่อน กล้องตัดสินได้เหมือนเดิม) และได้หันไปทางวงไปในตัว
            turned = abs(nav.norm_deg(th - self.pick_heading))
            zx, zy = self.zones.get(self.color, (105, 60))
            want = nav.heading_to(ax, ay, zx, zy)
            err = nav.norm_deg(want - th)
            if turned >= 40.0 or (turned >= 25.0 and abs(err) <= C.HEADING_DEADBAND_DEG) \
                    or self.in_state() > C.T_PICK_BACKOFF + 3.0:
                self._drive(0, 0)
                self._go("VERIFY_PICK")
                return "PICK_TURN done"
            if self.turn_dir == 0:
                # เลือกข้างที่จะหมุนครั้งเดียวตอนเริ่ม: นับหิน/วงที่มีหิน ที่ปลายก้ามจะกวาดผ่าน (รัศมี ~20 cm, 50°)
                # ข้างที่กวาดน้อยกว่าชนะ เท่ากันเอาข้างที่ไปทางวง
                def sweep_cost(sign):
                    cst = 0.0
                    for g in gems:
                        f, l = self._rel_to_robot(ax, ay, th, g["cm"])
                        rr, ang = math.hypot(f, l), math.degrees(math.atan2(l, f))
                        if 6.0 < rr < C.GRIP_REACH_CM + 8.0 and 0 < sign * ang < 55.0:
                            cst += 1.0
                    for zx, zy in self._filled_zones(gems):
                        f, l = self._rel_to_robot(ax, ay, th, (zx, zy))
                        rr, ang = math.hypot(f, l), math.degrees(math.atan2(l, f))
                        if rr < self.zr + C.GRIP_REACH_CM + 8.0 and 0 < sign * ang < 70.0:
                            cst += 5.0
                    return cst
                pref = 1 if err >= 0 else -1
                cp, cn = sweep_cost(pref), sweep_cost(-pref)
                self.turn_dir = pref if cp <= cn else -pref
            err = self.turn_dir * max(45.0, min(60.0, abs(err)))
            vl, vr = nav.turn_to_pulsed(ax, ay, th, nav.norm_deg(th + err), self.in_state())[:2]
            self._drive(vl, vr)
            return f"PICK_TURN {turned:.0f}deg"

        if st == "VERIFY_PICK":
            self._drive(0, 0)
            # v4.5: "แท็กหินในปาก" — ระหว่างรอ นับสีของ blob ที่อยู่ตรงจุดปากทุกเฟรม
            jc = self._jaw_class(ax, ay, th)
            self.jaw_seen[jc] = self.jaw_seen.get(jc, 0) + 1
            if self.in_state() < C.T_VERIFY:
                return "VERIFY_PICK ..."
            seen = {k: v for k, v in self.jaw_seen.items() if k is not None}
            jaw = max(seen, key=seen.get) if seen and max(seen.values()) >= 2 else None
            if self.raw_gems is None:
                jaw = "?"                      # ไม่มีข้อมูลปาก (ผู้เรียกเก่า) -> ใช้ตรรกะเดิมด้านล่าง
            elif jaw is None:
                # ปากเปล่า -> ไม่นับว่าได้ ไม่ว่าหินเป้าจะหายไปไหน (เดิมหินถูกเขี่ยหลุด = "หาย" = นับว่าได้ -> วิ่งไปวงเปล่า ๆ)
                self.retry += 1
                self.log(f"jaw empty after pick (seen {dict(self.jaw_seen)}), retry {self.retry}")
                if self.retry <= C.PICK_RETRY and self.tracker.still_there(self.pick_pos, self.target["class"], tol=8.0):
                    self.wiggle = 0
                    self._go("BACKOFF")
                else:
                    self.blacklist.append((tuple(self.pick_pos), self.target["class"]))
                    self._go("CHOOSE")
                return "pick FAILED (jaw empty)"
            if jaw != "?" and jaw != self.target["class"] and jaw in self.zones and jaw not in C.SKIP_COLORS:
                # ในปากเป็นสีอื่น (คว้าก้อนข้าง ๆ มา) -> ส่งไปวงของสีนั้นแทน ไม่เสียเที่ยว
                self.log(f"jaw holds {jaw} (target was {self.target['class']}) -> deliver {jaw}")
                self.color = jaw
                self.bin_count += 1
                self._go("GO_ZONE")
                return "pick OK (other colour)"
            if jaw == self.target["class"]:
                self.bin_count += 1
                self.log(f"picked {self.color}  bin={self.bin_count}  (jaw confirmed)")
                self._go("GO_ZONE")
                return "pick OK"
            # v4.1: tol 6 -> 3.5 (ในกองแน่น เพื่อนบ้านสีเดียวกันอยู่ใน 6 cm เสมอ -> เคยตัดสินว่า "ยังอยู่" ทั้งที่หยิบได้แล้ว
            #       แล้วหุ่นถือหินไปหยิบซ้ำ)  หินที่หนีบพลาดจริงจะอยู่ที่เดิมภายใน ~2-3 cm
            still = self.tracker.still_there(self.pick_pos, self.target["class"], tol=3.5)
            # v4.1: "ถูกดัน" = track เดิม (ก้อนเดียวกัน) ยังเห็นอยู่ แต่ขยับไปจากจุดหนีบ และไม่ได้อยู่ในปาก
            #       (v4.0 ใช้ "สีเดียวกันใน 18 cm" ซึ่งในกองหินเป็นจริงเสมอ -> ไม่เคยนับว่าหนีบได้เลย)
            pushed = None
            if not still and self.target.get("miss", 0) == 0 \
                    and nav.dist(*self.target["cm"], *self.pick_pos) > 6.0 \
                    and not self._in_jaw_zone(ax, ay, th, self.target["cm"]):
                pushed = self.target
            if still and self._count_same_near(self.pick_pos, self.target["class"], 8.0) < self.pick_nearby:
                # v4.1: ก้อนเป้ายังอยู่ แต่หินสีเดียวกันรอบ ๆ หายไป 1 ก้อน = ก้ามคว้าก้อนข้าง ๆ มาแทน (สีถูกอยู่ดี)
                #       ถ้าลองใหม่ PICK จะเปิดปากทำหินหล่นแล้ววนซ้ำ 3 รอบ -> นับว่าได้ แล้วไปส่ง
                self.log("target still there but a same-colour neighbour vanished -> assume grabbed neighbour")
                still = False
            if still or pushed is not None:
                self.retry += 1
                if pushed is not None:
                    # v4.0: หินไม่ได้หาย แค่ถูกดันไปข้าง ๆ -> เล็งก้อนที่ตำแหน่งใหม่ ไม่นับว่าหนีบได้
                    self.log(f"gem pushed to {tuple(round(v) for v in pushed['cm'])}, retry {self.retry}")
                    self.target = pushed
                    self.approach = self._keep_out_of_zones(self._approach_for(ax, ay, pushed), gems)
                else:
                    self.log(f"gem still there, retry {self.retry}")
                if self.retry <= C.PICK_RETRY:
                    self.wiggle = 0
                    self._go("BACKOFF")
                else:
                    self.log("give up this gem")
                    self.blacklist.append((tuple(self.target["cm"]), self.target["class"]))
                    self._go("CHOOSE")
                return "pick FAILED"
            self.bin_count += 1
            self.log(f"picked {self.color}  bin={self.bin_count}")
            if self.bin_count >= C.MAX_PER_TRIP or self.time_left() < 60:
                self._go("GO_ZONE")
            else:
                self._go("CHOOSE")
            return "pick OK"

        if st == "BACKOFF":
            self._drive(-C.V_CREEP, -C.V_CREEP)
            if self.in_state() > 1.0:
                self._go("GO_APPROACH")
            return "BACKOFF"

        if st == "GO_ZONE":
            ap, self.dump_pt, self.zone_heading = self._zone_approach(self.color, gems, ax, ay)
            self.zone_before = self._count_in_zone(gems, self.color)
            self.zone_ap = ap
            self.pre_pt = self._pre_point(ap, self.zones[self.color])          # v4.1j
            self._path_reset()
            self._go("GO_ZONE_AP")
            return "GO_ZONE"

        if st == "GO_ZONE_AP":
            if self.pre_pt is not None:                                         # v4.1j
                if nav.dist(ax, ay, *self.pre_pt) <= C.PATH_WP_TOL_CM:
                    self.pre_pt = None
                    self._path_reset()
                else:
                    vl, vr, _ = self._go_via(ax, ay, th, self.pre_pt, gems)
                    self._drive(vl, vr)
                    if self.in_state() > C.T_STATE_TIMEOUT:
                        self._go("ZONE_ALIGN")
                    return f"GO_ZONE_AP(pre) d={nav.dist(ax, ay, *self.pre_pt):.0f}cm"
            vl, vr, done = self._go_via(ax, ay, th, self.zone_ap, gems)
            self._drive(vl, vr)
            if done:
                self._go("ZONE_ALIGN")
            elif self.in_state() > C.T_STATE_TIMEOUT:
                self._go("ZONE_ALIGN")        # ใกล้พอแล้วก็ลอง
            return f"GO_ZONE_AP d={nav.dist(ax, ay, *self.zone_ap):.0f}cm"

        if st == "ZONE_ALIGN":
            # v2: หันหน้า (ปากหนีบ) เข้าหาวงโดยตรง ไม่มีกระบะให้ต้องหมุนกลับตัวแล้ว
            vl, vr, done = nav.turn_to_pulsed(ax, ay, th, self.zone_heading, self.in_state())
            self._drive(vl, vr)
            if done or self.in_state() > 6:
                self._go("REVERSE_IN")
            return "ZONE_ALIGN"

        if st == "REVERSE_IN":
            # v2: ชื่อ state เดิม (ไม่เปลี่ยนเพื่อลดจุดแก้) แต่ตอนนี้ "เดินหน้า" เข้าวง
            # ด้วยปากหนีบ ไม่ใช่ถอยด้วยท้ายกระบะเหมือนก่อน
            vl, vr, done = nav.creep_to(ax, ay, th, *self.dump_pt, C.GRIP_REACH_CM, C.GRAB_TOL_CM)
            self._drive(vl, vr)
            if done or self.in_state() > C.T_CREEP_TIMEOUT * 2:      # v4.1: ทางเข้าวงยาวกว่าทางเข้าหิน (~15-20 cm)
                self._drive(0, 0)
                self.link.dump()
                self.jaw_open = True      # v5.3: DUMP ค้างเปิด -> ก้อนถัดไปไม่ต้องสั่งอ้าซ้ำ
                self._go("DUMP")
            gp = nav.gripper_point(ax, ay, th)
            return f"REVERSE_IN gap={nav.dist(*gp, *self.dump_pt):.1f}cm"

        if st == "VERIFY_DUMP":
            # v3: หินที่เพิ่งปล่อยยังอยู่หน้าปากหนีบ (ในรัศมีตัดตัวหุ่น) นับตอนนี้จะไม่เจอ
            # -> ถอยออกก่อน (LEAVE) แล้วค่อยนับใน COUNT_DUMP
            self._go("LEAVE")
            return "released -> leaving"

        if st == "LEAVE":
            # หน้าหุ่นชี้เข้าวงอยู่ตอนปล่อย -> ถอยออก ให้กล้องเห็นหินที่วาง
            self._drive(-C.V_CREEP, -C.V_CREEP)
            if self.in_state() > C.T_LEAVE:
                self._drive(0, 0)
                self._go("COUNT_DUMP")
            return "LEAVE"

        if st == "COUNT_DUMP":
            self._drive(0, 0)
            if self.in_state() < C.T_VERIFY:
                return "COUNT_DUMP ..."
            after = self._count_in_zone(gems, self.color)
            got = max(0, after - self.zone_before)
            self.delivered += got
            if got > 0:
                self.delivered_by[self.color] = self.delivered_by.get(self.color, 0) + got
            self.log(f"placed {self.color}: zone {self.zone_before}->{after}  total={self.delivered}")
            self.zone_filled.add(self.color)         # v4.1h: วงนี้มีหินแล้ว
            self.bin_count = 0
            self.color = None
            self._go("CHOOSE")
            return "counted"

        return st
