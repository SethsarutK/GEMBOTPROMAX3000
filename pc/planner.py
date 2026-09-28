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
        self.pick_nearby = 0        # v4.1: จำนวนหินสีเป้ารอบจุดหนีบ ตอนสั่ง PICK
        self.contested = 0          # v4.1: ก้อนเป้าปัจจุบันโดน "ก้ามมีก้อนอื่นขวาง" มากี่ครั้ง
        self.contested_at = None
        self._last_cmd = (0, 0)
        self.path = []              # waypoint ที่เหลือ (cm) ตอน GO_APPROACH / GO_ZONE_AP
        self.path_t = 0.0

    # ---------- helpers ----------
    def _go(self, s):
        self.log(f"[{self.elapsed():5.1f}s] {self.state} -> {s}")
        self.state = s
        self.t_state = time.time()

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
        self._go("CHOOSE")

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
    def _obstacles(self, gems, exclude=None):
        """ตำแหน่งหินทุกก้อนที่ต้องหลบ (ยกเว้นก้อนเป้าหมาย)"""
        out = []
        for g in gems:
            if exclude is not None and g["class"] == exclude["class"] \
                    and nav.dist(*g["cm"], *exclude["cm"]) < 4.0:
                continue
            out.append(g["cm"])
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

    # ---------- เลือกหิน ----------
    def _choose(self, ax, ay, gems):
        loose = [g for g in gems if self._in_zone(g["cm"]) is None
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
        pool = [g for g in loose if not self.color or g["class"] == self.color]
        nb = {id(g): self._neighbors(g, loose) for g in pool}
        max_nb = C.MAX_NEIGHBORS
        if pool and min(nb.values()) > max_nb:
            max_nb = min(nb.values())
        cands = []
        for g in loose:
            if self.color and g["class"] != self.color:
                continue                      # กระบะมีของสี self.color อยู่ ต้องสีเดิม
            if nb[id(g)] > max_nb:
                continue
            zx, zy = self.zones[g["class"]]
            cost = nav.dist(ax, ay, *g["cm"]) + 0.7 * nav.dist(*g["cm"], zx, zy)
            # "ปอกกองจากขอบนอกเข้าใน": หินที่ไกลจากใจกลางกอง (=ขอบ) ได้ cost ลดลง
            # ทำให้ถูกเลือกก่อนหินที่อยู่ลึกเข้าไปกลางกอง แม้จะอยู่ใกล้หุ่นกว่าก็ตาม
            if pile:
                cost -= C.PILE_EDGE_BIAS * nav.dist(*g["cm"], *pile)
            if C.COLOR_PRIORITY and g["class"] in C.COLOR_PRIORITY:
                cost -= 40 * (len(C.COLOR_PRIORITY) - C.COLOR_PRIORITY.index(g["class"]))
            if g["area"] >= C.BIG_GEM_AREA_PX:
                cost -= 10                    # ก้อนใหญ่จับง่ายกว่า
            # v4.1: มีหินก้อนอื่นขวางในแถบก้ามระหว่างจุดตั้งต้นกับเป้า -> ก้ามจะคว้าก้อนนั้นแทน (ผิดสี)
            #       ให้ cost แพงมาก (ยังเลือกได้ถ้าไม่มีทางเลือกอื่น)
            ap_g = self._approach_point(ax, ay, g, pile)
            th_g = nav.heading_to(*ap_g, *g["cm"])
            if self._front_blockers(ap_g[0], ap_g[1], th_g, g, loose):
                cost += 200
            cands.append((cost, g))
        if not cands:
            return None
        cands.sort(key=lambda c: c[0])
        g = cands[0][1]
        return g, self._approach_point(ax, ay, g, pile)

    def _approach_point(self, ax, ay, g, pile):
        # จุด approach: ถอยจากหินออก "ด้านนอกกอง" ระยะ reach + standoff
        if pile and nav.dist(*g["cm"], *pile) > 1.0:
            dx, dy = g["cm"][0] - pile[0], g["cm"][1] - pile[1]
        else:
            dx, dy = ax - g["cm"][0], ay - g["cm"][1]
        n = math.hypot(dx, dy) or 1.0
        dx, dy = dx / n, dy / n
        back = C.GRIP_REACH_CM + C.PILE_STANDOFF_CM
        ap = (g["cm"][0] + dx * back, g["cm"][1] + dy * back)
        # อย่าให้จุด approach ออกนอกสนาม
        return (min(max(ap[0], 12), 210 - 12), min(max(ap[1], 12), 120 - 12))

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
        n = math.hypot(dx, dy) or 1.0
        dx, dy = dx / n, dy / n
        back = self.zr + C.ZONE_STANDOFF_CM + C.GRIP_REACH_CM
        ap = (zx - dx * back, zy - dy * back)
        if nav.dist(*ap, *pile) < pile_r + C.ROBOT_BODY_R_CM:
            back = self.zr + C.GRIP_REACH_CM      # จุดยืนทับกอง -> ยืนชิดขอบวงเลย
            ap = (zx - dx * back, zy - dy * back)
        ap = (min(max(ap[0], 12), 210 - 12), min(max(ap[1], 12), 120 - 12))
        place_pt = (zx - dx * self.zr * (1 - C.DUMP_DEPTH_FRAC),
                   zy - dy * self.zr * (1 - C.DUMP_DEPTH_FRAC))
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

        # ---- รอ sequence ของหุ่น (PICK/DUMP) ----
        if st in ("PICK", "DUMP"):
            wait = C.T_PICK_WAIT if st == "PICK" else C.T_DUMP_WAIT
            if (self.in_state() > 0.5 and not self.link.busy) or self.in_state() > wait:
                if self.in_state() > wait: self.log("sequence timeout")
                self._go("PICK_BACKOFF" if st == "PICK" else "VERIFY_DUMP")
            return f"{st} waiting robot busy={self.link.busy}"

        # =========================================================
        if st == "CHOOSE":
            self._drive(0, 0)
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
            self._path_reset()
            self._go("GO_APPROACH")
            return f"target {self.color} at {tuple(round(v) for v in self.target['cm'])}"

        if st == "GO_APPROACH":
            vl, vr, done = self._go_via(ax, ay, th, self.approach, gems, exclude=self.target)
            self._drive(vl, vr)
            if done:
                self._go("ALIGN")
            elif self.in_state() > C.T_STATE_TIMEOUT:
                self.log("approach timeout -> re-choose")
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
            # v4.1: ใกล้แล้วแต่หินเยื้องข้างเกินก้ามจะกิน -> คืบต่อไปก็แค่ดันหิน ถอยตั้งหลักแล้วหันใหม่ (ไม่เกิน 2 ครั้ง/ก้อน)
            if (not in_jaw and not lost and not done and fwd < C.GRIP_REACH_CM + 6.0
                    and abs(lat) > 3.5 and self.wiggle < 2):
                self.wiggle += 1
                self.log(f"creep misaligned lat={lat:.1f} -> realign {self.wiggle}")
                self._drive(0, 0)
                self._go("CREEP_BACK")
                return "CREEP realign"
            if in_jaw or lost:
                vl = vr = 0
            self._drive(vl, vr)
            if done or in_jaw or lost or self.in_state() > C.T_CREEP_TIMEOUT:
                why = "reach" if done else ("in jaw" if in_jaw else ("track lost" if lost else "timeout"))
                self._drive(0, 0)
                # v4.1: มีหินก้อนอื่นอยู่ในแถบก้ามด้วย -> หนีบไปก็ได้ก้อนผิด (ผิดสี) ถอยออกแล้วเลือกใหม่
                #       (ครั้งที่ 2 ของก้อนเดิม -> blacklist ไปเลย)
                blockers = self._front_blockers(ax, ay, th, self.target, gems)
                if blockers and not lost:
                    self.contested += 1
                    self.log(f"jaw contested by {len(blockers)} other gem(s) -> skip target ({self.contested})")
                    if self.contested >= 2:
                        self.blacklist.append((tuple(self.target["cm"]), self.target["class"]))
                    self._go("BACKOFF_SKIP")
                    return "CREEP contested"
                self.log(f"creep -> pick ({why})")
                self.pick_pos = tuple(self.target["cm"])   # จำตำแหน่งหินก่อนหนีบ ไว้เช็คหลังถอย
                # v4.1: จำว่ารอบ ๆ จุดหนีบมีหินสีนี้กี่ก้อน (ก้ามอาจคว้า "ก้อนข้าง ๆ สีเดียวกัน" แทนก้อนเป้า)
                self.pick_nearby = self._count_same_near(self.pick_pos, self.target["class"], 10.0)
                self.link.pick()
                self._go("PICK")
            gp = nav.gripper_point(ax, ay, th)
            return f"CREEP gap={nav.dist(*gp, *self.target['cm']):.1f}cm"

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
            # ถอยออกให้จุดที่หินเคยอยู่พ้นบริเวณตัดตัวหุ่น -> กล้องตัดสินได้ว่าหินติดมาหรือยังอยู่ที่พื้น
            self._drive(-C.V_CREEP, -C.V_CREEP)
            if self.in_state() > C.T_PICK_BACKOFF:
                self._drive(0, 0)
                self._go("VERIFY_PICK")
            return "PICK_BACKOFF"

        if st == "VERIFY_PICK":
            self._drive(0, 0)
            if self.in_state() < C.T_VERIFY:
                return "VERIFY_PICK ..."
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
            if still and self._count_same_near(self.pick_pos, self.target["class"], 10.0) < self.pick_nearby:
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
                    self.approach = self._approach_for(ax, ay, pushed)
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
            self._path_reset()
            self._go("GO_ZONE_AP")
            return "GO_ZONE"

        if st == "GO_ZONE_AP":
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
            self.log(f"placed {self.color}: zone {self.zone_before}->{after}  total={self.delivered}")
            self.bin_count = 0
            self.color = None
            self._go("CHOOSE")
            return "counted"

        return st
