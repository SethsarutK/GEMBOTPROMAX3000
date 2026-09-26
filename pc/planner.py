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
    MATCH_CM = 3.0

    def __init__(self):
        self.tracks = []          # [{"cm","class","area","n","miss"}]

    def update(self, gems_cm):
        used = [False] * len(gems_cm)
        for t in self.tracks:
            best, bd = -1, self.MATCH_CM
            for i, g in enumerate(gems_cm):
                if used[i] or g["class"] != t["class"]:
                    continue
                d = nav.dist(*t["cm"], *g["cm"])
                if d < bd:
                    best, bd = i, d
            if best >= 0:
                used[best] = True
                g = gems_cm[best]
                t["cm"] = g["cm"]; t["area"] = g["area"]
                t["n"] += 1; t["miss"] = 0
            else:
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
        self.bin_count = 0
        self.zone_before = 0
        self.delivered = 0
        self.lost_since = None
        self.blacklist = []         # หินที่หยิบไม่ได้ (cm, class)
        self.pick_pos = None
        self._last_cmd = (0, 0)

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
        cands = []
        for g in loose:
            if self.color and g["class"] != self.color:
                continue                      # กระบะมีของสี self.color อยู่ ต้องสีเดิม
            if self._neighbors(g, loose) > C.MAX_NEIGHBORS:
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
            cands.append((cost, g))
        if not cands:
            return None
        cands.sort(key=lambda c: c[0])
        g = cands[0][1]
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
        ap = (min(max(ap[0], 12), 210 - 12), min(max(ap[1], 12), 120 - 12))
        return g, ap

    def _zone_approach(self, color, gems):
        """
        v2: ไม่มีกระบะแล้ว ปากหนีบถือหินค้างมาตลอดทาง -> ต้อง "หันหน้าเข้าหาวงโดยตรง"
        แล้วเดินหน้าเข้าไปปล่อย (ตรงข้ามกับของเดิมที่หมุนกลับตัวแล้วถอยเข้าด้วยท้ายกระบะ)
        จุดยืน (approach) อยู่ฝั่งเดียวกับกอง ห่างจากวงตามระยะ standoff + ระยะเอื้อมปากหนีบ
        """
        zx, zy = self.zones[color]
        pile = self._pile_center([g for g in gems if self._in_zone(g["cm"]) is None]) or (105, 60)
        dx, dy = zx - pile[0], zy - pile[1]      # ทิศจาก "กอง" ไปหา "วง" (หน้าหุ่นจะชี้ทางนี้)
        n = math.hypot(dx, dy) or 1.0
        dx, dy = dx / n, dy / n
        back = self.zr + C.ZONE_STANDOFF_CM + C.GRIP_REACH_CM
        ap = (zx - dx * back, zy - dy * back)     # ยืนฝั่งกอง หันหน้าเข้าวง
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
            self._go("GO_APPROACH")
            return f"target {self.color} at {tuple(round(v) for v in self.target['cm'])}"

        if st == "GO_APPROACH":
            vl, vr, done = nav.go_to(ax, ay, th, *self.approach)
            self._drive(vl, vr)
            if done:
                self._go("ALIGN")
            elif self.in_state() > C.T_STATE_TIMEOUT:
                self.log("approach timeout -> re-choose")
                self._go("CHOOSE")
            return f"GO_APPROACH d={nav.dist(ax, ay, *self.approach):.0f}cm"

        if st == "ALIGN":
            want = nav.heading_to(ax, ay, *self.target["cm"])
            vl, vr, done = nav.turn_to(ax, ay, th, want)
            self._drive(vl, vr)
            if done:
                self._go("CREEP")
            elif self.in_state() > 6:
                self._go("CREEP")
            return f"ALIGN err={nav.norm_deg(want - th):.0f}"

        if st == "CREEP":
            # หินอาจขยับ (ถูกชน) — ตามตำแหน่งล่าสุดของ track เดิม
            vl, vr, done = nav.creep_to(ax, ay, th, *self.target["cm"], C.GRIP_REACH_CM, C.GRAB_TOL_CM)
            self._drive(vl, vr)
            if done or self.in_state() > C.T_CREEP_TIMEOUT:
                if not done: self.log("creep timeout -> pick anyway")
                self._drive(0, 0)
                self.pick_pos = self.target["cm"]      # จำตำแหน่งหินก่อนหนีบ ไว้เช็คหลังถอย
                self.link.pick()
                self._go("PICK")
            gp = nav.gripper_point(ax, ay, th)
            return f"CREEP gap={nav.dist(*gp, *self.target['cm']):.1f}cm"

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
            if self.tracker.still_there(self.pick_pos, self.target["class"], tol=6.0):
                self.retry += 1
                if self.retry <= C.PICK_RETRY:
                    self.log(f"gem still there, retry {self.retry}")
                    self._go("BACKOFF")
                else:
                    self.log("give up this gem")
                    self.blacklist.append((self.target["cm"], self.target["class"]))
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
            ap, self.dump_pt, self.zone_heading = self._zone_approach(self.color, gems)
            self.zone_before = self._count_in_zone(gems, self.color)
            self.zone_ap = ap
            self._go("GO_ZONE_AP")
            return "GO_ZONE"

        if st == "GO_ZONE_AP":
            vl, vr, done = nav.go_to(ax, ay, th, *self.zone_ap)
            self._drive(vl, vr)
            if done:
                self._go("ZONE_ALIGN")
            elif self.in_state() > C.T_STATE_TIMEOUT:
                self._go("ZONE_ALIGN")        # ใกล้พอแล้วก็ลอง
            return f"GO_ZONE_AP d={nav.dist(ax, ay, *self.zone_ap):.0f}cm"

        if st == "ZONE_ALIGN":
            # v2: หันหน้า (ปากหนีบ) เข้าหาวงโดยตรง ไม่มีกระบะให้ต้องหมุนกลับตัวแล้ว
            vl, vr, done = nav.turn_to(ax, ay, th, self.zone_heading)
            self._drive(vl, vr)
            if done or self.in_state() > 6:
                self._go("REVERSE_IN")
            return "ZONE_ALIGN"

        if st == "REVERSE_IN":
            # v2: ชื่อ state เดิม (ไม่เปลี่ยนเพื่อลดจุดแก้) แต่ตอนนี้ "เดินหน้า" เข้าวง
            # ด้วยปากหนีบ ไม่ใช่ถอยด้วยท้ายกระบะเหมือนก่อน
            vl, vr, done = nav.creep_to(ax, ay, th, *self.dump_pt, C.GRIP_REACH_CM, C.GRAB_TOL_CM)
            self._drive(vl, vr)
            if done or self.in_state() > C.T_CREEP_TIMEOUT:
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
