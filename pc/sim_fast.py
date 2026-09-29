"""sim_fast.py — sim แบบ "เวลาเสมือน" + โลกที่ใกล้ของจริงกว่า SimWorld ใน auto_main

ต่างจาก auto_main --sim:
  * นาฬิกาเสมือน: planner/link/world ใช้เวลาเดียวกัน -> รอบ 300 วิ จบใน ~10 วิจริง
    (auto_main --sim --speed N ทำให้ timeout ใน planner ใจดีเกินจริง เพราะใช้นาฬิกาจริง)
  * ของจริงที่ sim เดิมไม่มี:
      - dead zone มอเตอร์ (เดินหน้าเริ่มที่ 15, หมุนอยู่กับที่เริ่มที่ 45)
      - ล้อขวาช้ากว่าซ้าย (เป๋)
      - กล้อง/WiFi หน่วง ~150 ms, ตำแหน่งสั่น +-1 cm / +-2 องศา
      - ตรวจหินทุก 3 เฟรม (เหมือน run_real)
      - หินที่หนีบอยู่ "ยังถูกกล้องเห็น" ที่ปาก (ผ่าน filter_robot_blobs เหมือน run_real)
      - ปาก/ตัวหุ่นดันหินได้ (ฟิสิกส์ง่าย ๆ)
      - servo ใช้เวลา ~1.2 วิ

รัน:  python sim_fast.py [--seed 3] [--runs 3] [--ideal]   (--ideal = ปิดความจริงทั้งหมด)
"""
import argparse, math, random, time as _time

import auto_config as C
import nav
import auto_main
from planner import Planner


# ---------------- นาฬิกาเสมือน ----------------
class VClock:
    def __init__(self):
        self.t = 1000.0
    def time(self):
        return self.t
    def sleep(self, s):
        self.t += max(0.0, s)


CLOCK = VClock()
_time.time = CLOCK.time          # planner / link / auto_main.SimLink ล้วน import time แล้วเรียก time.time()
_time.sleep = CLOCK.sleep


# ---------------- โลกที่ใกล้ของจริง ----------------
class RealWorld(auto_main.SimWorld):
    DEAD_FWD = 19       # ต่ำกว่านี้ล้อไม่หมุน (28 ก.ย. บ่าย: 18 ไม่ออกตัว -> ทีมปรับเดิน 20 / เลี้ยว 55)
    DEAD_SPIN = 52      # หมุนอยู่กับที่ ต้องอย่างน้อยเท่านี้
    R_GAIN = 0.90       # ล้อขวาช้ากว่า 10%
    LATENCY_S = 0.15
    POS_NOISE = 1.0
    ANG_NOISE = 2.0
    GEM_R = 2.5

    def __init__(self, zones, ideal=False, seed=3):
        super().__init__(zones)
        random.seed(seed)
        self.ideal = ideal
        self.hist = []            # (t, pose) สำหรับ latency
        self.gems = []
        # หินกว้าง ~5 cm วางซ้อนกันไม่ได้ -> ศูนย์กลางห่างกันอย่างน้อย 2*GEM_R (กอง 54 ก้อนจึงกว้าง ~r 20)
        for c in auto_main.COLOR_CLASSES:
            for _ in range(9):
                for _try in range(200):
                    a = random.uniform(0, 2 * math.pi); r = random.uniform(0, 20)
                    p = (105 + r * math.cos(a), 60 + r * math.sin(a))
                    if all(nav.dist(*p, *g["cm"]) >= 2 * self.GEM_R for g in self.gems):
                        break
                big = random.random() < 0.5
                self.gems.append({"cm": p, "class": c, "area": 600 if big else 300})
        self.pushes = 0
        self.last_pick_d = 99
        self.hit_cm = {"body": 0.0, "seat": 0.0, "jaw": 0.0}     # ระยะรวมที่หินถูกดัน แยกตามส่วนที่ชน
        self.hit_ids = {"body": set(), "seat": set(), "jaw": set()}
        self.zone_exits = 0                                       # หิน (สีใดก็ได้) ถูกดันจนหลุดออกจากวง
        self.zone_exit_by = {}                                    # แยกตาม state ของ planner / ส่วนที่ชน
        self.hit_by = {}                                          # cm ที่หินถูกดัน แยกตาม state/ส่วนที่ชน
        self.cur_state = "?"
        self.slips = 0                                            # หินหลุดจากปากตอนถอย

    def score(self):
        """คะแนนจริง = หินสี X ที่อยู่ในวง X ตอนจบ (ไม่ใช่ที่ planner นับสะสม)"""
        return sum(1 for g in self.gems if self._zone_of(g["cm"]) == g["class"])

    def _zone_of(self, cm):
        for name, (zx, zy) in self.zones.items():
            if nav.dist(*cm, zx, zy) <= 10.0:
                return name
        return None

    # --- มอเตอร์ ---
    def _eff(self, vl, vr):
        if self.ideal:
            return vl, vr
        spin = vl * vr < 0
        dead = self.DEAD_SPIN if spin else self.DEAD_FWD
        el = vl if abs(vl) >= dead else 0
        er = vr if abs(vr) >= dead else 0
        return el, er * self.R_GAIN

    SLIP_PER_S = 0.6            # ของจริง 28 ก.ย.: คีบไม่แน่น ถอยแล้วหินหลุด (โอกาสหลุดต่อวินาทีที่ถอย)

    def update(self, dt):
        vl, vr = self._eff(self.vl, self.vr)
        v = (vl + vr) / 2 * self.SPEED_CM_S
        # ถอยหลังทั้งที่มีหินในปาก -> พื้นดึงหินหลุดจากก้าม (วางไว้ตรงหน้าปาก)
        # (v < -3 cm/s = ถอยจริง ๆ ไม่นับการหมุนอยู่กับที่ที่ล้อสองข้างไม่เท่ากันแล้วเลื่อนถอยนิดหน่อย)
        if not self.ideal and self.bin and v < -3.0 and random.random() < self.SLIP_PER_S * dt:
            gx, gy = nav.point_ahead(self.x, self.y, self.th, C.GRIP_REACH_CM + 2.0)
            for g in self.bin:
                self.gems.append({"cm": (gx + random.uniform(-1, 1), gy + random.uniform(-1, 1)),
                                  "class": g["class"], "area": g["area"]})
            self.bin = []
            self.slips += 1
        w = (vl - vr) / self.W_CM * self.SPEED_CM_S * C.TURN_SIGN
        self.th = nav.norm_deg(self.th + math.degrees(w) * dt * 0.35)
        r = math.radians(self.th)
        ox, oy = self.x, self.y
        self.x = min(max(self.x + v * math.cos(r) * dt, 5), 205)
        self.y = min(max(self.y + v * math.sin(r) * dt, 5), 115)
        if not self.ideal:
            self._push_gems()
        now_over = set()
        for g in self.gems:
            if nav.dist(self.x, self.y, *g["cm"]) < C.ROBOT_BODY_R_CM:
                now_over.add(id(g))
        self.runover += len(now_over - self._over)
        self._over = now_over
        self.hist.append((CLOCK.t, (self.x, self.y, self.th)))

    def _rel(self, gx, gy):
        r = math.radians(self.th)
        dx, dy = gx - self.x, gy - self.y
        return dx * math.cos(r) + dy * math.sin(r), -dx * math.sin(r) + dy * math.cos(r)

    def _abs(self, fwd, lat):
        r = math.radians(self.th)
        return (self.x + fwd * math.cos(r) - lat * math.sin(r),
                self.y + fwd * math.sin(r) + lat * math.cos(r))

    def _push_gems(self):
        """ตัวหุ่น (วงรัศมี BODY_R) และปาก (ที่นั่งหิน fwd=GRIP_REACH, ก้ามกว้าง +-4.5) ดันหิน"""
        # ก้ามปิด (ถือหินอยู่) แคบกว่าก้ามอ้า: ข้างก้ามอยู่ที่ lat 3.5-5 แทน 4.5-7
        jaw_in, jaw_out = (3.5, 5.0) if self.bin else (4.5, 7.0)
        for g in self.gems:
            fwd, lat = self._rel(*g["cm"])
            moved = None
            # ตัวหุ่น: หินที่อยู่ในวงตัวหุ่น (ยกเว้นด้านหน้าที่เป็นก้าม) ถูกดันออกจากศูนย์กลาง
            d = math.hypot(fwd, lat)
            if d < C.ROBOT_BODY_R_CM + self.GEM_R and not (fwd > C.ROBOT_BODY_FRONT_CM and abs(lat) < 6):
                k = (C.ROBOT_BODY_R_CM + self.GEM_R) / (d or 0.1)
                fwd, lat = fwd * k, lat * k; moved = "body"
            # ที่นั่งในปาก: หินเข้ามาลึกกว่า GRIP_REACH ไม่ได้ -> ถูกดันไปข้างหน้า
            elif abs(lat) < 4.5 and C.ROBOT_BODY_FRONT_CM < fwd < C.GRIP_REACH_CM:
                fwd = C.GRIP_REACH_CM; moved = "seat"
            # ปลายก้ามสองข้าง (fwd GRIP_REACH-2 .. GRIP_REACH+6, lat 4.5..7): ดันออกข้าง
            elif C.GRIP_REACH_CM - 2 < fwd < C.GRIP_REACH_CM + 6 and jaw_in <= abs(lat) < jaw_out:
                lat = math.copysign(jaw_out, lat); moved = "jaw"
            if moved:
                old = g["cm"]
                g["cm"] = self._abs(fwd, lat); self.pushes += 1
                # บัญชีการชน: ก้อนไหนโดนอะไร ขยับไปกี่ cm และหลุดออกจากวงไหม
                self.hit_cm[moved] += nav.dist(*old, *g["cm"])
                k = f"{self.cur_state}/{moved}"
                self.hit_by[k] = self.hit_by.get(k, 0.0) + nav.dist(*old, *g["cm"])
                if id(g) not in self.hit_ids[moved]:
                    self.hit_ids[moved].add(id(g))
                zo = self._zone_of(old); zn = self._zone_of(g["cm"])
                if zo is not None and zn != zo:
                    self.zone_exits += 1
                    key = f"{self.cur_state}/{moved}" + ("/correct" if zo == g["class"] else "/wrongzone")
                    self.zone_exit_by[key] = self.zone_exit_by.get(key, 0) + 1
                # หินที่ถูกดันไปชนเพื่อนบ้าน -> ดันเพื่อนบ้านออก (ไม่ซ้อนกัน)
                for o in self.gems:
                    if o is g:
                        continue
                    d = nav.dist(*g["cm"], *o["cm"])
                    if d < 2 * self.GEM_R:
                        ux, uy = ((o["cm"][0] - g["cm"][0]) / (d or 0.1), (o["cm"][1] - g["cm"][1]) / (d or 0.1))
                        o["cm"] = (g["cm"][0] + ux * 2 * self.GEM_R, g["cm"][1] + uy * 2 * self.GEM_R)

    # --- หยิบ/ปล่อย ---
    def do_action(self):
        if self.pending == "pick":
            gx, gy = nav.gripper_point(self.x, self.y, self.th)
            # firmware PICK = เปิดก่อนแล้วหนีบ -> ถ้ามีหินค้างในปากจะหล่นตรงนั้น
            for g in self.bin:
                self.gems.append({"cm": (gx + random.uniform(-1, 1), gy + random.uniform(-1, 1)),
                                  "class": g["class"], "area": g["area"]})
            self.bin = []
            best = min(self.gems, key=lambda g: nav.dist(gx, gy, *g["cm"]), default=None)
            self.last_pick_d = nav.dist(gx, gy, *best["cm"]) if best else 99
            if self.ideal:
                ok = best is not None and self.last_pick_d < 4.0
            else:
                # ก้ามปิดจะกวาดหินที่อยู่ "ระหว่างปลายก้ามกับที่นั่ง" (fwd GRIP_REACH-4..+2.5, เยื้อง <= 3) เข้ามาที่นั่ง
                fwd, lat = self._rel(*best["cm"]) if best else (99, 99)
                ok = (C.GRIP_REACH_CM - 4.0) <= fwd <= (C.GRIP_REACH_CM + 2.5) and abs(lat) <= 3.0
            if ok and random.random() < 0.9:
                self.gems.remove(best); self.bin.append(best)
        elif self.pending == "dump":
            gx, gy = nav.gripper_point(self.x, self.y, self.th)
            for g in self.bin:
                self.gems.append({"cm": (gx + random.uniform(-2, 2), gy + random.uniform(-2, 2)),
                                  "class": g["class"], "area": g["area"]})
            self.bin = []
        self.pending = None

    # --- สิ่งที่กล้องเห็น ---
    def pose(self):
        if self.ideal:
            return super().pose()
        want = CLOCK.t - self.LATENCY_S
        x, y, th = self.x, self.y, self.th
        for t, p in reversed(self.hist):
            if t <= want:
                x, y, th = p; break
        self.hist = self.hist[-40:]
        # v5.0 bugfix: "cm" ต้องเป็นตำแหน่ง "แท็ก" (เหมือน SimWorld.pose) ไม่ใช่เพลาตรง ๆ
        # ไม่งั้น nav.axle_pose บวก MARKER_TO_AXLE ซ้ำ -> geometry จริง (offset != 0) เพี้ยนทั้งระบบ
        r = math.radians(th)
        x -= C.MARKER_TO_AXLE_CM * math.cos(r); y -= C.MARKER_TO_AXLE_CM * math.sin(r)
        x += random.gauss(0, self.POS_NOISE * 0.5); y += random.gauss(0, self.POS_NOISE * 0.5)
        th = nav.norm_deg(th + random.gauss(0, self.ANG_NOISE * 0.5))
        return {"cm": (x, y), "angle_cm_deg": th, "px": (0, 0), "angle_deg": th}

    def seen_gems(self, pose, holding):
        """หินที่กล้องเห็น: บนพื้น + ที่หนีบอยู่ในปาก (แล้วตัดด้วย filter เดียวกับ run_real)"""
        out = [dict(g) for g in self.gems]
        if not self.ideal:
            gx, gy = nav.gripper_point(self.x, self.y, self.th)
            for g in self.bin:
                out.append({"cm": (gx, gy), "class": g["class"], "area": g["area"]})
            for g in out:
                g["cm"] = (g["cm"][0] + random.gauss(0, 0.4), g["cm"][1] + random.gauss(0, 0.4))
        out = nav.filter_outside_field(out)
        self.last_raw = list(out)                     # v4.5: ก่อนตัดตัวหุ่น (ให้ planner ดูในปาก)
        return nav.filter_robot_blobs(out, pose, holding=holding)


class RealLink(auto_main.SimLink):
    def pick(self):
        self.busy = True; self._busy_until = CLOCK.t + 1.2
        self.world.pending = "pick"
    def dump(self):
        self.busy = True; self._busy_until = CLOCK.t + 1.2
        self.world.pending = "dump"
    def grip(self, deg):
        if deg <= 10:
            self.busy = True; self._busy_until = CLOCK.t + 1.0
            self.world.pending = "pick"
        else:
            self.busy = True; self._busy_until = CLOCK.t + 0.3


ZONES = {"IRIDESCENT_VIOLET": (185, 34), "NEON_CYAN": (132, 18), "DEEP_CRIMSON": (151, 109),
         "MARIGOLD_ACCENT": (85, 18), "DEEP_SKY_BLUE": (85, 108), "LIME_GREEN": (182, 83)}


def run(seed=3, ideal=False, verbose=True):
    CLOCK.t = 1000.0                      # ให้ทุกรอบเริ่มเวลาเดียวกัน -> seed เดิมได้ผลเดิมเสมอ
    world = RealWorld(ZONES, ideal=ideal, seed=seed)
    link = RealLink(world)
    lines = []
    verdict = {"tp": 0, "fp": 0, "tn": 0, "fn": 0, "pick_d": []}
    def log(s):
        # ให้คะแนนการตัดสินของ VERIFY_PICK เทียบกับความจริงในโลก sim
        held_ok = bool(world.bin) and pl.color is not None and world.bin[-1]["class"] == pl.color
        held_wrong = bool(world.bin) and not held_ok
        if s.startswith("picked"):
            verdict["tp" if held_ok else "fp"] += 1
            s += f"   [sim: {'จริง' if held_ok else ('ผิด! ในปากเป็นสีอื่น' if held_wrong else 'ผิด! ปากเปล่า')} d={world.last_pick_d:.1f}]"
        elif ("still there" in s or "pushed" in s) and "assume" not in s:
            verdict["fn" if held_ok else "tn"] += 1
            s += f"   [sim: {'ผิด! หินสีถูกอยู่ในปาก' if held_ok else ('จริง (คว้าสีอื่นมา)' if held_wrong else 'จริง')} d={world.last_pick_d:.1f}]"
        if s.startswith("creep -> pick"):
            pass
        lines.append(s)
        if verbose: print(s)
    pl = Planner(link, ZONES, zone_radius_cm=10, log=log)
    dt = 1.0 / C.LOOP_HZ
    pl.start()
    frame = 0
    t_last = CLOCK.t
    while pl.state != "DONE":
        CLOCK.sleep(dt)
        world.cur_state = pl.state
        world.update(CLOCK.t - t_last); t_last = CLOCK.t
        link.tick()
        frame += 1
        pose = world.pose()
        holding = pl.holding() if hasattr(pl, "holding") else pl.bin_count > 0
        gems = world.seen_gems(pose, holding) if (ideal or frame % 3 == 1) else None
        if gems is not None:
            pl.raw_gems = world.last_raw
        pl.step(pose, "OK", gems)
        if CLOCK.t - t_last > 1000:
            break
    stats = {"delivered": pl.delivered, "score": world.score(), "zone_exit_by": world.zone_exit_by, "hit_by": world.hit_by,
             "runover": world.runover, "pushes": world.pushes,
             "picked": sum(1 for l in lines if l.startswith("picked")),
             "still": sum(1 for l in lines if "still there" in l),
             "pushed": sum(1 for l in lines if "pushed" in l),
             "giveup": sum(1 for l in lines if "give up" in l),
             "timeout": sum(1 for l in lines if "timeout" in l),
             "left_in_bin": len(world.bin),
             "verify TP/FP/TN/FN": (verdict["tp"], verdict["fp"], verdict["tn"], verdict["fn"]),
             "wrong_colour_grabs": sum(1 for l in lines if "คว้าสีอื่น" in l or "สีอื่น" in l),
             # การชน: (จำนวนก้อนที่โดน, ระยะรวม cm) แยก ตัวหุ่น / ที่นั่งปาก / ข้างก้าม + หินหลุดวง
             "hit_body": (len(world.hit_ids["body"]), round(world.hit_cm["body"])),
             "hit_seat": (len(world.hit_ids["seat"]), round(world.hit_cm["seat"])),
             "hit_jaw": (len(world.hit_ids["jaw"]), round(world.hit_cm["jaw"])),
             "zone_exits": world.zone_exits,
             "slips": world.slips}
    return stats, lines


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--runs", type=int, default=1)
    ap.add_argument("--ideal", action="store_true")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()
    tot = 0
    for i in range(a.runs):
        st, _ = run(seed=a.seed + i, ideal=a.ideal, verbose=not a.quiet)
        print(f"RUN seed={a.seed + i} ideal={a.ideal}: {st}")
        tot += st["delivered"]
    print(f"TOTAL delivered over {a.runs} runs = {tot}")
