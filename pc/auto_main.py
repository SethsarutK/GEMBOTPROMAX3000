"""auto_main.py — โปรแกรมรอบ auto

รันจริง:   python auto_main.py --source 0
ทดลอง:     python auto_main.py --sim        (หุ่นจำลอง ไม่ต้องมีกล้อง/หุ่น ใช้เช็ค logic)

คีย์:  SPACE = START/STOP    q = ออก    b = สลับ BIG_GEM_ONLY
ต้องมี color_profiles.json และ field_map.json (จาก Module 2) ก่อน
"""
import argparse, math, time, random
import cv2, numpy as np

import auto_config as C
import nav
from planner import Planner
from field_vision import (load_profiles, load_field_map, RobotTracker,
                          detect_gems, draw_overlay, open_source, COLOR_CLASSES, DRAW_BGR,
                          print_profile_report, resolve_ambiguous)


# =====================================================================
#  โหมดจำลอง: หุ่น + หิน + link ปลอม  (เพื่อทดสอบ planner/nav โดยไม่มีของจริง)
# =====================================================================
class SimLink:
    def __init__(self, world):
        self.world = world
        self.busy = False
        self.alive = True
        self.vbat = 7.4
        self._busy_until = 0

    def drive(self, vl, vr):
        self.world.vl, self.world.vr = vl, vr

    def stop(self):
        self.world.vl = self.world.vr = 0

    def pick(self):
        self.busy = True; self._busy_until = time.time() + 3.0
        self.world.pending = "pick"

    def dump(self):
        self.busy = True; self._busy_until = time.time() + 2.0
        self.world.pending = "dump"

    def tick(self):
        if self.busy and time.time() >= self._busy_until:
            self.busy = False
            self.world.do_action()

    def close(self): pass


class SimWorld:
    """หุ่นจำลอง: กลางเพลาที่ (x,y) heading th (y ลง)  ล้อ -100..100 -> cm/s"""
    W_CM = 10.5
    SPEED_CM_S = 0.6            # 100 -> 60 cm/s

    def __init__(self, zones):
        self.x, self.y, self.th = 40.0, 100.0, -60.0
        self.vl = self.vr = 0
        self.zones = zones
        self.bin = []
        self.pending = None
        self.gems = []
        random.seed(3)
        for c in COLOR_CLASSES:
            for _ in range(9):
                a = random.uniform(0, 2 * math.pi); r = random.uniform(0, 16)
                big = random.random() < 0.5
                self.gems.append({"cm": (105 + r * math.cos(a), 60 + r * math.sin(a)),
                                  "class": c, "area": 600 if big else 300})

    def update(self, dt):
        v = (self.vl + self.vr) / 2 * self.SPEED_CM_S
        w = (self.vl - self.vr) / self.W_CM * self.SPEED_CM_S * C.TURN_SIGN   # deg-ish/s
        self.th = nav.norm_deg(self.th + math.degrees(w) * dt * 0.35)
        r = math.radians(self.th)
        self.x = min(max(self.x + v * math.cos(r) * dt, 5), 205)
        self.y = min(max(self.y + v * math.sin(r) * dt, 5), 115)

    def do_action(self):
        if self.pending == "pick":
            gx, gy = nav.gripper_point(self.x, self.y, self.th)
            best = min(self.gems, key=lambda g: nav.dist(gx, gy, *g["cm"]), default=None)
            if best and nav.dist(gx, gy, *best["cm"]) < 4.0 and random.random() < 0.85:
                self.gems.remove(best); self.bin.append(best)
        elif self.pending == "dump":
            # v2: ไม่มีกระบะแล้ว ปล่อยหินตรงจุดปากหนีบ (จุดเดียวกับตอนหยิบ)
            gx, gy = nav.gripper_point(self.x, self.y, self.th)
            for g in self.bin:
                self.gems.append({"cm": (gx + random.uniform(-3, 3), gy + random.uniform(-3, 3)),
                                  "class": g["class"], "area": g["area"]})
            self.bin = []
        self.pending = None

    def pose(self):
        # marker อยู่ที่กลางเพลา - MARKER_TO_AXLE
        r = math.radians(self.th)
        mx = self.x - C.MARKER_TO_AXLE_CM * math.cos(r)
        my = self.y - C.MARKER_TO_AXLE_CM * math.sin(r)
        return {"cm": (mx, my), "angle_cm_deg": self.th, "px": (0, 0), "angle_deg": self.th}

    def draw(self, scale=4):
        img = np.full((120 * scale, 210 * scale, 3), (200, 215, 235), np.uint8)
        for name, (zx, zy) in self.zones.items():
            cv2.circle(img, (int(zx * scale), int(zy * scale)), int(12 * scale), DRAW_BGR[name], 2)
        for g in self.gems:
            cv2.circle(img, (int(g["cm"][0] * scale), int(g["cm"][1] * scale)),
                       5 if g["area"] > 450 else 3, DRAW_BGR[g["class"]], -1)
        p = (int(self.x * scale), int(self.y * scale))
        cv2.circle(img, p, int(8 * scale), (40, 40, 40), 2)
        # v2: จุดเดียว (ปากหนีบ) ใช้ทั้งหยิบและปล่อย ไม่มีจุด bin แยกอีกแล้ว
        gx, gy = nav.gripper_point(self.x, self.y, self.th)
        cv2.circle(img, (int(gx * scale), int(gy * scale)), 4, (0, 0, 255), -1)
        cv2.putText(img, f"holding={len(self.bin)}", (p[0] + 20, p[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        return img


# =====================================================================
def run_sim(args):
    zones = {"DEEP_CRIMSON": (75, 25), "NEON_CYAN": (130, 25), "LIME_GREEN": (30, 45),
             "IRIDESCENT_VIOLET": (30, 85), "DEEP_SKY_BLUE": (75, 100), "MARIGOLD_ACCENT": (130, 100)}
    world = SimWorld(zones)
    link = SimLink(world)
    pl = Planner(link, zones, zone_radius_cm=12)
    dt = 1.0 / C.LOOP_HZ
    t_last = time.time()
    pl.start()
    headless = args.headless
    while True:
        now = time.time()
        world.update(min(now - t_last, 0.1) * args.speed); t_last = now
        link.tick()
        msg = pl.step(world.pose(), "OK", [dict(g) for g in world.gems])
        if not headless:
            img = world.draw()
            cv2.putText(img, f"{pl.state}: {msg}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            cv2.putText(img, f"t={pl.elapsed():.0f}s delivered={pl.delivered}", (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            cv2.imshow("GEMBOT sim", img)
            k = cv2.waitKey(int(dt * 1000 / args.speed)) & 0xFF
            if k == ord('q'):
                break
        else:
            time.sleep(dt / args.speed)
        if pl.state == "DONE":
            print("SIM DONE delivered =", pl.delivered)
            break
    return pl.delivered


# =====================================================================

def draw_capsules(out, calib, pose, holding=False):
    """วาดบริเวณตัดตัวหุ่น (สีเทา) ให้เห็นว่าครอบหุ่น+แขน+หินที่หนีบพอดี"""
    if pose is None or "cm" not in pose:
        return
    for (x0, y0), (x1, y1), r in nav.robot_capsules(pose, holding):
        p0 = calib.to_pixel((x0, y0)); p1 = calib.to_pixel((x1, y1))
        rp = calib.to_pixel((x1 + r, y1)); rpx = int(math.hypot(rp[0] - p1[0], rp[1] - p1[1]))
        cv2.circle(out, p0, rpx, (90, 90, 90), 1); cv2.circle(out, p1, rpx, (90, 90, 90), 1)
        cv2.line(out, p0, p1, (90, 90, 90), 1)

def run_real(args):
    from link import RobotLink
    profiles = load_profiles()
    print_profile_report(profiles)
    fmap = load_field_map()
    if fmap is None:
        print("[ERROR] ไม่มี field_map.json — รัน field_vision.py --setup ก่อน"); return
    calib = fmap["calib"]; zones_px = fmap["zones"]
    zones_cm = {n: calib.to_field(z["px"]) for n, z in zones_px.items()}
    radii = []
    for n, z in zones_px.items():
        edge = calib.to_field((z["px"][0] + z["radius_px"], z["px"][1]))
        radii.append(nav.dist(*zones_cm[n], *edge))
    zone_r = float(np.median(radii)) if radii else 12.0
    print("zones (cm):", {n: tuple(round(v) for v in c) for n, c in zones_cm.items()}, " r=", round(zone_r, 1))
    missing = [c for c in COLOR_CLASSES if c not in zones_cm]
    if missing:
        print("[WARN] โซนที่ไม่มีในแผนที่ จะไม่เก็บสีนี้:", missing)

    cap, still = open_source(args.source, exposure=args.exposure)
    tracker = RobotTracker(marker_id=C.ARUCO_ID,
                           dict_name=getattr(cv2.aruco, C.ARUCO_DICT), calib=calib)
    link = RobotLink()
    pl = Planner(link, zones_cm, zone_r)

    # วงตัดตัวหุ่น+หินที่หนีบ (จาก ROBOT_FRONT_CM / ROBOT_REAR_CM)
    dt = 1.0 / C.LOOP_HZ
    GEM_EVERY = 3                       # ตรวจหินทุก N เฟรม (ArUco ทุกเฟรม)
    frame_i = 0
    gems_px, n_ambig = [], 0
    fps, fps_t, fps_n = 0.0, time.time(), 0
    print("SPACE = start/stop, q = quit")
    while True:
        t0 = time.time()
        frame = still.copy() if still is not None else cap.read()[1]
        if frame is None:
            print("[ERROR] frame drop"); link.stop(); continue

        tracker.detect(frame)
        pose, pstat = tracker.get_pose_or_last()
        frame_i += 1
        gems_cm = None                       # None = เฟรมนี้ไม่ตรวจหิน planner ใช้ track เดิม
        if frame_i % GEM_EVERY == 1:
            gems_px = detect_gems(frame, profiles, exclude_zones=zones_px)
            gems_px, n_ambig = resolve_ambiguous(gems_px)
            gems_cm = [{"cm": calib.to_field(g["px"]), "class": g["class"], "area": g["area"], "px": g["px"]} for g in gems_px]
            gems_cm = nav.filter_outside_field(gems_cm)          # ตัดกำแพง/นอกสนาม
            gems_cm = nav.filter_robot_blobs(gems_cm, pose, holding=pl.bin_count > 0)
            gems_px = [g for g in gems_px if any(g["px"] == h["px"] for h in gems_cm)]   # วาดเฉพาะที่ไม่ใช่ตัวหุ่น
        fps_n += 1
        if time.time() - fps_t >= 1.0:
            fps, fps_n, fps_t = fps_n / (time.time() - fps_t), 0, time.time()

        msg = pl.step(pose, pstat, gems_cm)

        lines = [f"[{pl.state}] {msg}",
                 f"t={pl.elapsed():.0f}s left={pl.time_left():.0f}s delivered={pl.delivered} bin={pl.bin_count} color={pl.color}",
                 f"link={'OK' if link.alive else 'LOST'}{' NET-ERR' if link.net_error else ''} busy={link.busy} fps={fps:.1f}  pose={pstat}"
                 + (f" ({pose['cm'][0]:.0f},{pose['cm'][1]:.0f}) {pose['angle_cm_deg']:.0f}deg" if pose and 'cm' in pose else ""),
                 f"gems stable={len(pl.tracker.stable())} ambiguous={n_ambig}  BIG_ONLY={C.BIG_GEM_ONLY}   SPACE=start/stop q=quit"]
        out = draw_overlay(frame, zones_px, gems_px, pose, calib, lines)
        draw_capsules(out, calib, pose, holding=pl.bin_count > 0)
        # วาดเป้าหมาย/จุด approach
        if pl.target is not None and pl.state not in ("IDLE", "DONE"):
            tp = calib.to_pixel(pl.target["cm"]); cv2.circle(out, tp, 12, (0, 0, 255), 2)
            if pl.approach: cv2.circle(out, calib.to_pixel(pl.approach), 6, (255, 0, 255), 2)
        cv2.imshow("GEMBOT auto", out)

        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        elif k == ord(' '):
            if pl.state == "IDLE": pl.start(); print("=== START ===")
            else: pl.stop(); print("=== STOP ===")
        elif k == ord('b'):
            C.BIG_GEM_ONLY = not C.BIG_GEM_ONLY

        el = time.time() - t0
        if el < dt: time.sleep(dt - el)

    link.close()
    if cap is not None: cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0")
    ap.add_argument("--marker-id", type=int, default=0)
    ap.add_argument("--exposure", type=float, default=None)
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--speed", type=float, default=1.0, help="sim: เร่งเวลา เช่น 4")
    ap.add_argument("--headless", action="store_true")
    a = ap.parse_args()
    run_sim(a) if a.sim else run_real(a)
