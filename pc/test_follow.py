"""test_follow.py — ทดสอบ "หุ่นเดินไปหาหินสีที่เลือก" จากกล้องสนาม (ไม่หยิบ ไม่เท)

รัน:    python test_follow.py --source 1
คีย์:
  1..6      เลือกสีเป้าหมาย (1=VIOLET 2=CYAN 3=CRIMSON 4=MARIGOLD 5=SKY_BLUE 6=LIME)
  c         สลับโหมดคลิก: เมื่อเปิด คลิกซ้ายบนภาพ = ตั้งเป้าที่จุดนั้น (จูน nav ไม่ต้องมีหิน)
            (ปิดอยู่ = คลิกเพื่อ focus หน้าต่างได้โดยไม่เปลี่ยนเป้า)
  SPACE     GO / STOP
  +/-       เพิ่ม/ลดความเร็วสูงสุด (V_MAX)
  t         สลับ TURN_SIGN (ใช้ตอนหุ่นหมุนหนีเป้า)
  k         AUTO-CALIBRATE ทิศ: หุ่นจะหมุน 1 วิ + วิ่งตรง 1 วิ เอง แล้วคำนวณ TURN_SIGN /
            HEADING_OFFSET_DEG จากที่กล้องเห็นจริง (ต้องมีที่ว่างรอบหุ่น ~50 cm, pose=OK)
  l         เริ่ม/หยุดบันทึก log CSV (follow_log.csv) ไว้ส่งให้วิเคราะห์
  w/s/a/d   ทดสอบมอเตอร์ผ่านโปรแกรมนี้: วิ่ง 1 วิ ที่ speed 45 (เท่า teleop) (หน้า/ถอย/หมุนซ้าย/หมุนขวา)
            ใช้เช็คว่าหุ่นรับคำสั่งจาก test_follow ได้เหมือน teleop ไหม

*** ปิด teleop.py และหน้าต่าง python อื่นทุกอันก่อนรัน — ถ้ามี 2 โปรแกรมส่งคำสั่งพร้อมกัน
    หุ่นจะได้รับ 0,0 สลับกับคำสั่งจริง = ไม่ขยับ/กระตุก ***
  q         ออก

ต้องมีก่อน: color_profiles.json, field_map.json (จาก field_vision.py --setup)
            หุ่นเปิดอยู่ + โน้ตบุ๊กต่อ WiFi GEMBOT
"""
import argparse, math, time
import cv2

import auto_config as C
import ui                     # v3.9: ตัวช่วยวาดจอ (ไม่มีค่าปรับจูน)
import nav
from link import RobotLink
from planner import GemTracker
from field_vision import (load_profiles, load_field_map, RobotTracker, detect_gems,
                          draw_overlay, open_source, COLOR_CLASSES, DRAW_BGR,
                          print_profile_report, resolve_ambiguous)

STOP_IN_FRONT_CM = C.GRIP_REACH_CM + 3.0   # หยุดให้ "ปากหนีบ" ห่างหิน ~3 cm (ไม่ทับหิน)

KEY_TO_COLOR = {ord(str(i + 1)): c for i, c in enumerate(COLOR_CLASSES)}

clicked = None


def on_mouse(event, x, y, flags, param):
    global clicked
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked = (x, y)



def draw_capsules(out, calib, pose):
    """วาดบริเวณตัดตัวหุ่น (สีเทา) ให้เห็นว่าครอบหุ่น+แขน+หินที่หนีบพอดี"""
    if pose is None or "cm" not in pose:
        return
    for (x0, y0), (x1, y1), r in nav.robot_capsules(pose):
        p0 = calib.to_pixel((x0, y0)); p1 = calib.to_pixel((x1, y1))
        rp = calib.to_pixel((x1 + r, y1)); rpx = int(math.hypot(rp[0] - p1[0], rp[1] - p1[1]))
        cv2.circle(out, p0, rpx, (90, 90, 90), 1); cv2.circle(out, p1, rpx, (90, 90, 90), 1)
        cv2.line(out, p0, p1, (90, 90, 90), 1)

def main():
    global clicked
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="1")
    ap.add_argument("--marker-id", type=int, default=0)
    ap.add_argument("--exposure", type=float, default=None)
    args = ap.parse_args()

    profiles = load_profiles()
    print_profile_report(profiles)
    fmap = load_field_map()
    if fmap is None:
        print("[ERROR] ไม่มี field_map.json -> รัน  python field_vision.py --source 1 --setup  ก่อน")
        return
    calib, zones_px = fmap["calib"], fmap["zones"]

    cap, still = open_source(args.source, exposure=args.exposure)
    tracker = RobotTracker(marker_id=C.ARUCO_ID,
                           dict_name=getattr(cv2.aruco, C.ARUCO_DICT), calib=calib,
                           cam_height_cm=C.CAM_HEIGHT_CM, tag_height_cm=C.TAG_HEIGHT_CM)
    gtrack = GemTracker()
    link = RobotLink()

    win = "GEMBOT follow test"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, on_mouse)

    color = None            # สีเป้าหมาย
    calib_step = None       # โหมด auto-calibrate: ("turn"|"turn_wait"|"fwd"|"fwd_wait", t0, data)
    pulse = None            # (vl, vr, t_end) ทดสอบมอเตอร์ด้วยปุ่ม w/s/a/d
    logf = None
    lines_extra = ""
    click_mode = False
    last_target = None      # (cm, time) จำหินล่าสุดไว้ 2 วิ กันหลุดตอนเข้าใกล้
    click_cm = None         # จุดที่คลิก (cm)
    running = False
    arrived = False
    lost_t = None
    dt = 1.0 / C.LOOP_HZ
    GEM_EVERY = 3                       # ตรวจหินทุก N เฟรม (ArUco ทุกเฟรม) ให้ loop เร็วขึ้น
    frame_i = 0
    gems_px, gems_cm, n_amb = [], [], 0
    fps, fps_t, fps_n = 0.0, time.time(), 0
    print(__doc__)
    print("[เช็ค] ถ้ากด w แล้วหุ่นไม่ขยับแต่ teleop ขยับ = มีโปรแกรมอื่นส่งคำสั่งอยู่ หรือ ESP32 ค้าง busy (กด O ใน teleop แล้วปิด)")

    while True:
        t0 = time.time()
        frame = still.copy() if still is not None else cap.read()[1]
        if frame is None:
            link.stop(); print("[ERROR] frame drop"); continue

        # ---------- vision ----------
        tracker.detect(frame)
        pose, pstat = tracker.get_pose_or_last()
        frame_i += 1
        if frame_i % GEM_EVERY == 1:
            gems_px = detect_gems(frame, profiles, exclude_zones=zones_px)
            gems_px, n_amb = resolve_ambiguous(gems_px)
            gems_cm = [{"cm": calib.to_field(g["px"]), "class": g["class"], "area": g["area"], "px": g["px"]}
                       for g in gems_px]
            gems_cm = nav.filter_outside_field(gems_cm)          # ตัดกำแพง/นอกสนาม
            gems_cm = nav.filter_robot_blobs(gems_cm, pose)
            gems_px = [g for g in gems_px if any(g["px"] == h["px"] for h in gems_cm)]
            gtrack.update(gems_cm)
        stable = gtrack.stable()
        fps_n += 1
        if time.time() - fps_t >= 1.0:
            fps, fps_n, fps_t = fps_n / (time.time() - fps_t), 0, time.time()

        # ---------- เลือกเป้า ----------
        target_cm, goal_cm, info = None, None, ""
        have_pose = pose is not None and "cm" in pose and pstat != "LOST"
        if have_pose:
            ax, ay, th = nav.axle_pose(pose)
            if click_cm is not None:
                target_cm = click_cm
                goal_cm = click_cm                       # คลิก = ให้กลางเพลาไปตรงนั้นเลย
            elif color is not None:
                cands = [g for g in stable if g["class"] == color]
                if cands:
                    g = min(cands, key=lambda g: nav.dist(ax, ay, *g["cm"]))
                    target_cm = g["cm"]; last_target = (target_cm, time.time())
                elif last_target and time.time() - last_target[1] < 2.0:
                    target_cm = last_target[0]          # หินเข้าใกล้ปากจนถูกตัดออก ใช้ตำแหน่งล่าสุด
                if target_cm is not None:
                    # จุดหยุด: ถอยจากหินมาทางหุ่น STOP_IN_FRONT_CM
                    dx, dy = ax - target_cm[0], ay - target_cm[1]
                    n = math.hypot(dx, dy) or 1.0
                    goal_cm = (target_cm[0] + dx / n * STOP_IN_FRONT_CM,
                               target_cm[1] + dy / n * STOP_IN_FRONT_CM)
                else:
                    info = f"no stable {color} gem"

        # ---------- auto-calibrate ทิศ ----------
        if calib_step is not None:
            step, t0, d = calib_step
            now = time.time()
            if not link.alive:
                info = "CALIB: link LOST -> abort"; calib_step = None
                try: link.stop()
                except OSError: pass
                step = None
            if not have_pose:
                info = "CALIB: pose lost -> abort"; calib_step = None; link.stop()
            elif step == "turn":
                # รอ pose นิ่งก่อนอ่านค่าเริ่ม (ต้องเป็น OK ไม่ใช่ STALE)
                if pstat == "OK" and now - t0 > 0.5:
                    d["th0"] = th; d["turn_lost"] = 0
                    calib_step = ("turn_run", now, d)
                info = "CALIB waiting stable pose..."
            elif step == "turn_run":
                link.drive(45, -45)                      # 45 = ระดับเดียวกับ teleop ที่พิสูจน์แล้วว่าวิ่ง
                if pstat != "OK": d["turn_lost"] += 1
                if now - t0 > 0.8:
                    link.drive(0, 0); calib_step = ("turn_wait", now, d)
            elif step == "turn_wait":
                link.drive(0, 0)
                if now - t0 > 1.0 and pstat == "OK":     # รอนิ่ง 1 วิ และ pose สดเท่านั้น
                    d["dth"] = nav.norm_deg(th - d["th0"])
                    d["x0"], d["y0"], d["thf"] = ax, ay, th; d["fwd_lost"] = 0
                    calib_step = ("fwd_run", now, d)
            elif step == "fwd_run":
                link.drive(40, 40)
                if pstat != "OK": d["fwd_lost"] += 1
                if now - t0 > 1.0:
                    link.drive(0, 0); calib_step = ("fwd_wait", now, d)
            elif step == "fwd_wait":
                link.drive(0, 0)
                if now - t0 > 1.0 and pstat == "OK":
                    moved = nav.dist(d["x0"], d["y0"], ax, ay)
                    mv_ang = nav.heading_to(d["x0"], d["y0"], ax, ay)
                    th_avg = nav.norm_deg(d["thf"] + nav.norm_deg(th - d["thf"]) / 2)
                    off = nav.norm_deg(mv_ang - th_avg)
                    print("\n===== AUTO-CALIBRATE RESULT (รายงานอย่างเดียว ไม่เปลี่ยนค่าเอง) =====")
                    print(f"หมุนด้วยคำสั่ง (L=+45,R=-45) 0.8 วิ -> heading เปลี่ยน {d['dth']:+.0f} deg   (ArUco หลุด {d['turn_lost']} เฟรม)")
                    if d["turn_lost"] > 5 or abs(d["dth"]) > 150:
                        print("  !! ค่าไม่น่าเชื่อถือ (ArUco หลุดระหว่างหมุน หรือหมุนเกิน 150 องศา) -> ไม่แนะนำค่า ลองใหม่")
                    elif abs(d["dth"]) < 8:
                        print("  !! หุ่นแทบไม่หมุน: ความเร็วต่ำไป (เพิ่ม MOTOR_MIN_PWM/V_TURN_MIN)")
                    else:
                        ts = +1 if d["dth"] > 0 else -1
                        print(f"  -> TURN_SIGN ควรเป็น {ts:+d}  (ตอนนี้ {C.TURN_SIGN:+d})  {'OK' if ts == C.TURN_SIGN else '<<< ถ้าซ้ำได้ 2 ครั้ง ค่อยแก้ใน auto_config.py'}")
                    print(f"เดินหน้าด้วยคำสั่ง (40,40) 1 วิ -> ขยับ {moved:.1f} cm ไปทิศ {mv_ang:.0f} deg, heading ที่กล้องอ่านได้ {th_avg:.0f} deg   (ArUco หลุด {d['fwd_lost']} เฟรม)")
                    if d["fwd_lost"] > 5 or moved > 60:
                        print("  !! ค่าไม่น่าเชื่อถือ (ArUco หลุด หรือขยับ > 60 cm ใน 1 วิ = pose กระโดด) -> ไม่แนะนำค่า ลองใหม่")
                    elif moved < 5:
                        print("  !! หุ่นแทบไม่ขยับ: ความเร็วต่ำไป")
                    else:
                        print(f"  ทิศเดินจริงต่างจาก heading {off:+.0f} deg")
                        if abs(off) <= 15:
                            print("  -> HEADING_OFFSET_DEG OK (ไม่ต้องแก้)")
                        elif abs(abs(off) - 180) < 25:
                            print("  -> ต่าง ~180: หุ่นวิ่ง 'ถอย' เมื่อสั่งเดินหน้า -> แก้ที่ config.h (กลับ L_INVERT/R_INVERT ทั้งคู่) ไม่ใช่ HEADING_OFFSET")
                        else:
                            print(f"  -> ถ้าซ้ำได้ 2 ครั้งใกล้เคียงกัน ค่อยตั้ง HEADING_OFFSET_DEG = {nav.norm_deg(C.HEADING_OFFSET_DEG + off):.0f} (ตอนนี้ {C.HEADING_OFFSET_DEG})")
                    print("ค่าในโปรแกรมไม่ถูกเปลี่ยน — ให้ดูผลซ้ำ 2 ครั้งก่อนแก้ auto_config.py ด้วยมือ")
                    print("=====================================================================\n")
                    calib_step = None
            if calib_step is not None:
                lines_extra = f"CALIB {calib_step[0]}"
            else:
                lines_extra = ""

        # ---------- ควบคุม ----------
        vl = vr = 0
        if calib_step is not None:
            pass                                          # calib คุมล้อเอง
        elif pulse is not None:
            if time.time() < pulse[2]:
                vl, vr = pulse[0], pulse[1]; info = f"PULSE test L={vl} R={vr}"
            else:
                pulse = None; print("pulse done")
        elif running:
            if not have_pose:
                lost_t = lost_t or time.time()
                info = f"POSE LOST {time.time() - lost_t:.1f}s -> stop"
            elif goal_cm is None:
                info = info or "no target"
            else:
                lost_t = None
                vl, vr, done = nav.go_to(ax, ay, th, *goal_cm)
                if done:
                    if not arrived:
                        print(f"ARRIVED  gap to goal={nav.dist(ax, ay, *goal_cm):.1f} cm")
                    arrived = True
                    vl = vr = 0
                else:
                    arrived = False
                err = nav.norm_deg(nav.heading_to(ax, ay, *goal_cm) - th)
                info = (f"d={nav.dist(ax, ay, *goal_cm):.1f}cm  heading_err={err:+.0f}deg  "
                        f"cmd L={vl} R={vr}  {'ARRIVED' if arrived else ''}")
        if calib_step is None:
            link.drive(vl, vr)           # keepalive ใน link.py ส่งซ้ำทุก 50 ms ให้เอง
            if link.net_error:
                info = "WIFI GEMBOT หลุด! ต่อใหม่แล้วโปรแกรมจะทำงานต่อเอง"
        if logf is not None:
            logf.write(f"{time.time():.3f},{pstat},{ax if have_pose else ''},{ay if have_pose else ''},"
                       f"{th if have_pose else ''},{goal_cm[0] if goal_cm else ''},{goal_cm[1] if goal_cm else ''},"
                       f"{vl},{vr},{running},{color}\n")

        # ---------- วาด ----------
        out = draw_overlay(frame, zones_px, gems_px, pose, calib, None)
        draw_capsules(out, calib, pose)
        if have_pose:
            a_px = calib.to_pixel((ax, ay))
            cv2.circle(out, a_px, 6, (0, 255, 255), -1)                       # กลางเพลา
            g_px = calib.to_pixel(nav.gripper_point(ax, ay, th))
            cv2.circle(out, g_px, 5, (0, 0, 255), -1)                         # ปากหนีบ
            cv2.line(out, a_px, g_px, (0, 0, 255), 2)
        if target_cm is not None:
            tp = calib.to_pixel(target_cm)
            cv2.circle(out, tp, 14, DRAW_BGR.get(color, (255, 255, 255)) if color else (255, 255, 255), 2)
        if goal_cm is not None:
            gp = calib.to_pixel(goal_cm)
            cv2.drawMarker(out, gp, (255, 0, 255), cv2.MARKER_CROSS, 18, 2)
            if have_pose:
                cv2.line(out, calib.to_pixel((ax, ay)), gp, (255, 0, 255), 1)
        # ---------- HUD (v3.9) : แสดงผลอย่างเดียว ไม่เปลี่ยนค่า/การทำงานใดๆ ----------
        seen_txt = "  ".join(f"{c.split('_')[-1][:4]}={sum(1 for g in stable if g['class'] == c)}"
                             for c in COLOR_CLASSES)
        rows = [
            ("สถานะ", "กำลังวิ่ง" if running else "หยุดอยู่", "ok" if running else "warn"),
            ("เป้าหมาย", "คลิกจุดบนจอ" if click_mode else ("สี " + color if color else "ยังไม่เลือกสี")),
            None,
            ("WiFi ถึงหุ่น", "ต่ออยู่" if link.alive else "ขาด!", "ok" if link.alive else "bad"),
            ("กล้องเห็นหุ่น", "เห็น" if pstat == "OK" else "ไม่เห็น!", "ok" if pstat == "OK" else "bad"),
            ("ตำแหน่งเพลา", f"({ax:.0f}, {ay:.0f}) cm   หัน {th:.0f}°" if have_pose else "-", "dim"),
            ("ความลื่นภาพ", f"{fps:.0f} fps", "ok" if fps >= 10 else "warn"),
            None,
            ("ความเร็วสูงสุด", f"{C.V_MAX}"),
            ("ทิศหมุน", f"{C.TURN_SIGN:+d}"),
            ("หินที่เห็น", seen_txt + (f"   ·   สีกำกวม {n_amb}" if n_amb else "")),
        ]
        if calib_step:
            rows.append(("กำลังวัดทิศ", lines_extra, "warn"))
        if logf:
            rows.append(("บันทึก log", "เปิดอยู่ (follow_log.csv)", "warn"))
        y = 12
        _, hh = ui.panel(out, 12, y, rows, title="TEST FOLLOW — ทดสอบเดินหาหิน")
        if info:
            state = "bad" if ("LOST" in info or "abort" in info or "no stable" in info) else "ok"
            ui.banner(out, 12, y + hh + 8, info, state, size=18)
        ui.keybar(out, [("1-6", "เลือกสี"), ("C", "โหมดคลิกจุด"), ("SPACE", "วิ่ง / หยุด"),
                        ("K", "วัดทิศ"), ("W/S/A/D", "ขยับ 1 วิ"), ("T", "กลับทิศหมุน"),
                        ("+/-", "ความเร็ว"), ("L", "log"), ("Q", "ออก")])
        cv2.imshow(win, out)

        # ---------- คีย์ ----------
        if clicked is not None:
            if click_mode:
                click_cm = calib.to_field(clicked)
                color = None; arrived = False
                print(f"click target = ({click_cm[0]:.1f}, {click_cm[1]:.1f}) cm")
            clicked = None
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        elif k in KEY_TO_COLOR:
            color = KEY_TO_COLOR[k]; click_cm = None; arrived = False
            print("target color =", color)
        elif k == ord(' '):
            running = not running; arrived = False
            print("=== GO ===" if running else "=== STOP ===")
            if not running:
                link.stop()
        elif k in (ord('+'), ord('=')):
            C.V_MAX = min(100, C.V_MAX + 5); print("V_MAX", C.V_MAX)
        elif k == ord('-'):
            C.V_MAX = max(C.V_MIN, C.V_MAX - 5); print("V_MAX", C.V_MAX)
        elif k in (ord('w'), ord('s'), ord('a'), ord('d')) and calib_step is None:
            v = {ord('w'): (45, 45), ord('s'): (-45, -45), ord('a'): (-45, 45), ord('d'): (45, -45)}[k]
            pulse = (v[0], v[1], time.time() + 1.0); running = False
            print(f"PULSE {chr(k).upper()}: L={v[0]} R={v[1]} 1 วิ  (ack ก่อนส่ง={link.seq_ack})")
        elif k == ord('c'):
            click_mode = not click_mode; print("click mode", click_mode)
            if not click_mode: click_cm = None
        elif k == ord('k'):
            if have_pose and link.alive and calib_step is None:
                running = False; click_cm = None
                calib_step = ("turn", time.time(), {})
                print("=== AUTO-CALIBRATE: หุ่นจะหมุน 0.8 วิ แล้ววิ่งตรง 1 วิ (ช้า) ===")
            else:
                print("calibrate ต้องมี pose=OK และ link=OK ก่อน (ตอนนี้ pose=%s link=%s)" % (pstat, "OK" if link.alive else "LOST"))
        elif k == ord('l'):
            if logf is None:
                logf = open("follow_log.csv", "w"); logf.write("t,pose,ax,ay,th,gx,gy,vl,vr,running,color\n"); print("LOG start")
            else:
                logf.close(); logf = None; print("LOG saved follow_log.csv")
        elif k == ord('t'):
            C.TURN_SIGN = -C.TURN_SIGN; print("TURN_SIGN", C.TURN_SIGN, "(แก้ถาวรใน auto_config.py)")

        el = time.time() - t0
        if el < dt:
            time.sleep(dt - el)

    if logf: logf.close()
    link.close()
    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
