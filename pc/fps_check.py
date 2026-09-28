"""fps_check.py — หาว่าอะไรทำให้ auto_main ช้า (จอบอก 4 fps ทั้งที่ควร ~20)

    python fps_check.py --source 1        (ต้องมี field_map.json + color_profiles.json)

วัดแยกทีละส่วน 60 เฟรม: อ่านกล้อง / ArUco / ตรวจสี / วาด HUD  แล้วบอกว่าตัวไหนกินเวลา
"""
import argparse, time, os
import cv2, numpy as np
os.chdir(os.path.dirname(os.path.abspath(__file__)))
import auto_config as C
import ui, nav
from field_vision import (load_profiles, load_field_map, RobotTracker, detect_gems, resolve_ambiguous,
                          open_source, draw_overlay)


def timed(fn, n):
    t = time.perf_counter()
    for _ in range(n):
        fn()
    return (time.perf_counter() - t) / n * 1000.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="1")
    a = ap.parse_args()
    profiles = load_profiles()
    fmap = load_field_map()
    cap, still = open_source(a.source)
    if cap is None or not cap.isOpened():
        print("เปิดกล้องไม่ได้"); return
    print(f"กล้อง: {cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f}  "
          f"fps ที่กล้องบอก={cap.get(cv2.CAP_PROP_FPS):.0f}  exposure={cap.get(cv2.CAP_PROP_EXPOSURE)}  "
          f"auto_exp={cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)}")
    for _ in range(10):
        cap.read()
    N = 60
    t_read = timed(lambda: cap.read(), N)
    ok, frame = cap.read()
    print(f"1) อ่านกล้อง (cap.read)      {t_read:6.1f} ms/เฟรม  -> สูงสุด {1000 / t_read:.0f} fps")
    if t_read > 60:
        print("   !! กล้องเองช้า: exposure ยาว (แสงน้อย/ตั้ง exposure ต่ำเกิน) หรือ USB hub ช้า -> ลอง --exposure -6 หรือเสียบ USB ตรง")

    if fmap is not None:
        calib = fmap["calib"]; zones_px = fmap["zones"]
        tr = RobotTracker(marker_id=C.ARUCO_ID, dict_name=getattr(cv2.aruco, C.ARUCO_DICT), calib=calib,
                          cam_height_cm=C.CAM_HEIGHT_CM, tag_height_cm=C.TAG_HEIGHT_CM)
        t_aruco = timed(lambda: tr.detect(frame), N)
        print(f"2) ArUco (ทุกเฟรม)          {t_aruco:6.1f} ms")
        t_gem = timed(lambda: resolve_ambiguous(detect_gems(frame, profiles, exclude_zones=zones_px)), 20)
        print(f"3) ตรวจสีหิน (ทุก 3 เฟรม)  {t_gem:6.1f} ms  -> เฉลี่ยต่อเฟรม {t_gem / 3:.1f} ms")
        gems_px = detect_gems(frame, profiles, exclude_zones=zones_px)
        pose, _ = tr.get_pose_or_last()
        def hud():
            out = draw_overlay(frame, zones_px, gems_px, pose, calib, None)
            ui.panel(out, 12, 12, [("สถานะ", "ทดสอบ", "ok")] * 10, title="AUTO")
            ui.banner(out, 12, 300, "CREEP — คืบเข้าหาหิน")
            ui.keybar(out, [("SPACE", "เริ่ม")])
            return out
        t_hud = timed(hud, 20)
        print(f"4) วาด overlay + HUD        {t_hud:6.1f} ms")
        t_show = timed(lambda: (cv2.imshow("fps_check", frame), cv2.waitKey(1)), 20)
        print(f"5) imshow + waitKey         {t_show:6.1f} ms")
        total = t_read + t_aruco + t_gem / 3 + t_hud + t_show
        print(f"รวมโดยประมาณ                {total:6.1f} ms/เฟรม  -> ~{1000 / total:.0f} fps  (เป้า 20 fps = 50 ms)")
        worst = max([("อ่านกล้อง", t_read), ("ArUco", t_aruco), ("ตรวจสี", t_gem / 3), ("HUD", t_hud), ("imshow", t_show)],
                    key=lambda kv: kv[1])
        print(f"ตัวที่กินเวลาที่สุด: {worst[0]} ({worst[1]:.1f} ms)")
        cv2.destroyAllWindows()
    cap.release()


if __name__ == "__main__":
    main()
