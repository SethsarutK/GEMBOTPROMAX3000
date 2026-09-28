"""smoke test gembot_app à¹à¸šà¸šà¹„à¸¡à¹ˆà¹€à¸›à¸´à¸”à¸«à¸™à¹‰à¸²à¸•à¹ˆà¸²à¸‡: à¸ªà¸£à¹‰à¸²à¸‡à¸ à¸²à¸žà¸ªà¸™à¸²à¸¡à¸›à¸¥à¸­à¸¡ à¹à¸¥à¹‰à¸§à¹„à¸¥à¹ˆà¸—à¸¸à¸à¸‚à¸±à¹‰à¸™"""
import os, json, shutil
import cv2, numpy as np
os.chdir(os.path.dirname(os.path.abspath(__file__)))
# à¸à¸±à¸™à¹„à¸Ÿà¸¥à¹Œà¸ˆà¸£à¸´à¸‡à¹€à¸ªà¸µà¸¢à¸«à¸²à¸¢: à¸ªà¸³à¸£à¸­à¸‡à¹„à¸§à¹‰à¸à¹ˆà¸­à¸™
for f in ("field_map.json", "color_profiles.json", "app_state.json", "grip_calib.json"):
    if os.path.exists(f): shutil.copy(f, f + ".bak_smoke")

import gembot_app as A
from field_vision import DRAW_BGR, COLOR_CLASSES

# ---- à¸ à¸²à¸žà¸ªà¸™à¸²à¸¡à¸›à¸¥à¸­à¸¡ 1280x720: à¸žà¸·à¹‰à¸™à¹„à¸¡à¹‰, à¸ªà¸™à¸²à¸¡ 210x120 -> scale 5 px/cm à¹€à¸£à¸´à¹ˆà¸¡ (100,80)
img = np.full((720, 1280, 3), (120, 160, 190), np.uint8)
ox, oy, sc = 100, 80, 5
cv2.rectangle(img, (ox, oy), (ox + 210 * sc, oy + 120 * sc), (140, 180, 210), -1)
zones_cm = {"IRIDESCENT_VIOLET": (185, 34), "NEON_CYAN": (132, 18), "DEEP_CRIMSON": (151, 109),
            "MARIGOLD_ACCENT": (85, 18), "DEEP_SKY_BLUE": (85, 108), "LIME_GREEN": (182, 83)}
zone_bgr = {"IRIDESCENT_VIOLET": (200, 60, 150), "NEON_CYAN": (230, 230, 60), "DEEP_CRIMSON": (40, 30, 220),
            "MARIGOLD_ACCENT": (30, 150, 250), "DEEP_SKY_BLUE": (230, 140, 40), "LIME_GREEN": (60, 220, 90)}
for n, (x, y) in zones_cm.items():
    cv2.circle(img, (ox + x * sc, oy + y * sc), 10 * sc, zone_bgr[n], -1)
gem_px = {}
for i, n in enumerate(COLOR_CLASSES):
    for j in range(3):
        p = (ox + (60 + i * 15) * sc, oy + (40 + j * 12) * sc)
        cv2.circle(img, p, 12, zone_bgr[n], -1)
        gem_px.setdefault(n, []).append(p)
# ArUco tag id 0 4x4 à¸‚à¸™à¸²à¸” 60 px à¸—à¸µà¹ˆ (ox+40cm, oy+90cm)
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
tag = cv2.aruco.generateImageMarker(d, 0, 60)
tx, ty = ox + 30 * sc, oy + 90 * sc
img[ty:ty + 60, tx:tx + 60] = cv2.cvtColor(tag, cv2.COLOR_GRAY2BGR)
cv2.imwrite("app_smoke_field.png", img)

app = A.App(still_path="app_smoke_field.png")
app.grab_ok = app.grab()
app.goto(0); app.render()
app.goto(1)
app.clicks = [(ox, oy), (ox + 210 * sc, oy), (ox + 210 * sc, oy + 120 * sc), (ox, oy + 120 * sc)]
app.handle_key(A.KEY_ENTER)               # -> set corners, auto zones, step 2
print("step", app.step, "zones auto:", len(app.zones), sorted(app.zones))
app.render()
# à¹ƒà¸ªà¹ˆà¸§à¸‡à¸—à¸µà¹ˆà¸«à¸²à¸¢à¸”à¹‰à¸§à¸¢à¸à¸²à¸£à¸„à¸¥à¸´à¸
for i, n in enumerate(COLOR_CLASSES):
    if n not in app.zones:
        app.sel_color = i
        app.clicks = [(ox + zones_cm[n][0] * sc, oy + zones_cm[n][1] * sc)]
        pt = app.clicks.pop(0)
        app.zones[n] = {"px": pt, "radius_px": app.zone_r_px(), "area": 0.0}
print("zones after manual:", len(app.zones))
app.handle_key(A.KEY_ENTER)               # save map -> step 3
print("step", app.step, "field_map saved:", os.path.exists("field_map.json"))
# à¸„à¸¥à¸´à¸à¸«à¸´à¸™à¸—à¸¸à¸à¸ªà¸µ 2 à¸à¹‰à¸­à¸™
for i, n in enumerate(COLOR_CLASSES):
    app.sel_color = i
    for p in gem_px[n][:2]:
        app.add_sample(p)
    print("  ", n, app.profiles[n])
app.update_gems(force=True)
seen = {n: sum(1 for g in app.gems if g["class"] == n) for n in COLOR_CLASSES}
print("gems seen per colour:", seen)
app.show_mask = True; app.render(); app.show_mask = False
app.handle_key(A.KEY_ENTER)               # save profiles -> step 4
print("step", app.step)
app.update_robot()
print("robot:", app.pstat, app.pose and app.pose.get("cm"))
app.render()
app.handle_key(A.KEY_ENTER)               # -> step 5
out = app.render()
print("checklist:", [(n, ok) for n, ok, _ in app.checklist()])
cv2.imwrite("app_smoke_out.png", out)
# F-key mapping sanity
print("F3 ->", A.KEY_F.get(0x700000 + 2 * 0x10000))

# ---- ขั้น 7 จูนวง ----
import math, nav, auto_config as C
if os.path.exists("grip_calib.json"): os.remove("grip_calib.json")
app.handle_key(0x700000 + 6 * 0x10000)      # F7
assert app.step == 6, app.step
app.update_robot(); app.update_gems(force=True)
assert app.pstat == "OK", "smoke: robot tag not seen"
ax, ay, th = nav.axle_pose(app.pose)
gp0 = app.calib.to_pixel(nav.gripper_point(ax, ay, th))
base = C.GRIP_REACH_CM
app.tune_sel = 0
app.handle_key(A.KEY_RIGHT); app.handle_key(A.KEY_RIGHT)          # +1.0 cm
assert abs(C.GRIP_REACH_CM - base - 1.0) < 1e-6, C.GRIP_REACH_CM
gp1 = app.calib.to_pixel(nav.gripper_point(ax, ay, th))
moved = math.hypot(gp1[0] - gp0[0], gp1[1] - gp0[1])
print(f"tune: GRIP_REACH {base} -> {C.GRIP_REACH_CM}, red dot moved {moved:.1f} px (expect ~5)")
assert 3.0 < moved < 7.0, moved
app.render()
# วัดอัตโนมัติ: หินปลอมที่ fwd=15, lat=+1 เทียบเพลา
off0 = C.HEADING_OFFSET_DEG
r = math.radians(th)
fake = (ax + 15 * math.cos(r) - 1 * math.sin(r), ay + 15 * math.sin(r) + 1 * math.cos(r))
app.gems = [{"cm": fake, "px": app.calib.to_pixel(fake), "class": "LIME_GREEN", "area": 100.0}]
app.handle_key(ord('a'))
for _ in range(30): app.tune_measure_tick()
print(f"auto-measure: GRIP_REACH {C.GRIP_REACH_CM}  HEADING_OFFSET {off0} -> {C.HEADING_OFFSET_DEG}")
assert abs(C.GRIP_REACH_CM - math.hypot(15, 1)) < 0.05, C.GRIP_REACH_CM
assert abs(nav.norm_deg(C.HEADING_OFFSET_DEG - off0) - math.degrees(math.atan2(1, 15))) < 0.1
assert app.measuring is False
app.render()
app.handle_key(ord('s'))
assert os.path.exists("grip_calib.json") and not app.tune_dirty
print("grip_calib.json:", json.load(open("grip_calib.json")))
app.handle_key(ord('r'))
assert abs(C.GRIP_REACH_CM - math.hypot(15, 1)) < 0.05      # R = กลับค่าที่บันทึกล่าสุด
out = app.render(); cv2.imwrite("app_smoke_tune.png", out)
app.handle_key(A.KEY_ENTER); assert app.step == 5
os.remove("grip_calib.json")
# à¸„à¸·à¸™à¹„à¸Ÿà¸¥à¹Œà¸ˆà¸£à¸´à¸‡
for f in ("field_map.json", "color_profiles.json", "app_state.json", "grip_calib.json"):
    if os.path.exists(f + ".bak_smoke"): shutil.move(f + ".bak_smoke", f)
print("SMOKE OK")
