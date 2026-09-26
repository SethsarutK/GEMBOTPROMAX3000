"""nav.py — closed-loop navigation จาก pose ที่กล้องบอก

ทุกฟังก์ชันคืน (vl, vr, done)  ส่งต่อให้ link.drive() ทุกเฟรม
พิกัด: cm, y ลงล่าง, heading 0° = +x (ขวา), +90° = +y (ลง)

หลัก: หมุนก่อนถ้า error มาก แล้วค่อยวิ่งพร้อมแก้ทิศ ชะลอเมื่อใกล้ หยุดเมื่อถึง
"""
import math
import auto_config as C


def norm_deg(a):
    """ทำให้อยู่ใน (-180, 180]"""
    while a > 180:  a -= 360
    while a <= -180: a += 360
    return a


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def axle_pose(pose):
    """
    pose จาก RobotTracker (มี "cm", "angle_cm_deg") = จุดกลาง ArUco
    -> คืน (x, y, θ) ของ "กลางเพลาล้อ" ซึ่งเป็นจุดที่หุ่นหมุนรอบจริง
    """
    x, y = pose["cm"]
    th = norm_deg(pose["angle_cm_deg"] + C.HEADING_OFFSET_DEG)
    r = math.radians(th)
    return (x + C.MARKER_TO_AXLE_CM * math.cos(r),
            y + C.MARKER_TO_AXLE_CM * math.sin(r), th)


def point_ahead(ax, ay, th, dist):
    """จุดที่อยู่หน้าหุ่น (dist>0) หรือหลังหุ่น (dist<0) ตามแนว heading"""
    r = math.radians(th)
    return (ax + dist * math.cos(r), ay + dist * math.sin(r))


def gripper_point(ax, ay, th):
    """
    จุดปลายปากหนีบ (หน้าหุ่น) — v2: ไม่มีกระบะแล้ว จุดนี้ใช้ทั้งตอนเข้าหา
    เม็ดอัญมณีตอนหยิบ และตอนเข้าหาโซนสีตอนปล่อย (คนละครั้งกัน แต่สูตรเดียวกัน)
    เพราะปากหนีบตัวเดียวทำหน้าที่ทั้งสองอย่าง ไม่ต้องมี bin_point() แยกอีกแล้ว
    """
    return point_ahead(ax, ay, th, C.GRIP_REACH_CM)


def _dist_to_segment(px, py, x0, y0, x1, y1):
    dx, dy = x1 - x0, y1 - y0
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / L2))
    return math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))


def robot_capsules(pose, holding=False):
    """
    บริเวณตัวหุ่น (cm) สำหรับตัด blob ของหุ่นเอง (ล้อเหลือง/แขน) ออกจาก gem
    v3.5: ท่อนแขนหยุดก่อนถึงปาก 8 cm (ปากสีดำ ไม่ถูกจับเป็นหินอยู่แล้ว) — หินที่วางอยู่หน้าปากต้อง "ยังเห็น" เพื่อให้เล็งได้
          วงหินที่หนีบ (ที่จุดปาก) จะเพิ่มเฉพาะตอน holding=True (planner รู้ว่ามีหินในปาก)
    คืน list ของ ((x0,y0),(x1,y1),r)
    """
    ax, ay, th = axle_pose(pose)
    rear = point_ahead(ax, ay, th, -C.ROBOT_REAR_CM)
    bodyf = point_ahead(ax, ay, th, C.ROBOT_BODY_FRONT_CM)
    arm_end = point_ahead(ax, ay, th, max(C.ROBOT_BODY_FRONT_CM, C.GRIP_REACH_CM - 8.0))
    caps = [(rear, bodyf, C.ROBOT_BODY_R_CM), (bodyf, arm_end, C.ARM_R_CM)]
    if holding:
        gp = point_ahead(ax, ay, th, C.GRIP_REACH_CM)
        caps.append((gp, gp, 4.5))
    return caps


def in_robot(cm, capsules):
    for (x0, y0), (x1, y1), r in capsules:
        if _dist_to_segment(cm[0], cm[1], x0, y0, x1, y1) <= r:
            return True
    return False


FIELD_W_CM, FIELD_H_CM = 210.0, 120.0

def filter_outside_field(gems_cm, margin_cm=2.0):
    """ตัด blob ที่อยู่นอกสนาม (กำแพงแดง/ของนอกสนาม) — พิกัด cm จาก homography"""
    return [g for g in gems_cm
            if margin_cm <= g["cm"][0] <= FIELD_W_CM - margin_cm
            and margin_cm <= g["cm"][1] <= FIELD_H_CM - margin_cm]


def filter_robot_blobs(gems_cm, pose, holding=False):
    """ตัด gem ที่อยู่ในตัวหุ่นออก (holding=True = มีหินในปาก ตัดตรงปากด้วย)"""
    if pose is None or "cm" not in pose:
        return gems_cm
    caps = robot_capsules(pose, holding)
    return [g for g in gems_cm if not in_robot(g["cm"], caps)]


def heading_to(ax, ay, tx, ty):
    return math.degrees(math.atan2(ty - ay, tx - ax))


def dist(ax, ay, tx, ty):
    return math.hypot(tx - ax, ty - ay)


# ---------------------------------------------------------------
def _turn_cmd(err_deg):
    """หมุนอยู่กับที่ให้ heading error -> 0"""
    v = clamp(abs(err_deg) * C.K_TURN, C.V_TURN_MIN, C.V_TURN_MAX)
    s = C.TURN_SIGN * (1 if err_deg > 0 else -1)
    return int(s * v), int(-s * v)


def turn_to(ax, ay, th, target_heading):
    err = norm_deg(target_heading - th)
    if abs(err) <= C.HEADING_DEADBAND_DEG:
        return 0, 0, True
    vl, vr = _turn_cmd(err)
    return vl, vr, False


def go_to(ax, ay, th, tx, ty):
    """ไปให้ "กลางเพลา" ถึงจุด (tx,ty)"""
    d = dist(ax, ay, tx, ty)
    if d <= C.ARRIVE_TOL_CM:
        return 0, 0, True
    err = norm_deg(heading_to(ax, ay, tx, ty) - th)
    if abs(err) > C.TURN_FIRST_DEG:
        vl, vr = _turn_cmd(err)
        return vl, vr, False
    v = clamp(d * C.K_DIST, C.V_MIN, C.V_MAX)
    if d < C.SLOWDOWN_CM:
        v = clamp(v * (0.4 + 0.6 * d / C.SLOWDOWN_CM), C.V_MIN, C.V_MAX)
    corr = C.TURN_SIGN * err * C.K_TURN * 0.6
    if abs(err) <= C.HEADING_DEADBAND_DEG:
        corr = 0
    return int(clamp(v + corr, -100, 100)), int(clamp(v - corr, -100, 100)), False


def creep_to(ax, ay, th, tx, ty, tool_offset, tol):
    """
    คืบช้า ๆ ให้ "จุดเครื่องมือ" (ปากหนีบ = +GRIP_REACH, ท้ายกระบะ = -BIN_REACH)
    ไปอยู่ที่ (tx,ty) — ใช้ตอนเข้าหาหิน (tool_offset>0) และถอยเข้าวง (tool_offset<0)
    ไม่หมุนมาก แค่แก้ทิศเล็กน้อย
    """
    px, py = point_ahead(ax, ay, th, tool_offset)
    d = dist(px, py, tx, ty)
    if d <= tol:
        return 0, 0, True
    forward = tool_offset > 0
    # ทิศที่ "เครื่องมือ" ต้องไป เทียบกับแนวหุ่น
    want = heading_to(ax, ay, tx, ty)
    if not forward:
        want = norm_deg(want + 180)         # ถอย: หลังหุ่นต้องชี้ไปเป้า
    err = norm_deg(want - th)
    if abs(err) > C.TURN_FIRST_DEG:
        vl, vr = _turn_cmd(err)
        return vl, vr, False
    v = C.V_CREEP if forward else -C.V_CREEP
    # หมายเหตุ: ล้อซ้าย>ขวา ทำให้หุ่นหมุนทางเดิมเสมอ ไม่ว่าจะเดินหน้าหรือถอย
    # ดังนั้นสัญญาณแก้ทิศไม่ต้องกลับด้านตอนถอย
    corr = C.TURN_SIGN * err * C.K_TURN * 0.5
    if abs(err) <= C.HEADING_DEADBAND_DEG:
        corr = 0
    return int(clamp(v + corr, -100, 100)), int(clamp(v - corr, -100, 100)), False
