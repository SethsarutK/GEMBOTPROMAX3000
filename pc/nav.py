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


def steer(v, corr):
    """v4.2b: แยกคำสั่งสองล้อจากความเร็วฐาน v และค่าแก้ทิศ corr
    ปกติ = (v+corr, v-corr)  แต่ถ้าล้อช้าจะต่ำกว่า WHEEL_MIN_CMD (ล้อหยุด = เลี้ยวหัก)
    ให้คงล้อช้าไว้ที่ v แล้วเร่งล้อเร็วเป็น v+2*corr แทน (ผลต่างล้อเท่ากัน หุ่นไม่หยุดล้อ)"""
    m = getattr(C, "WHEEL_MIN_CMD", 15)
    if v > 0 and v - abs(corr) < m:
        a, b = v + 2 * abs(corr), v
        vl, vr = (a, b) if corr >= 0 else (b, a)
    else:
        vl, vr = v + corr, v - corr
    return floor_wheels(clamp(vl, -100, 100), clamp(vr, -100, 100))


def floor_wheels(vl, vr):
    """v4.1: มอเตอร์จริงต่ำกว่า WHEEL_MIN_CMD (วัดได้ 15) ล้อไม่หมุนเลย
    -> คำสั่งล้อที่ไม่ใช่ 0 แต่ต่ำกว่านั้น ดันขึ้นให้ถึงขั้นต่ำ (คงเครื่องหมาย) ไม่งั้นล้อข้างหนึ่งหยุด
       หุ่นเลี้ยวหักแทนที่จะแก้ทิศเบา ๆ  (ใช้กับ go_to / creep_to เท่านั้น การหมุนอยู่กับที่ใช้ V_TURN_MIN อยู่แล้ว)"""
    m = getattr(C, "WHEEL_MIN_CMD", 15)
    def f(v):
        if v == 0 or abs(v) >= m:
            return int(v)
        return int(math.copysign(m, v))
    return f(vl), f(vr)


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
    # v4.1: ช่อง "หน้าปาก" ห้ามตัด — วงตัวหุ่น (r=BODY_R) ยื่นเลยหน้าตัวไปถึง BODY_FRONT+BODY_R (~17 cm)
    #       ซึ่งครอบที่นั่งหิน (GRIP_REACH 13) พอดี ทำให้ track หินหายตอนคืบเข้าใกล้ -> หนีบอากาศ
    #       ยกเว้นตอน holding: หินที่นั่งอยู่ที่ปาก (วง 4.5 cm รอบจุดปาก) ต้องตัดออกเหมือนเดิม
    ax, ay, th = axle_pose(pose)
    r = math.radians(th)
    gp = point_ahead(ax, ay, th, C.GRIP_REACH_CM)
    out = []
    for g in gems_cm:
        dx, dy = g["cm"][0] - ax, g["cm"][1] - ay
        fwd = dx * math.cos(r) + dy * math.sin(r)
        lat = -dx * math.sin(r) + dy * math.cos(r)
        if holding and dist(*g["cm"], *gp) <= 4.5:
            continue
        if fwd >= C.ROBOT_BODY_FRONT_CM:
            out.append(g)                  # หน้าตัวหุ่นทั้งแถบ (ปาก/ข้างปาก): เห็นเสมอ
        elif not in_robot(g["cm"], caps):
            out.append(g)
    return out


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


def turn_to_pulsed(ax, ay, th, target_heading, t_in_state):
    """v4.1: หมุนอยู่กับที่แบบ "จังหวะ" ตอนใกล้ถึงมุม
    หมุนต่อเนื่องที่ V_TURN_MIN 48 + กล้อง/WiFi หน่วง ~0.15 วิ = เลยมุมไป 10-20° ทุกครั้ง แล้วหมุนกลับ วนไม่จบ
    -> error ยังมาก: หมุนต่อเนื่อง / error < 25°: หมุน 0.12 วิ หยุด 0.25 วิ (ให้กล้องเห็นมุมจริงก่อนหมุนต่อ)"""
    err = norm_deg(target_heading - th)
    if abs(err) <= C.HEADING_DEADBAND_DEG:
        return 0, 0, True
    vl, vr = _turn_cmd(err)
    if abs(err) <= C.TURN_FIRST_DEG:
        period, on = 0.37, 0.12
        if (t_in_state % period) > on:
            return 0, 0, False
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
    vl, vr = steer(v, corr)
    return vl, vr, False


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
    if forward:
        # v4.1: ระยะใกล้ใช้ "เยื้องข้าง (lat)" ของเป้าเทียบแนวหุ่น แทนมุมจากเพลา
        #   เดิม: มุมจากเพลาไปเป้าโตขึ้นเร็วมากตอนใกล้ (เยื้อง 2 cm ที่ 5 cm = 22°) -> เกิน TURN_FIRST
        #   -> หมุนอยู่กับที่ที่ความเร็ว 48 -> เลยเป้า -> หมุนกลับ -> วนจน timeout แล้วปล่อย/หนีบผิดที่
        r = math.radians(th)
        dx, dy = tx - ax, ty - ay
        fwd = dx * math.cos(r) + dy * math.sin(r)
        lat = -dx * math.sin(r) + dy * math.cos(r)
        if fwd <= tool_offset and abs(lat) <= 2 * tol:
            return 0, 0, True                      # เลยที่นั่งแล้ว (เยื้องพอรับได้) = ถึง
        if fwd <= tool_offset - 3.0:
            return 0, 0, True                      # เลยไปแล้วแน่ ๆ คืบต่อมีแต่ไถหิน/ไถวง -> หยุด
        if fwd < tool_offset + 12.0:
            v = C.V_CREEP
            corr = C.TURN_SIGN * clamp(lat * 2.5, -8, 8)   # เยื้อง 1 cm -> ต่างล้อ 5
            if abs(lat) <= 0.8:
                corr = 0
            vl, vr = steer(v, corr)
            return vl, vr, False
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
    # v4.1: ตอนคืบใช้ deadband แคบกว่าตอนหมุน (ล้อไม่เท่ากันทำให้หุ่นโค้ง ถ้าปล่อยถึง 6° ปากเบี้ยว 2-3 cm)
    if abs(err) <= getattr(C, "CREEP_DEADBAND_DEG", 2.0):
        corr = 0
    vl, vr = steer(v, corr)
    return vl, vr, False


# ---------------------------------------------------------------
#  v3.8: หาเส้นทางหลบหิน (A* บนตาราง)  — ใช้ตอน GO_APPROACH / GO_ZONE_AP
#  หินทุกก้อนที่กล้องเห็น (ยกเว้นก้อนเป้าหมาย) = สิ่งกีดขวาง ขยายด้วย PATH_INFLATE_CM
#  คืน list waypoint (cm) ไม่รวมจุดเริ่ม แต่รวมจุดหมาย; [] ถ้าไปตรงได้; None ถ้าถูกล้อมหาทางไม่ได้
# ---------------------------------------------------------------
import heapq


def _los_free(free, a, b):
    """เดินเส้นตรงบนตารางจาก cell a ไป b ผ่านแต่ช่องว่างไหม (Bresenham)"""
    (x0, y0), (x1, y1) = a, b
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx, sy = (1 if x1 > x0 else -1), (1 if y1 > y0 else -1)
    err = dx - dy
    while True:
        if not free[y0][x0]:
            return False
        if (x0, y0) == (x1, y1):
            return True
        e2 = 2 * err
        if e2 > -dy: err -= dy; x0 += sx
        if e2 < dx:  err += dx; y0 += sy


def plan_path(sx, sy, gx, gy, obstacles_cm, inflate_cm=None, cell_cm=None):
    """
    ช่องใกล้หิน (ภายใน inflate) ไม่ได้ "ห้ามผ่าน" แต่ "แพง" (คูณ PATH_PENALTY) — เส้นทางจึงมีเสมอ
    แม้หุ่นจะยืนอยู่กลางกอง (หลัง PICK) หรือจุดหมายอยู่ชิดกอง: มันจะออก/เข้าทางที่ทับหินน้อยที่สุด
    ห้ามผ่านจริงเฉพาะขอบสนาม (กันชนกำแพง)
    """
    inflate = C.PATH_INFLATE_CM if inflate_cm is None else inflate_cm
    cell = C.PATH_CELL_CM if cell_cm is None else cell_cm
    W, H = int(FIELD_W_CM / cell) + 1, int(FIELD_H_CM / cell) + 1

    def to_cell(x, y):
        return (min(max(int(round(x / cell)), 0), W - 1), min(max(int(round(y / cell)), 0), H - 1))

    cost = [[1.0] * W for _ in range(H)]          # ค่าผ่านต่อช่อง
    free = [[True] * W for _ in range(H)]         # True = ไม่ติดหิน (ใช้ตอนย่อเส้นทาง)
    r_c = inflate / cell
    for (ox, oy) in obstacles_cm:
        cx, cy = ox / cell, oy / cell
        for yy in range(max(0, int(cy - r_c) - 1), min(H, int(cy + r_c) + 2)):
            for xx in range(max(0, int(cx - r_c) - 1), min(W, int(cx + r_c) + 2)):
                if (xx - cx) ** 2 + (yy - cy) ** 2 <= r_c * r_c:
                    cost[yy][xx] = C.PATH_PENALTY; free[yy][xx] = False
    m = int(C.ROBOT_BODY_R_CM / cell)
    wall = [[(xx < m or yy < m or xx >= W - m or yy >= H - m) for xx in range(W)] for yy in range(H)]
    s, g = to_cell(sx, sy), to_cell(gx, gy)
    wall[s[1]][s[0]] = wall[g[1]][g[0]] = False

    if _los_free(free, s, g):
        return []                                     # ไปตรงได้เลย ไม่ทับหิน

    def h(a):
        return math.hypot(a[0] - g[0], a[1] - g[1])
    openq = [(h(s), 0.0, s)]
    came, gcost = {s: None}, {s: 0.0}
    steps = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
    found = False
    while openq:
        _, gc, cur = heapq.heappop(openq)
        if cur == g:
            found = True; break
        if gc > gcost.get(cur, 1e9):
            continue
        for dx, dy, w in steps:
            nx, ny = cur[0] + dx, cur[1] + dy
            if not (0 <= nx < W and 0 <= ny < H) or wall[ny][nx]:
                continue
            ng = gc + w * cost[ny][nx]
            if ng < gcost.get((nx, ny), 1e9):
                gcost[(nx, ny)] = ng; came[(nx, ny)] = cur
                heapq.heappush(openq, (ng + h((nx, ny)), ng, (nx, ny)))
    if not found:
        return None
    cells = []
    c = g
    while c is not None:
        cells.append(c); c = came[c]
    cells.reverse()
    # ย่อเส้นทาง: ข้ามได้เฉพาะช่วงที่เส้นตรงไม่ทับหิน
    wp = []
    i = 0
    while i < len(cells) - 1:
        j = len(cells) - 1
        while j > i + 1 and not _los_free(free, cells[i], cells[j]):
            j -= 1
        wp.append(cells[j]); i = j
    out = [(x * cell, y * cell) for (x, y) in wp]
    if out:
        out[-1] = (gx, gy)
    return out
