"""ui.py — ตัวช่วย "วาดหน้าจอ" ให้อ่านง่าย ใช้ร่วมกันทุกโปรแกรมของ GEMBOT

*** ไฟล์นี้วาดจออย่างเดียว ไม่มีค่าปรับจูน ไม่ยุ่งกับการคุมหุ่น/กล้อง/planner ***
ถ้าลบไฟล์นี้ทิ้ง โปรแกรมอื่นจะพังแค่ตอนวาดจอ การคุมหุ่นไม่เปลี่ยน

วิธีใช้:
    import ui
    ui.panel(img, 12, 12, rows, title="AUTO")      # กล่องสถานะมุมซ้ายบน
    ui.keybar(img, [("SPACE", "เริ่ม/หยุด"), ("Q", "ออก")])   # แถบปุ่มล่างจอ

rows แต่ละแถวเป็นได้ 3 แบบ
    ("ป้ายชื่อ", "ค่า")            -> ป้ายสีจาง ค่าสีขาว
    ("ป้ายชื่อ", "ค่า", "ok")      -> ค่าเป็นสีเขียว ("ok"/"bad"/"warn"/"dim")
    ("ข้อความยาว", None, "dim")    -> บรรทัดเดียวเต็มความกว้าง กำหนดสีได้
    "ข้อความยาว"                   -> บรรทัดเดียวเต็มความกว้าง
    None                           -> เว้นบรรทัด
"""
import os

import cv2
import numpy as np

try:
    from PIL import Image, ImageDraw, ImageFont
    _HAVE_PIL = True
except Exception:                       # ไม่มี Pillow ก็ยังรันได้ แค่แสดงไทยไม่ได้
    _HAVE_PIL = False

# ---------------- สี (BGR) ----------------
COL = {
    "text": (245, 245, 245),
    "dim":  (160, 160, 160),
    "ok":   (120, 230, 140),    # เขียว = ปกติ
    "bad":  (70, 70, 255),      # แดง = มีปัญหา
    "warn": (60, 200, 255),     # เหลือง = รอ/ระวัง
    "key":  (255, 205, 110),    # ฟ้า = ชื่อปุ่ม
    "bg":   (22, 22, 22),
}

# ---------------- ฟอนต์ไทย ----------------
_FONT_FILES = [
    r"C:\Windows\Fonts\tahoma.ttf",
    r"C:\Windows\Fonts\leelawui.ttf",
    "/usr/share/fonts/truetype/tlwg/Loma.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/System/Library/Fonts/Supplemental/Tahoma.ttf",
]
_FONT_FILES_BOLD = [
    r"C:\Windows\Fonts\tahomabd.ttf",
    r"C:\Windows\Fonts\LeelaUIb.ttf",
    "/usr/share/fonts/truetype/tlwg/Loma-Bold.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
]
_font_cache = {}


def _font(size, bold=False):
    if not _HAVE_PIL:
        return None
    key = (size, bold)
    if key not in _font_cache:
        found = None
        for p in (_FONT_FILES_BOLD if bold else _FONT_FILES):
            if os.path.exists(p):
                try:
                    found = ImageFont.truetype(p, size)
                    break
                except Exception:
                    pass
        _font_cache[key] = found
    return _font_cache[key]


HAVE_THAI = _font(16) is not None      # โปรแกรมอื่นเช็คได้ว่าจะแสดงไทยได้ไหม


def _ascii_only(s):
    """สำรอง: ถ้าไม่มีฟอนต์ไทย ตัดอักษรที่วาดไม่ได้ออก (เหลือ ASCII)"""
    return "".join(c if 32 <= ord(c) < 127 else "" for c in s).strip() or "-"


_w_cache = {}


def _width(s, size, bold=False):
    k = (s, size, bold)
    v = _w_cache.get(k)
    if v is not None:
        return v
    f = _font(size, bold)
    if f is None:
        v = int(len(_ascii_only(s)) * size * 0.55)
    else:
        try:
            v = int(f.getlength(s))
        except Exception:
            v = int(f.getsize(s)[0])
    if len(_w_cache) > 4000:
        _w_cache.clear()
    _w_cache[k] = v
    return v


_tile_cache = {}


def _tile(txt, size, bold, color):
    """แปลงข้อความ 1 ชิ้นเป็นภาพเล็ก ๆ (เตรียมค่าผสมไว้ล่วงหน้า) แล้วเก็บไว้ใช้ซ้ำ
    ป้ายชื่อ/ชื่อปุ่มซ้ำทุกเฟรม จึงวาดจริงแค่ครั้งแรก HUD เลยแทบไม่กินเวลาลูปหลัก"""
    k = (txt, size, bold, color)
    t = _tile_cache.get(k)
    if t is None:
        w = max(1, _width(txt, size, bold)) + 3
        h = size + max(8, size // 2)
        im = Image.new("L", (w, h), 0)
        ImageDraw.Draw(im).text((0, 0), txt, font=_font(size, bold), fill=255)
        a1 = np.asarray(im).astype(np.float32) / 255.0
        a3 = cv2.merge([a1, a1, a1])
        t = (np.ascontiguousarray(1.0 - a3),
             (a3 * np.array(color, np.float32)).astype(np.uint8))
        if len(_tile_cache) > 2500:
            _tile_cache.clear()
        _tile_cache[k] = t
    return t


def _blit_text(img, items, size_default=17):
    """วาดข้อความหลายชิ้น  items = [(x, y, text, bgr, size, bold)]"""
    if not items:
        return
    if not _HAVE_PIL or not HAVE_THAI:
        for x, y, s, c, size, bold in items:
            cv2.putText(img, _ascii_only(s), (x, y + size), cv2.FONT_HERSHEY_SIMPLEX,
                        size / 30.0, c, 2 if bold else 1, cv2.LINE_AA)
        return

    H, W = img.shape[:2]
    for x, y, s, c, size, bold in items:
        if not s:
            continue
        inv3, fg = _tile(s, size, bold, tuple(int(v) for v in c))
        th, tw = fg.shape[0], fg.shape[1]
        x0, y0 = max(0, x), max(0, y)
        x1, y1 = min(W, x + tw), min(H, y + th)
        if x1 <= x0 or y1 <= y0:
            continue
        sy, sx, hh, ww = y0 - y, x0 - x, y1 - y0, x1 - x0
        roi = img[y0:y1, x0:x1]
        cv2.multiply(roi, inv3[sy:sy + hh, sx:sx + ww], dst=roi, dtype=cv2.CV_8U)
        cv2.add(roi, fg[sy:sy + hh, sx:sx + ww], dst=roi)


def _shade(img, x, y, w, h, alpha=0.74, color=None):
    """ทำพื้นหลังกล่องให้เข้ม เพื่อให้ตัวหนังสืออ่านออกบนภาพกล้อง"""
    H, W = img.shape[:2]
    x, y = max(0, x), max(0, y)
    w, h = min(w, W - x), min(h, H - y)
    if w <= 0 or h <= 0:
        return
    roi = img[y:y + h, x:x + w]
    bg = color or COL["bg"]
    if bg[0] == bg[1] == bg[2]:                     # สีเทา -> คำนวณรวดเดียว ไม่ต้องสร้างอาเรย์ใหม่
        cv2.addWeighted(roi, 1 - alpha, roi, 0.0, bg[0] * alpha, dst=roi)
    else:
        cv2.addWeighted(roi, 1 - alpha,
                        np.full_like(roi, np.array(bg, np.uint8)), alpha, 0, dst=roi)


# ==================================================================
#  กล่องสถานะ
# ==================================================================
def panel(img, x, y, rows, title=None, width=None, size=17, line_h=25, pad=12):
    """วาดกล่องสถานะ คืน (กว้าง, สูง) ที่ใช้ไป"""
    title_h = size + 12 if title else 0
    label_w = 0
    for r in rows:
        if isinstance(r, (tuple, list)) and len(r) >= 2 and r[1] is not None:
            label_w = max(label_w, _width(str(r[0]), size))
    gap = 14

    need = _width(title or "", size + 2, True) if title else 0
    for r in rows:
        if r is None:
            continue
        if isinstance(r, (tuple, list)) and len(r) >= 2 and r[1] is not None:
            need = max(need, label_w + gap + _width(str(r[1]), size))
        else:
            s = r[0] if isinstance(r, (tuple, list)) else r
            need = max(need, _width(str(s), size))
    w = width or (need + pad * 2)
    h = pad * 2 + title_h + len(rows) * line_h

    _shade(img, x, y, w, h)
    cv2.rectangle(img, (x, y), (x + w - 1, y + h - 1), (70, 70, 70), 1)

    items = []
    cy = y + pad
    if title:
        items.append((x + pad, cy, title, COL["key"], size + 2, True))
        cy += title_h
    for r in rows:
        if r is None:
            cy += line_h
            continue
        if isinstance(r, (tuple, list)) and len(r) >= 2 and r[1] is not None:
            col = COL.get(r[2] if len(r) > 2 else "text", COL["text"])
            items.append((x + pad, cy, str(r[0]), COL["dim"], size, False))
            items.append((x + pad + label_w + gap, cy, str(r[1]), col, size, False))
        else:
            s = r[0] if isinstance(r, (tuple, list)) else r
            ckey = "text"
            if isinstance(r, (tuple, list)):
                if len(r) > 2 and r[2]:
                    ckey = r[2]
                elif len(r) > 1 and isinstance(r[1], str):
                    ckey = r[1]
            items.append((x + pad, cy, str(s), COL.get(ckey, COL["text"]), size, False))
        cy += line_h
    _blit_text(img, items)
    return w, h


def banner(img, x, y, text, state="ok", size=22, width=None, pad=10):
    """แถบใหญ่ 1 บรรทัด (ไว้บอกว่า 'ตอนนี้ทำอะไรอยู่')"""
    w = width or (_width(text, size, True) + pad * 2)
    h = size + pad * 2
    _shade(img, x, y, w, h, alpha=0.8)
    cv2.rectangle(img, (x, y), (x + 5, y + h - 1), COL.get(state, COL["ok"]), -1)
    _blit_text(img, [(x + pad + 6, y + pad, text, COL.get(state, COL["ok"]), size, True)])
    return w, h


def keybar(img, keys, y=None, size=16, pad=10, gap=22):
    """แถบปุ่มล่างจอ  keys = [("SPACE", "เริ่ม/หยุด"), ...]"""
    H, W = img.shape[:2]
    h = size + pad * 2
    y = H - h if y is None else y
    _shade(img, 0, y, W, h, alpha=0.82)

    def _fits(sz, gp):
        cx = pad
        for k, meaning in keys:
            cx += _width(f"[{k}]", sz, True) + 6 + _width(meaning, sz) + gp
        return cx - gp <= W - pad

    for sz, gp in ((size, gap), (size, 14), (size - 1, 12), (size - 2, 10)):
        if sz >= 11 and _fits(sz, gp):
            size, gap = sz, gp
            break

    items, cx = [], pad
    for k, meaning in keys:
        kt = f"[{k}]"
        kw = _width(kt, size, True)
        mw = _width(meaning, size)
        if cx + kw + 6 + mw > W - pad:
            break
        items.append((cx, y + pad, kt, COL["key"], size, True))
        items.append((cx + kw + 6, y + pad, meaning, COL["text"], size, False))
        cx += kw + 6 + mw + gap
    _blit_text(img, items)
    return h


# ==================================================================
#  คำแปลสถานะ (ไว้แสดงบนจอเท่านั้น ไม่ได้ใช้ตัดสินใจอะไร)
# ==================================================================
STATE_TH = {
    "IDLE":          "รอเริ่ม — กด SPACE",
    "CHOOSE":        "เลือกหินก้อนถัดไป",
    "GO_APPROACH":   "วิ่งไปจุดตั้งต้นหน้าหิน",
    "ALIGN":         "หมุนหันหาหิน",
    "CREEP":         "คืบเข้าหาหิน",
    "CREEP_BACK":    "หินเบี้ยว ถอยตั้งหลัก",
    "BACKOFF_SKIP":  "มีก้อนอื่นขวางก้าม ถอยเลือกใหม่",
    "PICK":          "กำลังหนีบ",
    "PICK_BACKOFF":  "หนีบแล้ว หมุนตัวไปทางวง (ไม่ถอย)",
    "VERIFY_PICK":   "ตรวจว่าหนีบติดไหม",
    "BACKOFF":       "ถอยตั้งหลัก แล้วลองใหม่",
    "GO_ZONE":       "เล็งวงสีปลายทาง",
    "GO_ZONE_AP":    "ลากหินไปที่วงสี",
    "ZONE_ALIGN":    "หมุนหันเข้าวง",
    "REVERSE_IN":    "เข้าไปในวง",
    "DUMP":          "ปล่อยหิน",
    "VERIFY_DUMP":   "ตรวจว่าหินอยู่ในวงไหม",
    "LEAVE":         "ถอยออกจากวง",
    "COUNT_DUMP":    "นับคะแนน",
    "DONE":          "จบรอบแล้ว",
}

GESTURE_TH = {
    "FWD":   "เดินหน้า",
    "BACK":  "ถอยหลัง",
    "LEFT":  "หมุนซ้าย",
    "RIGHT": "หมุนขวา",
    "PICK":  "หนีบ",
    "DUMP":  "ปล่อย",
    "STOP":  "หยุด",
}


def state_line(state, msg=""):
    """'CREEP' -> 'CREEP — คืบเข้าหาหิน'  (เก็บชื่ออังกฤษไว้ด้วย จะได้เทียบกับ log ได้)"""
    th = STATE_TH.get(state)
    return f"{state} — {th}" if th else str(state)


def mmss(sec):
    sec = max(0, int(sec))
    return f"{sec // 60}:{sec % 60:02d}"
