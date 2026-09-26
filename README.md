# GEMBOT — Module 1: ESP32 firmware + teleop

## ESP32
1. Arduino IDE → Boards Manager ติดตั้ง **esp32 by Espressif** เวอร์ชัน **3.x ขึ้นไป** (โค้ดใช้ `ledcAttach` ของ core 3)
2. Library Manager → ติดตั้ง **ESP32Servo**
3. เปิด `esp32_gembot/esp32_gembot.ino` → แก้ `config.h` (ขามอเตอร์, ขา servo, MOTOR_MODE)
4. Board: "ESP32 Dev Module" (หรือรุ่นที่ตรงกับบอร์ด) → Upload
5. Serial Monitor 115200 ต้องเห็น `GEMBOT AP up ... IP=192.168.4.1`

## โน้ตบุ๊ก (Windows)
1. ติดตั้ง Python 3.10+ → `pip install opencv-python numpy`
2. ต่อ WiFi ชื่อ **GEMBOT** รหัส **gembot123**
3. `cd pc` → `python teleop.py`
4. บรรทัดแรกในหน้าต่างต้องขึ้น `link: OK`

## ลำดับทดสอบ (บันทึกผลทุกข้อ)
| # | ทดสอบ | ผ่านเมื่อ | ถ้าไม่ผ่านแก้ที่ |
|---|---|---|---|
| 1 | link: OK | สถานะมาทุก 0.2 s | WiFi ต่อถูกเครือข่ายไหม / firewall ปิด UDP 4210 |
| 2 | กด W | ล้อทั้งสองเดินหน้า | L_INVERT / R_INVERT / สลับ MOTOR_MODE |
| 3 | กด W ค้าง แล้วดึงสาย/ปิด WiFi | ล้อหยุดใน 0.5 s | WATCHDOG_MS |
| 4 | กด W วิ่งตรง 1 m | เบี่ยง < 10 cm | L_TRIM / R_TRIM |
| 5 | กด 1–6 | servo ขยับตามค่าบนจอ → จดค่า OPEN/CLOSE/DOWN/UP/HOLD/DUMP ลง config.h | ขา servo |
| 6 | กด P มีหินใหญ่อยู่หน้าปาก | หินไปอยู่ในกระบะ | มุมและ T_* ใน config.h |
| 7 | กด P 10 ครั้งติด | link ไม่ LOST เลย | ไฟ servo ดึงจน ESP32 รีเซ็ต → แยกไฟ/เพิ่ม capacitor |
| 8 | กด O (DUMP) | ปากเปิด หินถูกปล่อย | GRIP_OPEN |

ส่งผลตาราง + ค่าที่จูนได้กลับมา แล้วไป Module 2 (กล้อง + ArUco + calibration)

---
# Module 2: Vision (calibrate_colors.py + field_vision.py)

ไฟล์ของทีม + แก้ 4 จุด: มุมหุ่นใน field frame, ตัดตัวหุ่นออกจาก gem, ล็อก exposure/WB, คลิกโซนมือถ้า HSV หาไม่เจอ

## ไฟล์เครื่องมือของทีม (อยู่ใน pc/ แล้ว)
- `find_camera.py` หา index กล้องสนาม (ที่ผ่านมาคือ 1 — โน้ตบุ๊กมีกล้องในตัวเป็น 0) → ใช้ `--source 1` กับทุกโปรแกรม
- `test_aruco.py --source 1 [--dict DICT_6X6_50]` เช็คว่า tag ที่พิมพ์ตรง dictionary ไหม (ถ้าไม่ใช่ 4x4_50 ต้องแก้ `RobotTracker(dict_name=...)` ใน auto_main/field_vision)
- `compare_colors.py --source 1` คลิกดู HSV จริงของหินสองสีที่ใกล้กัน
- `apply_calibration.py` เขียน color_profiles.json จากค่าในไฟล์ (หรือใช้ `s` ใน calibrate_colors แทน)

## ⚠️ color_profiles.json ปัจจุบันมีช่วงสีทับกัน 3 คู่ (โปรแกรมจะพิมพ์เตือนตอนเปิด)
| คู่ | ช่วงที่ทับ | แนวแก้ |
|---|---|---|
| MARIGOLD ↔ LIME | H 19–54 | ส้มจริงอยู่ H ~8–20, เขียวจริง ~35–60 → ตั้ง MARIGOLD h_max ≈ 22, LIME h_min ≈ 30 แล้วดูด้วยตา |
| NEON_CYAN ↔ SKY_BLUE | H 89–95, S 175–210 | ใช้ compare_colors คลิกหินจริง 3–4 จุด/สี แล้วเลือกแกน (S หรือ V) ที่แยกชัด ตั้งเส้นแบ่งตรงกลาง |
| VIOLET ↔ CRIMSON | H 165–168 | VIOLET h_max ≈ 160, CRIMSON h_min ≈ 168 |
หินที่ถูกจับเป็น 2 สีพร้อมกันจะถูกตัดทิ้ง (แสดง `ambiguous=` บนจอ) ไม่ถูกหยิบ — ปลอดภัยแต่เสียโอกาส ควรแก้ให้ `ambiguous=0`
DEEP_CRIMSON s_min=16 กว้างมาก ถ้าเห็นพื้นไม้ถูกจับเป็นแดง ให้ขึ้นเป็น 60+

## ลำดับใช้งาน (ทำที่สนามจริงทุกครั้งที่กล้องขยับ)
1. `python calibrate_colors.py --source 1` → ปรับทีละสี 1–6 ให้เหลือสีนั้นสีเดียว → `s` บันทึก `color_profiles.json`
   - จด "top areas" ของหินเดี่ยว ๆ ก้อนใหญ่/เล็ก → ใช้ตั้ง `gem_min_area/gem_max_area` ใน field_vision.py
2. `python field_vision.py --source 1 --setup` → คลิก 4 มุมสนาม (TL→TR→BR→BL) → โซนที่หาไม่เจอคลิกกลางวง → ได้ `field_map.json`
3. `python field_vision.py --source 1` → ดูผล: หุ่น (x,y,heading cm), หินแยกสี, หินที่วางในวงแล้ว

## ต้องตรวจ (TODO ในโค้ด)
- exposure lock ทำงานกับกล้องสนามไหม (ดูบรรทัด `[CAM] ...` ตอนเปิด ค่า auto_exp ต้องไม่ใช่ 0.75/3)
- `robot_radius_px`: รัศมีตัดหุ่น ตั้งต้น 1.8×ขนาด ArUco — ถ้าล้อยังโผล่เป็น gem ให้เพิ่ม
- ArUco ต้องติดให้ "ขอบบน" ของ tag ชี้ไปหน้าหุ่น (heading คำนวณจาก corner 0-1)

## พิกัดที่ทั้งระบบใช้ (ห้ามเปลี่ยน)
- มุมซ้ายบนของภาพ = (0,0) cm, x ไปขวาถึง 210, y ลงล่างถึง 120
- heading 0° = ชี้ +x (ขวา), +90° = ชี้ +y (ลง), -90° = ชี้ขึ้น

---
# Module 3: Auto (auto_config.py + nav.py + planner.py + auto_main.py)

```
python auto_main.py --sim            # หุ่นจำลอง ดู logic ทำงาน (ไม่ต้องมีของจริง)
python auto_main.py --source 1       # ของจริง (index จาก find_camera)  SPACE=start/stop  q=quit  b=สลับหยิบเฉพาะก้อนใหญ่
```
Loop: CHOOSE → GO_APPROACH → ALIGN → CREEP → PICK → VERIFY_PICK → (GO_ZONE → ZONE_ALIGN → REVERSE_IN → DUMP → VERIFY_DUMP → LEAVE) → CHOOSE
- หนึ่งเที่ยว = สีเดียว (`MAX_PER_TRIP` = 1 ก่อน, ผ่านแล้วค่อยลอง 2–3)
- หยิบเฉพาะหินขอบกอง (เพื่อนบ้าน ≤ `MAX_NEIGHBORS`), ถอยเข้าวงด้วยท้ายกระบะ
- ไม่เห็น ArUco 0.5 s หยุด, 5 s หมุนช้าหา · ทุก state มี timeout · หยิบไม่ได้ 2 ครั้ง = ข้ามก้อนนั้น
- หยุดเองที่ 4:50

## ลำดับจูนของจริง (ทำตามลำดับ ห้ามข้าม)
1. **วัดเรขาคณิต** ใส่ `auto_config.py`: `MARKER_TO_AXLE_CM`, `GRIP_REACH_CM`, `ROBOT_FRONT_CM`, `ROBOT_REAR_CM`
2. **ทดสอบทิศหมุน**: รัน auto_main ของจริง ให้เลือกหินสักก้อน ดู state ALIGN — ถ้าหุ่นหมุน "หนี" เป้า → `TURN_SIGN = -1`
3. **จูน go-to-point**: ถ้าส่าย ลด `K_TURN`; ถ้าเลี้ยวไม่ทัน เพิ่ม; ถ้าวิ่งเลยเป้า ลด `V_MAX` / เพิ่ม `SLOWDOWN_CM`; ถ้าหยุดไม่ถึง เพิ่ม `V_MIN`
4. **จูนหยิบ**: ดูค่า `gap=` ตอน CREEP ถ้าหุ่นหยุดแล้วปากยังไม่ถึงหิน → `GRIP_REACH_CM` ผิด แก้ก่อน อย่าไปเพิ่ม `GRAB_TOL_CM`
5. **จูนเท**: ดู `gap=` ตอน REVERSE_IN + ดูว่าหินไหลไปตกในวงไหม ปรับ `DUMP_DEPTH_FRAC`
6. ถ้าจับก้อนเล็กหลุดบ่อย → `BIG_GEM_ONLY = True` (คะแนนเท่ากัน)
7. สีไหน calibrate แล้วยังสับสน → ใส่ `SKIP_COLORS`

---
# 📌 V3 UPDATE — ไม่มีกระบะ ไม่มี servo ยกแขน (คีบค้างระดับพื้นจนถึงโซน)

**เหตุผล:** ทีมยังไม่มีกลไกเก็บ (storage) จริง — CAD ล่าสุดของเพื่อนแสดง
ปากหนีบ 2 servo (ซ้าย/ขวาอิสระ) + servo ยกแขน 1 ตัว รวม **3 servo** โดยไม่มี
ช่องเก็บแยก จึงเปลี่ยนแนวคิดเป็น "คีบเม็ดค้างไว้ในปากหนีบตลอดทางจนถึงโซนสี
แล้วค่อยปล่อยตรงนั้น" (ไม่ใช่หยิบใส่กระบะแล้วค่อยไปเทSo)

## สิ่งที่เปลี่ยนจาก README เดิมด้านบน
- **Servo:** **grip 1 ตัว** (เฟืองคุมสองข้าง, GPIO 32) ไม่มี lift ไม่มี bin — คีบแล้วลากไปโซน
- **PICK sequence:** เปิด → หนีบค้าง · **DUMP:** เปิดปล่อย
- **Protocol `S`:** **`S,<grip>,<seq>`**
- **การเข้าโซนสี (planner.py):** จาก "หมุนกลับตัวแล้วถอยเข้าด้วยท้ายกระบะ" → **"หันหน้า (ปากหนีบ) เข้าวงตรงๆ แล้วเดินหน้าเข้าไปปล่อย"**
- **ลำดับเลือกหิน:** เพิ่ม `PILE_EDGE_BIAS` ใน `auto_config.py` — เลือกหินขอบนอกของกองก่อนเสมอ (ปอกกองจากนอกเข้าใน คล้ายวิ่งเกือกม้า) แทนที่จะพุ่งเข้ากลางกองทันที
- **`MAX_PER_TRIP` ต้องเป็น 1 เท่านั้น** ห้ามปรับเพิ่ม — ปากหนีบถือได้จริงแค่ 1 เม็ด (ข้อจำกัดทางกล ไม่ใช่แค่ตัวเลขปรับได้)

## ทำต่อจากนี้ (เรียงลำดับ)
1. **เช็คขา servo** `SERVO_GRIP_PIN` (32) ใน `esp32_gembot/config.h` ให้ตรงบอร์ดจริง
2. Upload `esp32_gembot.ino` ใหม่ทับของเดิม
3. รัน `python teleop.py` (ใน pc\) → ใช้ปุ่ม **1/2** จูน `grip` จนหาค่า OPEN/CLOSE ได้
4. กด **P** (PICK) แล้วดูว่าเม็ดถูกคีบค้างไว้จริง (ไม่หลุด) จากนั้นกด **O** (DUMP) ดูว่าปล่อยได้ตอนที่ต้องการ
5. จดค่ามุมที่ใช้ได้จริงลง `GRIP_OPEN/GRIP_CLOSE` ใน `config.h`
6. ไปต่อที่ **NEXT_STEPS.md** (Phase 2 เป็นต้นไป) ตามเดิม — เรื่อง vision/geometry/navigation ไม่เปลี่ยนจากที่เคยคุยกันไว้


---
# V3 (24 ก.ย.) สรุปที่แก้จากแพ็กเกจของทีม
- ตัด lift ออกจาก firmware/link/teleop ทั้งหมด (config.h กับ .ino เดิมขัดกัน คอมไพล์ไม่ผ่าน) · protocol `S,<grip>,<seq>` (v3.2 servo ตัวเดียว)
- planner: หลังปล่อยหินให้ **ถอยออกก่อน** (LEAVE) แล้วค่อยนับ (COUNT_DUMP) เพราะหินที่เพิ่งวางอยู่ในรัศมีตัดตัวหุ่น
- auto_main เตือนถ้า `ROBOT_RADIUS_CM` เล็กกว่า marker→axle + grip reach + 3
- ลบ `config.exe` (ไฟล์ขยะจากคอมไพเลอร์) · เพิ่ม `pc/test_follow.py` และ `esp32_gembot/motor_test/`

---
# สถานะล่าสุด 24 ก.ย. (v3.2 final) — ค่าที่ยืนยันแล้วกับหุ่นจริง
- มอเตอร์: L 27/26, R 16/17, `R_INVERT true` → W เดินหน้า A ซ้าย D ขวา S ถอย, วิ่งตรง 1 m ผ่าน
- ปากหนีบ: servo หลัก GPIO 32 ช่วงใช้งาน 0 (หุบสุด) – 80 (อ้าสุด) → `GRIP_OPEN 75`, `GRIP_CLOSE 0`, `T_GRIP 900`
- servo GPIO 19 ขบเฟืองเดียวกันแต่ **ไม่ใช้** (`GRIP_ASSIST 0`) — เปิด 2 ตัวแล้วสู้กัน ค่าไม่แน่นอน
- field_map.json + color_profiles.json ทำแล้ว (Phase 0 ผ่าน)
- auto_config: MARKER_TO_AXLE 8.6, GRIP_REACH 20.6, ROBOT_FRONT 24, ROBOT_REAR 10 (วัดยืนยัน FRONT/REAR อีกครั้ง)

## ที่ยังต้องทำ (เรียงตามลำดับ)
1. Phase 1 ข้อ 8: หนีบหินแล้ว W 1 m + A หมุน — ใหญ่/เล็ก หลุดไหม → เล็กหลุด = `BIG_GEM_ONLY = True`
2. `python test_follow.py --source 1` คลิกจุด → เช็ค TURN_SIGN, จูน nav → เลือกสีให้เดินหาหิน
3. `python auto_main.py --source 1` loop เต็ม

---
# 25 ก.ย. — แก้เรื่องทิศ/การมองเห็นหิน (v3.3)
- **บริเวณตัดตัวหุ่นเปลี่ยนจากวงกลมใหญ่เป็นแคปซูล 2 ท่อน** (ตัวหุ่นกว้าง + แขนแคบ) → เห็นหินบนพื้นได้จนปากห่างหิน ~4.5 cm (เดิมวงกลมกลืนหินหน้าหุ่นไป 26 cm ทำให้เป้าหายก่อนถึง)
- **test_follow: คลิกไม่ตั้งเป้าอีกแล้วจนกว่าจะกด `c`** (โหมด CLICK) — ก่อนหน้านี้คลิกเพื่อ focus หน้าต่างกลายเป็นสั่งหุ่นไปจุดที่คลิก ทำให้ "เดินทางเดียวไม่สนหิน"
- test_follow จำหินล่าสุด 2 วิ ตอนหินเข้าใกล้ปากจนถูกตัด
- planner: PICK → **PICK_BACKOFF** (ถอย 1.2 วิ) → VERIFY_PICK เช็คว่าหินยังอยู่ที่พื้นตรงเดิมไหม (ตัดสินได้จริงว่าหยิบติด)
- `HEADING_OFFSET_DEG` ใน auto_config ชดเชย ArUco หันผิดด้านโดยไม่ต้องแกะ tag
- ความเร็วรีเซ็ต (V_MAX 60 / TURN 15–40 / CREEP 25) และ `MOTOR_MIN_PWM 140`
- `config_swapLR.h` = config ที่สลับซ้าย-ขวา ใช้ถ้า teleop กด A แล้วหุ่นหมุนตามเข็ม

---
# v3.4 (25 ก.ย. เย็น) — แก้ 4 ไฟล์ใน pc/ จาก baseline v3.3 (config.h / firmware / nav / field_vision ไม่แตะ)
| ไฟล์ | แก้อะไร | ทำไม |
|---|---|---|
| link.py | keepalive thread ส่งคำสั่งล้อล่าสุดซ้ำทุก 50 ms + ส่ง PICK/DUMP ซ้ำเอง; ส่งไม่ได้ (WiFi หลุด) ไม่ crash ตั้ง `net_error` | โปรแกรมภาพช้า 2–4 fps → watchdog ESP32 ตัดมอเตอร์ระหว่างเฟรม หุ่นกระตุก/ไม่ขยับ |
| test_follow.py | ตรวจหินทุก 3 เฟรม, แสดง fps, ใช้ keepalive, `k` รายงานอย่างเดียว+ตรวจค่าเพี้ยน, `c` เปิดโหมดคลิก | loop เร็วขึ้น ~3 เท่า, calibrate ไม่ถูกหลอก |
| auto_main.py | ตรวจหินทุก 3 เฟรม, แสดง fps, ใช้ keepalive | เหมือนกัน |
| planner.py | `step()` รับ `gems_cm=None` = เฟรมที่ไม่ได้ตรวจหิน ใช้ track เดิม | รองรับตรวจหินทุก 3 เฟรม |

# v3.5 (25 ก.ย. ค่ำ) — แก้ 3 ไฟล์จาก v3.4
| ไฟล์ | แก้ | ทำไม |
|---|---|---|
| nav.py | ท่อนแขนของบริเวณตัดตัวหุ่นสั้นลง หยุดก่อนปาก 8 cm; วงตัด "หินในปาก" เพิ่มเฉพาะเมื่อ holding=True | หินที่วางหน้าปาก (~25 cm) เคยถูกตัดเป็น "ตัวหุ่น" → "no stable gem" ไม่ยอมเดิน |
| auto_main.py | ส่ง holding=(มีหินในปาก) ให้ nav | ตามข้อบน |
| test_follow.py | ปุ่ม w/s/a/d = พัลส์มอเตอร์ 1 วิ ผ่านโปรแกรมนี้; แสดง ack seq จาก ESP32; calibrate ใช้ speed 35/30; เตือนให้ปิด teleop ก่อน | แยกให้ออกว่าหุ่นไม่ขยับเพราะอะไร |

# v3.6 (26 ก.ย.) — แก้ 1 ไฟล์: pc/field_vision.py
- `--setup` (ไม่ใส่ --manual-zones) หาวงโซนด้วย **รูปร่าง** (blob สีอิ่มในสนาม กลม รัศมี ≈ 10 cm จาก homography) แล้วจำแนกสีจาก hue กลางวง เทียบ `ZONE_REF_HUE` — ไม่พึ่ง color_profiles ของหินอีกต่อไป
- ฟ้า 2 เฉด (NEON_CYAN / DEEP_SKY_BLUE) hue ใกล้กัน ตัดสินด้วยความอิ่มสี: เข้มกว่า = DEEP_SKY_BLUE
- ทดสอบกับภาพสนามจริง 3 ภาพ: จำแนกถูก 6/6, 6/6, 5/6 (วงเขียวถูกโน้ตบุ๊กบัง)
- วงที่หาไม่เจอยังให้คลิกมือเหมือนเดิม · `--manual-zones` ยังใช้ได้
