"""link.py — UDP link โน้ตบุ๊ก <-> ESP32 (GEMBOT)

ใช้:
    from link import RobotLink
    link = RobotLink()          # ต่อ WiFi "GEMBOT" ก่อน
    link.drive(40, 40)          # ล้อซ้าย/ขวา -100..100
    link.pick(); link.dump()
    link.busy                   # True ขณะหุ่นทำ sequence
    link.alive                  # False ถ้าไม่ได้ status > 1 s

v3.4: มี keepalive thread — คำสั่งล้อล่าสุดถูกส่งซ้ำทุก 50 ms เองแม้โปรแกรมหลักจะช้า
      (ประมวลผลภาพ 2-4 fps) ทำให้ watchdog บน ESP32 ไม่ตัดมอเตอร์ระหว่างรอเฟรม
"""
import socket, threading, time

ROBOT_IP   = "192.168.4.1"
ROBOT_PORT = 4210


class RobotLink:
    def __init__(self, ip=ROBOT_IP, port=ROBOT_PORT):
        self.addr = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.2)
        self.seq = 0
        # สถานะจากหุ่น
        self.busy = False
        self.seq_ack = -1
        self.vbat = -1.0
        self.last_status = 0.0
        # คำสั่งครั้งเดียว (PICK/DUMP) ที่ต้องการ ack — ส่งซ้ำจนกว่า ESP32 จะรายงาน seq >= ที่ส่ง
        self._pending = None          # (text, seq, last_sent_time)
        self._cmd = (0, 0)            # คำสั่งล้อล่าสุด (keepalive ส่งซ้ำ)
        self._cmd_t = 0.0
        self.net_error = False        # True เมื่อส่งไม่ได้ (WiFi หลุด)
        self._run = True
        threading.Thread(target=self._rx_loop, daemon=True).start()
        threading.Thread(target=self._keepalive_loop, daemon=True).start()
        self.send("PING")

    # ---------- ส่ง ----------
    def send(self, text: str):
        try:
            self.sock.sendto((text + "\n").encode(), self.addr)
            self.net_error = False
        except OSError:
            self.net_error = True     # WiFi GEMBOT หลุด — ไม่ crash โปรแกรม

    def _next(self):
        self.seq += 1
        return self.seq

    def drive(self, vl: int, vr: int):
        vl = max(-100, min(100, int(vl)))
        vr = max(-100, min(100, int(vr)))
        self._cmd = (vl, vr); self._cmd_t = time.time()
        self.send(f"M,{vl},{vr},{self._next()}")

    def stop(self):
        self._cmd = (0, 0); self._cmd_t = time.time()
        self.send("STOP")

    def _keepalive_loop(self):
        """ส่งคำสั่งล้อล่าสุดซ้ำทุก 50 ms + ส่ง PICK/DUMP ซ้ำถ้ายังไม่ได้ ack"""
        while self._run:
            time.sleep(0.05)
            try:
                if time.time() - self._cmd_t > 0.045:
                    vl, vr = self._cmd
                    self.send(f"M,{vl},{vr},{self._next()}")
                    self._cmd_t = time.time()
                self.tick()
            except Exception:
                pass

    def _send_reliable(self, cmd: str):
        seq = self._next()
        text = f"{cmd},{seq}"
        self.busy = True          # ตั้งเองก่อน กัน race จน status ตัวถัดไปมา
        self._pending = [text, seq, time.time()]
        self.send(text)

    def pick(self):
        self._send_reliable("PICK")

    def dump(self):
        """v3: DUMP = เปิดปากปล่อยหินที่โซนสี"""
        self._send_reliable("DUMP")

    def tick(self):
        """เรียกทุกเฟรม: ส่ง PICK/DUMP ซ้ำถ้ายังไม่ได้ ack ภายใน 0.3 วิ (กัน UDP หาย)
        ESP32 จะไม่เริ่ม sequence ซ้ำถ้ากำลัง busy อยู่ จึงส่งซ้ำได้ปลอดภัย"""
        if self._pending and time.time() - self._pending[2] > 0.3:
            self.send(self._pending[0]); self._pending[2] = time.time()

    def servo(self, grip: int):
        """servo หลักของปากหนีบ (GPIO 32)"""
        self.send(f"S,{grip},{self._next()}")

    def assist(self, angle: int):
        """servo ตัวช่วย (GPIO 19) ใช้ตอนจูนให้ตรงกับตัวหลัก"""
        self.send(f"A,{angle},{self._next()}")

    # ---------- รับ ----------
    @property
    def alive(self) -> bool:
        return time.time() - self.last_status < 1.0

    def wait_idle(self, timeout=6.0) -> bool:
        """รอจนหุ่นทำ sequence เสร็จ คืน False ถ้า timeout"""
        t0 = time.time()
        time.sleep(0.3)                       # ให้ status รอบแรกมาก่อน
        while time.time() - t0 < timeout:
            if self.alive and not self.busy:
                return True
            time.sleep(0.05)
        return False

    def _rx_loop(self):
        while self._run:
            try:
                data, _ = self.sock.recvfrom(128)
            except socket.timeout:
                continue
            except OSError:
                break
            for line in data.decode(errors="ignore").split("\n"):
                p = line.strip().split(",")
                if p[0] == "ST" and len(p) >= 5:
                    self.seq_ack = int(p[1])
                    if self._pending and self.seq_ack >= self._pending[1]:
                        self._pending = None              # ESP32 รับคำสั่งแล้ว
                    # ระหว่างรอ ack ให้ถือว่า busy ต่อ (status busy=0 ที่มาก่อน ack คือของเก่า)
                    self.busy = (p[2] == "1") or (self._pending is not None)
                    self.vbat = float(p[3])
                    self.last_status = time.time()

    def close(self):
        self._run = False
        try:
            self.stop()
        finally:
            self.sock.close()
