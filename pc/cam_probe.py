"""cam_probe.py — ลองตั้งค่ากล้องหลายแบบ ดูว่าแบบไหนได้ fps สูงสุด (แก้ปัญหา 4 fps)

    python cam_probe.py --source 1
"""
import argparse, time, os
import cv2

os.chdir(os.path.dirname(os.path.abspath(__file__)))


def measure(cap, n=30):
    for _ in range(5):
        cap.read()
    t = time.perf_counter()
    ok = 0
    for _ in range(n):
        if cap.read()[0]:
            ok += 1
    dt = (time.perf_counter() - t) / n * 1000
    return dt, ok


def fourcc_str(v):
    v = int(v)
    return "".join(chr((v >> (8 * i)) & 0xFF) for i in range(4))


def trial(idx, backend, name, w, h, mjpg, fps, exposure, auto_exp):
    cap = cv2.VideoCapture(idx, backend)
    if not cap.isOpened():
        print(f"{name:34s} เปิดไม่ได้"); return None
    if mjpg:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    if fps:
        cap.set(cv2.CAP_PROP_FPS, fps)
    if auto_exp is not None:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, auto_exp)
    if exposure is not None:
        cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
    dt, ok = measure(cap)
    got = f"{cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f} {fourcc_str(cap.get(cv2.CAP_PROP_FOURCC))}"
    print(f"{name:34s} {dt:6.1f} ms/เฟรม = {1000 / dt:4.0f} fps   ({got}, exp={cap.get(cv2.CAP_PROP_EXPOSURE)})")
    cap.release()
    time.sleep(0.5)
    return dt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", type=int, default=1)
    a = ap.parse_args()
    i = a.source
    D = cv2.CAP_DSHOW; M = cv2.CAP_MSMF
    res = {}
    res["ตอนนี้ (DSHOW 720p, manual exp -4)"] = trial(i, D, "ตอนนี้ (DSHOW 720p, manual exp -4)", 1280, 720, False, None, -4, 0.25)
    res["DSHOW 720p + MJPG"] = trial(i, D, "DSHOW 720p + MJPG", 1280, 720, True, 30, None, None)
    res["DSHOW 720p + MJPG + exp -4"] = trial(i, D, "DSHOW 720p + MJPG + exp -4", 1280, 720, True, 30, -4, 0.25)
    res["DSHOW 720p + MJPG + exp -6"] = trial(i, D, "DSHOW 720p + MJPG + exp -6", 1280, 720, True, 30, -6, 0.25)
    res["DSHOW 720p auto exposure"] = trial(i, D, "DSHOW 720p auto exposure", 1280, 720, False, 30, None, 0.75)
    res["DSHOW 640x480 (ไม่ MJPG)"] = trial(i, D, "DSHOW 640x480 (ไม่ MJPG)", 640, 480, False, 30, -4, 0.25)
    res["MSMF 720p + MJPG"] = trial(i, M, "MSMF 720p + MJPG", 1280, 720, True, 30, None, None)
    res["MSMF 720p + MJPG + exp -4"] = trial(i, M, "MSMF 720p + MJPG + exp -4", 1280, 720, True, 30, -4, 0.25)
    best = min((v, k) for k, v in res.items() if v is not None)
    print(f"\nเร็วสุด: {best[1]}  ({1000 / best[0]:.0f} fps)")


if __name__ == "__main__":
    main()
