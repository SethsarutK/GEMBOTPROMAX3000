"""
find_camera.py
===============
เช็คว่ากล้องสนามที่เสียบ USB ไปคือ index ไหน (เครื่องอาจมีกล้องในตัว
โน้ตบุ๊กเป็น index 0 อยู่แล้ว กล้องสนามอาจไปโผล่ที่ index 1, 2, ...)

วิธีใช้:
    python find_camera.py

จะไล่เปิดกล้อง index 0-4 ทีละตัว โชว์ภาพให้ดูว่าใช่กล้องสนามไหม
กด n = ไปกล้องตัวถัดไป, q = จบ (จำ index ที่ใช่ไว้)

บน Windows ใช้ cv2.CAP_DSHOW แทน backend default เพราะ default (MSMF)
บางทีเปิดกล้อง USB ภายนอกช้ามาก หรือได้ความละเอียดผิดเพี้ยน
"""

import cv2

MAX_INDEX_TO_TRY = 5


def try_camera(idx):
    # CAP_DSHOW เร็วกว่าและเสถียรกว่าบน Windows สำหรับกล้อง USB ภายนอก
    cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap.release()
        return None
    ok, frame = cap.read()
    if not ok or frame is None:
        cap.release()
        return None
    return cap


def main():
    print("กำลังไล่หากล้อง... กด n = ตัวถัดไป, q = จบ (ปิดโปรแกรม)\n")
    idx = 0
    cap = None

    while idx < MAX_INDEX_TO_TRY:
        if cap is not None:
            cap.release()
        cap = try_camera(idx)
        if cap is None:
            print(f"index {idx}: เปิดไม่ได้ / ไม่มีกล้อง")
            idx += 1
            continue

        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"index {idx}: เปิดได้ -> ความละเอียดปัจจุบัน {w}x{h}")

        win = f"camera index {idx}  (n=next, q=quit)"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            cv2.putText(frame, f"index={idx}  {w}x{h}  [n]=next [q]=quit",
                        (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
            cv2.putText(frame, f"index={idx}  {w}x{h}  [n]=next [q]=quit",
                        (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.imshow(win, frame)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("n"):
                break
            if k == ord("q"):
                cap.release()
                cv2.destroyAllWindows()
                print(f"\n--> จำไว้: ถ้า index {idx} คือกล้องสนาม ใช้ "
                      f"--source {idx} ตอนรัน calibrate_colors.py / field_vision.py")
                return
        cv2.destroyWindow(win)
        idx += 1

    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()
    print("\nลองครบทุก index แล้ว ถ้ายังไม่เจอกล้องสนาม ลองถอด/เสียบ USB ใหม่ "
          "หรือเช็คใน Device Manager (Windows) ว่ากล้องถูกจำเป็นอุปกรณ์หรือยัง")


if __name__ == "__main__":
    main()
