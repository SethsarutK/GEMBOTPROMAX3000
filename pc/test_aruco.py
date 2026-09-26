"""
test_aruco.py
==============
ทดสอบตรวจจับ ArUco marker จากกล้องสนาม (แยกไฟล์ไว้ทดสอบเร็ว ๆ
ก่อนไปรวมกับ field_vision.py จริง)

⚠️ สำคัญ: dictionary ต้องตรงกับ tag ที่พิมพ์ออกมาจริง ถ้าพิมพ์มาคนละ
dictionary กับที่ตั้งในโค้ด จะตรวจจับไม่เจอเลย (ไม่ error แค่ไม่เจอ)
field_vision.py (RobotTracker) ในโปรเจกต์นี้ตั้งค่าเริ่มต้นเป็น
DICT_4X4_50 ไว้ — ถ้าจะใช้ tag เดียวกันกับที่ track หุ่นจริง ให้พิมพ์
tag จาก dictionary นี้ หรือถ้าพิมพ์ 6x6 มาแล้ว ก็สั่ง --dict DICT_6X6_50
ตอนรันไฟล์นี้ แล้วอย่าลืมไปแก้ RobotTracker ให้ตรงกันด้วย (ดูท้ายไฟล์)

วิธีใช้:
    python test_aruco.py --source 1                    # dict เริ่มต้น 4x4
    python test_aruco.py --source 1 --dict DICT_6X6_50  # ถ้า tag เป็น 6x6
"""

import cv2
import argparse

# dictionary ที่เลือกได้ทั้งหมด (พิมพ์ให้ตรงกับที่จะใช้จริง)
DICT_MAP = {
    "DICT_4X4_50":  cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_5X5_50":  cv2.aruco.DICT_5X5_50,
    "DICT_6X6_50":  cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="1",
                    help="camera index ของกล้องสนาม (จาก find_camera.py)")
    ap.add_argument("--dict", default="DICT_4X4_50", choices=list(DICT_MAP.keys()),
                    help="ต้องตรงกับ dictionary ของ tag ที่พิมพ์จริง")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    args = ap.parse_args()

    cap = cv2.VideoCapture(int(args.source), cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        print(f"[ERROR] เปิดกล้อง index {args.source} ไม่ได้")
        return

    aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_MAP[args.dict])
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    print(f"[INFO] source=index {args.source}   dict={args.dict}")
    print("กด q เพื่อออก\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] อ่านเฟรมจากกล้องไม่ได้")
            break

        corners, ids, rejected = detector.detectMarkers(frame)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            status = f"เจอ {len(ids)} tag: {ids.flatten().tolist()}"
        else:
            status = "ไม่เจอ tag"

        cv2.putText(frame, f"{status}   [{args.dict}]", (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
        cv2.putText(frame, f"{status}   [{args.dict}]", (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

        cv2.imshow("ArUco Marker Detection (field camera)", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

# -----------------------------------------------------------------
# หลังจากรู้แน่ชัดแล้วว่าใช้ dictionary ไหน ให้ไปแก้ที่
# field_vision.py -> class RobotTracker -> __init__(...)
# ให้ dict_name ตรงกับที่ทดสอบเจอในไฟล์นี้ เช่น
#     tracker = RobotTracker(marker_id=0, dict_name=cv2.aruco.DICT_6X6_50)
# ไม่งั้นตอนรัน field_vision.py จริง จะหา tag ไม่เจอทั้งที่ไฟล์นี้เจอ
# -----------------------------------------------------------------
