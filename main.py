import cv2
import math
import numpy as np

from qr_localization import QRLocalizer
from path_planner import PathDrawer
from pure_pursuit import PurePursuit
from robot_client import RobotClient
import config as cfg



def draw_heading_arrow(frame, cx, cy, heading_deg, pts=None, color=(255, 0, 0)):
    if pts is not None:
        length = float(np.linalg.norm(pts[0] - pts[1]))
    else:
        length = 10
    length = max(length, 10)

    rad = math.radians(heading_deg)
    start = (int(cx), int(cy))
    tip = (int(cx + length * math.cos(rad)), int(cy + length * math.sin(rad)))

    cv2.arrowedLine(frame, start, tip, color, 3, tipLength=0.3)

def main():
    cap = cv2.VideoCapture(cfg.CAM_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FPS, 60)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.50)   # manual untuk DSHOW
    cap.set(cv2.CAP_PROP_EXPOSURE, -4)

    if not cap.isOpened():
        print("Gagal membuka kamera. Cek CAM_INDEX di config.py.")
        return
    
    localizer = QRLocalizer()
    drawer = PathDrawer(cfg.WINDOW_NAME)
    robot = RobotClient(cfg.ESP32_IP, cfg.ESP32_PORT)

    pursuit = None
    following = False
    marker_lost = False

    print("Kontrol: [drag mouse] gambar path | 'g' mulai jalan | 's' stop | 'r' reset | 'q' keluar")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Gagal membaca frame kamera.")
                break

            pos, heading = None, None
            result = localizer.detect(frame)
            if result:
                cx, cy, heading, pts = result
                pos = (cx, cy)
                cv2.polylines(frame, [pts.astype(int)], True, (0, 255, 0), 2)
                cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                draw_heading_arrow(frame, cx, cy, heading, pts)
                #cv2.putText(frame, f"heading: {heading:.1f}", (int(cx) + 10, int(cy) + 20),
                #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            frame = drawer.draw_overlay(frame)

            if pursuit is not None:
                drawer.draw_smoothed_path(frame, pursuit.path)
  
            if following and pursuit is not None:
                if pos is not None:
                    if marker_lost:               
                        pursuit.reset_controller()
                        marker_lost = False       

                    left, right, target = pursuit.compute(pos, heading)
                    robot.send_motor(left, right)
                    cv2.circle(frame, tuple(target.astype(int)), 6, (255, 0, 255), -1)

                    kp, ki, kd = pursuit.controller.last_gains
                    cv2.putText(frame, f"Kp={kp:.2f} Ki={ki:.2f} Kd={kd:.2f}",
                                (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                    cv2.putText(frame, f"slow={pursuit.last_slow_factor:.2f}", (10, 75),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
                    if pursuit.is_finished(pos):
                        following = False
                        robot.stop()
                        print("Path selesai.")
                else:
                    marker_lost = True
                    robot.stop()
                    cv2.putText(frame, "QR TIDAK TERDETEKSI - ROBOT BERHENTI", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            status = "FOLLOWING" if following else "IDLE"
            cv2.putText(frame, f"Status: {status}", (10, frame.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            cv2.imshow(cfg.WINDOW_NAME, frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('r'):
                drawer.reset()
                pursuit = None
                following = False
                robot.stop()
                print("Path direset.")
            elif key == ord('g'):
                smoothed = drawer.get_smoothed_path()
                if smoothed is not None and len(smoothed) >= cfg.PATH_MIN_POINTS_TO_RUN:
                    pursuit = PurePursuit(smoothed)
                    following = True
                    marker_lost = False
                    print(f"Mulai mengikuti path ({len(smoothed)} titik).")
                else:
                    print("Path belum digambar / terlalu pendek. Drag mouse dulu di window kamera.")
            elif key == ord('s'):
                following = False
                robot.stop()
                print("Stop dipaksa.")

    finally:
        robot.close()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()