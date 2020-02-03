import numpy as np
import cv2
import time
from cv2 import aruco
from glob import glob

caps = glob('./computer_vision/test_caps/*')

for cap in caps:
    frame = cv2.imread(cap)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(
        gray, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(frame, corners, ids)

    if corners:

        c1 = corners[0][0, 0]
        c2 = corners[0][0, 1]
        c3 = corners[0][0, 2]
        c4 = corners[0][0, 3]

        e1 = (c2-c1)
        e2 = (c3-c2)
        e3 = (c4-c3)
        e4 = (c1-c4)

        m1 = c1 + 0.5*e1
        m2 = c2 + 0.5*e2
        m3 = c3 + 0.5*e3
        m4 = c4 + 0.5*e4

        b1 = c1 - e1 * 0.65 + e4 * 1.20
        b2 = c2 + e1 * 0.65 + e4 * 1.20
        b3 = c3 + e1 * 0.65 - e4 * 0.60
        b4 = c4 - e1 * 0.65 - e4 * 0.60

        central_parallel = (m1-m3)/np.linalg.norm(m1-m3)
        cp_f_l = 300
        cp_f_e = m1 + cp_f_l * central_parallel

        cp_r_l = 50
        cp_r_e = m3 - cp_r_l * central_parallel

        cv2.circle(frame, tuple(m1), 5, (0, 255, 0), 2)
        cv2.circle(frame, tuple(m2), 5, (0, 255, 0), 2)
        cv2.circle(frame, tuple(m3), 5, (0, 255, 0), 2)
        cv2.circle(frame, tuple(m4), 5, (0, 255, 0), 2)

        cv2.line(frame, tuple(m3), tuple(m1), (0, 128, 0), 2)

        cv2.arrowedLine(frame, tuple(m1), tuple(cp_f_e), (0, 128, 0), 2)
        cv2.line(frame, tuple(m3), tuple(cp_r_e), (0, 128, 0), 2)
        cv2.circle(frame, tuple(cp_r_e), 5, (0, 128, 0), 2)

        cv2.circle(frame, tuple(b1), 5, (255, 0, 0), 2)
        cv2.circle(frame, tuple(b2), 5, (255, 0, 0), 2)
        cv2.circle(frame, tuple(b3), 5, (255, 0, 0), 2)
        cv2.circle(frame, tuple(b4), 5, (255, 0, 0), 2)

    cv2.imshow("Aruco Tracking", frame)
    #frame = cv2.flip(frame, 1)

    key = cv2.waitKey(0)
    if key & 0xFF == ord('n'):
        continue
    elif key & 0xFF == ord('s'):
        cv2.imwrite('processed_capture_{}.png'.format(
            str(int(time.time()))), frame)
    elif key & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
