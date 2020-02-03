import numpy as np
import cv2
from cv2 import aruco
import time

bot_width = 0.65
bot_front = 1.20
bot_rear = 0.60

def bot_width_trackbar(val):
    global bot_width
    bot_width = val/100
    cv2.setTrackbarPos('Width', 'Bars', int(bot_width*100))

def bot_front_trackbar(val):
    global bot_front
    bot_front = val/100
    cv2.setTrackbarPos('Front', 'Bars', val)

def bot_rear_trackbar(val):
    global bot_rear
    bot_rear = val/100
    cv2.setTrackbarPos('Rear', 'Bars', val)

cv2.namedWindow('Bars')
cv2.createTrackbar('Width', 'Bars', int(bot_width*100), 200, bot_width_trackbar)
cv2.createTrackbar('Front', 'Bars', int(bot_front*100), 200, bot_front_trackbar)
cv2.createTrackbar('Rear', 'Bars', int(bot_rear*100), 200, bot_rear_trackbar)

#cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
cap = cv2.VideoCapture('http://localhost:8080/')
#cap.set(6, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(3,800)
cap.set(4,600)

while True:
    _, frame = cap.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
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

        b1 = c1 - e1 * bot_width + e4 * bot_front
        b2 = c2 + e1 * bot_width + e4 * bot_front
        b3 = c3 + e1 * bot_width - e4 * bot_rear
        b4 = c4 - e1 * bot_width - e4 * bot_rear

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

        cv2.line(frame, tuple(b1), tuple(b2), (255, 0, 0), 2)
        cv2.line(frame, tuple(b2), tuple(b3), (255, 0, 0), 2)
        cv2.line(frame, tuple(b3), tuple(b4), (255, 0, 0), 2)
        cv2.line(frame, tuple(b4), tuple(b1), (255, 0, 0), 2)
    
    #frame = cv2.flip(frame, 1)
    cv2.imshow("Aruco Tracking", frame)

    key = cv2.waitKey(1)
    if key & 0xFF == ord('n'):
        continue
    elif key & 0xFF == ord('s'):
        cv2.imwrite('processed_capture_{}.png'.format(
            str(int(time.time()))), frame)
    elif key & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()