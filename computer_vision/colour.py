import numpy as np
import cv2
import time

cap = cv2.VideoCapture(1)

cap.set(3,1024)
cap.set(4,768)
#cap.set(10, 50)
#160.0 x 120.0
#176.0 x 144.0
#320.0 x 240.0
#352.0 x 288.0
#640.0 x 480.0
#1024.0 x 768.0
#1280.0 x 1024.0
time.sleep(2)

font = cv2.FONT_HERSHEY_SIMPLEX

victim_lower = np.array([35, 60, 200], np.uint8)
victim_upper = np.array([60, 255, 255], np.uint8)

cyan_lower = np.array([60, 60, 200], np.uint8)
cyan_upper = np.array([140, 255, 255], np.uint8)

magenta_lower = np.array([125, 60, 200], np.uint8)
magenta_upper = np.array([165, 255, 255], np.uint8)

while True:
    _, img = cap.read()

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    cyan = cv2.inRange(hsv, cyan_lower, cyan_upper)

    kernel_cyan = np.ones((5, 5), "uint8")
    blue_cyan = cv2.dilate(cyan, kernel_cyan)
    res_cyan = cv2.bitwise_and(img, img, mask=cyan)

    (_, contours, hierarchy) = cv2.findContours(cyan, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 30:
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(img, [box], 0, (0, 0, 255), 2)

            M = cv2.moments(contour)
            Rcx = int(M['m10']/M['m00'])
            Rcy = int(M['m01']/M['m00'])
            cv2.circle(img, (Rcx, Rcy), 2, (0, 0, 0), 2)

    magenta = cv2.inRange(hsv, magenta_lower, magenta_upper)

    kernel_magenta = np.ones((5, 5), "uint8")
    blue_magenta = cv2.dilate(magenta, kernel_magenta)
    res_magenta = cv2.bitwise_and(img, img, mask=magenta)

    (_, contours, hierarchy) = cv2.findContours(magenta, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 30:
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(img, [box], 0, (0, 0, 255), 2)

            M = cv2.moments(contour)
            Lcx = int(M['m10']/M['m00'])
            Lcy = int(M['m01']/M['m00'])
            cv2.circle(img, (Lcx, Lcy), 2, (0, 0, 0), 2)
    
    '''if Lcx and Lcy and Rcx and Rcy:
        #Lcx, Lcy, Rcx, Rcy = 0, 0, 0, 0

        cv2.line(img, (Lcx, Lcy), (Rcx, Rcy), (0, 0, 0), 3)
        mx = 0.5*(Lcx + Rcx)
        my = 0.5*(Lcy + Rcy)
        cv2.circle(img, (int(mx), int(my)), 2, (0, 255, 255), 2)

        #cv2.line(img, (x1, y1), (x2, y2), (0, 0, 0), 3)

        v_dir = np.array([Lcx - Rcx, Lcy - Rcy])
        v_dir_rot = np.array([-v_dir[1], v_dir[0]])
        vu_dir_rot = v_dir_rot / np.linalg.norm(v_dir_rot)
        
        l = 50
        
        ex = mx + v_dir_rot[0] * l
        ey = my + v_dir_rot[1] * l

        #print([ex, ey])

        cv2.line(img, (int(mx), int(my)), (int(ex), int(ey)), (0, 0, 0), 3)
    '''
    victim = cv2.inRange(hsv, victim_lower, victim_upper)

    kernel_victim = np.ones((5, 5), "uint8")
    blue_victim = cv2.dilate(victim, kernel_victim)
    res_victim = cv2.bitwise_and(img, img, mask=victim)

    (_, contours, hierarchy) = cv2.findContours(victim, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        # if(area>5):
        x, y, w, h = cv2.boundingRect(contour)
        img = cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 3)

    cv2.imshow("Color Tracking", img)
    img = cv2.flip(img, 1)

    key = cv2.waitKey(1)
    if key & 0xFF == ord('n'):
        continue
    elif key & 0xFF == ord('s'):
        cv2.imwrite('processed_capture_{}.png'.format(
            str(int(time.time()))), img)
    elif key & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()