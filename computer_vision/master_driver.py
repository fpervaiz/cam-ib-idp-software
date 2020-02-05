import numpy as np
import paho.mqtt.client as mqtt

import cv2
import time

from cv2 import aruco

bot_width = 0.40
bot_front = 0.75
bot_rear = 0.60

victim_lower = np.array([35, 70, 150], np.uint8)
victim_upper = np.array([60, 255, 255], np.uint8)

victim_detection_region = np.array([[230, 25],[750, 25],[750, 240],[570, 240],[570, 450],[750, 450],[750, 700],[230, 700]], np.int32).reshape((-1,1,2))
triage_region = np.array([[810, 570], [1000, 573], [1000, 718], [808, 718]], np.int32).reshape((-1,1,2))

victim_contour_min_area = 25

victims = [(0,0), (0,0), (0,0), (0,0)]

send_vectors = False
stage = -1
prev_cmd_move = -1

angle_threshold = 10
angle_threshold_pickup = 5

point_cave_exit_positioning = np.array([480, 335])
point_cave_exit_line = np.array([545, 335])

topic_bot_stt = "/idp/bot/stt"
topic_bot_serial = "/idp/bot/serial"
topic_bot_cmd = "/idp/bot/cmd"
topic_bot_stage = "/idp/bot/stage"

font = cv2.FONT_HERSHEY_SIMPLEX

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def unit_cross_prod(v1, v2):
    return np.cross(unit_vector(v1), unit_vector(v2))


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

def vic_min_area(val):
    global victim_contour_min_area
    victim_contour_min_area = val
    cv2.setTrackbarPos('Rear', 'Bars', val)

def print_mouse_coords(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONUP:
        print([x, y])

def on_disconnect(client, userdata,rc=0):
    #logging.debug("DisConnected result code "+str(rc))
    client.loop_stop()
    
def on_message(client, userdata, message):
    global stage

    payload = str(message.payload.decode("utf-8"))
    
    print("MQTT received: message topic=", message.topic, "payload=", payload, "\n")

    if message.topic == topic_bot_stt:
        if int(payload) == 0:#and stage == -1:
            # Arduino start ping
            stage = 0

        elif int(payload) == 1 and stage == 0:
            # Exiting start box
            stage = 1
            
        elif int(payload) == 2 and stage == 1:
            # Line following - start to cave entry
            stage = 2

        elif int(payload) == 3 and stage == 2:
            # Search and pick up
            stage = 3

        elif int(payload) == 4 and stage == 3:
            # Detecting victim health
            stage = 4

        elif int(payload) == 5 and stage == 4:
            # Loading victim
            stage = 5

        elif int(payload) == 6 and stage == 5:
            # Navigating to cave exit
            stage = 6
            
        elif int(payload) == 7 and stage == 6:
            # Bot line following - cave exit to triage area
            stage = 7

        elif int(payload) == 8 and stage == 7:
            # Unloading victim
            stage = 8

        else:
            # Undefined
            pass
            
        #print("Entered stage {}".format(stage))

def send_move_command(curr, prev):
    global prev_cmd_move
    if not curr == prev:
        mqc.publish(topic_bot_cmd, curr)
        prev_cmd_move = curr
        print("Command move: {}".format(curr))

mqc = mqtt.Client("MASTER")
mqc.connect("192.168.137.1")
mqc.on_message = on_message
mqc.on_disconnect = on_disconnect
mqc.loop_start()

mqc.subscribe(topic_bot_stt)
#mqc.subscribe(topic_bot_serial)
'''
cv2.namedWindow('Bars')
cv2.createTrackbar('Width', 'Bars', int(bot_width*100), 200, bot_width_trackbar)
cv2.createTrackbar('Front', 'Bars', int(bot_front*100), 200, bot_front_trackbar)
cv2.createTrackbar('Rear', 'Bars', int(bot_rear*100), 200, bot_rear_trackbar)
cv2.createTrackbar('Min Area', 'Bars', int(victim_contour_min_area), 200, vic_min_area)
'''
cv2.namedWindow('Vision')
cv2.setMouseCallback('Vision', print_mouse_coords)

cap = cv2.VideoCapture(0)
#cap = cv2.VideoCapture('http://localhost:8080/')
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

while True:
    ### FRAME UPDATE ###

    _, frame = cap.read()
    
    out = frame.copy()

    cv2.drawContours(out,[victim_detection_region],0,(255,0,0),1)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    victim = cv2.inRange(hsv, victim_lower, victim_upper)

    kernel_victim = np.ones((5, 5), "uint8")
    blue_victim = cv2.dilate(victim, kernel_victim)
    res_victim = cv2.bitwise_and(frame, frame, mask=victim)

    (contours, hierarchy) = cv2.findContours(victim, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    victim_index = 0
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 25:
            x, y, w, h = cv2.boundingRect(contour)
            if cv2.pointPolygonTest(victim_detection_region, (x, y), False) == 1.0:        
                cv2.rectangle(out, (x, y), (x+w, y+h), (255, 0, 0), 3)
                M = cv2.moments(contour)
                Vx = int(M['m10']/M['m00'])
                Vy = int(M['m01']/M['m00'])
                victims[victim_index] = (Vx, Vy)
                victim_index += 1
                if victim_index > 3:
                    break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(out, corners, ids)
    
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

        cv2.circle(out, tuple(m1), 5, (0, 255, 0), 2)
        cv2.circle(out, tuple(m2), 5, (0, 255, 0), 2)
        cv2.circle(out, tuple(m3), 5, (0, 255, 0), 2)
        cv2.circle(out, tuple(m4), 5, (0, 255, 0), 2)

        cv2.line(out, tuple(m3), tuple(m1), (0, 128, 0), 2)

        cv2.arrowedLine(out, tuple(m1), tuple(cp_f_e), (0, 128, 0), 2)
        cv2.line(out, tuple(m3), tuple(cp_r_e), (0, 128, 0), 2)
        cv2.circle(out, tuple(cp_r_e), 5, (0, 128, 0), 2)

        cv2.circle(out, tuple(b1), 5, (255, 0, 0), 2)
        cv2.circle(out, tuple(b2), 5, (255, 0, 0), 2)
        cv2.circle(out, tuple(b3), 5, (255, 0, 0), 2)
        cv2.circle(out, tuple(b4), 5, (255, 0, 0), 2)

        cv2.line(out, tuple(b1), tuple(b2), (255, 0, 0), 2)
        cv2.line(out, tuple(b2), tuple(b3), (255, 0, 0), 2)
        cv2.line(out, tuple(b3), tuple(b4), (255, 0, 0), 2)
        cv2.line(out, tuple(b4), tuple(b1), (255, 0, 0), 2)

    '''
    if send_vectors:
        cv2.line(out, victims[0], tuple(m1), (0, 255, 255), 2)
        target_vector = np.array(victims[0]) - np.array(tuple(m1))
        
        angle_error = angle_between(target_vector, central_parallel)

        #print(angle_error * 180/np.pi)

        
        if angle_error > angle_threshold:
            if 

    '''

    ### DRIVING ###

    if stage == -1:
        status = "Waiting for Arduino..."
        has_cmd = "Arduino"
    elif stage == 0:
        status = "Waiting for start button push..."
        has_cmd = "Arduino"
    elif stage == 1:
        status = "Bot line following - exiting start box"
        has_cmd = "Arduino"
    elif stage == 2:
        status = "Bot line following - start box to cave entrance"
        has_cmd = "Arduino"

    elif stage == 3:
        status = "Navigating to victim"
        has_cmd = "Python"
        
        if victims:
            cv2.line(out, victims[0], tuple(m1), (0, 255, 255), 2)
            target_vector = np.array(tuple(m1)) - np.array(victims[0])

            angle_error = np.degrees(
                angle_between(-target_vector, central_parallel))
            #print("Angle error: {}".format(angle_error))

            if angle_error > angle_threshold_pickup:
                if unit_cross_prod(target_vector, central_parallel) > 0:
                    cmd_move = 6  # Clockwise - rotate right
                else:
                    cmd_move = 5  # Anticlockwise - rotate left
            else:
                distance = np.linalg.norm(target_vector)
                if distance > 30:
                    cmd_move = 2  # Forward
                else:
                    cmd_move = 0  # Stop

                    # Move to detection stage
                    stage = 4
                    mqc.publish(topic_bot_stage, stage)

            send_move_command(cmd_move, prev_cmd_move)

    elif stage == 4:
        status = "Detecting victim health"
        has_cmd = "Arduino"
    
    elif stage == 5:
        status = "Positioning to load victim"
        has_cmd = "Python"

        distance = np.linalg.norm(target_vector)

        if distance < 50:
            cmd_move = 1 # Reverse away for turn
        else:
            angle_error = np.degrees(
                angle_between(target_vector, central_parallel))
            
            if angle_error > angle_threshold:
                # Turn around
                if unit_cross_prod(target_vector, central_parallel) > 0:
                    cmd_move = 6  # Clockwise - rotate right
                else:
                    cmd_move = 5  # Anticlockwise - rotate left
            else:
                # Reverse towards victim for pickup
                if distance > 5:
                    cmd_move = 1
                else:
                    # Ready to pick up
                    cmd_move = 0

                    # Move to load victim stage
                    stage = 6
                    mqc.publish(topic_bot_stage, stage)

        send_move_command(cmd_move, prev_cmd_move)
    
    elif stage == 6:
        status = "Loading victim"
        has_cmd = "Arduino"
    
    elif stage == 7:
        status = "Navigating to cave exit"
        has_cmd = "Python"

        cv2.line(out, tuple(m1), tuple(point_cave_exit_line),(0, 255, 255), 2)
        target_vector = np.array(tuple(m1)) - point_cave_exit_line

        angle_error = np.degrees(angle_between(-target_vector, central_parallel))
        if angle_error > angle_threshold:
                if unit_cross_prod(target_vector, central_parallel) > 0:
                    cmd_move = 6  # Clockwise - rotate right
                else:
                    cmd_move = 5  # Anticlockwise - rotate left
        else:
                distance = np.linalg.norm(target_vector)
                if distance > 5:
                    cmd_move = 2  # Forward
                else:
                    cmd_move = 0  # Stop

                    # Turn parallel to exit line
                    target_vector = np.array([1, 0])
                    angle_error = np.degrees(angle_between(-target_vector, central_parallel))
                    if angle_error > angle_threshold:
                        if unit_cross_prod(target_vector, central_parallel) > 0:
                            cmd_move = 4  # Clockwise - pivot right
                        else:
                            cmd_move = 3  # Anticlockwise - pivot left
                    else:
                        # Lined up with exit line
                        cmd_move = 0 # Stop

                        # Move to cave exit line follow stage
                        stage = 8
                        mqc.publish(topic_bot_stage, stage)

    elif stage == 8:
        status = "Bot line following - cave exit to triage area"
        has_cmd = "Arduino"

        b_front_midpoint = tuple(0.5*(np.array(b1) + np.array(b2)))

        if cv2.pointPolygonTest(triage_region, b_front_midpoint, False) == 1.0:
            # Entered triage region
            cmd_move = 0 # Stop

            # Move to reposition to unload stage
            stage = 9
            mqc.publish(topic_bot_stage, stage)

    elif stage == 9:
        status = "Repositioning for unloading"
        has_cmd = "Python"

        target_vector = np.array([0, 1])
        angle_error = np.degrees(angle_between(-target_vector, central_parallel))

        if angle_error > angle_threshold:
            if unit_cross_prod(target_vector, central_parallel) > 0:
                cmd_move = 6  # Clockwise - rotate right
            else:
                cmd_move = 5  # Anticlockwise - rotate left
        else:
            # Lined up
            cmd_move = 0 # Stop

            # Move to unload stage
            stage = 10
            mqc.publish(topic_bot_stage, stage)

    elif stage == 10:
        status = "Unloading victim"
        has_cmd = "Arduino"

    elif stage == 11:
        status = "Returning to start box"
        has_cmd = "Python"
    
    else:
        status = "Undefined"
    

    ### DISPLAY ###

    cv2.putText(out, "{} {}".format(stage, status), (50, 50), font, 0.5, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.putText(out, "{} in command".format(has_cmd), (50, 75), font, 0.5, (0, 0, 255), 2, cv2.LINE_AA)

    #frame = cv2.flip(frame, 1)
    #display_frame = cv2.resize(frame, None, fx=0.8, fy=0.8)

    cv2.imshow("Vision", out)

    key = cv2.waitKey(1)
    if key & 0xFF == ord('n'):
        continue
    elif key & 0xFF == ord('s'):
        cv2.imwrite('processed_capture_{}.png'.format(
            str(int(time.time()))), frame)
    elif key & 0xFF == ord('v'):
        send_vectors = not send_vectors
    elif key & 0xFF == ord('q'):
        break

mqc.loop_stop()
cap.release()
cv2.destroyAllWindows()