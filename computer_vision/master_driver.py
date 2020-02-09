import numpy as np
import paho.mqtt.client as mqtt

import cv2
import time

from cv2 import aruco

red = (0, 0, 255)
yellow = (0, 255, 255)
green = (0, 255, 0)
blue = (255, 0, 0)
dark_green = (0, 128, 0)

stop = 0
straight_forward = 1
straight_reverse = 2
pivot_left = 3
pivot_right = 4
rotate_left = 5
rotate_right = 6

nav_speed_low = 48
nav_speed_high = 192

bot_width = 0.40
bot_front = 0.75
bot_rear = 0.60

bot_detected = False
angle_control_allowed = True

victim_lower = np.array([35, 70, 150], np.uint8)
victim_upper = np.array([60, 255, 255], np.uint8)

victim_detection_region = np.array([[230, 25],[750, 25],[750, 240],[570, 240],[570, 450],[750, 450],[750, 700],[230, 700]], np.int32).reshape((-1,1,2))
triage_region = np.array([[810, 500], [1000, 500], [1000, 720], [810, 720]], np.int32).reshape((-1,1,2))

risk_zone_upper = np.array([[500, 25],[750, 25],[750, 240],[500, 240]], np.int32).reshape((-1,1,2))
risk_zone_lower = np.array([[500, 450],[750, 450],[750, 700],[500, 700]], np.int32).reshape((-1,1,2))

victim_contour_min_area = 25

#victims = [(0,0), (0,0), (0,0), (0,0)]
victims = []

send_vectors = False
stage = -1
prev_cmd_move = -1
prev_cmd_speed = 255

angle_threshold = 4

point_cave_exit_positioning = np.array([500, 335])
point_cave_exit_line = np.array([590, 335])
point_start_box_end = np.array([965, 10])
point_cave_reentry = np.array([905, 445])

topic_bot_cmd_stage = "/idp/bot/cmd_stage"
topic_bot_stt_stage = "/idp/bot/stt_stage"
topic_bot_stt_drop_stage = "/idp/bot/stt_drop_stage"
topic_bot_debug = "/idp/bot/debug"
topic_bot_cmd_move = "/idp/bot/cmd_move"
topic_bot_cmd_speed = "/idp/bot/cmd_speed"
topic_bot_cmd_mech = "/idp/bot/cmd_mech"

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

    if message.topic == topic_bot_stt_stage:
        if int(payload) == 0:
            # Arduino start ping
            stage = 0

            # Reset to allow future transmission
            prev_cmd_move = 0
            prev_cmd_speed = -1

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
            # Positioning to load
            stage = 4

        elif int(payload) == 5 and stage == 4:
            # Health detection wait
            stage = 5

        elif int(payload) == 6 and stage == 5:
            # Load victim
            stage = 6
            
        elif int(payload) == 7 and stage == 6:
            # Navigate to cave exit
            stage = 7

        elif int(payload) == 8 and stage == 7:
            # Positioning for cave exit line following
            stage = 8

        elif int(payload) == 9 and stage == 8:
            # Bot line following - cave exit to triage area
            stage = 9

        elif int(payload) == 10 and stage == 9:
            # Reposition for unloading
            stage = 10

        elif int(payload) == 11 and stage == 10:
            # Unloading victim
            stage = 11

        elif int(payload) == 12 and stage == 11:
            # Decision
            stage = 12

        elif int(payload) == 13 and stage == 12:
            # Return to start box
            stage = 13

        else:
            # Undefined
            pass
            
    elif message.topic == topic_bot_stt_drop_stage:
        stage = int(payload)
        print("Set stage to {} on receiving dropped connection restate".format(payload))

        if stage in [3, 4, 7, 8, 10, 12, 13]:
            # Re-command speed and move when reconnected
            send_speed_command(cmd_speed, force=True)
            send_move_command(cmd_move, force=True)

def calculate_turn_command(target_vector, bot_vector, error, threshold):
    if unit_cross_prod(target_vector, bot_vector) > 0:
        move = rotate_right
    else:
        move = rotate_left

    if error < (threshold * 5):
        speed = nav_speed_low
    else:
        speed = nav_speed_high

    return move, speed

def send_move_command(curr, force=False):
    global prev_cmd_move
    if (force) or (not curr == prev_cmd_move):
        mqc.publish(topic_bot_cmd_move, curr)
        prev_cmd_move = curr
        print("Command move: {}".format(curr))

def send_speed_command(curr, force=False):
    global prev_cmd_speed
    if not curr == prev_cmd_speed:
        mqc.publish(topic_bot_cmd_speed, curr)
        prev_cmd_speed = curr
        print("Command speed: {}".format(curr))

def window_write_nav_info(angle, distance, move, speed):
    cv2.putText(out, "Angle {} Distance {}".format(angle, distance), (50, 125), font, 0.5, red, 2, cv2.LINE_AA)
    cv2.putText(out, "Move {} Speed {}".format(move, speed), (50, 150), font, 0.5, red, 2, cv2.LINE_AA)

def in_risk_zone(point):
    return cv2.pointPolygonTest(risk_zone_lower, point, False) == 1.0 or cv2.pointPolygonTest(risk_zone_upper, point, False) == 1.0

mqc = mqtt.Client("MASTER")
mqc.connect("192.168.137.1")
mqc.on_message = on_message
mqc.on_disconnect = on_disconnect
mqc.loop_start()

mqc.subscribe(topic_bot_stt_stage)
mqc.subscribe(topic_bot_stt_drop_stage)

'''
cv2.namedWindow('Bars')
cv2.createTrackbar('Width', 'Bars', int(bot_width*100), 200, bot_width_trackbar)
cv2.createTrackbar('Front', 'Bars', int(bot_front*100), 200, bot_front_trackbar)
cv2.createTrackbar('Rear', 'Bars', int(bot_rear*100), 200, bot_rear_trackbar)
cv2.createTrackbar('Min Area', 'Bars', int(victim_contour_min_area), 200, vic_min_area)
'''
cv2.namedWindow('Vision')
cv2.moveWindow('Vision', 20, 20)
cv2.setMouseCallback('Vision', print_mouse_coords)

cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

while True:
    ### FRAME UPDATE ###

    victims = []

    _, frame = cap.read()
    
    out = frame.copy()

    cv2.drawContours(out, [victim_detection_region], 0, green, 1)
    cv2.drawContours(out, [triage_region], 0, red, 1)
    cv2.drawContours(out, [risk_zone_lower], 0, red, 1)
    cv2.drawContours(out, [risk_zone_upper], 0, red, 1)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    victim = cv2.inRange(hsv, victim_lower, victim_upper)

    kernel_victim = np.ones((5, 5), "uint8")
    blue_victim = cv2.dilate(victim, kernel_victim)
    res_victim = cv2.bitwise_and(frame, frame, mask=victim)

    (contours, hierarchy) = cv2.findContours(victim, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #victim_index = 0
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area >= victim_contour_min_area:
            x, y, w, h = cv2.boundingRect(contour)
            if cv2.pointPolygonTest(victim_detection_region, (x, y), False) == 1.0:        
                M = cv2.moments(contour)
                Vx = int(M['m10']/M['m00'])
                Vy = int(M['m01']/M['m00'])
                distance = np.linalg.norm(np.array((Vx, Vy)) - point_cave_exit_line)

                if in_risk_zone((Vx, Vy)):
                    # Pick these up last
                    distance += 500
                    colour = red
                else:
                    colour = blue

                cv2.rectangle(out, (x, y), (x+w, y+h), colour, 3)

                victims.append([Vx, Vy, distance])
                if len(victims) > 4:
                    break
    
    # Sort victims by distance and rebuild
    victims = sorted(victims, key=lambda tup: tup[2])
    for i in range(len(victims)):
        victims[i] = (victims[i][0], victims[i][1])

    #print(victims)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(out, corners, ids)
    
    if corners:
        bot_detected = True

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

        cv2.circle(out, tuple(m1), 5, green, 2)
        cv2.circle(out, tuple(m2), 5, green, 2)
        cv2.circle(out, tuple(m3), 5, green, 2)
        cv2.circle(out, tuple(m4), 5, green, 2)

        cv2.line(out, tuple(m3), tuple(m1), dark_green, 2)

        cv2.arrowedLine(out, tuple(m1), tuple(cp_f_e), dark_green, 2)
        cv2.line(out, tuple(m3), tuple(cp_r_e), dark_green, 2)
        cv2.circle(out, tuple(cp_r_e), 5, dark_green, 2)

        cv2.circle(out, tuple(b1), 5, blue, 2)
        cv2.circle(out, tuple(b2), 5, blue, 2)
        cv2.circle(out, tuple(b3), 5, blue, 2)
        cv2.circle(out, tuple(b4), 5, blue, 2)

        cv2.line(out, tuple(b1), tuple(b2), blue, 2)
        cv2.line(out, tuple(b2), tuple(b3), blue, 2)
        cv2.line(out, tuple(b3), tuple(b4), blue, 2)
        cv2.line(out, tuple(b4), tuple(b1), blue, 2)

    else:
        bot_detected = False
    
    ### DRIVING ###

    if stage == -1:
        status = "Waiting for Arduino..."
        has_cmd = "Arduino"
    elif stage == 0:
        status = "Waiting for start button push..."
        has_cmd = "Arduino"

        load_mech_opened = False

    elif stage == 1:
        status = "Bot line following - exiting start box"
        has_cmd = "Arduino"
    elif stage == 2:
        status = "Bot line following - start box to cave entrance"
        has_cmd = "Arduino"

        risk_zone = False

    elif stage == 3:
        status = "Navigating to victim"
        has_cmd = "Python"
        
        if victims:

            orig = tuple(m1)
            if in_risk_zone(victims[0]):
                dest = (100, victims[0][1])
                risk_zone = True
            else:
                dest = victims[0]
                #risk_zone = False

            cv2.line(out, dest, orig, yellow, 2)

            target_vector = np.array(dest) - np.array(orig)

            distance = np.linalg.norm(target_vector)
            angle_error = np.degrees(angle_between(target_vector, central_parallel))

            if angle_error > angle_threshold:
                cmd_move, cmd_speed = calculate_turn_command(target_vector, central_parallel, angle_error, angle_threshold)

            else:
                if risk_zone:
                    target_distance = 70
                else:
                    target_distance = 150

                if distance > target_distance:
                    cmd_move = straight_forward
                    '''
                    if distance > 50:
                        cmd_speed = nav_speed_high
                    else:
                        cmd_speed = nav_speed_low
                    '''
                    cmd_speed = nav_speed_high
                else:
                    cmd_move = stop
                    cmd_speed = nav_speed_high

                    # Move to positioning stage
                    move_away_allowed = True
                    stage = 4
                    mqc.publish(topic_bot_cmd_stage, stage)
            
            window_write_nav_info(angle_error, distance, cmd_move, cmd_speed)            
            
            send_speed_command(cmd_speed)
            send_move_command(cmd_move)
        
        else:
            # No victims found! Exit cave.
            stage = 7
            mqc.publish(topic_bot_cmd_stage, stage)

    elif stage == 4:
        status = "Positioning to load victim"
        has_cmd = "Python"

        orig = tuple(m1)
        dest = victims[0]

        cv2.line(out, dest, orig, yellow, 2)

        target_vector = np.array(orig) - np.array(dest)
        distance = np.linalg.norm(target_vector)
        angle_error = np.degrees(angle_between(target_vector, central_parallel))

        # Calibration
        #angle_error += 10

        if distance < 150 and move_away_allowed:
            cmd_speed = nav_speed_high
            cmd_move = straight_reverse
        else:
            move_away_allowed = False
            if angle_error > angle_threshold:
                # Turn around
                '''
                if unit_cross_prod(target_vector, central_parallel) > 0:
                    cmd_move = 6  # Clockwise - rotate right
                else:
                    cmd_move = 5  # Anticlockwise - rotate left
                '''
                # cmd_move = 6
                cmd_move, cmd_speed = calculate_turn_command(target_vector, central_parallel, angle_error, angle_threshold)
            
            else:
                if not load_mech_opened:
                    mqc.publish(topic_bot_cmd_mech, "Open sesame")
                    load_mech_opened = True
                
                # Reverse towards victim for pickup
                if distance > 110:
                    cmd_move = straight_reverse
                    if distance > 120:
                        cmd_speed = nav_speed_high
                    else:
                        cmd_speed = nav_speed_low
                else:
                    # Ready to pick up
                    cmd_move = stop

                    # Move to load victim stage
                    stage = 5
                    mqc.publish(topic_bot_cmd_stage, stage)

        window_write_nav_info(angle_error, distance, cmd_move, cmd_speed)

        send_speed_command(cmd_speed)
        send_move_command(cmd_move)
    
    elif stage == 5:
        status = "Detecting victim health"
        has_cmd = "Arduino"
    
    elif stage == 6:
        status = "Loading victim"
        has_cmd = "Arduino"

        # For next stage
        angle_control_allowed = True
    
    elif stage == 7:
        status = "Navigating to cave exit"
        has_cmd = "Python"

        orig = tuple(m1)
        dest = tuple(point_cave_exit_positioning)

        cv2.line(out, orig, dest, yellow, 2)

        target_vector = point_cave_exit_positioning - np.array(orig) 
        distance = np.linalg.norm(target_vector)
        angle_error = np.degrees(angle_between(target_vector, central_parallel))

        if angle_error > angle_threshold and angle_control_allowed:
            cmd_move, cmd_speed = calculate_turn_command(target_vector, central_parallel, angle_error, angle_threshold)
        
        else:
                
            if distance > 5:
                cmd_move = straight_forward
                if distance > 20:
                    cmd_speed = nav_speed_high
                    angle_control_allowed = True
                else:
                    cmd_speed = nav_speed_low
                    angle_control_allowed = False
            else:

                cmd_move = stop
                angle_control_allowed = True

                # Move to cave exit line follow stage
                stage = 8
                mqc.publish(topic_bot_cmd_stage, stage)

        window_write_nav_info(angle_error, distance, cmd_move, cmd_speed)
        
        send_speed_command(cmd_speed)
        send_move_command(cmd_move)

    elif stage == 8:
        status = "Positioning for cave exit line following"
        has_cmd = "Python"

        orig = tuple(m1)
        dest = tuple(point_cave_exit_line)

        cv2.line(out, orig, dest, yellow, 2)

        target_vector = point_cave_exit_line - np.array(orig) 
        distance = np.linalg.norm(target_vector)
        angle_error = np.degrees(angle_between(target_vector, central_parallel))

        if angle_error > angle_threshold and angle_control_allowed:
            cmd_move, cmd_speed = calculate_turn_command(target_vector, central_parallel, angle_error, angle_threshold)

        else:
                
            if distance > 4:
                cmd_move = straight_forward
                if distance > 20:
                    cmd_speed = nav_speed_high
                    angle_control_allowed = True
                else:
                    cmd_speed = nav_speed_low
                    angle_control_allowed = False
            else:
                # Lined up with exit line
                cmd_move = stop

                # Move to cave exit line follow stage
                stage = 9
                mqc.publish(topic_bot_cmd_stage, stage)

        window_write_nav_info(angle_error, distance, cmd_move, cmd_speed)
        
        send_speed_command(cmd_speed)
        send_move_command(cmd_move)

    
    elif stage == 9:
        status = "Bot line following - cave exit to triage area"
        has_cmd = "Arduino"

        b_front_midpoint = tuple(0.5*(np.array(b1) + np.array(b2)))

        if cv2.pointPolygonTest(triage_region, b_front_midpoint, False) == 1.0:
            # Entered triage region
            cmd_move = stop # Unnecessary

            # Move to reposition to unload stage
            stage = 10
            mqc.publish(topic_bot_cmd_stage, stage)

    elif stage == 10:
        status = "Repositioning for unloading"
        has_cmd = "Python"

        orig = tuple((int(m1[0]), int(m1[1])))
        dest = tuple((int(m1[0]), int(m1[1] - 200)))

        target_vector = np.array(dest) - np.array(orig)
        angle_error = np.degrees(angle_between(target_vector, central_parallel))        

        cv2.line(out, orig, dest, yellow, 2)

        if angle_error > angle_threshold:
            cmd_move, cmd_speed = calculate_turn_command(target_vector, central_parallel, angle_error, angle_threshold)
            
        else:
            # Lined up
            cmd_move = stop

            # Move to unload stage
            stage = 11
            mqc.publish(topic_bot_cmd_stage, stage)

        window_write_nav_info(angle_error, distance, cmd_move, cmd_speed)

        send_speed_command(cmd_speed)
        send_move_command(cmd_move)

    elif stage == 11:
        status = "Unloading victim"
        has_cmd = "Arduino"

        # For next stage
        angle_control_allowed = True

    elif stage == 12:
        status = "Decision to continue or end"
        has_cmd = "Python"

        #del victims[0]

        if victims:
            # More to rescue: re-enter cave
            
            orig = tuple(m1)
            dest = tuple(point_cave_reentry)

            target_vector = np.array(dest) - np.array(orig)
            cv2.line(out, orig, dest, yellow, 2)

            distance = np.linalg.norm(target_vector)
            angle_error = np.degrees(angle_between(target_vector, central_parallel))

            if angle_error > angle_threshold and angle_control_allowed:
                cmd_move, cmd_speed = calculate_turn_command(target_vector, central_parallel, angle_error, angle_threshold)

            else:
                if distance > 20:
                    cmd_move = straight_forward
                    if distance > 30:
                        cmd_speed = nav_speed_high
                        angle_control_allowed = True
                    else:
                        
                        cmd_speed = nav_speed_low
                        angle_control_allowed = False
                else:
                    cmd_move = stop

                    # Move to line following cave reentry stage
                    angle_control_allowed = True
                    move_away_allowed = True
                    stage = 2
                    mqc.publish(topic_bot_cmd_stage, stage)
            
            window_write_nav_info(angle_error, distance, cmd_move, cmd_speed)            
            
            send_speed_command(cmd_speed)
            send_move_command(cmd_move)

        else:
            # Return to start box
            stage = 13

        mqc.publish(topic_bot_cmd_stage, stage)
    
    elif stage == 13:
        status = "Returning to start box"
        has_cmd = "Python"

        
        orig = tuple(m1)
        dest = tuple(point_start_box_end)

        cv2.line(out, dest, orig, yellow, 2)

        target_vector = point_start_box_end - np.array(orig)

        distance = np.linalg.norm(target_vector)
        angle_error = np.degrees(angle_between(target_vector, central_parallel))

        if angle_error > angle_threshold:
            cmd_move, cmd_speed = calculate_turn_command(target_vector, central_parallel, angle_error, angle_threshold)

        else:
            cmd_move = straight_forward
            cmd_speed = nav_speed_high
            
            if distance < 20:    
                # Inside box
                cmd_move = stop
                cmd_speed = 0

                stage = 14
                #mqc.publish(topic_bot_stt_stage, stage)

        window_write_nav_info(angle_error, distance, cmd_move, cmd_speed)     
        
        send_speed_command(cmd_speed)
        send_move_command(cmd_move)

    elif stage == 14:
        status = "Complete"
        has_cmd = "N/A"
    
    else:
        status = "Undefined"
    

    ### DISPLAY ###

    cv2.putText(out, "{} {}".format(stage, status), (50, 50), font, 0.5, red, 2, cv2.LINE_AA)
    cv2.putText(out, "{} in command".format(has_cmd), (50, 75), font, 0.5, red, 2, cv2.LINE_AA)

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
    elif key & 0xFF == ord('o'):
        stage -= 1
    elif key & 0xFF == ord('p'):
        stage += 1
    elif key & 0xFF == ord('q'):
        break

mqc.loop_stop()
cap.release()
cv2.destroyAllWindows()