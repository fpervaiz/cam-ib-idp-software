# -*- coding: utf-8 -*-
"""
Created on Tue Oct 7 11:41:42 2018

@author: Caihao.Cui
"""
from __future__ import print_function

import pyzbar.pyzbar as pyzbar
import numpy as np
import cv2
import time
import csv

# get the webcam:  
#cap = cv2.VideoCapture('http://localhost:8080')
cap = cv2.VideoCapture(1)
out = cv2.VideoWriter('out.avi', cv2.VideoWriter_fourcc(*'XVID'), 25, (640, 480))

# open csv output
#f = open('qrxy.csv', 'w')
#xyWriter = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

leftTops = [[0, 0], [0, 0]]

#cap.set(3,640)
#cap.set(4,480)
#cap.set(10, 50)
#160.0 x 120.0
#176.0 x 144.0
#320.0 x 240.0
#352.0 x 288.0
#640.0 x 480.0
#1024.0 x 768.0
#1280.0 x 1024.0
time.sleep(2)

def decode(im) : 
    # Find barcodes and QR codes
    decodedObjects = pyzbar.decode(im)
    # Print results
    for obj in decodedObjects:
        print('Type : ', obj.type)
        print('Data : ', obj.data,'\n')     
    return decodedObjects


font = cv2.FONT_HERSHEY_SIMPLEX

while(cap.isOpened()):
    # Capture frame-by-frame
    ret, frame = cap.read()
    # Our operations on the frame come here
    im = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
         
    decodedObjects = decode(im)

    for decodedObject in decodedObjects: 
        points = decodedObject.polygon
     
        # If the points do not form a quad, find convex hull
        if len(points) > 4 : 
          hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
          hull = list(map(tuple, np.squeeze(hull)))
        else : 
          hull = points
         
        # Number of points in the convex hull
        n = len(hull)     
        # Draw the convext hull
        for j in range(0,n):
          cv2.line(frame, hull[j], hull[ (j+1) % n], (255,0,0), 3)
          cv2.putText(frame, str(j), (hull[j][0], hull[j][1]), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        x = decodedObject.rect.left
        y = decodedObject.rect.top

        print(x, y)
        #xyWriter.writerow([x, y])
        cv2.circle(frame, (x, y), 5, (0, 255, 0), 2)

        print('Type : ', decodedObject.type)
        print('Data : ', decodedObject.data,'\n')

        barCode = str(decodedObject.data.decode())
        cv2.putText(frame, barCode, (x, y), font, 1, (0,255,255), 2, cv2.LINE_AA)

        if barCode == 'F':
          leftTops[0][0] = x
          leftTops[0][1] = y
        else:
          leftTops[1][0] = x
          leftTops[1][1] = y

    # Draw connecting line
    cv2.line(frame, tuple(leftTops[0]), tuple(leftTops[1]), (0, 0, 0), 10)

    # Calculate angle
    x1, y1 = leftTops[1]
    x2, y2 = leftTops[0]
    if x1 and y1 and x2 and y2:
      try:
        angle = np.arctan((x2-x1)/(y2-y1))*180.0/np.pi
      except ZeroDivisionError:
        angle = "Infinity"
    else:
      angle = "N/A"

    # Show info
    cv2.putText(frame, 'Top left: {}'.format(leftTops[0]), (10, 30), font, 0.5, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, 'Angle: {} deg'.format(angle), (10, 70), font, 0.5, (0, 0, 255), 2, cv2.LINE_AA)

    # Display the resulting frame
    cv2.imshow('frame',frame)
    out.write(frame)
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q'):
        break
    elif key & 0xFF == ord('s'): # wait for 's' key to save 
        cv2.imwrite('Capture.png', frame)     

# When everything done, release the capture
#f.close()
cap.release()
out.release()
cv2.destroyAllWindows()