import numpy as np
import cv2
import time

cap = cv2.VideoCapture(1)

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

font = cv2.FONT_HERSHEY_SIMPLEX

victim_lower = np.array([35,60,200],np.uint8)
victim_upper = np.array([60,255,255],np.uint8)

cyan_lower = np.array([60,60,200],np.uint8)
cyan_upper = np.array([140,255,255],np.uint8)

magenta_lower = np.array([125,60,200],np.uint8)
magenta_upper = np.array([165,255,255],np.uint8)

while True:
    _, img = cap.read()
    
    #converting frame(img) from BGR (Blue-Green-Red) to HSV (hue-saturation-value)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
    #defining the range of Yellow color

    #finding the range yellow colour in the image
    cyan = cv2.inRange(hsv, cyan_lower, cyan_upper)

    #Morphological transformation, Dilation         
    kernal_cyan = np.ones((5 ,5), "uint8")

    blue_cyan=cv2.dilate(cyan, kernal_cyan)

    res_cyan=cv2.bitwise_and(img, img, mask = cyan)

    #Tracking Colour (Yellow) 
    (_,contours,hierarchy)=cv2.findContours(cyan,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
    for pic, contour in enumerate(contours):
        
        area = cv2.contourArea(contour)
        if(area>30):
                        
            x,y,w,h = cv2.boundingRect(contour)     
            img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),3)
        

        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(img,[box],0,(0,0,255),2)    

    #finding the range magenta colour in the image
    magenta = cv2.inRange(hsv, magenta_lower, magenta_upper)

    #Morphological transformation, Dilation         
    kernal_magenta = np.ones((5 ,5), "uint8")

    blue_magenta=cv2.dilate(magenta, kernal_magenta)

    res_magenta=cv2.bitwise_and(img, img, mask = magenta)

    #Tracking Colour (magenta) 
    (_,contours,hierarchy)=cv2.findContours(magenta,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area>30):
                        
            x,y,w,h = cv2.boundingRect(contour)     
            img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),3)

    #finding the range victim colour in the image
    victim = cv2.inRange(hsv, victim_lower, victim_upper)

    #Morphological transformation, Dilation         
    kernal_victim = np.ones((5 ,5), "uint8")

    blue_victim=cv2.dilate(victim, kernal_victim)

    res_victim=cv2.bitwise_and(img, img, mask = victim)

    #Tracking Colour (magenta) 
    (_,contours,hierarchy)=cv2.findContours(victim,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        #if(area>5):
                        
        x,y,w,h = cv2.boundingRect(contour)     
        img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),3)
                   
    cv2.imshow("Color Tracking",img)
    img = cv2.flip(img,1)
    
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q'):
        break
    elif key & 0xFF == ord('s'): # wait for 's' key to save 
        cv2.imwrite('Capture_{}.png'.format(str(int(time.time()))), img)   

cap.release()
cv2.destroyAllWindows()