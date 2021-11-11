import cv2
import numpy as np
import imutils
import serial
import time

p = 0
if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB1',115200,timeout=1)
    ser.flush()
ser.write ("1#".encode('ascii'))
time.sleep(0.5)
cap = cv2.VideoCapture(0)

lower1 = np.array([0, 90, 0])
upper1 = np.array([2, 255, 255])
 
# upper boundary RED color range values; Hue (160 - 180)
lower2 = np.array([176,120,0])
upper2 = np.array([179,255,255])


lower_green = np.array([51, 60, 26])#woooooooooooo!!!!!!!!!!
upper_green = np.array([85, 255 ,245])

i = 3
j = 8
while True:
    p = 0
    success,img = cap.read()
    image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        #mask1 = cv2.inRange(image, lower_red, upper_red)
    lower_mask = cv2.inRange(image, lower1, upper1)
    upper_mask = cv2.inRange(image, lower2, upper2)
    full_mask = lower_mask + upper_mask;
    
    mask2 = cv2.inRange(image, lower_green, upper_green)

    
    contours1,hierarchy1  = cv2.findContours(full_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
    contours2,hierarchy2  = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

   
    if len(contours1)!= 0:
         for contour in contours1:
             if cv2.contourArea(contour) > 500:
                 p = 2
                 x, y , w, h = cv2.boundingRect(contour)
                 area = cv2.contourArea(contour)
                 dis = 4532.2*pow(area,-0.532)
                 m = cv2.moments(contour)
                 cx = int (m["m10"]/m["m00"])
                 cy = int (m["m01"]/m["m00"])      
                 if dis < 48:
                    if j != i :
                        ser.write ("6#".encode('ascii'))
                        print("6")
                        i = 3
                        j=3
                 if dis > 52 and i == 3:
                    ser.write ("8#".encode('ascii'))
                    print("8")
                    i = 8
                    j=3
                    
    if len(contours2)!= 0:
        for contour in contours2:
            if cv2.contourArea(contour) > 5000:
                p = 2
                x, y , w, h = cv2.boundingRect(contour)
                AREA = cv2.contourArea(contour)
                DIS = 6165.4*pow(AREA,-0.562)
                m = cv2.moments(contour)
                cx = int (m["m10"]/m["m00"])
                cy = int (m["m01"]/m["m00"])
                if DIS < 25:
                    if j != i :
                        ser.write ("3#".encode('ascii'))
                        print("3")
                        i = 3
                        j=3
                if DIS > 28 and i == 3:
                    ser.write ("8#".encode('ascii'))
                    print("8")
                    i = 8
                    j=3
    
    if p == 0 and i == 3:
        ser.write ("8#".encode('ascii'))
        print("8")
        i=8
        j=3
    k = cv2.waitKey(5)
    if k == 27:
        break
cap.release()
cv2.destroyAllWindows()


