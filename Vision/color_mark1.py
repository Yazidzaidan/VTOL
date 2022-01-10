import cv2
import numpy as np
import imutils

cap = cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)

while True:
    _, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #Red color
    high_red = np.array([16, 255, 255])
    low_red = np.array([1, 115, 0])
    red_mask = cv2.inRange(hsv_frame, low_red, high_red)
    #red = cv2.bitwise_and(frame, frame, mask=red_mask)
    cnts1 = cv2.findContours(red_mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cnts1 = imutils.grab_contours(cnts1)
    for c in cnts1:
        area1 = cv2.contourArea(c)
        if area1 > 5000:
            cv2.drawContours(frame, [c],-1,(0,255,0), 3)
            #print(c)
            M = cv2.moments(c)
            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])
            cv2.circle(frame, (cx,cy),7,(255,255,255),-1)
            # displaying the coordinates
            # on the Shell
            print(cx, ' ', cy)
  
            # displaying the coordinates
            # on the image window
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, str(cx) + ',' +
                str(cy), (cx,cy), font,
                1, (255, 0, 0), 2)
            #cv2.putText(frame, "Red", (cx-20, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255,255,255), 3)


    #Add Rectangle
    #cv2.rectangle(red_mask, (x,y),(x + w, y +w),(0,0,255), 2)

    #show frame
    cv2.imshow("Frame", frame)
    cv2.imshow("Red Mask", red_mask)
    if cv2.waitKey(5) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()