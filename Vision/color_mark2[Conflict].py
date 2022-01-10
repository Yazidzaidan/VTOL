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
    lower_red = np.array([90, 173, 84])
    upper_red = np.array([119, 193, 104])
    img_inrange = cv2.inRange(hsv_frame, lower_red, upper_red)
    
    cnts, _ = cv2.findContours(img_inrange, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        area = cv2.contourArea(c)
        if area > 5000:
            #Draw line contour image
            #cv2.drawContours(frame, [c],-1,(0,255,0), 3)
            area = cv2.minAreaRect(c)
            center = (int(area[0][0]), int(area[0][1]))
            width = int(max(area[1])/2)
            frame = cv2.circle(frame, center, width, (0, 0, 255), 2)
            #Draw center circle
            M = cv2.moments(c)
            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])
            print(M)
            cv2.circle(frame, (cx,cy),7,(255,255,255),-1)
            #frame_center = round(frame.shape[1] / 2), round(frame.shape[0] / 2)
            #frame = cv2.circle(frame, frame_center, 3, (0, 255, 0), -1)

    #show frame
    cv2.imshow("Frame", frame)
    #cv2.imshow("Red Mask", red_mask)
    if cv2.waitKey(5) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()