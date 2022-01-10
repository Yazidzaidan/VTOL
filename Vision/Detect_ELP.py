import cv2
import numpy as np
#import imutils

cap = cv2.VideoCapture(1)
cap.set(3,640)
cap.set(4,480)

while True:
    _, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #frame_center = round(frame.shape[1] / 2), round(frame.shape[0] / 2)
    #image = cv2.line(frame, frame_center, (round(frame.shape[1] / 2), round(frame.shape[0])), (255, 255, 255), 2)
    #ELP
    lower_elp = np.array([25, 62, 190])
    upper_elp = np.array([35, 138, 240])
    img_inrange = cv2.inRange(hsv_frame, lower_elp, upper_elp)
   
    #ballWidthCm:int = 20
    #focalLength:int = (70 * 100) / ballWidthCm
    #print(focalLength)

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
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.circle(frame, (cx,cy),7,(255,255,255),-1)
            if cx < 290:
                print(cx, ' ', cy, "Kiri")
                cv2.putText(frame, "Kiri - ", (cx-10, cy-25), font, 0.5, (255, 0, 0),1)
            elif cx > 390:
                print(cx, ' ', cy, "Kanan")
                cv2.putText(frame, "Kanan - ", (cx-30, cy-25), font, 0.5, (255, 0, 0),1)
            else :
                print("Center")
                cv2.putText(frame, "None - ", (cx-10, cy-25), font, 0.5, (255, 0, 0),1)
            if cy < 170:
                print(cx, ' ', cy, "Maju")
                cv2.putText(frame, "Maju", (cx+40, cy-25), font, 0.5, (255, 0, 0),1)
            elif cy > 310:
                print(cx, ' ', cy, "Mundur")
                cv2.putText(frame, "Mundur", (cx+40, cy-25), font, 0.5, (255, 0, 0),1)
            else :
                print("None")
                cv2.putText(frame, "None", (cx+40, cy-25), font, 0.5, (255, 0, 0),1)

            cv2.putText(frame, str(cx) + ',' +
                str(cy), (cx,cy), font,
                1, (255, 0, 0), 2)

            
    #show frame
    cv2.imshow("Frame", frame)
    cv2.imshow("ELP Mask", img_inrange)
    if cv2.waitKey(5) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()