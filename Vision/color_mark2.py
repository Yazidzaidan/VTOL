import cv2
import numpy as np
import pandas as pd

df = pd.read_excel('hsv_GD_C.xlsx')
df.info()
print("Data HSV ELP")
#print(df)
h_min = df['h'].min()
s_min = df['s'].min()
v_min = df['v'].min()
h_max = df['h'].max()
s_max = df['s'].max()
v_max = df['v'].max()
print("lower HSV :", h_min, s_min, v_min)
print("upper HSV :", h_max, s_max, v_max)

cap = cv2.VideoCapture(1)
xCenter = 320
yCenter = 240

def rescale_frame(frame, percent):
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)

while True:
    _, frame = cap.read()
    frame30 = rescale_frame(frame, percent=30)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #Red color
    high_red = np.array([h_max, s_max, v_max])
    low_red = np.array([h_min, s_min, v_min])
    red_mask = cv2.inRange(hsv_frame, low_red, high_red)
    red_mask30 = rescale_frame(red_mask, percent=30)
    
    cnts, _ = cv2.findContours(red_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
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
            #print(cx, ' ', cy)
            #titik tengah cx 320
            #titik tengah cy 240
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.circle(frame, (cx,cy),7,(255,255,255),-1)
            if cx < 250:
                print("Left",  cx, cy, "desire :", xCenter, yCenter)
                cv2.putText(frame, "Kiri - ", (cx-10, cy-25), font, 0.5, (255, 0, 0),1)
            elif cx > 390:
                print("Right",  cx, cy, "desire :", xCenter, yCenter)
                cv2.putText(frame, "Kanan - ", (cx-30, cy-25), font, 0.5, (255, 0, 0),1)
            if cy < 170:
                print("Forward",  cx, cy, "desire :", xCenter, yCenter)
                cv2.putText(frame, "Maju", (cx+40, cy-25), font, 0.5, (255, 0, 0),1)
            elif cy > 310:
                print("Backward",  cx, cy, "desire :", xCenter, yCenter)
                cv2.putText(frame, "Mundur", (cx+40, cy-25), font, 0.5, (255, 0, 0),1)
            else :
                print("Reach Target ELP",  cx, cy, "desire :", xCenter, yCenter)
                cv2.putText(frame, "None - ", (cx-10, cy-25), font, 0.5, (255, 0, 0),1)
                cv2.putText(frame, "None", (cx+40, cy-25), font, 0.5, (255, 0, 0),1)

            #Koordinat tengah frame
            #frame_center = round(frame.shape[1] / 2), round(frame.shape[0] / 2)
            #frame = cv2.circle(frame, frame_center, 3, (0, 255, 0), -1)

            # displaying the coordinates
            # on the image window
            cv2.putText(frame, str(cx) + ',' +
                str(cy), (cx,cy), font,
                1, (255, 0, 0), 2)
            #cv2.putText(frame, "Red", (cx-20, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255,255,255), 3)

    #show frame
    cv2.imshow("Frame", frame30)
    cv2.imshow("Red Mask", red_mask30)
    if cv2.waitKey(5) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()