import cv2
import numpy as np
xCenter = 180
yCenter = 100
import pandas as pd

df = pd.read_excel('hsv_ELP.xlsx')
df.info()
print("Data HSV ELP")
print(df)
h_min = df['h'].min()
s_min = df['s'].min()
v_min = df['v'].min()
h_max = df['h'].max()
s_max = df['s'].max()
v_max = df['v'].max()
print("lower HSV :", h_min, s_min, v_min)
print("upper HSV :", h_max, s_max, v_max)

def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def rescale_frame(frame, percent):
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)

def showHSV():
    print(gstreamer_pipeline(flip_method=0))
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    toleransi = 50
    land = False
    if cap.isOpened():
        #Red color
        high_red = np.array([h_max, s_max, v_max])
        low_red = np.array([h_min, s_min, v_min])
        window_handle = cv2.namedWindow("image", cv2.WINDOW_AUTOSIZE)
        # Window
        while cv2.getWindowProperty("image", 0) >= 0:
            
            ret_val, frame = cap.read()
            frame30 = rescale_frame(frame, percent=30)
            
            hsv_frame = cv2.cvtColor(frame30, cv2.COLOR_BGR2HSV)
            red_mask = cv2.inRange(hsv_frame, low_red, high_red)
            cnts, _ = cv2.findContours(red_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                area = cv2.contourArea(c)
                if area > 5000:
                    #Draw line contour image
                    #cv2.drawContours(frame, [c],-1,(0,255,0), 3)
                    area = cv2.minAreaRect(c)
                    center = (int(area[0][0]), int(area[0][1]))
                    width = int(max(area[1])/2)
                    frame30 = cv2.circle(frame30, center, width, (0, 0, 255), 2)
                    #Draw center circle
                    M = cv2.moments(c)
                    cx = int(M["m10"]/M["m00"])
                    cy = int(M["m01"]/M["m00"])
                    #print(cx, ' ', cy)
                    #titik tengah cx 180
                    #titik tengah cy 100
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.circle(frame30, (cx,cy),7,(255,255,255),-1)
                    if cx < xCenter - toleransi:
                        print("Left",  cx, cy, "desire :", xCenter, yCenter)
                        cv2.putText(frame30, "Left - ", (cx-10, cy-25), font, 0.5, (255, 0, 0),1)
                    elif cx > xCenter + toleransi:
                        print("Right",  cx, cy, "desire :", xCenter, yCenter)
                        cv2.putText(frame30, "Right - ", (cx-30, cy-25), font, 0.5, (255, 0, 0),1)
                    elif cy < yCenter - toleransi:
                        print("Forward",  cx, cy, "desire :", xCenter, yCenter)
                        cv2.putText(frame30, "Forward", (cx+40, cy-25), font, 0.5, (255, 0, 0),1)
                    elif cy > yCenter:
                        print("Backward",  cx, cy, "desire :", xCenter, yCenter)
                        cv2.putText(frame30, "Backward", (cx+40, cy-25), font, 0.5, (255, 0, 0),1)
                    else :
                        print("Reach Target ELP",  cx, cy, "desire :", xCenter, yCenter)
                        cv2.putText(frame30, "None - ", (cx-10, cy-25), font, 0.5, (255, 0, 0),1)
                        cv2.putText(frame30, "None", (cx+40, cy-25), font, 0.5, (255, 0, 0),1)
                        land = True
                        break
                    cv2.putText(frame30, str(cx) + ',' +
                        str(cy), (cx,cy), font,
                        1, (255, 0, 0), 2)
            if land == True:
                print("ELP detected, Ready to land.......")
                break

                    #Koordinat tengah frame
                    #frame_center = round(frame.shape[1] / 2), round(frame.shape[0] / 2)
                    #frame = cv2.circle(frame, frame_center, 3, (0, 255, 0), 2)

                    # displaying the coordinates
                    # on the image window
                    
                    #cv2.putText(frame, "Red", (cx-20, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (255,255,255), 3)
            cv2.imshow("image", frame30)
            cv2.imshow("Red Mask", red_mask)
            # This also acts as
            keyCode = cv2.waitKey(1) & 0xFF == ord('q')
            # Stop the program on the ESC key
            if keyCode == 27:
                break
        cap.release()
        cv2.destroyAllWindows()
    else:
        print("Unable to open camera")

    
   
#while True:
showHSV()
