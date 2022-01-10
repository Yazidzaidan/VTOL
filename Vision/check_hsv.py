import numpy as np
import cv2

cap = cv2.VideoCapture(0)
h = s = v = xpos = ypos = 0


def rescale_frame(frame, percent):
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)
def draw_function(event, x, y, flags, frame):
    if event == cv2.EVENT_LBUTTONUP:
        global b, g, r, xpos, ypos, clicked
        # clicked = True
        xpos = x
        ypos = y
        imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, s, v = imgHSV[y, x]
        print(h,s,v)
        
#cv2.namedWindow('frame')


while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    frame40 = rescale_frame(frame, percent=40)
    cv2.setMouseCallback('frame', draw_function, frame40)
    cv2.imshow('frame', frame40)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.destroyAllWindows()