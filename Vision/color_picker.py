import cv2
import numpy as np

colors =[]
posx =10
posy =10

def nothing(x):
    pass
refPt = []
cropping = False
mask = []
beres = False
toleransi1 =0
toleransi2 =0
toleransi3 =0

capture = cv2.VideoCapture(0)
capture.set(3,480)
capture.set(4,480)

track_min = (10, 240, 225) #hsv bola 161,101,204  (176,148,255)
track_max = (17, 255, 245)
cv2.namedWindow('mask')
cv2.createTrackbar('toleransi h', 'mask',10,100,nothing)
cv2.createTrackbar('toleransi s', 'mask',40,100,nothing)
cv2.createTrackbar('toleransi v', 'mask',50,100,nothing)
def click_and_crop(event, x, y, flags, param):
    # grab references to the global variables
    global refPt, cropping
    global average
    # if the left mouse button was clicked, record the starting
    # (x, y) coordinates and indicate that cropping is being
    # performed
    if event == cv2.EVENT_LBUTTONDOWN:
        refPt = [(x, y)]
        cropping = True
 
    # check to see if the left mouse button was released
    elif event == cv2.EVENT_LBUTTONUP:
        cv2.namedWindow('ROI')
        refPt.append((x, y))
        cropping = False
        clone = mask.copy()   
        # draw a rectangle around the region of interest
        image = cv2.rectangle(mask, refPt[0], refPt[1], (0, 255, 0), 2)
        cv2.imshow('frame', image)
        if len(refPt) >1:
            roi = clone[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]
            cv2.imshow("ROI", roi)
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            rata=np.average(hsv, axis=0)
            average = np.average(rata, axis=0)
            average= np.uint8(average)
            #print average
            average_color_img = np.array([[average]*200]*200, np.uint8)
            average_color_img = cv2.cvtColor(average_color_img, cv2.COLOR_HSV2BGR)
            cv2.putText(average_color_img , str(average), (5,20), cv2.FONT_HERSHEY_DUPLEX, 0.5,(0, 255, 255))
            cv2.imshow( "average_color.png", average_color_img )
            
def on_mouse_click (event, x, y, flags, frame):
    if event == cv2.EVENT_LBUTTONDOWN:
        global posx 
        global posy 
        posx = x
        posy = y
        colors.append(frame[y,x].tolist())
        
def main():
    global mask
    global hsv
    global beres
    _, frame = capture.read()
    mask = frame
        #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS_FULL)
        #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #if colors:
          #  cv2.putText(frame, str(colors[-1]), (posx,posy), cv2.FONT_HERSHEY_DUPLEX, 0.5,(0, 255, 255))
    cv2.imshow('frame', frame)
        #cv2.setMouseCallback('frame', on_mouse_click, hsv)
    cv2.setMouseCallback('frame', click_and_crop)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
       # capture.release()
       # cv2.destroyAllWindows()
        beres = True
        print ("TRACKING")
       # break

#if __name__ == "__main__":
while 1:
    while not beres  :
        main()
    while beres :
        hmin= average[0]-toleransi1
        smin=average[1]-toleransi2
        vmin=average[2]-toleransi3
        hmax= average[0]+toleransi1
        smax=average[1]+toleransi2
        vmax=average[2]+toleransi3
        if hmin < 0 :
            hmin=0
        if smin <0 :
            smin=0
        if vmin < 0:
            vmin =0
        if hmax > 255 :
            hmax = 255
        if smax >255 :
            smax =255
        if vmax >255 :
            vmax =255
        #print " ",(hmin,smin,vmin,hmax,smax,vmax)
        _, frame = capture.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (hmin,smin,vmin),(hmax,smax,vmax))
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
            
                if radius >5 :
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    x = (M["m10"] / M["m00"])
                    y = (M["m01"] / M["m00"])
                    cv2.circle(frame, (int(x), int(y)), int(radius),
                                (0, 255, 255), 2)
                    print (y)

        toleransi1 = cv2.getTrackbarPos('toleransi h','mask')
        toleransi2 = cv2.getTrackbarPos('toleransi s','mask')
        toleransi3 = cv2.getTrackbarPos('toleransi v','mask')
        cv2.imshow('mask', mask)
        cv2.imshow('frame', frame)
        
        #cv2.waitKey(1)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            beres = False
            break
        
