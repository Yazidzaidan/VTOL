import KeyPressModule as kp
from time import sleep
import numpy as np
import cv2
import math

##################### PARAMETERS #########################
fspeed = 117/10 #forward speed in cm/s based research (ideal = 15 cm/s)
aspeed = 360/10 #angular speed deegrees/sec (50 cm/s)
interval = 0.25

dInterval = fspeed*interval
aInterval = aspeed*interval
##########################################################
x, y = 500, 500
a = 0
yaw = 0
points = [(0,0), (0,0)]


kp.init()

def getKeyboardInput():
    lr, fb, ud, yv = 0, 0, 0, 0 #left rigth, forward backward, up down, yaw velocity
    speed = 15
    aspeed = 50
    d = 0
    global x,y,yaw,a # memasukkan variabel global ke dalam function

    if kp.getKey("LEFT"): 
        lr = -speed
        d = dInterval
        a = -180 #kepala mengarah ke barat
        #print("Left")
        #print(lr)
    elif kp.getKey("RIGHT"):
        
        lr = speed
        #print("Right")
        d = -dInterval
        a = 180 #kepala mengarah ke timur
        #print(lr)

    if kp.getKey("UP"): 
        
        fb = speed
        #print("Forward")
        d = dInterval
        a = 270 #kepala mengarah ke utara
        #print(fb)

    elif kp.getKey("DOWN"): 
        fb = -speed
        #print("Backward")
        d = -dInterval
        a = -90 #kepala mengarah ke selatan
        #print(fb)



    if kp.getKey("w"): 
        ud = speed
        print("Up")
    elif kp.getKey("s"): 
        ud = -speed
        print("Down")

    if kp.getKey("a"): 
        yv = aspeed
        #print("yaw")
        yaw -= aInterval
    elif kp.getKey("d"): 
        yv = -aspeed
        #print("velocity")
        yaw += aInterval

    if kp.getKey("q"): 
        #me.land()
        print("Land")
    #elif kp.getKey("e"): me.takeoff()

    sleep(0.1)
    a += yaw
    x += int(d*math.cos(math.radians(a)))
    y += int(d*math.sin(math.radians(a)))

    return [lr, fb, ud, yv, x, y]

def drawPoints(img, points):
    for point in points:
        cv2.circle(img, point, 5, (0, 0, 255), cv2.FILLED) #tracking pointer
    #cv2.circle(img, (points[0], points[1]), 5, (0, 0, 255), cv2.FILLED) #pointer pada frame
    cv2.circle(img, points[-1], 8, (0,255,0), cv2.FILLED)
    #cv2.putText(img, f'({(points[-1][0]-500)/100},{(points[-1][1]-500)/100})m', ##penentuan titik #dikurangi 500 karena titik awal 500, dibagi 100 karena ingin mendapatkan dalam meter
    #    (points[-1][0]+10,points[-1][1]+30), cv2.FONT_HERSHEY_PLAIN, 1, #penempatan kalimatnya
    #    (255,0,255), 1)
    posX = (points[-1][0]-500)/100
    posY = (points[-1][1]-500)/100
    cv2.putText(img, '(' + str(posX) + ',' + str(posY) + ')', ##penentuan titik #dikurangi 500 karena titik awal 500, dibagi 100 karena ingin mendapatkan dalam meter
        (points[-1][0]+10,points[-1][1]+30), cv2.FONT_HERSHEY_PLAIN, 1, #penempatan kalimatnya
        (255,0,255), 1)
    #print(posX, posY)
    


while True:
    #me.send_rc_control(vals[0], vals[1], vals[2], vals[3])
    vals = getKeyboardInput()

    img = np.zeros((1000, 1000, 3), np.uint8)
    if (points[-1][0] != vals[4] or points[-1][1] != vals[5]):
        points.append((vals[4], vals[5])) #get value from getKeyboardInput
    #print(points)
    drawPoints(img, points)
    cv2.imshow("Output", img)
    cv2.waitKey(1)



