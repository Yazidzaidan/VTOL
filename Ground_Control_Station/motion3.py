import KeyPressModule as kp
from time import sleep
import numpy as np
import cv2
import math
import pyrealsense2 as rs
import MovementModule as mv
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions

# Prettier prints for reverse-engineering
from pprint import pprint


##################### PARAMETERS #########################
connection_string = '/dev/ttyACM0'
x,y = 0,0
points = [(0,0), (0,0)]
a,b,c,d = (0,0),(0,0),(0,0),(0,0)
count = 0
fw,bw,lf,rh,u,d = None,None,None,None,None,None
##########################################################
connection_string = '/dev/ttyACM0'
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)
print("connected")

#get pygame initialisasi
kp.init()


# Get realsense pipeline handle
pipe = rs.pipeline()

# Print all connected devices and find the T265
devices = rs.context().devices
for i in range(len(devices)):
    print('Found device:', devices[i].get_info(rs.camera_info.name), ', with serial number: ', devices[i].get_info(rs.camera_info.serial_number))

# Configure the pipeline
cfg = rs.config()

# Prints a list of available streams, not all are supported by each device
print('Available streams:')
pprint(dir(rs.stream))

# Enable streams you are interested in
cfg.enable_stream(rs.stream.pose) # Positional data (translation, rotation, velocity etc)
cfg.enable_stream(rs.stream.fisheye, 1) # Left camera
cfg.enable_stream(rs.stream.fisheye, 2) # Right camera

# Start the configured pipeline
pipe.start(cfg)

#################################### Realsense Input #################################################
def getRealsenseInput():
    global x,y
    frames = pipe.wait_for_frames()

    # Left fisheye camera frame
    left = frames.get_fisheye_frame(1)
    left_data = np.asanyarray(left.get_data())

    # Right fisheye camera frame
    right = frames.get_fisheye_frame(2)
    right_data = np.asanyarray(right.get_data())

    #print('Left frame', left_data.shape)
    #print('Right frame', right_data.shape)
    
    # Positional data frame
    pose = frames.get_pose_frame()
    if pose:
        pose_data = pose.get_pose_data()
        x = math.ceil(pose_data.translation.x * 100)
        y = math.ceil(pose_data.translation.z * 100)
        
        #print("Position xyz: % 2.4f % 2.4f" % (pose_data.translation.x, pose_data.translation.y))
        #print("Position x:  % 2.4f"%(x)) #2.4f artinya memberikan float, 4 angka dibelakang koma
        #print("Position y: % 2.4f"%(y))
    
    #x += int(fb)
    #y += int(lr)

    return [x, y]
########################################################################################################



###################################### Draw Pointer ####################################################
def drawPoints(img, points):
    #point = np.round(points).astype("int")
    for point in points:
        cv2.circle(img, point, 1, (150, 150, 150), cv2.FILLED) #tracking pointer
    cv2.circle(img, points[-1], 5, (0,0,255), cv2.FILLED)
    #cv2.circle(img, (x, y), 5, (0, 0, 255), cv2.FILLED) #pointer pada frame
    #cv2.circle(img, points, 5, (0,0,255), cv2.FILLED)
    #posX = (points[0])
    #posY = (points[1])
    cv2.putText(img, '(' + str(x) + ',' + str(y) + ')', ##penentuan titik #dikurangi 500 karena titik awal 500, dibagi 100 karena ingin mendapatkan dalam meter
        (points[-1][0]+10,points[-1][1]+30), cv2.FONT_HERSHEY_PLAIN, 1, #penempatan kalimatnya
        (255,0,255), 1)
    #print(posX, posY)
########################################################################################################



###################################### Draw Gedung ####################################################
def drawHome():
    cv2.rectangle(img, ((-50+500)/2, (-50+1000)/2), ((50+500)/2, (50+1000)/2), (255,255,255), cv2.FILLED) #+500 agar x di tengah +1000 agar z di tengah
def drawGdB():
    cv2.rectangle(img, ((-100+500)/2, (-900+1000)/2), ((100+500)/2, (-750+1000)/2), (0,255,0), cv2.FILLED)
def drawGdA():
    cv2.rectangle(img, ((-500+500)/2, (-900+1000)/2), ((-300+500)/2, (-700+1000)/2), (0,255,0), cv2.FILLED)
def drawGdC():
    cv2.rectangle(img, ((300+500)/2, (-900+1000)/2), ((500+500)/2, (-700+1000)/2), (0,255,0), cv2.FILLED)
def drawGD():
    drawHome()
    drawGdB()
    drawGdA()
    drawGdC()
########################################################################################################

def getKeyboardInput():
    global fb,lr,ud
    if kp.getKey("UP"): 
        fb = 1350
        #mv.forwardRemoteValue(1350)
        #print("Forward")
    elif kp.getKey("DOWN"): 
        fb = 1650
        #mv.backwardRemoteValue(1650)
        #print("Backward")
    #else:
    #    fw, bw = None, None
    elif kp.getKey("LEFT"): 
        lr = 1310
        #mv.leftRemoteValue(1310)
        #print("Left")
    elif kp.getKey("RIGHT"):
        lr = 1650
        #mv.rightRemoteValue(1650)
        #print("Right")
    #else:
    #    lf, rh = None, None
    elif kp.getKey("w"): 
        ud = 1600
        #mv.upRemoteValue(1600)
        #print("Up")
    elif kp.getKey("s"): 
        ud = 1250
        #mv.downRemoteValue(1580)
        #print("Down")
    else:
        fb, lr, ud = None, None, None

    return [fb, lr, ud]
        
        
start = raw_input("READY Keyboard Control Movement")


while True:
    
    
    value = getKeyboardInput()
    print(value[0], value[1], value[2])
    mv.sendRemoteValue(value[0], value[1], value[2], vehicle)

    img = np.zeros((550, 500, 3), np.uint8)
    drawGD()
    vals = getRealsenseInput()
    #print(vals[0], vals[1])
    
    #if (points[0] != vals[0] or points[1] != vals[1]):
     #get value from getKeyboardInput
    x = np.round(vals[0]).astype("int")
    y = np.round(vals[1]).astype("int")
    #print(x, y)
    points.append(((x+500)/2, (y+1000)/2))
    #points = ((vals[0]+500)/2, (vals[1]+1000)/2)
    #print(points)
    drawPoints(img, points)
    cv2.imshow("Output", img)
    cv2.waitKey(1)