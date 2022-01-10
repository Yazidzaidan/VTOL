#contoh penarikan data dari excel

from time import sleep
import time
import numpy as np
import cv2
import math
import sys
sys.path.append("/usr/local/lib/")

from numpy.lib.function_base import _rot90_dispatcher
import pyrealsense2 as rs
import pandas as pd
from pprint import pprint
import MovementModule as mv
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import serial
import pyzbar.pyzbar as pyzbar


##################### PARAMETERS #########################
x,z,pose = 0,0,0
points = [(0,0), (0,0)]
coordinate = (0,0)
forwardFast = 1400 #1400#1450#1370 #1350 
backwardFast = 1600 #1600#1550 #1600 #1650
leftFast = 1400 #1450 #1400 #1370
rightFast = 1600 #1550 #1600 #1650 #1650

forward = 1400 #1350#1400#1450#1370 #1350 
backward = 1550 #1600#1550 #1600 #1650
left = 1460 #1450 #1400 #1370
right = 1570 #1550 gakan kanan #1600 #1650 #1650
up = 1650 #1700
down = 1390
yawRight = 1550
yawLeft = 1450
targetPose = 0

#fb, lr, ud = None, None, None

#tfmini
ser = serial.Serial('COM13', baudrate = 115200, timeout=1)
detectWall = False
detectGD = '2'
detectGDA = '1'
detectGDB = '0' #di 95 kurang, drop di luar taman
detectGDC = '1'
#objectAvoid = 200
detectWall = '3'

#altitude
altitudeA = 1.7
altitudeB = 0.7
altitudeC = 1.3
altitudeUPA = 2.7
altitudeUPB = 2.0
altitudeUPC = 2.1

yaw = 0
pose = 0

#camera
cap = cv2.VideoCapture(1) #webcam
detected = False
ELP = False
#CSI-Camera
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

#succes the mission?
succes = False
##########################################################


##################### LOAD DATA ##########################
df = pd.read_excel('coordinate and area.xlsx')
#df.info()
#print(df)
df_sort = df.tail(14)
#df_sort.info()
print(df_sort)
#print(df_sort.iat[-2,0])
coorAx = df_sort['CoordX'].values[0]
coorAz = df_sort['CoordY'].values[0]
coorBx = df_sort['CoordX'].values[1]
coorBz = df_sort['CoordY'].values[1]
coorCx = df_sort['CoordX'].values[2]
coorCz = df_sort['CoordY'].values[2]
coorObsx = df_sort['CoordX'].values[3]
coorObsz = df_sort['CoordY'].values[3]
area1Ax = df_sort['CoordX'].values[4]
area1Az = df_sort['CoordY'].values[4]
area1Bx = df_sort['CoordX'].values[5]
area1Bz = df_sort['CoordY'].values[5]
area2Ax = df_sort['CoordX'].values[6]
area2Az = df_sort['CoordY'].values[6]
area2Bx = df_sort['CoordX'].values[7]
area2Bz = df_sort['CoordY'].values[7]
area3Ax = df_sort['CoordX'].values[8]
area3Az = df_sort['CoordY'].values[8]
area3Bx = df_sort['CoordX'].values[9]
area3Bz = df_sort['CoordY'].values[9]
area4Ax = df_sort['CoordX'].values[10]
area4Az = df_sort['CoordY'].values[10]
area4Bx = df_sort['CoordX'].values[11]
area4Bz = df_sort['CoordY'].values[11]
area5Ax = df_sort['CoordX'].values[12]
area5Az = df_sort['CoordY'].values[12]
area5Bx = df_sort['CoordX'].values[13]
area5Bz = df_sort['CoordY'].values[13]
print("Gd A : ", (coorAx,coorAz))
print("Gd B : ", (coorBx,coorBz))
print("Gd C : ", (coorCx,coorCz))
print("Obstacle : ", (coorObsx,coorObsz))
print("Area 1A : ", (area1Ax,area1Az))
print("Area 1B : ", (area1Bx,area1Bz))
print("Area 2A : ", (area2Ax,area2Az))
print("Area 2B : ", (area2Bx,area2Bz))
print("Area 3A : ", (area3Ax,area3Az))
print("Area 3B : ", (area3Bx,area3Bz))
print("Area 4A : ", (area4Ax,area4Az))
print("Area 4B : ", (area4Bx,area4Bz))
print("Area 5A : ", (area5Ax,area5Az))
print("Area 5B : ", (area5Bx,area5Bz))

df_2 = pd.read_excel('hsv_ELP.xlsx')
#df_2.info()
print("Data HSV ELP")
#print(df_2)
h_min = df_2['h'].min()
s_min = df_2['s'].min()
v_min = df_2['v'].min()
h_max = df_2['h'].max()
s_max = df_2['s'].max()
v_max = df_2['v'].max()
print("lower HSV :", h_min, s_min, v_min)
print("upper HSV :", h_max, s_max, v_max)
xCenter = 180
yCenter = 100
area1X = -321
area1Y = -282
area2X = -8
area2Y = -313
area3X = 289
area3Y = -349

df_3 = pd.read_excel('hsv_GD_C.xlsx')
#df_2.info()
print("Data HSV GD C")
#print(df_2)
h_min3 = df_3['h'].min()
s_min3 = df_3['s'].min()
v_min3 = df_3['v'].min()
h_max3 = df_3['h'].max()
s_max3 = df_3['s'].max()
v_max3 = df_3['v'].max()
print("lower HSV :", h_min3, s_min3, v_min3)
print("upper HSV :", h_max3, s_max3, v_max3)
##########################################################


################### Connect to Vehicle ###################
connection_string = '/dev/ttyACM0'
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)
print("connected")

def arm_and_takeoff_nogps(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """

    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.7
    SMOOTH_TAKEOFF_THRUST = 0.6

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check,
    # just comment it with your own responsibility.
    #while not vehicle.is_armable:
    #    print(" Waiting for vehicle to initialise...")
    #    time.sleep(1)


    print("Arming motors")
    # Copter should arm in GUIDED_NOGPS mode
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        sleep(1)

    print("Taking off!")

    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        current_altitude = vehicle.rangefinder.distance
        print(" Altitude: %f  Desired: %f" %
              (current_altitude, aTargetAltitude))
        if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
            print("Reached target altitude")
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust = thrust)
        sleep(0.2)

def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,
                         yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                         thrust = 0.5):
    """
    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                  When one is used, the other is ignored by Ardupilot.
    thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
            Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
            the code for maintaining current altitude.
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        math.radians(yaw_rate), # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)

def set_attitude(roll_angle = 0.0, pitch_angle = 0.0,
                 yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                 thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent more often than every
    second, as an ATTITUDE_TARGET order has a timeout of 1s.
    In AC3.2.1 and earlier the specified attitude persists until it is canceled.
    The code below should work on either version.
    Sending the message multiple times is the recommended way.
    """
    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)
    start = time.time()
    while time.time() - start < duration:
        send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
        sleep(0.1)
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, 0,
                         0, 0, True,
                         thrust)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]
##########################################################


##################### Realsense ##########################                             
# Get realsense pipeline handle
pipe = rs.pipeline()

# Print all connected devices and find the T265
devices = rs.context().devices
for i in range(len(devices)):
    print('Found device:', devices[i].get_info(rs.camera_info.name), ', with serial number: ', devices[i].get_info(rs.camera_info.serial_number))

# Configure the pipeline
cfg = rs.config()

# Prints a list of available streams, not all are supported by each device
#print('Available streams:')
#pprint(dir(rs.stream))

# Enable streams you are interested in
cfg.enable_stream(rs.stream.pose) # Positional data (translation, rotation, velocity etc)
cfg.enable_stream(rs.stream.fisheye, 1) # Left camera
cfg.enable_stream(rs.stream.fisheye, 2) # Right camera

# Start the configured pipeline
pipe.start(cfg)


#################################### Realsense Input #################################################
def getRealsenseInput():
    global x,z, pose
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
        z = math.ceil(pose_data.translation.z * 100)
        pose = math.ceil(pose_data.rotation.y * 100)
        #if rotate < 0:
        #    rotate = rotate * -1
        
        #print("Position xyz: % 2.4f % 2.4f" % (pose_data.translation.x, pose_data.translation.y))
        #print("Position x:  % 2.4f"%(x)) #2.4f artinya memberikan float, 4 angka dibelakang koma
        #print("Position y: % 2.4f"%(y))
    
    #x += int(fb)
    #y += int(lr)

    return [x, z, pose]
########################################################################################################

###################################### Draw Pointer ####################################################
def drawPoints(img, points, x, z):
    #point = np.round(points).astype("int")
    for point in points:
        cv2.circle(img, point, 1, (150, 150, 150), cv2.FILLED) #tracking pointer
    cv2.circle(img, points[-1], 5, (0,0,255), cv2.FILLED)
    #cv2.circle(img, (x, y), 5, (0, 0, 255), cv2.FILLED) #pointer pada frame
    #cv2.circle(img, points, 5, (0,0,255), cv2.FILLED)
    #posX = (points[0])
    #posY = (points[1])
    cv2.putText(img, '(' + str(x) + ',' + str(z) + ')', ##penentuan titik #dikurangi 500 karena titik awal 500, dibagi 100 karena ingin mendapatkan dalam meter
        (points[-1][0]+10,points[-1][1]+30), cv2.FONT_HERSHEY_PLAIN, 1, #penempatan kalimatnya
        (255,0,255), 1)
    #print(posX, posY)
########################################################################################################


###################################### Draw Gedung ####################################################
def drawHome(img):
    cv2.rectangle(img, ((-50+500)/2, (-50+1000)/2), ((50+500)/2, (50+1000)/2), (255,255,255), cv2.FILLED) #+500 agar x di tengah +1000 agar z di tengah
def drawGdB(img):
    cv2.rectangle(img, ((-100+500)/2, (-900+1000)/2), ((100+500)/2, (-750+1000)/2), (0,255,0), cv2.FILLED)
def drawGdA(img):
    cv2.rectangle(img, ((-500+500)/2, (-900+1000)/2), ((-300+500)/2, (-700+1000)/2), (0,255,0), cv2.FILLED)
def drawGdC(img):
    cv2.rectangle(img, ((300+500)/2, (-900+1000)/2), ((500+500)/2, (-700+1000)/2), (0,255,0), cv2.FILLED)
def drawGD(img):
    drawHome(img)
    drawGdB(img)
    drawGdA(img)
    drawGdC(img)
########################################################################################################


def rescale_frame(frame, percent):
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)
def detectQR():
    #global detected
    #sleep(1)
    _, frame = cap.read()
    frame40 = rescale_frame(frame, percent=30)
    decode_QR = pyzbar.decode(frame40)
    detected = False
    for qrcode in decode_QR:
        (x,y,w,h) = qrcode.rect
        cv2.rectangle(frame40, (x,y),(x + w, y +w),(0,0,255), 2)
        cv2.putText(frame40, str(qrcode.data), (100, 100), cv2.FONT_HERSHEY_PLAIN,2,(255,0,0), 3)
        print(qrcode.data)
        detected = True
    cv2.imshow("QR Code Scanner",frame40)
    cv2.waitKey(1) 
    return detected
def detectELP( coorX, coorZ):
    print("vehicle goto : ", coorX, coorZ)
    sleep(0.5)
    toleransi = 50
    #ELP = False
    print(gstreamer_pipeline(flip_method=0))
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    toleransiELP = 90
    detectedELP = False
    if cap.isOpened():
        #Red color
        high_red = np.array([h_max, s_max, v_max])
        low_red = np.array([h_min, s_min, v_min])
        window_handle = cv2.namedWindow("image", cv2.WINDOW_AUTOSIZE)
        # Window
        while cv2.getWindowProperty("image", 0) >= 0:
            showMaps()
            ret_val, frame = cap.read()
            frame30 = rescale_frame(frame, percent=30)
            
            hsv_frame = cv2.cvtColor(frame30, cv2.COLOR_BGR2HSV)
            red_mask = cv2.inRange(hsv_frame, low_red, high_red)
            cnts, _ = cv2.findContours(red_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            
            for c in cnts:
                
                #land = True
                #break
                area = cv2.contourArea(c)
                if area > 1000:
                    #ELP = True
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
                    cv2.circle(frame30, (cx,cy),5,(255,255,255),-1)
                    killProgram()
                    if cx < xCenter - toleransiELP and cy < yCenter - toleransiELP:
                        print("Left Forward",  cx, cy, "desire :", xCenter, yCenter)
                        cv2.putText(frame30, "Left Forward- ", (cx-30, cy-25), font, 0.5, (255, 0, 0),1)
                        mv.sendRemoteValue(forward, left, None, None, vehicle)
                    elif cx < xCenter - toleransiELP and cy > yCenter + toleransiELP:
                        print("Left Backward",  cx, cy, "desire :", xCenter, yCenter)
                        cv2.putText(frame30, "Left Backward- ", (cx+40, cy-25), font, 0.5, (255, 0, 0),1)
                        mv.sendRemoteValue(backward, left, None, None, vehicle)
                    elif cx > xCenter + toleransiELP and cy < yCenter - toleransiELP:
                        print("Right Forward",  cx, cy, "desire :", xCenter, yCenter)
                        cv2.putText(frame30, "Right Forward- ", (cx+40, cy-25), font, 0.5, (255, 0, 0),1)
                        mv.sendRemoteValue(forward, right, None, None, vehicle)
                    elif cx > xCenter + toleransiELP and cy > yCenter + toleransiELP:
                        print("Right Backward",  cx, cy, "desire :", xCenter, yCenter)
                        cv2.putText(frame30, "Right Backward- ", (cx-10, cy-25), font, 0.5, (255, 0, 0),1)
                        mv.sendRemoteValue(backward, right, None, None, vehicle)
                   
                    elif cx > xCenter + toleransiELP:
                        print("Right",  cx, cy, "desire :", xCenter, yCenter)
                        cv2.putText(frame30, "Right - ", (cx-30, cy-25), font, 0.5, (255, 0, 0),1)
                        mv.sendRemoteValue(None, right, None, None, vehicle)
                    elif cx < xCenter - toleransiELP:
                        print("Left",  cx, cy, "desire :", xCenter, yCenter)
                        cv2.putText(frame30, "Left - ", (cx-30, cy-25), font, 0.5, (255, 0, 0),1)
                        mv.sendRemoteValue(None, left, None, None, vehicle)
                    elif cy < yCenter - toleransiELP:
                        print("Forward",  cx, cy, "desire :", xCenter, yCenter)
                        cv2.putText(frame30, "Forward", (cx+40, cy-25), font, 0.5, (255, 0, 0),1)
                        mv.sendRemoteValue(forward, None, None, None, vehicle)
                    elif cy > yCenter + toleransiELP:
                        print("Backward",  cx, cy, "desire :", xCenter, yCenter)
                        cv2.putText(frame30, "Backward", (cx+40, cy-25), font, 0.5, (255, 0, 0),1)
                        mv.sendRemoteValue(backward, None, None, None, vehicle)
                    else :
                        mv.sendRemoteValue(None, None, None, None, vehicle)
                        print("Reach Target ELP",  cx, cy, "desire :", xCenter, yCenter)
                        cv2.putText(frame30, "None - ", (cx-10, cy-25), font, 0.5, (255, 0, 0),1)
                        cv2.putText(frame30, "None", (cx+40, cy-25), font, 0.5, (255, 0, 0),1)
                        detectedELP = True
                        break
                    cv2.putText(frame30, str(cx) + ',' +
                        str(cy), (cx,cy), font,
                        1, (255, 0, 0), 2)

            
            if detectedELP == False:
                if  pose > targetPose + 20:
                    print("Right Yaw", pose, "desire :", targetPose)
                    mv.sendRemoteValue(None, None, None, yawRight, vehicle)
                elif pose < targetPose - 20:
                    print("Left Yaw",  pose, "desire :", targetPose)
                    mv.sendRemoteValue(None, None, None, yawLeft, vehicle)
                elif  x > coorX + toleransi and z > coorZ + toleransi:
                    print("Left Forward",  x, z, "desire :", coorX, coorZ, "QR", detectedELP, "Yaw:", pose)
                    mv.sendRemoteValue(forward, left, None, None, vehicle)
                elif x < coorX - toleransi and z > coorZ + toleransi:
                    print("Right Forward",  x, z, "desire :", coorX, coorZ, "QR", detectedELP, "Yaw:", pose)
                    mv.sendRemoteValue(forward, right, None, None, vehicle)
                elif x > coorX + toleransi and z < coorZ - toleransi:
                    print("Left Backward",  x, z, "desire :", coorX, coorZ, "QR", detectedELP, "Yaw:", pose)
                    mv.sendRemoteValue(backward, left, None, None, vehicle)
                elif x < coorX - toleransi and z < coorZ - toleransi:
                    print("Right Backward",  x, z, "desire :", coorX, coorZ, "QR", detectedELP, "Yaw:", pose)
                    mv.sendRemoteValue(backward, right, None, None, vehicle)

                elif x < coorX - toleransi:
                    print("Right",  x, z, "desire :", coorX, coorZ, "QR", detectedELP, "Yaw:", pose)
                    mv.sendRemoteValue(None, right, None, None, vehicle)
                elif x > coorX + toleransi:
                    print("left",  x, z, "desire :", coorX, coorZ, "QR", detectedELP, "Yaw:", pose)
                    mv.sendRemoteValue(None, left, None, None, vehicle)
                elif z > coorZ + toleransi:
                    print("forward",  x, z, "desire :", coorX, coorZ, "QR", detectedELP, "Yaw:", pose)
                    mv.sendRemoteValue(forward, None, None, None, vehicle)
                elif z < coorZ - toleransi:
                    print("backward",  x, z, "desire :", coorX, coorZ, "QR", detectedELP, "Yaw:", pose)
                    mv.sendRemoteValue(backward, None, None, None, vehicle)
                else:
                    mv.sendRemoteValue(None, None, None, None, vehicle)
                    print("reach target coordinate : ", x, z, "Desire: ", coorX, coorZ, "QR", detectedELP, "Yaw:", pose)
                    mv.sendRemoteValue(None, None, None, None, vehicle)
                    print("stabilizing.....")
                    #for i in range(200):
                    #    showMaps()
                    #sleep(3)
                    print("goto", coorX, coorZ,"finish")
                    break  
            
                killProgram() 

def ELP():
    print("vehicle ELP")
    sleep(0.5)
    toleransi = 50
    #ELP = False
    print(gstreamer_pipeline(flip_method=0))
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    toleransiELP = 90
    detectedELP = False
    current_altitude = vehicle.rangefinder.distance
    if cap.isOpened():
        #Red color
        high_red = np.array([h_max, s_max, v_max])
        low_red = np.array([h_min, s_min, v_min])
        window_handle = cv2.namedWindow("image", cv2.WINDOW_AUTOSIZE)
        # Window
        while cv2.getWindowProperty("image", 0) >= 0:
            showMaps()
            ret_val, frame = cap.read()
            frame30 = rescale_frame(frame, percent=30)
            
            hsv_frame = cv2.cvtColor(frame30, cv2.COLOR_BGR2HSV)
            red_mask = cv2.inRange(hsv_frame, low_red, high_red)
            cnts, _ = cv2.findContours(red_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            
            for c in cnts:
                
                #land = True
                #break
                area = cv2.contourArea(c)
                if area > 1000:
                    #ELP = True
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
                    cv2.circle(frame30, (cx,cy),5,(255,255,255),-1)
                    killProgram()
                    if cx < xCenter - toleransiELP and cy < yCenter - toleransiELP:
                        print("Left Forward",  cx, cy, "desire :", xCenter, yCenter)
                        cv2.putText(frame30, "Left Forward- ", (cx-30, cy-25), font, 0.5, (255, 0, 0),1)
                        mv.sendRemoteValue(forward, left, None, None, vehicle)
                    elif cx < xCenter - toleransiELP and cy > yCenter + toleransiELP:
                        print("Left Backward",  cx, cy, "desire :", xCenter, yCenter)
                        cv2.putText(frame30, "Left Backward- ", (cx+40, cy-25), font, 0.5, (255, 0, 0),1)
                        mv.sendRemoteValue(backward, left, None, None, vehicle)
                    elif cx > xCenter + toleransiELP and cy < yCenter - toleransiELP:
                        print("Right Forward",  cx, cy, "desire :", xCenter, yCenter)
                        cv2.putText(frame30, "Right Forward- ", (cx+40, cy-25), font, 0.5, (255, 0, 0),1)
                        mv.sendRemoteValue(forward, right, None, None, vehicle)
                    elif cx > xCenter + toleransiELP and cy > yCenter + toleransiELP:
                        print("Right Backward",  cx, cy, "desire :", xCenter, yCenter)
                        cv2.putText(frame30, "Right Backward- ", (cx-10, cy-25), font, 0.5, (255, 0, 0),1)
                        mv.sendRemoteValue(backward, right, None, None, vehicle)
                   
                    elif cx > xCenter + toleransiELP:
                        print("Right",  cx, cy, "desire :", xCenter, yCenter)
                        cv2.putText(frame30, "Right - ", (cx-30, cy-25), font, 0.5, (255, 0, 0),1)
                        mv.sendRemoteValue(None, right, None, None, vehicle)
                    elif cx < xCenter - toleransiELP:
                        print("Left",  cx, cy, "desire :", xCenter, yCenter)
                        cv2.putText(frame30, "Left - ", (cx-30, cy-25), font, 0.5, (255, 0, 0),1)
                        mv.sendRemoteValue(None, left, None, None, vehicle)
                    elif cy < yCenter - toleransiELP:
                        print("Forward",  cx, cy, "desire :", xCenter, yCenter)
                        cv2.putText(frame30, "Forward", (cx+40, cy-25), font, 0.5, (255, 0, 0),1)
                        mv.sendRemoteValue(forward, None, None, None, vehicle)
                    elif cy > yCenter + toleransiELP:
                        print("Backward",  cx, cy, "desire :", xCenter, yCenter)
                        cv2.putText(frame30, "Backward", (cx+40, cy-25), font, 0.5, (255, 0, 0),1)
                        mv.sendRemoteValue(backward, None, None, None, vehicle)
                    else :
                        mv.sendRemoteValue(None, None, None, None, vehicle)
                        print("Reach Target ELP",  cx, cy, "desire :", xCenter, yCenter)
                        cv2.putText(frame30, "None - ", (cx-10, cy-25), font, 0.5, (255, 0, 0),1)
                        cv2.putText(frame30, "None", (cx+40, cy-25), font, 0.5, (255, 0, 0),1)
                        detectedELP = True
                        break
                    cv2.putText(frame30, str(cx) + ',' +
                        str(cy), (cx,cy), font,
                        1, (255, 0, 0), 2)

                killProgram()
            
            if detectedELP == True:
                #print("ELP detected, Ready to land.......")
                print("down", current_altitude, "Yaw:", pose)
                mv.sendRemoteValue(None, None, down, None, vehicle)
                break
            
             
           
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
    return detectedELP

#"""
def progress(string):
    print(string)
    sys.stdout.flush()
def send_msg_to_gcs(text_to_be_sent):
    # MAV_SEVERITY: 0=EMERGENCY 1=ALERT 2=CRITICAL 3=ERROR, 4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG, 8=ENUM_END
    text_msg = 'Mission: ' + text_to_be_sent
    conn.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text_msg.encode())
    progress("INFO: %s" % text_to_be_sent)

progress("INFO: Starting Vehicle communications")
conn = mavutil.mavlink_connection(
    connection_string,
    autoreconnect = True,
    source_system = 1,
    #source_component = 93,
    force_connected=True,
)
#"""
def killProgram():
    if vehicle.channels['8']>1500 :
        mv.sendRemoteValue(None, None, None, None, vehicle)
        print("Kill Program...")
        exit()
    else:
        pass

def showMaps():
    img = np.zeros((550, 500, 3), np.uint8)
    drawGD(img)
    vals = getRealsenseInput()
    #print(vals[0], vals[1])
    x = np.round(vals[0]).astype("int")
    z = np.round(vals[1]).astype("int")
    rotate = np.round(vals[2]).astype("int")
    points.append(((x+500)/2, (z+1000)/2))
    drawPoints(img, points, x, z)
    cv2.imshow("Output", img)
    cv2.waitKey(1)
    return rotate


def goto(coorX, coorZ, string):
    print("vehicle goto : ", coorX, coorZ)
    sleep(0.5)
    toleransi = 20
    while True:
        showMaps()
        count = ser.in_waiting
        if count > 8:
            recv = ser.read(9)    
            ser.reset_input_buffer()        
            if recv[0] == 'Y' and recv[1] == 'Y':     #python2
                lowD = int(recv[2].encode('hex'), 16)      
                highD = int(recv[3].encode('hex'), 16)
                distance = lowD + highD * 256
        
        killProgram()
        detectedQR = detectQR()
        #print(detectedQR)
        if detectedQR == True:
            print("QR Detected")
            print("QR Detected")
            print("QR Detected")
            mv.sendRemoteValue(None, None, None, None, vehicle)
            print("stabilizing.....")
            #for i in range(200):
            #    showMaps()
            sleep(3)
            #cap.release()
            #cv2.destroyWindow("QR Code Scanner")
            print("goto", coorX, coorZ,"finish")
            #break 

                     
        if  pose > targetPose + 10:
            print("Right Yaw", pose, "desire :", targetPose, distance, string)
            mv.sendRemoteValue(None, None, None, yawRight, vehicle)
        elif pose < targetPose - 10:
            print("Left Yaw",  pose, "desire :", targetPose, distance, string)
            mv.sendRemoteValue(None, None, None, yawLeft, vehicle)
        elif  x > coorX + toleransi and z > coorZ + toleransi:
            print("Left Forward",  x, z, "desire :", coorX, coorZ, "QR", detectedQR, "Yaw:", pose, distance, string)
            mv.sendRemoteValue(forward, left, None, None, vehicle)
        elif x < coorX - toleransi and z > coorZ + toleransi:
            print("Right Forward",  x, z, "desire :", coorX, coorZ, "QR", detectedQR, "Yaw:", pose, distance, string)
            mv.sendRemoteValue(forward, rightFast, None, None, vehicle)
        elif x > coorX + toleransi and z < coorZ - toleransi:
            print("Left Backward",  x, z, "desire :", coorX, coorZ, "QR", detectedQR, "Yaw:", pose, distance, string)
            mv.sendRemoteValue(backward, left, None, None, vehicle)
        elif x < coorX - toleransi and z < coorZ - toleransi:
            print("Right Backward",  x, z, "desire :", coorX, coorZ, "QR", detectedQR, "Yaw:", pose, distance, string)
            mv.sendRemoteValue(backward, rightFast, None, None, vehicle)

        elif x < coorX - toleransi:
            print("Right",  x, z, "desire :", coorX, coorZ, "QR", detectedQR, "Yaw:", pose, distance, string)
            mv.sendRemoteValue(None, rightFast, None, None, vehicle)
        elif x > coorX + toleransi:
            print("left",  x, z, "desire :", coorX, coorZ, "QR", detectedQR, "Yaw:", pose, distance, string)
            mv.sendRemoteValue(None, left, None, None, vehicle)
        elif z > coorZ + toleransi:
            print("forward",  x, z, "desire :", coorX, coorZ, "QR", detectedQR, "Yaw:", pose, distance, string)
            mv.sendRemoteValue(forward, None, None, None, vehicle)
        elif z < coorZ - toleransi:
            print("backward",  x, z, "desire :", coorX, coorZ, "QR", detectedQR, "Yaw:", pose, distance, string)
            mv.sendRemoteValue(backward, None, None, None, vehicle)
        
            #break
        else:
            mv.sendRemoteValue(None, None, None, None, vehicle)
            print("reach target coordinate : ", x, z, "Desire: ", coorX, coorZ, "QR", detectedQR, "Yaw:", pose, distance, string)
            mv.sendRemoteValue(None, None, None, None, vehicle)
            print("stabilizing.....")
            #for i in range(200):
            #    showMaps()
            #sleep(3)
            print("goto", coorX, coorZ, string, "finish")
            break  

def getValuesTFMini(char):
    global detectWall, backward
    ser.write(char)
    if (ser.in_waiting > 0):
        arduinoData = ser.readline().decode('ascii').strip()
        if arduinoData == u'1':
            detectWall = True
            backward = True
        else:
            detectWall = False
            backward = False
        return arduinoData
def straight(userInput, coorX, string):
    global detectWall
    print("vehicle goto forward :", userInput)
    toleransiX = 15
    #userInput = raw_input('Get data point?')
    current_altitude = vehicle.rangefinder.distance
    while True:
        showMaps()
        killProgram()

        if detectWall == True:
            mv.sendRemoteValue(None, None, None, None, vehicle)
            print("reach distance : ", detectWall, "Desire: ", userInput, "Yaw:", pose, "x:", coorX, "alt:", current_altitude, string)
            break
        if  pose > targetPose + 11:
            mv.sendRemoteValue(None, None, None, yawRight, vehicle)
            print("Right Yaw", pose, "desire :", targetPose, "detect:", detectWall, "desire:", userInput, "x:", coorX, "alt:", current_altitude, string)
        elif pose < targetPose - 11:
            mv.sendRemoteValue(None, None, None, yawLeft, vehicle)
            print("Left Yaw",  pose, "desire :", targetPose, "detect:", detectWall, "desire:", userInput, "x:", coorX, "alt:", current_altitude, string)
        elif x < coorX - toleransiX:
            mv.sendRemoteValue(None, right, None, None, vehicle)
            print("Right",  x, "desire :", coorX, "detect:", detectWall, "desire:", userInput, "Yaw:", pose, "x:", coorX, "alt:", current_altitude, string)
        elif x > coorX + toleransiX:
            mv.sendRemoteValue(None, left, None, None, vehicle)
            print("left",  x, "desire :", coorX, "detect:", detectWall, "desire:", userInput, "Yaw:", pose, "x:", coorX, "alt:", current_altitude, string)
        elif userInput == '0': # GD B
            mv.sendRemoteValue(forward, None, None, None, vehicle)
            val = getValuesTFMini(b'0')
            print("goto GD B", val, "detectGD_B:", detectWall, "desire:", userInput, "Yaw:", pose, "x:", coorX, "alt:", current_altitude, string)
        elif userInput == '1': # GD AC
            mv.sendRemoteValue(forward, None, None, None, vehicle)
            val = getValuesTFMini(b'1')
            print("goto GD A & C", val, "detectGD_A_C:", detectWall, "desire:", userInput, "Yaw:", pose, "x:", coorX, "alt:", current_altitude, string)
        elif userInput == '2': # GD AC
            mv.sendRemoteValue(forward, None, None, None, vehicle)
            val = getValuesTFMini(b'2')
            print("goto Buildings", val, "detectWall:", detectWall, "desire:", userInput, "Yaw:", pose," x:", coorX, "alt:", current_altitude, string)

def Back(userInput, string):
    global backward
    backward = True
    print("vehicle goto Backward :", userInput)
    #sleep(0.5)
    toleransi = 5
    current_altitude = vehicle.rangefinder.distance
    while True:
        showMaps()
        killProgram()
        if backward == False:
            mv.sendRemoteValue(None, None, None, None, vehicle)
            print("reach distance : ", backward, "Desire: ", userInput, "Yaw:", pose, "alt:", current_altitude, string)
            print("Backward done....")
            break
        if  pose > targetPose + 11:
            mv.sendRemoteValue(None, None, None, yawRight, vehicle)
            print("Right Yaw", pose, "desire :", targetPose, "detect:", backward, "desire:", userInput, "alt:", current_altitude, string)
        elif pose < targetPose - 11:
            mv.sendRemoteValue(None, None, None, yawLeft, vehicle)
            print("Left Yaw",  pose, "desire :", targetPose, "detect:", backward, "desire:", userInput, "alt:", current_altitude, string)
        elif userInput == '3': # backward
            mv.sendRemoteValue(backward, None, None, None, vehicle)
            val = getValuesTFMini(b'3')
            print("backward", val, "back", backward, "desire:", userInput, "Yaw:", pose, "alt:", current_altitude, string)

def drop():
    vehicle.channels.overrides['6'] = 990
    mv.sendRemoteValue(backward, None, None, None, vehicle)
    sleep(0.5)
    vehicle.channels.overrides['6'] = 2000
    mv.sendRemoteValue(None, None, 1500, None, vehicle)
    print("drop log")
    print("drop log done.....!")
def setAltitude(aTargetAltitude):
    toleransiAlt = 0.3
    print("vehicle goto altitude :", aTargetAltitude)
    sleep(0.5)
    while True:
        killProgram()
        showMaps()
        count = ser.in_waiting
        
        if count > 8:
            recv = ser.read(9)    
            ser.reset_input_buffer()        
            if recv[0] == 'Y' and recv[1] == 'Y':     #python2
                lowD = int(recv[2].encode('hex'), 16)      
                highD = int(recv[3].encode('hex'), 16)
                distance = lowD + highD * 256

        current_altitude = vehicle.rangefinder.distance
        if  pose > targetPose + 9:
            mv.sendRemoteValue(None, None, None, yawRight, vehicle)
            print("Right Yaw", pose, "desire :", targetPose, distance)
        elif pose < targetPose - 9:
            mv.sendRemoteValue(None, None, None, yawLeft, vehicle)
            print("Left Yaw",  pose, "desire :", targetPose, distance)
        elif vehicle.channels['8']>1500 :
            mv.sendRemoteValue(None, None, None, None, vehicle)
            print("break")
            break

        elif current_altitude <= aTargetAltitude - toleransiAlt: # Trigger just below target alt.
            print("up", current_altitude, "desire :", aTargetAltitude, "Yaw:", pose, distance)
            mv.sendRemoteValue(None, None, up, None, vehicle)
        elif current_altitude >= aTargetAltitude + toleransiAlt: # Trigger just below target alt.
            print("down", current_altitude, "desire :", aTargetAltitude, "Yaw:", pose, distance)
            mv.sendRemoteValue(None, None, down, None, vehicle)
        else :
            print("reach target altitude : ", current_altitude, "Desire: ", aTargetAltitude, "Yaw:", pose, distance)
            mv.sendRemoteValue(None, None, None, None, vehicle)
            print("stabilizing.....")
            #for i in range(200):
            #    showMaps()
            #sleep(1)
            break

#yaw pixhawk
def set_pose(aTargetPose):
    print("vehicle set pose : ", aTargetPose)
    #sleep(0.5)
    toleransi = 5
    while True:
        showMaps()

        killProgram()
        
        current_pose = pose
        if  current_pose > aTargetPose + toleransi:
            print("Right Yaw", current_pose, "desire :", targetPose)
            mv.sendRemoteValue(None, None, None, yawRight, vehicle)
        elif current_pose < aTargetPose - toleransi:
            print("Left Yaw",  current_pose, "desire :", targetPose)
            mv.sendRemoteValue(None, None, None, yawLeft, vehicle)
        else:
            print("reach target Yaw: ", current_pose, "Desire: ", aTargetPose)
            mv.sendRemoteValue(None, None, None, None, vehicle)
            break



def goto_GD_A():
    print("Heading to waypoint 1")
    send_msg_to_gcs('1 WP')
    progress('1 WP')
    sleep(0.1)
    print("Set pose to", targetPose)
    sleep(0.1)
    set_pose(targetPose)
    goto(coorAx, coorAz, "goto GD_A")
    print("Taking of to altitude !", altitudeA)
    sleep(0.1)
    set_pose(targetPose)
    setAltitude(altitudeA)
    print("Heading to Building")
    sleep(0.1)
    set_pose(targetPose)
    straight(detectGD, coorAx, "forward GD_A")
    setAltitude(altitudeUPA)
    straight(detectGDA, coorAx, "forward GD_A")
    drop()
    set_pose(targetPose)
    Back(detectWall)
def goto_GD_B():
    print("Heading to waypoint 2")
    send_msg_to_gcs('2 WP')
    sleep(0.1)
    print("Set pose to", targetPose)
    sleep(0.1)
    set_pose(targetPose)
    goto(coorBx, coorBz, "goto GD_B")
    print("Taking of to altitude!", altitudeB)
    sleep(0.1)
    print("Set pose to", targetPose)
    sleep(0.1)
    set_pose(targetPose)
    setAltitude(altitudeB)
    print("Heading to Building")
    sleep(0.1)
    straight(detectGD, coorBx, "forward GD_B")
    setAltitude(altitudeUPB)
    set_pose(targetPose)
    straight(detectGDB, coorBx, "forward Wall GD_B")
    drop()
    set_pose(targetPose)
    Back(detectWall)
def goto_GD_C():
    print("Heading to waypoint 3")
    send_msg_to_gcs('3 WP')
    sleep(0.1)
    print("Set pose to", targetPose)
    sleep(0.1)
    set_pose(targetPose)
    goto(coorCx, coorCz, "goto GD_C")
    print("Taking of to altitude !", altitudeC)
    sleep(0.1)
    set_pose(targetPose)
    setAltitude(altitudeC)
    print("Heading to Building")
    sleep(0.1)
    set_pose(targetPose)
    straight(detectGD, coorCx, "forward GD_C")
    setAltitude(altitudeUPC)
    set_pose(targetPose)
    straight(detectGDC, coorCx, "forward GD_C")
    drop()
    set_pose(targetPose)
    Back(detectWall)
def goto_ELP():
    print("Heading to ELP")
    sleep(0.1)
    print("Set pose to", targetPose)
    sleep(0.1)
    set_pose(targetPose)
    goto(coorObsx, coorObsz, "Goto ELP")
def goto_area1():
    global succes
    print("Heading to ELP")
    sleep(0.1)
    print("Set pose to", targetPose)
    sleep(0.1)
    set_pose(targetPose)
    land = detectELP(area1Ax, area1Az)
    land = detectELP(area1Bx, area1Bz)
    if land == True:
        print("Ready to land...")
        succes = True
    return succes
def goto_area2():
    global succes
    print("Heading to ELP")
    sleep(0.1)
    print("Set pose to", targetPose)
    sleep(0.1)
    set_pose(targetPose)
    #setAltitude(3)
    land = detectELP(area2Ax, area2Az)
    land = detectELP(area2Bx, area2Bz)
    if land == True:
        print("Ready to land...")
        succes = True
    return succes
def goto_area3():
    global succes
    print("Heading to ELP")
    sleep(0.1)
    print("Set pose to", targetPose)
    sleep(0.1)
    set_pose(targetPose)
    #setAltitude(3)
    land = detectELP(area3Ax, area3Az)
    land = detectELP(area3Bx, area3Bz)
    if land == True:
        print("Ready to land...")
        succes = True
    return succes
def goto_area4():
    global succes
    print("Heading to ELP")
    sleep(0.1)
    print("Set pose to", targetPose)
    sleep(0.1)
    set_pose(targetPose)
    #setAltitude(3)
    land = detectELP(area4Ax, area4Az)
    land = detectELP(area4Bx, area4Bz)
    if land == True:
        print("Ready to land...")
        succes = True
    return succes
def goto_area5():
    global succes
    print("Heading to ELP")
    sleep(0.1)
    print("Set pose to", targetPose)
    sleep(0.1)
    set_pose(targetPose)
    #setAltitude(3)
    land = detectELP(area5Ax, area5Az)
    land = detectELP(area5Bx, area5Bz)
    if land == True:
        print("Ready to land...")
        succes = True
    return succes
def ujicoba():
    straight(400, coorAx, "forward GD_B")
def ujicobaCam():
    global succes
    goto_area2()
    setAltitude(3.5)
    detectELP()
    setAltitude(1)
    succes = True
    return succes

def mode1() :
    #A B C
    global succes #succes mission
    goto_GD_A()
    goto_GD_B()
    goto_GD_C()
    goto_ELP()
    succes = True
    return succes
def mode2() :
    #A C B
    global succes #succes mission
    goto_GD_A()
    goto_GD_C()
    goto_GD_B()
    goto_ELP()
    succes = True
    return succes
def mode3() :
    #B A C
    global succes #succes mission
    #setAltitude(2.5)
    #goto_GD_B()
    #goto(coorAx, coorAz, objectAvoid)
    #goto_GD_A()
    #goto(coorCx, coorCz, objectAvoid)
    #goto_GD_C()
    goto_ELP()
    setAltitude(2.3)
    ELP()
    setAltitude(0.7)
    ELP()
    succes = True
    return succes
def mode4() :
    #C A B
    global succes #succes mission
    goto_GD_C()
    goto_GD_A()
    goto_GD_B()
    goto_ELP()
    succes = True
    return succes
def mode5() :
    #B C A
    global succes #succes mission
    goto_GD_B()
    goto_GD_C()
    goto_GD_A()
    goto_ELP()
    succes = True
    return succes
def mode6() :
    #C B A
    global succes #succes mission
    goto_GD_C()
    goto_GD_B()
    goto_GD_A()
    goto_ELP()
    succes = True
    return succes

def start():
    global succes
    while True:
        #print(succes)

        killProgram()

        if succes == True:
            mv.sendRemoteValue(None, None, None, None, vehicle)
            vehicle.mode = VehicleMode("LAND")
            print("landing mission succes......")
            sleep(5)
            print("Land")
            break
        

        img = np.zeros((550, 500, 3), np.uint8)
        drawGD(img)
        vals = getRealsenseInput()
        #print(vals[0], vals[1])
        x = np.round(vals[0]).astype("int")
        z = np.round(vals[1]).astype("int")
        rotate = np.round(vals[2]).astype("int")
        points.append(((x+500)/2, (z+1000)/2))
        drawPoints(img, points, x, z)
        cv2.imshow("Output", img)
        cv2.waitKey(1)

        cv2.setMouseCallback('Output', startMission, img)
        cv2.imshow("Output", img)
        cv2.waitKey(1)
        

def startMission(event, x, y, flags, frame):
    if event == cv2.EVENT_LBUTTONUP:
        missionMode = int(input('what mode is this?? --> 1(ABC) 2(ACB) 3(BAC) 4(CAB) 5(BCA) 6(CBA)'))
        if missionMode == 1 :
            print("start Mode 1....")
            print(".")
            print(".")
            sleep(0.5)
            mode1()
        if missionMode == 2 :
            print("start Mode 2....")
            print(".")
            print(".")
            sleep(0.5)
            mode2()
        if missionMode == 3 :
            arm_and_takeoff_nogps(1)
            vehicle.mode = VehicleMode("LOITER")
            #goto(0, -50, 100, "take off")
            setAltitude(2.5)
            print("hold")
            sleep(5)
            print("start Mode 3....")
            print("Bismillah")
            print(".")
            print(".")
            sleep(0.5)
            mode3()
        if missionMode == 4 :
            print("start Mode 4....")
            print("Bismillah")
            print(".")
            print(".")
            sleep(0.5)
            mode4()
        if missionMode == 5 :
            print("start Mode 5....")
            print("Bismillah")
            print(".")
            print(".")
            sleep(0.5)
            mode5()
        if missionMode == 6 :
            print("start Mode 6....")
            print("Bismillah")
            print(".")
            print(".")
            sleep(0.5)
            mode6()
        if missionMode == 7 :
            print("start Uji coba....")
            print("Bismillah")
            print(".")
            print(".")
            sleep(0.5)
            ujicoba()
        if missionMode == 8 :
            ujicoba()
        if missionMode ==  9:
            goto_area1()
        if missionMode ==  10:
            goto_area4()

            
        

#start = raw_input("READY to way point")
while True:
    start()
    print("Mission Complete!")
    print("Well done!")
    print("Alhamdulillah....!")
    break

    
    


"""
#yaw pixhawk
def set_yaw(aYaw):
    print("vehicle set yaw : ", aYaw)
    sleep(0.5)
    toleransi = 0.05
    while True:
        showMaps()

        if vehicle.channels['8']>1500 :
            mv.sendRemoteValue(None, None, None, None, vehicle)
            print("break")
            break
        
        current_yaw = vehicle.attitude.yaw
        if  current_yaw > aYaw + toleransi:
            print("Left Yaw",  current_yaw, "desire :", aYaw)
            mv.sendRemoteValue(None, None, None, yawLeft, vehicle)
        elif current_yaw < aYaw - toleransi:
            print("Right Yaw", current_yaw, "desire :", aYaw)
            mv.sendRemoteValue(None, None, None, yawRight, vehicle)
        else:
            print("reach target Yaw: ", current_yaw, "Desire: ", aYaw)
            mv.sendRemoteValue(None, None, None, None, vehicle)
            print("stabilizing.....")
            for i in range(75):
                showMaps()
            #sleep(3)
            break
"""
