from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import Odroid.GPIO as GPIO
import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
from imutils.video import VideoStream
import imutils

#CAMERA
cap = cv2.VideoCapture(0)
font = cv2.FONT_HERSHEY_PLAIN
detected = False

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect',
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
connection_string = '/dev/ttyACM0'
sitl = None
GPIO.setmode(GPIO.BOARD)
GPIO.setup(8, GPIO.IN)
GPIO.setup(10, GPIO.IN)
# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()
# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, baud=115200, wait_ready=True)

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
        time.sleep(1)

    print("Taking off!")

    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(vehicle.mode)
        print(" Altitude: %f  Desired: %f" %
              (current_altitude, aTargetAltitude))
        if vehicle.channels['8']<1500 :
            print("break")
            break;
        if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
            print("Reached target altitude")
            print(" Altitude: %f  Desired: %f" %
              (current_altitude, aTargetAltitude))
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust = thrust)
        time.sleep(0.2)

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
        time.sleep(0.1)
        if vehicle.channels['8']<1500 :
            print("break")
            break;
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

#gerak 1
def maju():
    print("Motion maju")
    print("Motion maju")
    print("Prox 1 stand by")
    print(vehicle.mode)
    while GPIO.input(8) == 1 :
        if vehicle.channels['8']<1500 :
            print("break")
            break;
        print("Maju")
        cam()
        vehicle.channels.overrides['2'] = 1350
        key = cv2.waitKey(1)
        if key == 27:
            break
    servo()
    #vehicle.channels.overrides['2'] = None
    print("mau mencari QR")
    #time.sleep(1)

def cariQR():
    print("Mencari QR")
    print("Mencari QR")
    print("Prox 2 stand by")
    while GPIO.input(10) == 1 :
        if vehicle.channels['8']<1500 :
            print("break")
            break;
        print("Menuju QR")
        cam()
        vehicle.channels.overrides['2'] = 1450
        key = cv2.waitKey(1)
        if key == 27:
            break
    vehicle.channels.overrides['2'] = None
    print("QR terdeteksi")
    print("QR terdeteksi")
    #time.sleep(1)
    servo()   
        
def mundurbroo():
    #start = raw_input("READY Mundur")
    for x in range(300):
        print(x, "mundur")
        if vehicle.channels['8']<1500 :
            print("break")
            break;
        vehicle.channels.overrides['2'] = 1700
        vehicle.channels.overrides['3'] = 1800
    vehicle.channels.overrides['2'] = None
    vehicle.channels.overrides['6'] = 990

def servo():
    if vehicle.channels['8']>1500 :
        print("jatuhin log")
        vehicle.channels.overrides['6'] = 2000
        #time.sleep(1)
        print("servo balik")
        #time.sleep(1)
        
#gerak 1
def kiri():
    #start = raw_input("READY Kiri")
    print("gerak 3")
    #time.sleep(1)
    for x in range(55):
        vehicle.channels.overrides['1'] = 1310
        vehicle.channels.overrides['3'] = 1700
        if vehicle.channels['8']<1500 :
            print("break")
            break;
        print(x, "gerak kiri")
        _, frame = cap.read()
        detected= False
        print ("detected di cam", detected)
        decodedObject = pyzbar.decode(frame)
        for  obj in decodedObject:
            print ("Data", obj.data)
            print ("Terdeteksi")
            detected= True
            print ("detected di cam", detected)
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1)
        if detected == True:
            print("break")
            break;
        #vehicle.channels.overrides['3'] = 1800
    #print("sampai target 2")
    vehicle.channels.overrides['1'] = None  
    #time.sleep(1)
    print("aman")

#gerak 2
def kananAtas():
    #start = raw_input("READY Kanan")
    print("gerak 3")
    #time.sleep(1)
    for x in range(110): 
        vehicle.channels.overrides['1'] = 1700
        vehicle.channels.overrides['3'] = 2000
        print(x, "gerak kanan")
        if vehicle.channels['8']<1500 :
            print("break")
            break;
        _, frame = cap.read()
        detected= False
        print ("detected di cam", detected)
        decodedObject = pyzbar.decode(frame)
        for  obj in decodedObject:
            print ("Data", obj.data)
            detected= True
            print ("detected di cam", detected)
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1)
        if detected == True:
            print("break")
            break;
    vehicle.channels.overrides['1'] = None

#gerak 3
def kiriBawah():
    #start = raw_input("READY Kanan")
    print("gerak 2")
    #time.sleep(1)
    for x in range(55): 
        vehicle.channels.overrides['1'] = 1300
        vehicle.channels.overrides['3'] = 1500
        print(x, "gerak kiri")
        if vehicle.channels['8']<1500 :
            print("break")
            break;
        _, frame = cap.read()
        detected= False
        print ("detected di cam", detected)
        decodedObject = pyzbar.decode(frame)
        for  obj in decodedObject:
            print ("Data", obj.data)
            detected= True
            print ("detected di cam", detected)
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1)
        if detected == True:
            print("break")
            break;
    vehicle.channels.overrides['1'] = None

#gerak 4
def balikTengah():
    #start = raw_input("READY Balik")
    for x in range(900):
        print(x, "mundur tengah")
        if vehicle.channels['8']<1500 :
            print("break")
            break;
        vehicle.channels.overrides['2'] = 1700
        #vehicle.channels.overrides['3'] = 2010
    vehicle.channels.overrides['2'] = None
    print("sampai target mau balik")
    time.sleep(2)
    print("aman")
    
def cam():
    _, frame = cap.read()
    frame = imutils.resize(frame, width=400)
    decodedObject = pyzbar.decode(frame)
    for obj in decodedObject:
        cv2.putText(frame, str(obj.data), (50,50), font, 2, (255, 0, 0), 3)
        (x, y, w, h) = obj.rect
        a = x + w/2
        cv2.putText(frame, str(obj.data), (50,50), font, 2, (255, 0, 0), 3)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(frame, (a, y + h/2), 7, (0,0,255), -1)
        #print (a)
        if a>245:
            print("kanan")
            vehicle.channels.overrides['1'] = 1570
        elif a<225:
            print("kiri")
            vehicle.channels.overrides['1'] = 1430
        elif a>225 and a<245:
            print("break")
            vehicle.channels.overrides['1'] = None
            #break
        #print ("rect", obj.rect) 
        #print ("polygon", obj.polygon) #data posisi
        #print (decodedObject)
    cv2.imshow("Frame", frame) 

#Pemanggilan program
print("gerak 1")
print("Bismillah Maju")
start = raw_input("READY Motion")
print ("stand by")
#majunaik()

while vehicle.channels['8']>1500 :
    arm_and_takeoff_nogps(2.5)
    vehicle.mode = VehicleMode("LOITER")
    time.sleep(1)
    print(vehicle.mode)
    kiri()
    if vehicle.channels['8']<1500 :
        print("break")
        break;
    maju()
    if vehicle.channels['8']<1500 :
        print("break")
        break;
    mundurbroo()
    if vehicle.channels['8']<1500 :
        print("break")
        break;
    print("DELAYYY")
    time.sleep(3)
    kananAtas()
    if vehicle.channels['8']<1500 :
        print("break")
        break;
    maju()
    if vehicle.channels['8']<1500 :
        print("break")
        break;
    mundurbroo()
    if vehicle.channels['8']<1500 :
        print("break")
        break;
    print("DELAYYY")
    time.sleep(3)
    kiriBawah()
    if vehicle.channels['8']<1500 :
        print("break")
        break;
    maju()
    if vehicle.channels['8']<1500 :
        print("break")
        break;
    mundurbroo()
    if vehicle.channels['8']<1500 :
        print("break")
        break;
    print("DELAYYY")
    time.sleep(3)
    balikTengah()
    if vehicle.channels['8']<1500 :
        print("break")
        break;
    print("Setting LAND mode...")
    vehicle.mode = VehicleMode("LAND")
    time.sleep(5)
    break
print("selesai")
print("selesai")
print("selesai")
print("selesai")
print("selesai")
print("selesai")
print("selesai")
