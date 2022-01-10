#Maju, deteksi gedung,landing
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import serial
import cv2
import pyzbar.pyzbar as pyzbar

cap = cv2.VideoCapture(0)
detected = False

#Declare tfmini
ser = serial.Serial("/dev/ttyTHS1", 115200)

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect',
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
connection_string = '/dev/ttyACM0'
delay = 2

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
def rescale_frame(frame, percent):
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)

def getTFminiData():
    start = input("READY MAJU")
    print("Maju")
    print("Maju")
    print("Maju")
    print("Maju")
    detected= False
    while True:
        #time.sleep(0.1)
        count = ser.in_waiting
        _, frame = cap.read()
        frame40 = rescale_frame(frame, percent=40)
        decode_QR = pyzbar.decode(frame40)
        for qrcode in decode_QR:
            (x,y,w,h) = qrcode.rect
            cv2.rectangle(frame40, (x,y),(x + w, y +w),(0,0,255), 2)
            cv2.putText(frame40, str(qrcode.data), (100, 100), cv2.FONT_HERSHEY_PLAIN,2,(255,0,0), 3)
        cv2.imshow("QR Code Scanner",frame40)
        if cv2.waitKey(1) & 0xFF == ord("q") :
            break 
        if count > 8:
            vehicle.channels.overrides['2'] = 1350
            vehicle.channels.overrides['3'] = 1520
            print("Maju")
            recv = ser.read(9)  
            ser.reset_input_buffer()  
            # type(recv), 'str' in python2(recv[0] = 'Y'), 'bytes' in python3(recv[0] = 89)
            # type(recv[0]), 'str' in python2, 'int' in python3 
            
            if recv[0] == 0x59 and recv[1] == 0x59:     #python3
                distance = recv[2] + recv[3] * 256
                strength = recv[4] + recv[5] * 256
                print('(', distance, ',', strength, ')')
                ser.reset_input_buffer()
            if recv[0] == 'Y' and recv[1] == 'Y':     #python2
                lowD = int(recv[2].encode('hex'), 16)      
                highD = int(recv[3].encode('hex'), 16)
                lowS = int(recv[4].encode('hex'), 16)      
                highS = int(recv[5].encode('hex'), 16)
                distance = lowD + highD * 256
                strength = lowS + highS * 256
                print(distance, strength)

            if distance < 70 :
                vehicle.channels.overrides['2'] = None
                vehicle.channels.overrides['3'] = None
                print("Stabilizing drone")
                
                time.sleep(5)
                #detected= False
                print("Mau mencari QR")
                while True:
                    #for qrcode in decode_QR:
                    #    detected= True
                    #    print ("detected di cam", detected)
                    for x in range(40): 
                        vehicle.channels.overrides['1'] = 1550
                        vehicle.channels.overrides['3'] = 1520
                        print(x, "gerak kanan", "detected di cam", detected)
                        _, frame = cap.read()
                        frame40 = rescale_frame(frame, percent=40)
                        decode_QR = pyzbar.decode(frame40)
                        for qrcode in decode_QR:
                            (x,y,w,h) = qrcode.rect
                            cv2.rectangle(frame40, (x,y),(x + w, y +w),(0,0,255), 2)
                            cv2.putText(frame40, str(qrcode.data), (100, 100), cv2.FONT_HERSHEY_PLAIN,2,(255,0,0), 3)
                            detected = True
                        if detected == True:
                            break
                        cv2.imshow("QR Code Scanner",frame40)
                        if cv2.waitKey(1) & 0xFF == ord("q") :
                            break 
                    for x in range(40): 
                        vehicle.channels.overrides['1'] = 1420
                        vehicle.channels.overrides['3'] = 1520
                        print(x, "gerak kiri", "detected di cam", detected)
                        _, frame = cap.read()
                        frame40 = rescale_frame(frame, percent=40)
                        decode_QR = pyzbar.decode(frame40)
                        for qrcode in decode_QR:
                            (x,y,w,h) = qrcode.rect
                            cv2.rectangle(frame40, (x,y),(x + w, y +w),(0,0,255), 2)
                            cv2.putText(frame40, str(qrcode.data), (100, 100), cv2.FONT_HERSHEY_PLAIN,2,(255,0,0), 3)
                            detected = True
                        cv2.imshow("QR Code Scanner",frame40)
                        if detected == True:
                            break
                        if cv2.waitKey(1) & 0xFF == ord("q") :
                            break 
                    if detected == True:
                        vehicle.channels.overrides['1'] = None
                        vehicle.channels.overrides['2'] = None
                        vehicle.channels.overrides['3'] = None
                        print("Segera lakukan drop")
                        #time.sleep(5)
                        vehicle.channels.overrides['6'] = 2000
                        time.sleep(0.5)
                        vehicle.channels.overrides['6'] = 990
                        break
                if detected == True:
                    print("Maju Selesai")
                    break;
                #vehicle.channels.overrides['2'] = 1350
                #print("Maju mau drop")

            
    cap.release()
    cv2.destroyAllWindows()
            # you can also distinguish python2 and python3: 
            #import sys
            #sys.version[0] == '2'    #True, python2
            #sys.version[0] == '3'    #True, python3
            #vehicle.channels.overrides['2'] = None
            #vehicle.channels.overrides['3'] = None
    
###########################################################
def maju():
    start = input("READY MAJU")
    print("Motion maju")
    print("Motion maju")
    getTFminiData()
    vehicle.channels.overrides['2'] = 1350
    vehicle.channels.overrides['3'] = 1520
    time.sleep(3)
    vehicle.channels.overrides['2'] = None
    vehicle.channels.overrides['6'] = 2000
    print("Maju selesai")

def mundurbroo():
    #start = input("READY Mundur")
    print("Motion mundur")
    print("Motion mundur")
    vehicle.channels.overrides['6'] = 990
    vehicle.channels.overrides['2'] = 1650
    vehicle.channels.overrides['3'] = 1520
    time.sleep(8)
    vehicle.channels.overrides['2'] = None
    print("Mundur selesai")
def kanan():
    start = input("READY Kanan")
    print("Motion kanan")
    print("Motion kanan")
    vehicle.channels.overrides['1'] = 1650
    vehicle.channels.overrides['3'] = 1520
    time.sleep(2)
    vehicle.channels.overrides['1'] = None
    print("Kanan selesai")
def kiri():
    start = input("READY Kiri")
    print("Motion kiri")
    print("Motion kiri")
    vehicle.channels.overrides['1'] = 1310
    vehicle.channels.overrides['3'] = 1520
    time.sleep(8
    )
    vehicle.channels.overrides['1'] = None
    print("Kiri selesai")

getTFminiData()
mundurbroo()
#arm_and_takeoff_nogps(2)
#print("Take off complete")
#print(vehicle.mode)
#vehicle.mode = VehicleMode("LOITER")
#time.sleep(1)
#print(vehicle.mode)
#maju()
#mundurbroo()
#kanan()
#maju()
#mundurbroo()
#kiri()
# maju()
#mundurbroo()
#kanan()

print("Now let's land")
time.sleep(1)
vehicle.mode = VehicleMode("LAND")
time.sleep(5)
print("Land")
# Close vehicle object
#vehicle.close()