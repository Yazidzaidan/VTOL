from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math

# Connect to the Vehicle
print ("Connecting")
connection_string = '/dev/ttyACM0'
vehicle = connect(connection_string, wait_ready=True)

while True:
# Display basic vehicle state
#print (" Type: %s" % vehicle._vehicle_type)
#print (" Armed: %s" % vehicle.armed)
#print (" System status: %s" % vehicle.system_status.state)
#print (" GPS: %s" % vehicle.gps_0)
#print (" Alt: %s" % vehicle.location.global_relative_frame.alt)
<<<<<<< HEAD
    print (" Alt1: ", vehicle.rangefinder.distance)
#print (" Alt1: ", vehicle.rangefinder2.distance)
=======
print (" Alt1: ", vehicle.rangefinder.distance)
print (" Alt1: ", vehicle.ra
>>>>>>> 6a17770f6687036201324496b0469c1d762b675b
#print(" Rangefinder: %s" % vehicle.rangefinder)




    