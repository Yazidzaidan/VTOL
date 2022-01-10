import serial
<<<<<<< HEAD
import numpy as np 
ser = serial.Serial('/dev/ttyACM1', baudrate = 9600, timeout=1)
=======

ser = serial.Serial('COM17', baudrate = 9600, timeout=1)
from time import sleep
a =5
>>>>>>> 6a17770f6687036201324496b0469c1d762b675b
 
while True:
    arduinoData = ser.readline().strip()
    #length = int(arduinoData.strip('\0'))  
    #print(arduinoData, type(arduinoData))
    d = float(arduinoData)
    print(arduinoData)
    print(d)
    
    #string = arduinoData.decode()
    #stripped_string = string.strip()
    #num_int = int(stripped_string)
    #print(type(num_int))

"""
wallRight = 20
def side(gd):
    print("vehicle goto forward :", gd)
    #sleep(0.5)
    toleransi = 5
    while True:
        line = ser.readline()   # read a byte
<<<<<<< HEAD
        if line:
            string = line.decode().strip()# convert the byte string to a unicode string
            num = int(float(string)) # convert the unicode string to an int
=======
        if line is not ValueError:
            string = line.decode().strip() # convert the byte string to a unicode string
            #num = float(string) # convert the unicode string to an int
            d = int(string)
            #sleep(0.1)
>>>>>>> 6a17770f6687036201324496b0469c1d762b675b
            #print(type(num))
            #print(num)
            try:
                raise ValueError
            except ValueError:
                print("")

            if d > gd + toleransi:
                print("forward",  d, "desire :", gd)
            elif d <= gd - toleransi:
                print("backward",  d, "desire :", gd)
            else :
                print("ready drop....")
                #break
<<<<<<< HEAD

=======
        else:
>>>>>>> 6a17770f6687036201324496b0469c1d762b675b

side(wallRight)
"""
