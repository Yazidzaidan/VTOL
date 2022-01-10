# -*- coding: utf-8 -*
import serial
import time

ser = serial.Serial("/dev/ttyTHS1", 115200)


while True:
    #time.sleep(0.1)
    count = ser.in_waiting
    if count > 8:
        recv = ser.read(9)  
        ser.reset_input_buffer()  
        # type(recv), 'str' in python2(recv[0] = 'Y'), 'bytes' in python3(recv[0] = 89)
        # type(recv[0]), 'str' in python2, 'int' in python3 
            
        
                
        if recv[0] == 'Y' and recv[1] == 'Y':     #python2
            lowD = int(recv[2].encode('hex'), 16)      
            highD = int(recv[3].encode('hex'), 16)
            distance = lowD + highD * 256
            print(distance)
        #if distance < 300:
        #    break
            
        # you can also distinguish python2 and python3: 
        #import sys
        #sys.version[0] == '2'    #True, python2
        #sys.version[0] == '3'    #True, python3

#while True:
#    getTFminiData()
print ("Selesai")

#if __name__ == '__main__':
#    try:
#        if ser.is_open == False:
#            ser.open()
#        getTFminiData()
#    except KeyboardInterrupt:   # Ctrl+C
#        if ser != None:
#            ser.close()
