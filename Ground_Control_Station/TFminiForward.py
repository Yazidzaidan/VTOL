# -*- coding: utf-8 -*
import serial
import time
distance = 0

ser = serial.Serial("/dev/ttyTHS1", 115200)


while True:
    #time.sleep(0.1)
    count = ser.in_waiting
    if count > 8:
        recv = ser.read(9)  
                
        if recv[0] == 'Y' and recv[1] == 'Y':     #python2
            lowD = int(recv[2].encode('hex'), 16)      
            highD = int(recv[3].encode('hex'), 16)
            distance = lowD + highD * 256
            print(distance)

    
    

#print ("Selesai")

#if __name__ == '__main__':
#    try:
#        if ser.is_open == False:
#            ser.open()
#        getTFminiData()
#    except KeyboardInterrupt:   # Ctrl+C
#        if ser != None:
#            ser.close()
