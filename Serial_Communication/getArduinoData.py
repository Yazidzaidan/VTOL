import serial

ser = serial.Serial('COM13', baudrate = 115200, timeout=1)

while(1):
    #ser.write(char)
    arduinoData = ser.readline().decode('ascii')
    if arduinoData <= u'100\r\n':
        print("mundur", arduinoData)
    elif arduinoData >= u'110\r\n':
        print("maju", arduinoData)
    else:
        print("Reach", arduinoData)
 
