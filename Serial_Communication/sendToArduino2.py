#backward
import serial

ser = serial.Serial('COM13', baudrate = 115200, timeout=1)
backward = True

def getValuesTFMini(char):
    global backward
    ser.write(char)
    
    arduinoData = ser.readline().decode('ascii').strip()
    if arduinoData == u'':
        backward = True
    elif arduinoData == u'1':
        backward = True
    else:
        backward = False
    return arduinoData
 
def straight(userInput):
    #userInput = raw_input('Get data point?')
    global backward
    backward = True
    while True:
        if userInput == '3': # GD AC
            val = getValuesTFMini(b'3')
            print("backward from Buildings", val, "backward:", backward)
        if backward == False:
            print("reach Backward ")
            break
        

#while True:
straight('3')
    