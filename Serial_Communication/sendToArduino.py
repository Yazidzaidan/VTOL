import serial

ser = serial.Serial('COM13', baudrate = 115200, timeout=1)
detectWall = False
backward = True

def getValuesTFMini(char):
    global detectWall, backward
    ser.write(char)
    if (ser.in_waiting > 0):
        arduinoData = ser.readline().decode('ascii').strip()
        if arduinoData == u'1':
            detectWall = True
        else:
            detectWall = False
        return arduinoData
 
def straight(userInput):
    #userInput = raw_input('Get data point?')\
    global detectWall
    while True:
        if detectWall == True:
            print("reach distance ")
            break
        if backward == False:
            print("reach Backward ")
            break
        if userInput == '0': # GD B
            val = getValuesTFMini(b'0')
            print("goto GD B", val, "detectGD_B:", detectWall)
        elif userInput == '1': # GD AC
            val = getValuesTFMini(b'1')
            print("goto GD A & C", val, "detectGD_A_C:", detectWall)
        elif userInput == '2': # GD AC
            val = getValuesTFMini(b'2')
            print("goto Buildings", val, "detectWall:", detectWall)

#while True:
straight('1')
    