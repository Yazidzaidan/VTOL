import serial

readData = serial.Serial('COM17', 9600)

while True:
    if (readData.in_waiting > 0):
        line = readData.readline()
        print (line)
