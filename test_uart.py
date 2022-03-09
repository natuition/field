import serial
ser = serial.Serial('/dev/ttyTHS1', baudrate=115200)  # open serial port
try:
    while True:
        print(ser.readline())     
except KeyboardInterrupt:
    print("Close...")
    ser.close() 