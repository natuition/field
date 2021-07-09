import time
import serial

print("***** Starting reset *****")

ser = serial.Serial(
 port='/dev/ttyTHS1',
 baudrate = 19200,
 parity=serial.PARITY_NONE,
 stopbits=serial.STOPBITS_ONE,
 bytesize=serial.EIGHTBITS,
 timeout=1
)

ser.write(bytearray.fromhex("B5 62 06 04 04 00 FF B9 00 00 C6 8B"))
time.sleep(0.5)

print("***** Reset are send ! *****")