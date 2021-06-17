import serial

arduino = serial.Serial("/dev/ttyUSB0", baudrate=115200)
while True:
	print(arduino.readline())
