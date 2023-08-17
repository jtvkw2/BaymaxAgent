import serial
arduino = serial.Serial('/dev/cu.usbserial-AL01EJ6E', 9600)

while True:
    command = str(input("Servo position: "))
    if command == '-1':
        break
    arduino.write(str.encode(command))
    reachedPos = str(arduino.readline().decode())
    print(reachedPos)