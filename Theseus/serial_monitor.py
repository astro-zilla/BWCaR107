import serial

COM = 'COM9'  # /dev/ttyACM0 (Linux)
BAUD = 9600

ser = serial.Serial(COM, BAUD, timeout=.1)

print('Waiting for device')
print(ser.name)

ser.flushInput()

while True:
    try:
        ser_bytes = ser.readline()
        if ser_bytes != b'':
            print(str(ser_bytes[:-2], 'utf-8'))
    except Exception as e:
        print("Keyboard Interrupt")
        print(e)
        break
