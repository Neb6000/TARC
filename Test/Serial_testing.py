import serial
import time
arduino = serial.Serial(port='COM4', baudrate=115200, timeout=.1)
def write_read(x):
    arduino.write(bytes(x, 'utf-8'))
    #while arduino.in_waiting == 0:
    #    pass
    #data = arduino.readline()
    #return data
#while True:
#    num = input("Enter a number: ") # Taking input from user
#    value = write_read(num)
#    print(value) # printing the value
data = None

y = 10.5
#write_read(str(1))
while True:
    if arduino.in_waiting > 0:
        data = arduino.readline()
        try:
            print(float(data))
        except:
            pass
    #arduino.write(bytes(x, 'utf-8'))
    write_read(str(y))    
    time.sleep(0.01)