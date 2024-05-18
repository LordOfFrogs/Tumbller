import serial
from XboxController import XboxController
import time


COM_PORT = "COM9"
BAUD_RATE = 9600

controller = XboxController()

s = serial.Serial(COM_PORT, BAUD_RATE)
if not s.is_open:
    s.open()

while True:
    left_x, left_y = controller.getRightJoystick()
    
    left_x = int((left_x+1)/2 * 255)
    left_y = int((left_y+1)/2 * 255)
    
    left_x = min(max(left_x, 0), 255)
    left_y = min(max(left_y, 0), 255)
    
    serial_msg = f'D{left_y:03}, S{left_x:03}'
    print(serial_msg)
    s.write(bytes(serial_msg, 'ASCII'))
    time.sleep(0.05)