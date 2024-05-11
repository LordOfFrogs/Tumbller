import serial
import pygame
import time

COM_PORT = "com0"
BAUD_RATE = 9600

pygame.joystick.init()
if (pygame.joystick.get_count() > 0):
    raise RuntimeError("Joystick not detected")
controller = pygame.joystick.Joystick(0)

s = serial.Serial(COM_PORT, BAUD_RATE)
s.open()

while True:
    left_x = controller.get_axis(0)
    left_y = controller.get_axis(1)
    
    left_x = int((left_x+1)/2 * 255)
    left_y = int((left_y+1)/2 * 255)
    
    left_x = min(max(left_x, 0), 255)
    left_y = min(max(left_y, 0), 255)
    
    serial_msg = f'D{left_y}, S{left_x}'
    
    s.write(bytes(serial_msg))
    
    time.sleep(0.05)