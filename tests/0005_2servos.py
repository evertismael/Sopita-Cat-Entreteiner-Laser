from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep
import cv2

# configure servos:
factory = PiGPIOFactory()
srv_1 = Servo(17, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
srv_2 = Servo(18, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)

srv_1.mid()
srv_2.mid()
sleep(1)


def update_servo_value(key):
    angle_step = 1.0/90.0
    if key == 81:#left 
        srv_1.value+=angle_step
    if key == 83:#right 
        srv_1.value-=angle_step
    
    if key == 82:#up  
        srv_2.value+=angle_step
    if key == 84:#down 
        srv_2.value-=angle_step



frame = cv2.imread('./libs/cat.jpg')
while True:
    cv2.imshow('sopita',frame)
    kp = cv2.waitKey(1)
    update_servo_value(kp)
    print(f'you pressed:{kp} ')
    
    if kp==ord('q'):
        break

    





