from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import cv2, time, random, math
import libs.sopita as sopita
from libs import myservo as ms
import itertools
import numpy as np

""" This scripts alows to move the laser left-right-up-down so that the beam
    can be located in the center of the image.
    Notice that we assume the camera is well positioned (manually)
"""


# init camera:
cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_BUFFERSIZE,1)
a=cam.get(cv2.CAP_PROP_BUFFERSIZE) 

# init servo:
factory = PiGPIOFactory()
servo1 = Servo(17, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
servo2 = Servo(18, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
servo1.mid()
servo2.mid()

# init laser:
laser = Servo(27, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
laser.max()

cal_path = './cal_laser'
cal_servos_file = cal_path+'/servos.txt'
idx = 0
while True:
    # read camera frame:  
    valid, frame = cam.read()
    cv2.imshow('Calibration',frame)

    inkey = cv2.waitKey(1)
    print(f'servo1, servo2 values:{servo1.value:.3f}  , {servo2.value:.3f}, angles:{ms.VAL2ANGLE*servo1.value:.2f}  , {ms.VAL2ANGLE*servo2.value:.2f}')
    
    # get key to move up down or sides:
    ms.manual_left_right(inkey,servo1, step=ms.ANGLE2VAL*0.5)
    ms.manual_up_down(inkey,servo2, step=ms.ANGLE2VAL*0.5)

    if inkey == ord('q'):
        break
    elif inkey == ord('c'):
        # save image:
        filename = cal_path+'/'+str(idx)+'.png'
        cv2.imwrite(filename, frame)

        ang1 = servo1.value*ms.VAL2ANGLE
        ang2 = servo2.value*ms.VAL2ANGLE
        
        with open(cal_servos_file, "a") as f:
            tof = f'{idx:d}\t{ang1:.3f}\t{ang2:.3f}\t{servo1.value:.3f}\t{servo2.value:.3f}\n'
            f.write(tof)
        print('-----')
        time.sleep(1)
        idx+=1

laser.value = -1
time.sleep(0.1)