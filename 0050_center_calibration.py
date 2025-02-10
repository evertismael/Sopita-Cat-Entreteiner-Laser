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


while True:
    # read camera frame:  
    valid, frame = cam.read()
    cv2.imshow('Calibration',frame)

    inkey = cv2.waitKey(1)
    print(f'servo1, servo2 values:{servo1.value:.2f}  , {servo2.value:.2f}, angles:{ms.VAL2ANGLE*servo1.value:.2f}  , {ms.VAL2ANGLE*servo2.value:.2f}')
    
    # get key to move up down or sides:
    ms.manual_left_right(inkey,servo1, step=ms.ANGLE2VAL)
    ms.manual_up_down(inkey,servo2, step=ms.ANGLE2VAL)

    if inkey==ord('q'):
        break

"""
print(servo1.value, servo2.value)
servo1.value=-0.1
servo2.value=-0.1
"""

cal_flg = 'yes'# input('Obtain Calibration [no]: yes/no')
while 'yes' in cal_flg:
    cal_path = sopita.create_folder_number()

    # define calibration path:
    off_moves_ang = sopita.get_calibration_path_serpent(min_angle=-10, max_angle=10,step=3)
    print(f'taking {len(off_moves_ang)} calibration moves', off_moves_ang)
    sopita.collect_calibration_dataset(off_moves_ang, cal_path, servo1, servo2, cam)

    # ask again: 
    cal_flg = input('Obtain Calibration AGAIN [no]: yes/no')


cam.release()
cv2.destroyAllWindows()







