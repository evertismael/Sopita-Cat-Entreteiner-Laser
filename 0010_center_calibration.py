from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import cv2, time, random, math
import libs.sopita as sopita
from libs import myservo as ms
import itertools

""" This scripts alows to move the laser left-right-up-down so that the beam
    can be located in the center of the image.
    Notice that we assume the camera is well positioned (manually)
"""


# init camera:
cam = cv2.VideoCapture(0)

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
    print(f'servo1, servo2 values:{servo1.value:.2f}  , {servo2.value:.2f}')
    
    # get key to move up down or sides:
    ms.manual_left_right(inkey,servo1,.05)
    ms.manual_up_down(inkey,servo2,.05)

    if inkey==ord('q'):
        break



cal_flg = 'yes'# input('Obtain Calibration [no]: yes/no')
while 'yes' in cal_flg:
    cal_path = sopita.create_folder_number()

    off_moves = itertools.product(list(range(-5,5)),list(range(-5,5)))
    off_moves = [(a/100.0, b/100.0) for (a,b) in list(off_moves)] 

    sopita.collect_calibration_dataset(off_moves, cal_path, servo1, servo2, cam)

    # ask again: 
    cal_flg = input('Obtain Calibration AGAIN [no]: yes/no')


cam.release()
cv2.destroyAllWindows()







