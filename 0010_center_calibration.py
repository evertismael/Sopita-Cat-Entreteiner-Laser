from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import cv2, time, random, math
import libs.sopita as sopita
from libs import myservo as ms

""" This scripts alows to move the laser left-right-up-down so that the beam
    can be located in the center of the image.
    Notice that we assume the camera is well positioned (manually)
"""


# init camera:
cam = cv2.VideoCapture(0)

# init servo:
factory = PiGPIOFactory()
servo = Servo(17, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
servo.mid()


while True:
    # read camera frame:  
    valid, frame = cam.read()
    cv2.imshow('Calibration',frame)

    inkey = cv2.waitKey(1)
    print(f'servo value:{servo.value:.2f}')
    
    # get key to move up down or sides:
    ms.manual_left_right(inkey,servo,.1)

    if inkey==ord('q'):
        break



cal_flg = input('Obtain Calibration [no]: yes/no')
while 'yes' in cal_flg:
    cal_path = sopita.create_folder_number()
    sopita.collect_calibration_dataset(cal_path, servo, cam)

    # ask again: 
    cal_flg = input('Obtain Calibration AGAIN [no]: yes/no')


cam.release()
cv2.destroyAllWindows()







