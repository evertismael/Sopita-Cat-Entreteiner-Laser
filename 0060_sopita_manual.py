from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import cv2, time, random, math
import libs.sopita as sopita
from libs import myservo as ms
import numpy as np
# my libs
import libs.cam_calib_utils as ccu
import libs.laser_calib_utils as lcu
import libs.chess_utils as chu


""" 
    This script manipulates the pixel coordinates (x,y) and draws that into the image.
    Based on this (x,y) computes the appropriate angles for the servo motots such that
    the laser points to that point.
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


# load camera and laser R,T matrices:
R_cam, T_cam, A_cam, Ainv_cam = ccu.load_RTA_camera_matrices('./out/camera')
R_las, T_las = lcu.load_RT_laser_matrices('./out/laser')


Ppxl = np.array([30,30],dtype=int)
while True:
    # read camera frame:  
    valid, frame = cam.read()
    cv2.circle(frame,Ppxl,10,(255,0,0),1,1)
    cv2.imshow('Sopita Manual',frame)
    inkey = cv2.waitKey(1)


    # update values of the pixel target:
    Ppxl = sopita.manual_left_right(inkey,Ppxl,frame.shape[1],0)
    Ppxl = sopita.manual_up_down(inkey,Ppxl,frame.shape[0],0)
    
    print(f'target position in pixel coords: {Ppxl}')
    if inkey == ord('q'):
        break
    elif inkey == ord('m'):
        print('Moving laser:')
        # 1. find the world coords of the pixel point.
        # 2. find the directional angles.
        # 3. activate the servos:

        # 1.
        Ppxl_ = Ppxl.reshape(2,1)
        P_w = ccu.uv2XYZ(Ppxl_,Ainv_cam,R_cam,T_cam.T)
        # 2.
        P_l = R_las.dot(P_w) + T_las
        U_l = P_l/np.sqrt(np.sum(P_l**2,axis=0,keepdims=True))

        phi = np.arccos(U_l[2,0])
        theta = np.arctan2(U_l[1,0],U_l[0,0])
        
        alpha = np.rad2deg(theta) - 90
        beta = 90 - np.rad2deg(phi)

        value1 = ms.ANGLE2VAL*alpha
        value2 = ms.ANGLE2VAL*beta

        servo1.value = ms.sanitate_max_values(ms.ANGLE2VAL*alpha)
        servo2.value = ms.sanitate_max_values(ms.ANGLE2VAL*beta)

       # time.sleep(1)
        print(f'servo1, servo2 values:{servo1.value:.3f}  , {servo2.value:.3f}, angles:{ms.VAL2ANGLE*servo1.value:.2f}  , {ms.VAL2ANGLE*servo2.value:.2f}')
      #  time.sleep(5)

        