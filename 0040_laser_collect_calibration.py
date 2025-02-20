from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import cv2, time, random, math
import libs.sopita as sopita
from libs import myservo as ms
import itertools
import numpy as np
from libs.cam_calib_utils import detect_chess_board_points

""" This scripts alows to move the laser left-right-up-down so that the beam
    can be located in the center of the image.
    Notice that we assume the camera is well positioned (manually)
"""


# init camera:
cam = cv2.VideoCapture(0)

try:
    cam.set(cv2.CAP_PROP_BUFFERSIZE,1)
    a=cam.get(cv2.CAP_PROP_BUFFERSIZE) 

    # init servo:
    factory = PiGPIOFactory()
    servo1 = Servo(17, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
    servo2 = Servo(18, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
    servo1.mid()
    servo2.mid()
    val1_th = servo1.value
    val2_th = servo2.value
    step_angle = 1


    # init laser:
    laser = Servo(27, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
    laser.max()

    cal_path = './cal_laser'
    cal_servos_file = cal_path+'/servos.txt'
    idx = 0
    while True:
        # read camera frame:  
        valid, frame = cam.read()
        frame_to_save = frame.copy()

        # try to detect the chessboard
        ptrn_size = ((6,4))
        ret, P_chs, P_pxl = detect_chess_board_points(frame, ptrn_size)
        
        # if detected draw:
        if ret==True:
            cv2.drawChessboardCorners(frame,ptrn_size,P_pxl,ret)
            for p_idx, p in enumerate(P_pxl):
                p = [int(a) for a in p]
                cv2.putText(frame,str(p_idx),p,cv2.FONT_HERSHEY_COMPLEX,.4,(0,0,0),1,1)

        cv2.imshow('Calibration',frame)

        inkey = cv2.waitKey(1)
        print(f'servo1, servo2 values:{servo1.value:.3f}  , {servo2.value:.3f}, angles:{ms.VAL2ANGLE*servo1.value:.2f}  , {ms.VAL2ANGLE*servo2.value:.2f}')
        
        # get key to move up down or sides:
        val1_th = ms.manual_left_right_angle(inkey,servo1,prev_val = val1_th, step_angle=step_angle)
        val2_th = ms.manual_up_down_angle(inkey,servo2,prev_val = val2_th, step_angle=step_angle)

        if inkey == ord('q'):
            break
        elif inkey == ord('c'):
            if ret==True:
                # save image:
                filename = cal_path+'/'+str(idx)+'_chess.png'
                cv2.imwrite(filename, frame)

                filename = cal_path+'/'+str(idx)+'_img.png'
                cv2.imwrite(filename, frame_to_save)
                

                ang1 = servo1.value*ms.VAL2ANGLE
                ang2 = servo2.value*ms.VAL2ANGLE
                
                with open(cal_servos_file, "a") as f:
                    tof = f'{idx:d}\t{ang1:.3f}\t{ang2:.3f}\t{servo1.value:.3f}\t{servo2.value:.3f}\n'
                    f.write(tof)
                print(f'Image and Servo-values saved {idx}')
                time.sleep(1)
                idx+=1
            else:
                print('Chess not in image, NOT SAVED')

    laser.value = -1
    time.sleep(0.1)
except:
    cam.release()
    print('error')
else:
    cam.release()