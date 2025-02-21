import cv2
import multiprocessing
import numpy as np
from libs.cam_calib_utils import detect_chess_board_points

from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from libs import myservo as ms
import time

def camera_process(cal_path, running_flg, degrees_1, degrees_2, save_flg, move_idx):
    ptrn_size = ((6,4))
    cam = cv2.VideoCapture(0)  
    cam.set(cv2.CAP_PROP_BUFFERSIZE,1)
    axx=cam.get(cv2.CAP_PROP_BUFFERSIZE)       
    
    cal_servos_file = cal_path+'/servos.txt'
    cnt = 0
    idx = 0
    while running_flg.value==True:
        valid, frame = cam.read()
        
        if valid==True:
            # try to detect the chessboard and draw
            ret, P_chs, P_pxl = detect_chess_board_points(frame, ptrn_size)
            if ret==True:
                frame_to_save = frame.copy()
                cv2.drawChessboardCorners(frame,ptrn_size,P_pxl,ret)
                for p_idx, p in enumerate(P_pxl):
                    p = [int(a) for a in p]
                    cv2.putText(frame,str(p_idx),p,cv2.FONT_HERSHEY_COMPLEX,.4,(0,0,0),1,1)
            cv2.imshow('Calibration',frame)
            cv2.waitKey(1)
        
        # act on pressed key
        if save_flg.value == True:
            if ret==True and valid==True:
                # save image:
                filename = cal_path+'/'+str(move_idx.value)+'/'+str(idx)+'_chess.png'
                cv2.imwrite(filename, frame)

                filename = cal_path+'/'+str(move_idx.value)+'/'+str(idx)+'_img.png'
                cv2.imwrite(filename, frame_to_save)

                # write file:
                with open(cal_servos_file, "a") as f:
                    tof = f'{idx:d}\t{move_idx.value:d}\t{degrees_1.value:.3f}\t{degrees_2.value:.3f}\t{degrees_1.value*ms.ANGLE2VAL:.3f}\t{degrees_2.value*ms.ANGLE2VAL:.3f}\n'
                    f.write(tof)
                    print(tof)

                idx+=1
                print(f'from camera: images saved {idx}')
            else:
                print('Chess not in image, NOT SAVED')
            
            save_flg.value = False
        
        cnt += 1
        if cnt%30000 == 0:
            print(f'from camera {valid}')
    
    cam.release()
    cv2.destroyAllWindows()


def laser_process(cal_path, running_flg, degrees_1, degrees_2):
    # init servo:
    factory = PiGPIOFactory()
    servo1 = Servo(17, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
    servo2 = Servo(18, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
    servo1.mid()
    servo2.mid()

    # init laser:
    laser = Servo(27, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
    laser.max()


    prev_degrees_1 = 0.
    prev_degrees_2 = 0.
    
    while running_flg.value==True:
        
        if prev_degrees_1 !=degrees_1.value or prev_degrees_2 !=degrees_2.value:
            
            # get key to move up down or sides:
            degrees_1.value = ms.manual_move_servo(servo1, prev_degrees_1, degrees_1.value)
            degrees_2.value = ms.manual_move_servo(servo2, prev_degrees_2, degrees_2.value)
            
            prev_degrees_1 = degrees_1.value
            prev_degrees_2 = degrees_2.value
            print(f'from lsr proc: 1-2 values:{servo1.value:.3f}  , {servo2.value:.3f}, angles:{ms.VAL2ANGLE*servo1.value:.2f}  , {ms.VAL2ANGLE*servo2.value:.2f}')


if __name__=='__main__':

    cal_path = './cal_laser'

    # define variables to communicate the processes:
    running_flg = multiprocessing.Value('b')
    save_flg = multiprocessing.Value('b')
    degrees_1 = multiprocessing.Value('f')
    degrees_2 = multiprocessing.Value('f')
    move_idx = multiprocessing.Value('i')
    # init:
    running_flg.value = True
    save_flg.value = False
    degrees_1.value = 0
    degrees_2.value = 0
    move_idx.value=0

    # launch process,
    p_cam = multiprocessing.Process(target=camera_process,args=(cal_path,running_flg, degrees_1, degrees_2, save_flg, move_idx),name='cam_proc')
    p_cam.start()
    
    p_lsr = multiprocessing.Process(target=laser_process,args=(cal_path,running_flg, degrees_1, degrees_2),name='lsr_proc')
    p_lsr.start()
    
    
    # move laser to new positions
    moves = [(10,20),(0,20),(-10,20), (10,10),(0,10),(-10,10), (10,0), (0,0), (-10,0)]
    for i in range(10):
        
        for idx, mv_deg in enumerate(moves):
            
            move_idx.value = idx

            degrees_1.value = 0
            degrees_2.value = 0

            time.sleep(1)
            degrees_1.value = mv_deg[0]
            degrees_2.value = mv_deg[1]
            time.sleep(1)
            save_flg.value = True
            time.sleep(1)

    running_flg.value = False

    # join all process:
    p_cam.join()
    p_lsr.join()
    
    
    