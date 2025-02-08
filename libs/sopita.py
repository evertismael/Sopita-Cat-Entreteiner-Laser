import os
import time
import cv2
from libs.myservo import sanitate_max_values, ANGLE2VAL, VAL2ANGLE

def get_calibration_path_serpent(min_angle:-5, max_angle:5,step=1):
    assert(abs(min_angle)>=step,'step too big')
    assert(abs(max_angle)>=step,'step too big')
    assert(min_angle<max_angle,'min max values are wrong')

    angle_grid = range(min_angle, max_angle,step)
    tmp = list(angle_grid).copy()
    mvs = []
    for i,v in enumerate(angle_grid):
        tmp = tmp[::-1] 
        new_mvs = [(a,v) for a in tmp]
        mvs +=new_mvs
    return mvs

def create_folder_number(path:str='./calibration/'):
    if path is None:
        raise('path not provided')
    
    current_trial = 1
    prev_folders = [d for d in os.listdir(path) if os.path.isdir(path+'/'+d)]
    if len(prev_folders)>0:
        prev_folders.sort(reverse=True)
        current_trial = int(prev_folders[0])+1
    os.makedirs(path +'/'+ str(current_trial))

    return path +'/'+ str(current_trial)


def collect_calibration_dataset(off_moves_ang, cal_path, servo1, servo2, cam):
    ref_ang1 = servo1.value*VAL2ANGLE
    ref_ang2 = servo2.value*VAL2ANGLE
    
    for idx, mv_ang_off in enumerate(off_moves_ang):
        mv_ang = (ref_ang1 + mv_ang_off[0], ref_ang2 + mv_ang_off[1])
        mv_val = (ANGLE2VAL*mv_ang[0], ANGLE2VAL*mv_ang[1])
        # move servo
        servo1.value = sanitate_max_values(mv_val[0])
        servo2.value = sanitate_max_values(mv_val[1])
        print(f'Collecting {idx} of {len(off_moves_ang)}')
        print(f'move{idx} angles: s1: {mv_ang[0]:.2f} , s2: {mv_ang[1]:.2f}; values: s1: {mv_val[0]:.2f} , s2: {mv_val[1]:.2f}')
        print(f'getting ready to shoot')


        # read camera frame:
        for i in range(20):
            valid, frame = cam.read()
            if not valid:
                print('invalid frame')
                continue
            cv2.imshow('Calibration',frame)
            cv2.waitKey(1)
        
        print(f'showing captured image{idx}')
        #save image
        filename = cal_path+'/'+str(idx)+'.png'
        cv2.imwrite(filename, frame)
        # save file 
        filename = cal_path+'/servos.txt'
        with open(filename, "a") as f:
            tof = f'{idx:d}\t{ref_ang1:.3f}\t{ref_ang2:.3f}\t{mv_val[0]:.3f}\t{mv_val[1]:.3f}\t{mv_ang[0]:.3f}\t{mv_ang[1]:.3f}\n'
            f.write(tof)
        print('-----')
        for ii in range(1):
            time.sleep(1)
            print(ii)
        



def collect_calibration_dataset_single(off_moves, cal_path, servo, cam):
    ref_val = servo.value
    
    for idx, mv in enumerate(off_moves):
        # move servo
        servo.value = ref_val + mv
        print(f'move{idx}')
        for i in range(100):
            # read camera frame:  
            valid, frame = cam.read()
            if not valid:
                print('invalid frame')
                continue
            print(f'shooting in {100-i}',end='\n')
            cv2.imshow('Calibration',frame)
            cv2.waitKey(1)
        print('')
        #save 
        filename = cal_path+'/'+str(idx)+'.png'
        cv2.imwrite(filename, frame)