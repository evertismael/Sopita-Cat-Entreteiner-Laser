import os
import time
import cv2
from libs.myservo import sanitate_max_values

def get_calibration_path_serpent(min_val:-5, max_val:5,step=1):
    assert(abs(min_val)>=step,'step too big')
    assert(abs(max_val)>=step,'step too big')
    assert(min_val<max_val,'min max values are wrong')

    side_grid = range(min_val, max_val,step)
    tmp = list(side_grid).copy()
    mvs = []
    for i,v in enumerate(side_grid):
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


def collect_calibration_dataset(off_moves, cal_path, servo1, servo2, cam):
    ref_val1 = servo1.value
    ref_val2 = servo2.value
    
    for idx, mv in enumerate(off_moves):
        # move servo
        servo1.value = sanitate_max_values(ref_val1 + mv[0])
        servo2.value = sanitate_max_values(ref_val2 + mv[1])
        print(f'Collecting {idx} of {len(off_moves)}')
        print(f'move{idx}: s1: {mv[0]} , s2: {mv[1]}')
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
            tof = f'{ref_val1}\t{ref_val2}\t{servo1.value:.3f}\t{servo2.value:.3f}\t{idx:d}\n'
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