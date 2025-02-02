import os
import time
import cv2

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
        servo1.value = ref_val1 + mv[0]
        servo2.value = ref_val2 + mv[1]

        print(f'move{idx}: s1: {mv[0]} , s2: {mv[1]}')
        for i in range(10):
            # read camera frame:  
            valid, frame = cam.read()
            if not valid:
                print('invalid frame')
                continue
            print(f'shooting in {10-i}',end='\n')
            cv2.imshow('Calibration',frame)
            cv2.waitKey(1)
        print('')
        #save 
        filename = cal_path+'/'+str(idx)+'.png'
        cv2.imwrite(filename, frame)


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