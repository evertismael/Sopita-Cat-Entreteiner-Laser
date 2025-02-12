import os
import time
import cv2
from libs.myservo import sanitate_max_values, ANGLE2VAL, VAL2ANGLE

def sanitate_max_values(value, max_val, min_val):
    return max([min([value, max_val]), min_val])


def manual_left_right(ch:str, Px, max_val=100, min_val=0, step=10):
    if ch==81: # use left arrrow
        Px[0] = sanitate_max_values(Px[0] - step, max_val, min_val)
        print('left')
    elif ch==83: # use right arrow
        Px[0] = sanitate_max_values(Px[0] + step, max_val, min_val)
        print('right')
    return Px

def manual_up_down(ch:str, Px, max_val=100, min_val=0,step=10):
    if ch==82: # use up arrrow
        Px[1] = sanitate_max_values(Px[1] - step, max_val, min_val)
        print('up')
    elif ch==84: # use down arrow
        Px[1] = sanitate_max_values(Px[1] + step, max_val, min_val)
        print('down')
    return Px