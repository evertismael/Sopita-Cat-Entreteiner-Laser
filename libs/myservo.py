from gpiozero import Servo

MAX_VALUE = 1
MIN_VALUE = -1

def manual_left_right(ch:str, servo:Servo, step:float):

    if ch==81: # use right arrrow
        servo.value = min([servo.value + step, MAX_VALUE])
        print('right')
    elif ch==83: # use left arrow
        servo.value = max([servo.value - step, MIN_VALUE])
        print('left')
    return