from gpiozero import Servo

MAX_VALUE = 1
MIN_VALUE = -1

def manual_left_right(ch:str, servo:Servo, step:float):

    if ch==81: # use left arrrow
        servo.value = min([servo.value + step, MAX_VALUE])
        print('left')
    elif ch==83: # use right arrow
        servo.value = max([servo.value - step, MIN_VALUE])
        print('right')
    return

def manual_up_down(ch:str, servo:Servo, step:float):
    if ch==82: # use up arrrow
        servo.value = min([servo.value + step, MAX_VALUE])
        print('up')
    elif ch==84: # use down arrow
        servo.value = max([servo.value - step, MIN_VALUE])
        print('down')
    return