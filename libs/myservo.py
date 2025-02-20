from gpiozero import Servo
VAL2ANGLE = 90./1.0
ANGLE2VAL = 1.0/90.0
MAX_VALUE = 1.0
MIN_VALUE = -1.0

def sanitate_max_values(value):
    return max([min([value, MAX_VALUE]), MIN_VALUE])

def manual_left_right(ch:str, servo:Servo, step:float):
    if ch==81: # use left arrrow
        servo.value = sanitate_max_values(servo.value + step)
        print('left')
    elif ch==83: # use right arrow
        servo.value = sanitate_max_values(servo.value - step)
        print('right')
    return

def manual_up_down(ch:str, servo:Servo, step:float):
    if ch==82: # use up arrrow
        servo.value = sanitate_max_values(servo.value + step)
        print('up')
    elif ch==84: # use down arrow
        servo.value = sanitate_max_values(servo.value - step)
        print('down')
    return


