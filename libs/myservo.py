from gpiozero import Servo

VAL2ANGLE = 90.0/1.0
ANGLE2VAL = 1.0/90.0
MAX_DEGREES = 90.0
MIN_DEGREES = -90.0


def sanitate_max_min_angles(prev_angle, new_angle):
    if new_angle > MAX_DEGREES:
        return prev_angle
    if new_angle < MIN_DEGREES:
        return prev_angle
    return new_angle

def manual_move_servo(servo:Servo, prev_angle: float, new_angle:float):
    new_angle = sanitate_max_min_angles(prev_angle, new_angle)
    servo.value = new_angle*ANGLE2VAL
    print(f'moved to values: {prev_angle*ANGLE2VAL:.3f}, new: {new_angle*ANGLE2VAL:.3f}, angles prev: {prev_angle:.3f}, new: {new_angle:.3f}')
    return new_angle


def manual_left_right_angle(ch:str, servo:Servo, prev_angle: float,  step_angle:float):
    if ch==81: # use left arrrow
        new_angle = prev_angle + step_angle
        new_angle = sanitate_max_min_angles(prev_angle, new_angle)
        servo.value = new_angle*ANGLE2VAL
        print(f'left prev values: {prev_angle*ANGLE2VAL:.3f}, new: {new_angle*ANGLE2VAL:.3f}, angles prev: {prev_angle:.3f}, new: {new_angle:.3f}')
    elif ch==83: # use right arrow
        new_angle = prev_angle - step_angle*ANGLE2VAL
        new_angle = sanitate_max_min_angles(prev_angle, new_angle)
        servo.value = new_angle*ANGLE2VAL
        print(f'right prev values: {prev_angle*ANGLE2VAL:.3f}, new: {new_angle*ANGLE2VAL:.3f}, angles prev: {prev_angle:.3f}, new: {new_angle:.3f}')
    else:
        new_angle = prev_angle
    return new_angle

def manual_up_down_angle(ch:str, servo:Servo, prev_angle:float, step_angle:float):
    if ch==82: # use up arrrow
        new_angle = prev_angle + step_angle
        new_angle = sanitate_max_min_angles(prev_angle, new_angle)
        servo.value = new_angle*ANGLE2VAL
        print(f'up prev values: {prev_angle*ANGLE2VAL:.3f}, new: {new_angle*ANGLE2VAL:.3f}, angles prev: {prev_angle:.3f}, new: {new_angle:.3f}')
    elif ch==84: # use down arrow
        new_angle = prev_angle - step_angle*ANGLE2VAL
        new_angle = sanitate_max_min_angles(prev_angle, new_angle)
        servo.value = new_angle*ANGLE2VAL
        print(f'down prev values: {prev_angle*ANGLE2VAL:.3f}, new: {new_angle*ANGLE2VAL:.3f}, angles prev: {prev_angle:.3f}, new: {new_angle:.3f}')
    else:
        new_angle = prev_angle
    return new_angle