import cv2
import multiprocessing
import numpy as np
import libs.cam_calib_utils as ccu

from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from libs import myservo as ms
from libs import servo_mechanics as sm
import time


'''
This function only sets the servos to the positions given in theta-phi lists that are the angles of the given pointsx,y 
in pixel coords that are projected into the wall plane, that are later translated into the laser coords.
'''
def laser_process(theta_list, phi_list, running_flg):
    # init servo:
    factory = PiGPIOFactory()
    servo1 = Servo(17, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
    servo2 = Servo(18, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
    servo1.mid()
    servo2.mid()

    # init laser:
    laser = Servo(27, min_pulse_width=0./1000, max_pulse_width=2.5/1000, pin_factory=factory)
    laser.min()
    
    # loop over all movements
    while running_flg.value==True:
        for tht, phi in zip(theta_list,phi_list):
            ms.manual_move_servo(servo1, tht, tht, False)
            ms.manual_move_servo(servo2, phi, phi, False)
            time.sleep(0.03)
            laser.max()
        laser.min()
        time.sleep(0.07)

'''
This function shifts the info contained in Px, Py and adds x,y pixel coords as last element.
Notice that the first element of the array is lost every time this func is run.
'''
def add_point_in_tht_phi_lists(event,x,y,flags,param):
    global Pxy_pxl_list
    if event == cv2.EVENT_LBUTTONDBLCLK:
        # shift
        Pxy_pxl_list[:-1] = Pxy_pxl_list[1:]
        # add:
        Pxy_pxl_list[-1] = (x,y)
    elif event== cv2.EVENT_MOUSEMOVE:
        # shift
        Pxy_pxl_list[:-1] = Pxy_pxl_list[1:]
        # add:
        Pxy_pxl_list[-1] = (x,y)


Pxy_pxl_list =[]
for i in range(130) :
    Pxy_pxl_list.append((300,200))


cal_file_pref = './out/cal1'
M_file_pref = './out/'
if __name__=='__main__':
    # load camera/laser translational matrices:
    
    _, dist, img_size, mtx_new, _ = ccu.load_camera_calibration_matrices(cal_file_pref)
    M_cam_wall = np.load(M_file_pref + 'final__M_cam_wall.npy')
    M_lsr_wall = np.load(M_file_pref + 'final__M_lsr_wall.npy')
    M_world_wall = np.load(M_file_pref + 'final__M_world_wall.npy')
    Ainv = ccu.inv_svd(mtx_new)
    
    # define variables to communicate the processes:
    running_flg = multiprocessing.Value('b')
    theta_list = multiprocessing.Array('f',len(Pxy_pxl_list))
    phi_list = multiprocessing.Array('f',len(Pxy_pxl_list))

    # init:
    running_flg.value = True
    
    # start laser process:
    p_lsr = multiprocessing.Process(target=laser_process,args=(theta_list, phi_list, running_flg),name='lsr_proc')
    p_lsr.start()
    

    # start camera:s
    cam = cv2.VideoCapture(0)  
    cam.set(cv2.CAP_PROP_BUFFERSIZE,1)
    
    cv2.namedWindow('Sopita-mouse')
    cv2.setMouseCallback('Sopita-mouse',add_point_in_tht_phi_lists)
    while running_flg.value==True:
        valid, frame = cam.read()

        # convert point to theta and phi:
        P_pxl = np.stack(Pxy_pxl_list).reshape((-1,2)).T
        P_wall = ccu.uv2XYZ(P_pxl, Ainv, R_cam_wall=M_cam_wall[:3,:3], T_cam_wall=M_cam_wall[:3,3,None])
        P1_wall = np.row_stack((P_wall, np.ones((1,P_wall.shape[1]))))
        P1_lsr = M_lsr_wall.dot(P1_wall)

        tmp =[]
        for xy_idx in range(P1_lsr.shape[1]):
            Pxy_L = P1_lsr[:2,xy_idx,None]
            _, theta_deg = sm.get_theta(Pxy_L, Lx=0.7)
            theta_deg = theta_deg[0]
            phi_deg = sm.get_phi(P1_lsr[:,xy_idx,None], Lx=0.7, theta_deg=theta_deg) 

            tmp.append((theta_deg, phi_deg)) 
            theta_list[xy_idx] = theta_deg 
            phi_list[xy_idx] = phi_deg

        # print(tmp)

        for xy_idx in range(P_pxl.shape[1]):
            cv2.putText(frame,str(xy_idx),(int(P_pxl[0,xy_idx]),int(P_pxl[1,xy_idx])),cv2.FONT_HERSHEY_COMPLEX,.4,(255,0,0),1,1)
           # cv2.putText(frame,str(p_idx),  p,cv2.FONT_HERSHEY_COMPLEX,.4,(0,0,0),1,1)
        if valid==True:
            cv2.imshow('Sopita-mouse',frame)
        
        # act on pressed key
        key_pressed = cv2.waitKey(1)
        if key_pressed == ord('q'):
            running_flg.value = False
            break

    print('closing everything / releasing resources')
    p_lsr.join()
    cam.release()
    cv2.destroyAllWindows()

    
    
    
    
    