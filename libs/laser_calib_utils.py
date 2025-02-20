import numpy as np
import cv2
from scipy.optimize import minimize, Bounds
import imutils



def get_laser_pixel_coords_from_black_squares(frm, thr):
    
    # step 1: threshold the image
    mask = cv2.threshold(frm, thr, 255, cv2.THRESH_BINARY)[1]

    # step 2: find the center of the blob (the blob should be only one at the beam location)
    kernel = np.ones((4,4))
    mask = cv2.dilate(mask,kernel,iterations = 2)

    mask_contour = mask.copy()
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    x_pxl, y_pxl = -1, -1
    bigest_area = -np.Inf
    for cntr in contours:
        M = cv2.moments(cntr)
        if bigest_area < M['m00'] and M['m00'] > 0:
            bigest_area = M['m00']
            x_pxl = M["m10"] / M["m00"]
            y_pxl = M["m01"] / M["m00"]

    # output
    ret = bigest_area != -np.Inf
    L_pxl = np.array([[x_pxl],[y_pxl]])

    cv2.imshow('contours', mask_contour)
    #cv2.waitKey(0)

    return ret, L_pxl



def error_angles_with_Ul(ang_and_T, Pw, Ul):
    angles_hat = np.array(ang_and_T[0:3])
    T_hat = np.array(ang_and_T[3:])
    R_hat,_ = cv2.Rodrigues(np.deg2rad(angles_hat))

    Pl_hat = R_hat.dot(Pw) + T_hat.reshape(3,1)
    Ul_hat = Pl_hat/np.sqrt(np.sum(Pl_hat**2,axis=0,keepdims=True))

    error = np.mean(np.sum((Ul - Ul_hat)**2,axis=0,keepdims=True), axis=1, keepdims=True)
    return error

def estimate_R_T_optimization(angles_0, T0, Pw,Ul):
    x0mins = [-90,-90,-90,-100,-100,-100]
    x0max =  [ 90, 90, 90, 100, 100, 100]
    bounds = Bounds(x0mins, x0max)

    x0 = np.column_stack((angles_0, T0))
    res = minimize(error_angles_with_Ul,x0,args=(Pw,Ul),method='nelder-mead',options={'display':False}, bounds=bounds)
    
    angles_hat = np.array(res.x[:3])
    T_hat = np.array(res.x[3:]).reshape(3,1)
    R_hat,_ = cv2.Rodrigues(angles_hat)

    return R_hat, T_hat, res

def load_RT_laser_matrices(path='./out/laser'):
    R_las = np.load(path + '_R_las.npy')
    T_las = np.load(path + '_T_las.npy')
    return R_las, T_las