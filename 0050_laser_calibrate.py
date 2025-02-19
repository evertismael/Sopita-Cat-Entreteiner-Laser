import cv2, glob
import libs.cam_calib_utils as ccu
import libs.laser_calib_utils as lcu
import libs.chess_utils as chu
import numpy as np
import time


# load servos file:
folder_path = './cal_laser/'
srv_file = folder_path + 'servos.txt'
with open(srv_file) as f:
    srvs_data = f.read()
srvs_data = srvs_data.split('\n')
srvs_data = srvs_data[:-1]


# Load camera matrices
cal_file_pref = './out/cal1'
_, dist, img_size, mtx_new, _ = ccu.load_camera_calibration_matrices(cal_file_pref)

''' 
  ---------------------------------------------------------------------------------
  Step 1: Compute camera R_cam_wall, T_cam_wall. 
  They are needed to compute wall coordinates of laser beam detected on the image
  ---------------------------------------------------------------------------------
  Each row in file contains the name of one image.
  Assumption: All images contain the chessboard in the same location.
  Hence we use them to compute R_cam_wall and T_cam_wall
'''
chess_img_files = [folder_path + row.split('\t')[0] + '_img.png' for row in srvs_data]
ptrn_size = ((6,4))
ret_list, P_wall_list, P_pxl_list,img_size = ccu.find_chessboard_on_image_files(chess_img_files, ptrn_size,False)
# select the pairs that are valid:
P_wall_list = [P_wall for P_wall,rl in zip(P_wall_list,ret_list) if rl==True]
P_pxl_list = [P_pxl for P_pxl,rl in zip(P_pxl_list,ret_list) if rl==True]
srvs_data = [row for row,rl in zip(srvs_data,ret_list) if rl==True] # filter out images with no chessboard pattern

rvec_list = []
T_cam_list = []
for P_wall, P_pxl in zip(P_wall_list, P_pxl_list):
    ret, rvec, T_cam =cv2.solvePnP(P_wall,P_pxl,mtx_new,dist)
    rvec_list.append(rvec)
    T_cam_list.append(T_cam)

rvec_mean, rvec_std = np.mean(np.array(rvec_list),axis=0), np.std(np.array(rvec_list),axis=0)
T_cam_mean, T_cam_std = np.mean(np.array(T_cam_list),axis=0), np.std(np.array(T_cam_list),axis=0)

R_cam_wall,_ = cv2.Rodrigues(rvec_mean)
T_cam_wall = T_cam_mean
Ainv = ccu.inv_svd(mtx_new)


'''
  ---------------------------------------------------------------------------------
  Step 2: Gather all detected beam points:
  ---------------------------------------------------------------------------------
'''
chess_img_files = [folder_path + row.split('\t')[0] + '_img.png' for row in srvs_data]

L_pxl_list = []
L_wall_list = []
ret_list = []
for img_file, P_pxl in zip(chess_img_files,P_pxl_list):
    # A. load image, convert to grey, draw pattern
    frame = cv2.imread(img_file)
    frame_grey = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    cv2.drawChessboardCorners(frame, ptrn_size, P_pxl,True)
    cv2.putText(frame,'0', P_pxl[0,:].astype(int),cv2.FONT_HERSHEY_COMPLEX,.4,(0,0,0),1,1)
    cv2.imshow('', frame)
    #cv2.waitKey(0)

    # B. mask only black squares
    fr_sqr_black, roi_mask = chu.get_chess_black_squares(frame_grey, P_pxl.T, ptrn_size)
    cv2.imshow('black', fr_sqr_black)
    #cv2.waitKey(0)

    # C. get Laser beam in pixel coords
    ret, L_pxl = lcu.get_laser_pixel_coords_from_black_squares(fr_sqr_black,thr = 130)
    ret_list.append(ret)
    if ret==True: # some beams are very small.
        cv2.circle(frame_grey,L_pxl.astype(int)[:,0],10,(255,0,0),3,1)
        cv2.imshow('laser beam', frame_grey)
        
        # D. get wall coords of laser beam:
        L_wall = ccu.uv2XYZ(L_pxl, Ainv, R_cam_wall, T_cam_wall)

        # save for later processing:
        L_pxl_list.append(L_pxl)
        L_wall_list.append(L_wall)
    else:
        cv2.imshow('laser beam', frame_grey)
        L_pxl_list.append([])
        L_wall_list.append([])

    print(f'Processed : {img_file}')
    cv2.waitKey(0)

L_pxl_list = [L_pxl for L_pxl,rl in zip(L_pxl_list,ret_list) if rl==True]
L_wall_list = [L_wall for L_wall,rl in zip(L_wall_list,ret_list) if rl==True]


'''
  ---------------------------------------------------------------------------------
  Step 3: Estimate R_lsr_wall,T_lsr_wall using optimization.
  ---------------------------------------------------------------------------------
'''







# --------------------------------------------------------------------------------
# --------------------------------------------------------------------------------
# --------------------------------------------------------------------------------
# Up to here we have laser in pixel, world coords, and Ul unitary vectors. 
# now se compute R and T using optimization
Ul  = np.stack(tuple(Ul_list),axis=1).reshape(3,-1)
L_w = np.stack(tuple(L_w_list),axis=1).reshape(3,-1)

# find the angles and T using optimization:
R0 = cam2w_info_list[0][0]
T0 = cam2w_info_list[0][1]
angles_0,_ = cv2.Rodrigues(R0)
angles_0 = angles_0.reshape(1,3)
T0 = T0.reshape(1,3)
R_hat,T_hat,res = lcu.estimate_R_T_optimization(angles_0, T0, L_w,Ul)

# Compute error:
# 1. project Pw to Pl
# 2. compute unitary vectors and compare:
Pl_hat = R_hat.dot(L_w) + T_hat
Ul_hat = Pl_hat/np.sqrt(np.sum(Pl_hat**2,axis=0,keepdims=True))

er = np.sum(np.sum((Ul_hat - Ul)**2,axis=0,keepdims=True))
print(f'approximation error of unitary vectors: {er}')

# ------------------------------------------
# -----------------------------------------
# Finally just save the calibration matrices:
R_las = R_hat
T_las = T_hat
out_file_pref = './out/laser'
np.save(out_file_pref + '_R_las.npy', R_las)
np.save(out_file_pref + '_T_las.npy', T_las)
print(f'Laser saved in {out_file_pref}')


R_cam = cam2w_info_list[0][0]
T_cam = cam2w_info_list[0][1].reshape(1,3)
Ainv_cam = Ainv
A_cam = mtx_new
out_file_pref = './out/camera'
np.save(out_file_pref + '_R_cam.npy', R_cam)
np.save(out_file_pref + '_T_cam.npy', T_cam)
np.save(out_file_pref + '_A_cam.npy', A_cam)
np.save(out_file_pref + '_Ainv_cam.npy', Ainv_cam)
print(f'Laser saved in {out_file_pref}')
