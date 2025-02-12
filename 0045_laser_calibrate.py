import cv2, glob
import libs.cam_calib_utils as ccu
import libs.laser_calib_utils as lcu
import libs.chess_utils as chu
import numpy as np
import time


# load servos file:
folder_path = './cal_laser/cal_laser_second_try/'
srv_file = folder_path + 'servos.txt'
with open(srv_file) as f:
    srvs_data = f.read()


# Load camera matrices
cal_file_pref = './out/out_first_try/cal1'
_, dist, img_size, mtx_new, _ = ccu.load_camera_calibration_matrices(cal_file_pref)


# Each row in the file contain the name of the image and the angles of the servos.
# Notice that only one point is recovered per row (Since we have a single beam).
srvs_data = srvs_data.split('\n')
srvs_data = srvs_data[:-1]

Ul_list = []
L_pxl_list = []
L_w_list = []
cam2w_info_list = []
for row in srvs_data:
    #row = srvs_data[1]
    row = row.split('\t')

    # load the image
    img_name = f'{folder_path}{row[0]}.png' 
    frame = cv2.imread(img_name)

    # -------------------------------------------------------------
    # find the pixel point of the laser.
    # -------------------------------------------------------------
    # Step 1: find the chess pattern in the image to remove everything from around.
    frame_grey = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    ptrn_size = ((11,7))
    ret_list, P_chs_list, P_pxl_list, img_size = ccu.find_sequence_chessboard_points([img_name], ptrn_size,False)
    if ret_list[0]==False: # some pictures do not find the chessboard.
        continue
    
    # Step 2: mask and retrieve only the black squares in the chessboard
    fr_sqr_black, roi_mask = chu.get_chess_black_squares(frame_grey, P_pxl_list[0].T, ptrn_size)
    ret, L_pxl = lcu.get_laser_pixel_coords_from_black_squares(fr_sqr_black,thr = 140)
    if ret==False: # some beams are very small.
        continue

    # -------------------------------------------------------------
    # find the world coordinates point of the laser beam from the pixel coords
    # -------------------------------------------------------------
    # Step 3: Using the image pixel and world points, compute R and T.
    ret, rvec, T=cv2.solvePnP(P_chs_list[0],P_pxl_list[0],mtx_new,dist)
    R,_ = cv2.Rodrigues(rvec)
    Ainv = ccu.inv_svd(mtx_new)

    # Step 4: with R,T,Ainv known we can map any point in the image 
    #         to the corresponding point in the plane of the chessboard in world coords.
    L_w = ccu.uv2XYZ(L_pxl, Ainv, R, T)

    # Step 5: lastly recover the rotation angles from the servos and compute the directional vector:
    theta_deg = 90 + float(row[1])
    phi_deg = 90 - float(row[2])
    sin_phi = np.sin(np.deg2rad(phi_deg))
    cos_phi = np.cos(np.deg2rad(phi_deg))
    sin_tht = np.sin(np.deg2rad(theta_deg))
    cos_tht = np.cos(np.deg2rad(theta_deg))

    # unitary directional vector in Laser coords:
    ul = np.array([sin_phi*cos_tht, sin_phi*sin_tht, cos_phi])

    # store for further processing:
    Ul_list.append(ul)
    L_pxl_list.append(L_pxl)
    L_w_list.append(L_w)
    cam2w_info_list.append((R,T,mtx_new,Ainv))

    print(row)    
    #cv2.circle(frame_grey,L_pxl.astype(int)[:,0],10,(255),10,1)
    #cv2.imshow('', frame_grey)
    #cv2.waitKey(0)


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