import cv2, glob
import libs.cam_calib_utils as ccu
import libs.laser_calib_utils as lcu
import libs.chess_utils as chu
import libs.servo_mechanics as sm
import numpy as np
import time
import matplotlib.pylab as plt
import numpy as np


# numpy printing options:
np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)},linewidth=np.inf)


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
chess_img_files = [folder_path + row.split('\t')[1] + '/' + row.split('\t')[0] + '_img.png' for row in srvs_data]

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

M_cam_wall = np.eye(4)
M_cam_wall[:3,:3] = R_cam_wall
M_cam_wall[:3,3,None] = T_cam_wall

M_wall_cam = ccu.inv_svd(M_cam_wall)
Ainv = ccu.inv_svd(mtx_new)


print(f'----------------------------------------------------------')
print(f'Step 1:')
print(f"used images: {[row[0:2] for row in srvs_data]}")
print(f'rvec_mean_deg:{np.rad2deg(rvec_mean.T)} ; rvec_std: {np.rad2deg(rvec_std.T)}')
print(f'T_cam_mean:{T_cam_mean.T} ; T_cam_std: {T_cam_std.T}')
print(f'----------------------------------------------------------')


'''
  ---------------------------------------------------------------------------------
  Step 2: Gather all detected beam points:
  ---------------------------------------------------------------------------------
  Each image in chess_img_files contains a red-point over a black square over the 
  chessboard. We collect this point in the image (L_pxl) and the corresponding wall
  coords (L_wall). This L_wall is computed using R_cam_wall, T_cam_wall and exploiting
  the fact that all points lay over the same plane z_wall = 0.
'''
chess_img_files = [folder_path + row.split('\t')[1] + '/' + row.split('\t')[0] + '_img.png' for row in srvs_data]
block_idx_list = [int(row.split('\t')[1]) for row in srvs_data]

# read and parsing file with block mask info:
with open(folder_path+'laser_point_masks.txt') as f:
    blck_mask_info_list = f.read()
blck_mask_info_list = blck_mask_info_list.split('\n')[:-1]
blck_mask_info_list = [(int(row.split('\t')[0]), int(row.split('\t')[1]), int(row.split('\t')[2]), int(row.split('\t')[3]),int(row.split('\t')[4])) for row in blck_mask_info_list]


L_pxl_list = []
L_wall_list = []
ret_list = []
for img_file, P_pxl,blk_idx in zip(chess_img_files,P_pxl_list, block_idx_list):
    # A. load image, convert to grey, draw block
    frame = cv2.imread(img_file)
    frame_grey = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    mask_info = blck_mask_info_list[blk_idx]
    _, mouseX, mouseY, rw, rh = mask_info
    p0 = (mouseX - rw//2, mouseY- rh//2)
    pf = (mouseX + rw//2, mouseY+ rh//2) 
    cv2.rectangle(frame,p0,pf,0,1)
    cv2.imshow('', frame)
    #cv2.waitKey(0)

    # B. mask only black squares
    mask = np.zeros(frame.shape[:2], dtype="uint8")
    cv2.rectangle(mask,p0,pf,255,-1)
    mask = cv2.erode(mask,np.ones((4,4),dtype=np.uint8), iterations=3)
    frame_masked = cv2.bitwise_and(frame_grey,frame_grey,mask=mask)

    cv2.imshow('masked', frame_masked)
    #cv2.waitKey(0)

    # C. get Laser beam in pixel coords
    ret, L_pxl = lcu.get_laser_pixel_coords_from_black_squares(frame_masked,thr = 130)
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
cv2.waitKey(1)

L_pxl_list = [L_pxl for L_pxl,rl in zip(L_pxl_list,ret_list) if rl==True]
L_wall_list = [L_wall for L_wall,rl in zip(L_wall_list,ret_list) if rl==True]
srvs_data = [row for row,rl in zip(srvs_data,ret_list) if rl==True]

print(f'----------------------------------------------------------')
print(f'Step 2:')
print(f"used images: {[row[0:2] for row in srvs_data]}")
print(f'L_pxl_list (10 first):')
print(np.stack(L_pxl_list[:10]).reshape(-1,2).T)
print(f'L_wall_list (10 first):')
print(np.stack(L_wall_list[:10]).reshape(-1,3).T)
print(f'----------------------------------------------------------')


'''
  ---------------------------------------------------------------------------------
  Step 3: Estimate R_lsr_wall,T_lsr_wall using optimization.
  ---------------------------------------------------------------------------------
'''
Lx = 0.7
L_wall = np.stack(L_wall_list).reshape(-1,3).T
L1_wall = np.row_stack((L_wall,np.ones((1,L_wall.shape[1]))))
theta_phi_deg_list = [np.array([float(row.split('\t')[2]), float(row.split('\t')[3])]) for row in srvs_data]

M_lsr_wall_list = []
for idx, theta_phi_deg in enumerate(theta_phi_deg_list):
    theta_deg, phi_deg = theta_phi_deg[0], theta_phi_deg[1]

    _, _, M_lsr_wall = sm.move_set_up(theta_deg, phi_deg, M_W_L=np.eye(4),Lx=Lx)
    M_lsr_wall_list.append(M_lsr_wall)

cam_lsr_delta_wall = np.array([[0,-1,-1.5]]).T
T_wall_lsr_0 = M_wall_cam[:3,3,None] + cam_lsr_delta_wall # notice this is not T_lsr_wall_0
angles_hat, R_lsr_wall_hat, T_lsr_wall_hat = sm.estimate_R_l_w_and_T_l_w(L1_wall, M_lsr_wall_list, T_wall_lsr_0, options={'maxiter':1e4, 'disp':True})

M_lsr_wall = np.eye(4)
M_lsr_wall[:3,:3] = R_lsr_wall_hat
M_lsr_wall[:3,3] = T_lsr_wall_hat

M_wall_lsr = ccu.inv_svd(M_lsr_wall)

'''
  ---------------------------------------------------------------------------------
  Step 4: Plot the computed matrices.
  ---------------------------------------------------------------------------------
'''

plt.ion()
plt.figure("Test Figure")

# M_W: wall:
M_wall_world = np.eye(4)
M_wall_world[:3,:3] = np.array([[0,-1,0],[0,0,-1],[1,0,0]])
M_wall_world[:3,3,None] = np.array([[0,0,-10]]).T
M_world_wall = ccu.inv_svd(M_wall_world)

ax = sm.make_figure([-25,10], [-20,10], [-20,10])
sm.plot_coord_sys(np.eye(4),10,'world', ax, 0.2)
sm.plot_coord_sys(M_world_wall,4,'wall', ax, 0.6)
sm.plot_coord_sys(M_world_wall.dot(M_wall_cam),10,'cam', ax, 1)
sm.plot_coord_sys(M_world_wall.dot(M_wall_lsr),10,'lsr', ax, 1)

plt.show()


# save matrices:
path = './out/final_'
np.save(path + '_M_cam_wall.npy', M_cam_wall)
np.save(path + '_M_lsr_wall.npy', M_lsr_wall)
np.save(path + '_M_world_wall.npy', M_world_wall)
a=2