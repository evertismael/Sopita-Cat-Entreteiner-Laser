import cv2, glob
import numpy as np
from libs.cam_calib_utils import inv_svd, find_sequence_chessboard_points

img_folder = './cal_imgs/'
chess_img_files = glob.glob(img_folder+'*.png')


# Step1: Collect all sequence of points for each chessboard, 
#        as well as the ones in chess coords.
ptrn_size = ((11,7))
scale_down = False
P_chs_list, P_pxl_list,img_size = find_sequence_chessboard_points(chess_img_files, ptrn_size,scale_down)

# Step2: Compute the calibration, and return the matrices: (img_size(is reverted))
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(P_chs_list,P_pxl_list,img_size[::-1],None,None)

# -------------------------------------------------------------
# -------------------------------------------------------------
# Step3: save results and show location of points on one image:
for i in [33]:
    frame = cv2.imread(chess_img_files[i])
    frame = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)
    #frame = cv2.resize(frame,(640,480))
    cv2.drawChessboardCorners(frame,ptrn_size,P_pxl_list[i],True)
    for p_idx, p in enumerate(P_pxl_list[i]):
        p = [int(a) for a in p]
        cv2.putText(frame,str(p_idx),p,cv2.FONT_HERSHEY_COMPLEX,.4,(0,0,0),1,1)
    cv2.imshow('example',frame)
    cv2.waitKey(0)


if scale_down:
    out_file_pref = './out/cal1_scaled'
else:
    out_file_pref = './out/cal1'


np.save(out_file_pref + '_mtx.npy', mtx)
np.save(out_file_pref + '_dist.npy', dist)
np.save(out_file_pref + '_rvecs.npy', rvecs)
np.save(out_file_pref + '_tvecs.npy', tvecs)
np.save(out_file_pref + '_imgsize.npy', img_size)



