import cv2, glob
import libs.cam_calib_utils as ccu
import numpy as np
import time
import imutils
from scipy.linalg import null_space

# load servos file:
folder_path = './cal_laser/cal_laser_second_try/'
srv_file = folder_path + 'servos.txt'
with open(srv_file) as f:
    srvs_data = f.read()


def ind2sub(ptrn_size, sqr_idx):
    N,_ = ptrn_size
    u = int(sqr_idx/(N-1))
    v = int(sqr_idx%(N-1))
    return u,v
def corner_indexes(ptrn_size, sqr_idx):
    '''
    Return the indexes of the points that are the corners of the square with index sqr_idx.
    The returned indexes idxs are in the clockwise direction.
    Example:
            for sqr_idx=0 (first index), idxs=[0,1,9,8] for a ptrn_size = (8,4)
    '''
    N,_ = ptrn_size
    u,v = ind2sub(ptrn_size, sqr_idx)
    
    idxs = []
    idxs.append(sqr_idx+u     )
    idxs.append(sqr_idx+u   +1)
    idxs.append(sqr_idx+u+N +1)
    idxs.append(sqr_idx+u+N   )
    return idxs
def get_black_sqr_idxs(ptrn_size,first_black_idx):
    '''
    Returns the indexes of the black squares. Consider the following pattern:
        b w b w
        w b w b    
    The output is [0,2,5,7]
    Whereas: 
        w b w b
        b w b w
    The output is [1,3,4,6]


    '''
    sqr_idx_list = np.array([]).astype(int)
    for v in range(ptrn_size[1]-1):
        new_idx_list = np.array(range(first_black_idx,(ptrn_size[0]-1),2)).astype(int) + int(v*(ptrn_size[0]-1))
        sqr_idx_list = np.concatenate((sqr_idx_list,new_idx_list), axis=0)

        if (ptrn_size[0]%2-1)==0:
            first_black_idx+=1
            first_black_idx = first_black_idx%2

    return sqr_idx_list
def get_chess_black_squares(frame, P_pxl, ptrn_size, erode_flag=False):
    '''
    Return a mask that only contains the black squares from the chessboard.
    
    inputs: 
            frame: gray scaled image.

    '''
    # Step 1: Determine if the pattern starts with a black or white square.
    idx1 = corner_indexes(ptrn_size, 0)
    sqr1_pnts = P_pxl[:,idx1].astype(int)
    mask_sqr1 = np.zeros(frame.shape[:2],dtype="uint8")
    cv2.fillPoly(mask_sqr1,[sqr1_pnts.T],(255))

    idx2 = corner_indexes(ptrn_size, 1)
    sqr2_pnts = P_pxl[:,idx2].astype(int)
    mask_sqr2 = np.zeros(frame.shape[:2],dtype="uint8")
    cv2.fillPoly(mask_sqr2,[sqr2_pnts.T],(255))
    
    # compute the intensity value I: (Lowest 'I' identifies the black square)
    I_sqr1 = np.sum(cv2.bitwise_and(frame,frame,mask=mask_sqr1))
    I_sqr2 = np.sum(cv2.bitwise_and(frame,frame,mask=mask_sqr2))
    
    first_black_idx = 0 if I_sqr1<I_sqr2 else 1

    # Step 2: Recover the corner indexes for all squares in the chessboard that are black.
    sqr_idx_list = get_black_sqr_idxs(ptrn_size,first_black_idx)

    final_mask = np.zeros(frame.shape[:2],dtype="uint8")
    for sqr_idx in list(sqr_idx_list):
        idxs = corner_indexes(ptrn_size, sqr_idx)
        sqr_pnts = P_pxl[:,idxs].astype(int)
        cv2.fillPoly(final_mask,[sqr_pnts.T],(255))

    # Step 3: erode to reduce the white spaces:
    kernel = np.ones((8,8),np.uint8)
    final_mask = cv2.erode(final_mask,kernel,iterations = 1)

    frame_black_sqrs = cv2.bitwise_and(frame,frame,mask=final_mask)  
    
    return frame_black_sqrs, final_mask
def get_laser_pixel_coords_from_black_squares(frm, thr):
    
    # step 1: threshold the image
    mask = cv2.threshold(frm, thr, 255, cv2.THRESH_BINARY)[1]

    # step 2: find the center of the blob (the blob should be only one at the beam location)
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
    return ret, L_pxl


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
    fr_sqr_black, roi_mask = get_chess_black_squares(frame_grey, P_pxl_list[0].T, ptrn_size)
    ret, L_pxl = get_laser_pixel_coords_from_black_squares(fr_sqr_black,thr = 140)
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
# Up to here we have laser in pixel, world coords. 

def compute_matrix_Lo(L_w, Ul, k0):
    # step 1:
    Pl = np.multiply(Ul,k0)


    # step 2:

    Rl, Tl = 0,0
    return Rl, Tl

k0 = np.stack([np.sqrt(sum(c[1]**2)) for c in cam2w_info_list]).reshape(1,-1)
Ul  = np.stack(tuple(Ul_list),axis=1).reshape(3,-1)
L_w = np.stack(tuple(L_w_list),axis=1).reshape(3,-1)
Rl, Tl = compute_matrix_Lo(L_pxl, Ul, k0)

#cam2w_info = []
a=2
