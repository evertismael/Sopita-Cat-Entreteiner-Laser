import cv2, glob
import libs.cam_calib_utils as ccu
import numpy as np
import time

# load servos file:
folder_path = './cal_laser/'
srv_file = folder_path + 'servos.txt'
with open(srv_file) as f:
    srvs_data = f.read()



def chess_roi_mask(frame, P_pxl, ptrn_size):
    # find up-left, up-right as well as down-left down-right.
    chss_corners = P_pxl[:,[0, ptrn_size[0]-1, ptrn_size[0]*(ptrn_size[1]-1), ptrn_size[0]*ptrn_size[1]-1]]
    chss_corners = chss_corners.astype(int)
    
    for c_idx in range(chss_corners.shape[1]):
        cv2.circle(frame,chss_corners[:,c_idx],4,(0,0,0),1,1,0)
    
    # make mask:
    xymin = np.min(chss_corners,axis=1)-10*np.ones((2,)).astype(int)
    xymax = np.max(chss_corners,axis=1)+10*np.ones((2,)).astype(int)
    mask = np.zeros(frame.shape[:2], dtype="uint8")
    cv2.rectangle(mask, xymin, xymax, 255, -1)
    frm = cv2.bitwise_and(frame, frame, mask=mask)

    # threshold the image to find only the black rectangles.
    thr = 175
    mask = cv2.inRange(frm, np.array([0]), np.array([thr])) 
    
    # invert the mask and do a dilation to remove the white lines that are not yet removed.
    mask_inv = 255-mask
    kernel = np.ones((4,4),np.uint8)
    mask_inv = cv2.dilate(mask_inv,kernel,iterations = 1)
    mask = 255-mask_inv
    frm_black = cv2.bitwise_and(frm, frm,mask=mask)

    # now remove the black parts: after this point only the beam is shown.
    thr = 135
    mask = cv2.inRange(frm_black, np.array([thr]), np.array([255])) 
    for c_idx in range(chss_corners.shape[1]):
        cv2.circle(mask,chss_corners[:,c_idx],4,(255,255,255),1,1,0)

    cv2.imshow('', frame)
    cv2.imshow('m', mask)
    cv2.waitKey(0)
    a=2
    return 2

# Each row in the file contain the name of the image and the angles of the servos.
# Notice that only one point is recovered per row (Since we have a single beam).
srvs_data = srvs_data.split('\n')
for row in srvs_data:
    #row = srvs_data[2]
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
    P_chs_list, P_pxl_list,img_size = ccu.find_sequence_chessboard_points([img_name], ptrn_size,False)
    frame_roi = chess_roi_mask(frame_grey, P_pxl_list[0].T, ptrn_size)



    #for i in range(300):

        #cv2.imshow('', frame_grey)




    #print(row)
    #cv2.waitKey(0)
    


