import cv2, glob
import libs.cam_calib_utils as ccu
import numpy as np
import time

# load servos file:
folder_path = './cal_laser/'
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
    kernel = np.ones((7,7),np.uint8)
    final_mask = cv2.erode(final_mask,kernel,iterations = 1)

    frame_black_sqrs = cv2.bitwise_and(frame,frame,mask=final_mask)  
    
    return frame_black_sqrs, final_mask

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
    fr_sqr_black, roi_mask = get_chess_black_squares(frame_grey, P_pxl_list[0].T, ptrn_size)
    
    #cv2.imshow('', frame)
    #cv2.imshow('a', final_mask)
    cv2.imshow('dd', fr_sqr_black)
    cv2.waitKey(0)
    a=2
    a=2

    #for i in range(300):

        #cv2.imshow('', frame_grey)




    #print(row)
    #cv2.waitKey(0)
    


