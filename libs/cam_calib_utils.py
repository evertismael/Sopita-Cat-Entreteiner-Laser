import numpy as np
import cv2

def inv_svd(A):
    ''' 
    Notice that this function returns the inverse, but it's the user who decides if it's the left or right pseudoinverse.
    If A is squared then A_inv=left=right inverses
    '''
    """
    Example:
    A = np.array([[3,2,2],[2,3,-2]])
    A_inv = inv_svd(A)
    print(A)
    print(A_inv)
    print("Notice that in this case, A_inv is the right pseudo inverse since A in wide and not tall matrix")
    print(np.dot(A,A_inv))
    """
    U, S, Vh = np.linalg.svd(A, full_matrices=True)

    Sm = np.zeros((U.shape[0],Vh.shape[1])) # cause (dimU, dimV)
    Sm[:len(S),:len(S)] = np.diag(S)
    
    Sm_inv = np.zeros((Vh.shape[1],U.shape[0])) # cause (dimV, dimU)
    Sm_inv[:len(S),:len(S)] = np.diag(1./S)
    
    A_inv = np.dot(np.dot(Vh.T,Sm_inv),U.T)

    return A_inv


def detect_chess_board_points(frame, ptrn_size):
    # Precompute Pw positions: By default cv2 locates the origin at the left-upper corner.
    # format: (0,0,0),(1,0,0),(2,0,0)......(last,last,0)
    # world coords is over the paper hence z=0 for all poins.
    P_chs = np.zeros((ptrn_size[0]*ptrn_size[1],3), np.float32)
    P_chs[:,:2] = np.mgrid[0:ptrn_size[0],0:ptrn_size[1]].T.reshape(-1,2)

    # convert to grey
    frame_gray = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)
    ret, P_pxl_tmp = cv2.findChessboardCorners(frame_gray, ptrn_size,None)

    if ret==True:
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        P_pxl = cv2.cornerSubPix(frame_gray,P_pxl_tmp,(10,10),(0,0),criteria)
        P_pxl = P_pxl.reshape(-1,2)
        
        # Notice that since each point is taken from different images, the origin of the world
        # coordinates (corner of chessboard with 0 index) might be different.
        # Ensure that the origin is the left most corner.
        if P_pxl[0,1] > P_pxl[-1,1]: # P0 to the right of Pf (INVERT ORDER)
            P_pxl = P_pxl[::-1,:]
    else:
        P_chs = []
        P_pxl = []
    return ret, P_chs, P_pxl




def find_chessboard_on_image_files(chess_img_files, ptrn_size, scale_down:False):
    ''' Collects the pair of points in chess and pixel coords.
        Each element in the lists are the associated ones to each image.
         
             ptrn_size: This is very important!!! wrong param->unable to decode.
    '''
       
    # creating the lists (one sequence per image)
    P_chs_list = []
    P_pxl_list = []
    ret_list = []
    img_size = 0,0
    for img_file in chess_img_files:
        frame = cv2.imread(img_file)
        if scale_down:
            frame = cv2.resize(frame,(640,480))

        ret, P_chs, P_pxl = detect_chess_board_points(frame, ptrn_size)
        
        # append results
        ret_list.append(ret)
        P_chs_list.append(P_chs)
        P_pxl_list.append(P_pxl)
        img_size = frame.shape[:2]
    return ret_list, P_chs_list, P_pxl_list,img_size

def load_camera_calibration_matrices(cal_file_pref):
    mtx = np.load(cal_file_pref + '_mtx.npy')
    dist = np.load(cal_file_pref + '_dist.npy')
    img_size = np.load(cal_file_pref + '_imgsize.npy')
    # refine mtx:
    mtx_new, roi =cv2.getOptimalNewCameraMatrix(mtx, dist, img_size[::-1], 1, img_size[::-1])

    return mtx, dist, img_size, mtx_new, roi

def uv2XYZ(P_pxl, Ainv, R_cam_wall, T_cam_wall):
    """ 
    Here we compute the XYZ coordinates in world coords, that are over the plane Z=0
    Notice that Z_c is the different for each point.
    Dim(P_Pxl) = (2,Npoints)
    Ex:
        # Z_c = <r_3^T, T> / <r_3^T, Ainv U>
        # with U = [u,v,1]^T
        # r_3 is the third column of R
    """
    # to homogeneous coords
    U = np.row_stack((P_pxl,np.ones((1,P_pxl.shape[1]))))

    r3 = R_cam_wall[:,2,None]
    Z_c = r3.T.dot(T_cam_wall)/r3.T.dot(Ainv.dot(U))

    # compute XYZ:
    P_w_hat = R_cam_wall.T.dot(Z_c*Ainv.dot(U) - T_cam_wall)

    return P_w_hat




def load_RTA_camera_matrices(path='./out/camera'):
    R_cam = np.load(path + '_R_cam.npy')
    T_cam = np.load(path + '_T_cam.npy')
    A_cam = np.load(path + '_A_cam.npy')
    Ainv_cam = np.load(path + '_Ainv_cam.npy')
    return R_cam, T_cam, A_cam, Ainv_cam