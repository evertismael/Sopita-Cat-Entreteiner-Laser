import cv2
import time
from libs.cam_calib_utils import detect_chess_board_points

cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_BUFFERSIZE,1)
a=cam.get(cv2.CAP_PROP_BUFFERSIZE) 


cal_imgs_path = './cal_imgs'

idx = 0
while True:
    ret, frame = cam.read()
    
    frame_to_save = frame.copy()
    
    # try to detect the chessboard
    ptrn_size = ((10,7))
    ret, P_chs, P_pxl = detect_chess_board_points(frame, ptrn_size)

    # if detected draw:
    if ret==True:
        cv2.drawChessboardCorners(frame,ptrn_size,P_pxl,ret)
        for p_idx, p in enumerate(P_pxl):
            p = [int(a) for a in p]
            cv2.putText(frame,str(p_idx),p,cv2.FONT_HERSHEY_COMPLEX,.4,(0,0,0),1,1)

    cv2.imshow('camera',frame)


    pressed_key = cv2.waitKey(1)
    if pressed_key==ord('q'):
        break
    elif pressed_key==ord('c'):
        if ret==True:
            print('capturing image')
            filename = cal_imgs_path+'/'+str(idx)+'_img.png'
            cv2.imwrite(filename, frame_to_save)

            filename = cal_imgs_path+'/'+str(idx)+'_chess.png'
            cv2.imwrite(filename, frame)
            
            print('-------------------------------------------------')
            print(f'image {filename} save')
            print('-------------------------------------------------')
            time.sleep(1)
            idx+=1
        else:
            print(f'Chess not in image - NOT SAVED')

cam.release()
cv2.destroyAllWindows()
