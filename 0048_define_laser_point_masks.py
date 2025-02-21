import cv2
import numpy as np
'''
Notice that when collecting the laser calibration images, the camera laser setup is the same. 
Hence the additional squares on the wall are at the same location for all images.
Hence, we can manually define all locations and masks in a single image.
'''

cal_path = './cal_laser/'
image_path = cal_path + '/0/0_img.png'
laser_point_masks_file = cal_path + '/laser_point_masks.txt'

frame = cv2.imread(image_path)
frame_copy = frame.copy()

mouseX = 0
mouseY = 0
rw = 30
rh = 30
idx = 0
def draw_circle(event,x,y,flags,param):
    global mouseX,mouseY, rw, rh
    if event == cv2.EVENT_LBUTTONDBLCLK:
        mouseX,mouseY = x,y   

cv2.namedWindow('image')
cv2.setMouseCallback('image',draw_circle)


while(1):
    
    mask = np.zeros(frame.shape[:2], dtype="uint8")
    p0 = (mouseX - rw//2, mouseY- rh//2)
    pf = (mouseX + rw//2, mouseY+ rh//2)
    cv2.rectangle(mask,p0,pf,255,-1)
    frame_masked = cv2.bitwise_and(frame_copy,frame_copy,mask=mask)

    frame = frame_copy.copy()
    cv2.rectangle(frame,p0,pf,(255,0,0),1)
    
    cv2.imshow('image', frame)
    cv2.imshow('image_masked', frame_masked)

    k = cv2.waitKey(1)
    # quit
    if k == ord('q'):
        break
    # reload
    elif k == ord('r'):
        frame = cv2.imread(image_path)    
    # reload
    elif k == ord('a'):
        print(mouseX,mouseY)
    # increase width
    elif k == ord('x'):
        rw+=1
    # decrease width
    elif k == ord('w'):
        rw-=1
    # increase height
    elif k == ord('j'):
        rh+=1
    # decrease height
    elif k == ord('h'):
        rh-=1
    elif k == ord('s'):
        # write file:
        with open(laser_point_masks_file, "a") as f:
            tof = f'{idx:d}\t{mouseX:d}\t{mouseY:d}\t{rw:d}\t{rh:d}\n'
            f.write(tof)
            print(tof)
        idx+=1

cv2.destroyAllWindows()