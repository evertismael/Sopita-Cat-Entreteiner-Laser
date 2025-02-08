import cv2
import time
import numpy as np

cal_folder = './calibration/1/'
file_name = cal_folder + 'servos.txt'

# read file:
with open(file_name) as f:
    data = f.read()

pairs_iim = []
for row in data.split('\n'):    
    # read data in row:
    dt = row.split('\t')
    if len(dt)<3:
        break
    print(row)

    for i in range(1):
        # load image
        img = cv2.imread(cal_folder + f'{dt[4]}.png')

        # thresolding image:
        # convert to hsv, and apply a mask.
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_red = np.array([160,70,150])
        upper_red = np.array([180,255,255])
        mask = cv2.inRange(img_hsv, lower_red, upper_red)
        
        """ Use this part to remove unwanted parts"""
        # erase superior part:
        mask[0:290, 0:640] = 0
        mask[470:480, 0:640] = 0
        mask[0:480, 0:30] = 0

        # open and close:
        kernel = np.ones((2,2),np.uint8)
        closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        dilation = cv2.dilate(closing,kernel,iterations = 5)
        img_result = cv2.bitwise_and(img, img, mask=mask)


        # extract indexes:
        canny_edges = cv2.Canny(dilation,30,255)
        M = cv2.moments(canny_edges)
        if M['m00']==0:
            cx=-1
            cy=-1
        else:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

        cv2.circle(dilation, center=(cx,cy), radius=50, color=(255,255,255),thickness=1)

        print(i, cx, cy)
        cv2.imshow('data', img)
        cv2.imshow('mask', mask)
        cv2.imshow('result', dilation)
        
        #cv2.waitKey(0)
        if cv2.waitKey(1)==ord('q'):
            break
        time.sleep(0.1)
    pairs_iim.append((cx,cy,dt[2],dt[3],dt[4]))


hlines = [list(range(0,7)), list(range(7,14)), list(range(14,21))]
for hl in hlines: 
    # compute funtion for one line
    a=1


cv2.destroyAllWindows()
print(data)
