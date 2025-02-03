import cv2
import time
import numpy as np

cal_folder = './calibration/1/'
file_name = cal_folder + 'servos.txt'

# read file:
with open(file_name) as f:
    data = f.read()

for row in data.split('\n'):
    
    # read data in row:
    dt = row.split('\t')
    if len(dt)<3:
        break
    print(row)
    
    # load image
    img = cv2.imread(cal_folder + f'{dt[4]}.png')

    # thresolding image:
    # convert to hsv, and apply a mask.
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red = np.array([80,0,0])
    upper_red = np.array([255,255,255])
    mask = cv2.inRange(img_hsv, lower_red, upper_red)

    img_result = cv2.bitwise_and(img, img, mask=mask)


    cv2.imshow('data', img)
    cv2.imshow('mask', mask)
    cv2.imshow('result', img_result)
    
    
    
    cv2.waitKey(0)
    if cv2.waitKey(1)==ord('q'):
        break
    time.sleep(1)
    

cv2.destroyAllWindows()
print(data)
