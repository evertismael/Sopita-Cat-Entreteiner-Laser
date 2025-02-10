import cv2
import time

cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_BUFFERSIZE,1)
a=cam.get(cv2.CAP_PROP_BUFFERSIZE) 


cal_imgs_path = './cal_imgs'

idx = 0
while True:
    ret, frame = cam.read()
    cv2.imshow('cam',frame)

    pressed_key = cv2.waitKey(1)
    if pressed_key==ord('q'):
        break
    elif pressed_key==ord('c'):
        print('capturing image')
        filename = cal_imgs_path+'/'+str(idx)+'.png'
        cv2.imwrite(filename, frame)
        print(f'image {filename} save')
        time.sleep(1)
        idx+=1

cam.release()
cv2.destroyAllWindows()
