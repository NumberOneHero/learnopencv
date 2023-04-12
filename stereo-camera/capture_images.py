import numpy as np
import cv2
import time

import sys
sys.path.insert(1,'C://Users//Ruben Silva//Documents//GitHub//learnopencv//Depth-Perception-Using-Stereo-Camera//python')
from MyFunctions import *



output_path = "./data/"

start = time.time()
T = 10
count = 0

while True:
    timer = T - int(time.time() - start)
    btsL, imgL, retL = Esp32Frame(urlLeft, imgL, btsL, retL)
    btsR, imgR, retR = Esp32Frame(urlRight, imgR, btsR, retR)

    if (retR == True) and (retL == True):
        img1_temp = imgL.copy()
        cv2.putText(img1_temp,"%r"%timer,(50,50),1,5,(55,0,0),5)
        cv2.imshow('imgR',imgR)
        cv2.imshow('imgL',img1_temp)

        grayR= cv2.cvtColor(imgR,cv2.COLOR_BGR2GRAY)
        grayL= cv2.cvtColor(imgL,cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        CL, cornersR = cv2.findChessboardCorners(grayR,(9,6),None)
        CR, cornersL = cv2.findChessboardCorners(grayL,(9,6),None)

        # If corners are detected in left and right image then we save it.
        if (CL == True) and (CR == True) and timer <=1:
            count+=1
            cv2.imwrite(output_path+'stereoR/img%d.png'%count,imgR)
            cv2.imwrite(output_path+'stereoL/img%d.png'%count,imgL)

    if timer <=0:
        start = time.time()
    
    # Press esc to exit
    if cv2.waitKey(1) & 0xFF == 27:
        print("Closing the cameras!")
        break

# Release the Cameras

cv2.destroyAllWindows()