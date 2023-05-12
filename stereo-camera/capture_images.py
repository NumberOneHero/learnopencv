
import cv2

import numpy as np
import urllib.request
import time
pTime = 0

dTime = time.time()
sTime= 0


btsR= b''
btsL = b''
# change to your ESP32-CAM ip
urlLeft = "http://192.168.50.16/capture"
urlRight = "http://192.168.50.87/capture"
CAMERA_BUFFRER_SIZE = 15000

num=0
retL = False
retR = False
ret = None

imgR = None
imgL = None
def Esp32Frame(url,bts,ret):
    jpghead =-1
    jpgend = -1

    stream = urllib.request.urlopen(url)
    while (jpghead < 0 or jpgend < 0):
        bts += stream.read()

        if jpghead < 0 :
            jpghead = bts.find(b'\xff\xd8')
            print("jpghead < 0")
            print(jpghead)
        if jpgend < 0:
            print("jpgend < 0")

            jpgend = bts.find(b'\xff\xd9')
            print(jpgend)

        if jpghead > -1 and jpgend > -1:
            jpg = bts[jpghead:jpgend + 2]
            bts = bts[jpgend + 2:]
            print("got hereeeeeeeeee")
            print(jpgend)
            img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
            # img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)


            k = cv2.waitKey(1)
            ret = True
        else:
            ret = False

    return bts,img,ret


output_path = "./data/"

start = time.time()
T = 5
count = 0
oneTurn = False

while True:
    print("START of code")
    sTime = time.time() - dTime
    print(sTime)
    dTime = time.time()


    timer = T - int(time.time() - start)

    btsL, imgL, retL = Esp32Frame(urlLeft, btsL, retL)
    btsR, imgR, retR = Esp32Frame(urlRight, btsR, retR)
    print("after images ")
    sTime = time.time() - dTime
    print(sTime)
    dTime = time.time()
    if (retR == True) and (retL == True):
        img1_temp = imgL.copy()
        cv2.putText(img1_temp,"%r"%timer,(50,50),1,5,(0,255,0),5)
        cv2.imshow('imgR',imgR)
        cv2.imshow('imgL',img1_temp)

        grayR= cv2.cvtColor(imgR,cv2.COLOR_BGR2GRAY)
        grayL= cv2.cvtColor(imgL,cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        CL, cornersR = cv2.findChessboardCorners(grayR,(9,6),None)
        CR, cornersL = cv2.findChessboardCorners(grayL,(9,6),None)

        # If corners are detected in left and right image then we save it.
        if (CL == True) and (CR == True) and timer <=1 and oneTurn == False:
            count+=1
            cv2.imwrite(output_path+'stereoR/img%d.png'%count,imgR)
            cv2.imwrite(output_path+'stereoL/img%d.png'%count,imgL)
            oneTurn = True
        
    if timer > 1 and timer <= 5 and timer <= 0:
        oneTurn =False
    if timer <=0:
        start = time.time()
    
    # Press esc to exit
    if cv2.waitKey(1) & 0xFF == 27:
        print("Closing the cameras!")
        break
    print("end of code")
    sTime = time.time() - dTime
    print(sTime)
    dTime = time.time()
# Release the Cameras

cv2.destroyAllWindows()