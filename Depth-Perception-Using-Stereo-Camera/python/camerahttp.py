import cv2 as cv
import cv2
import numpy as np
from urllib.request import urlopen
import os
import datetime
import time
import sys
import mediapipe as mp
import cvzone





pTime = 0

# change to your ESP32-CAM ip
url = "http://192.168.137.179:81/stream"
CAMERA_BUFFRER_SIZE = 18432
stream = urlopen(url)
num=0
bts = b''
i = 0
fpsReader = cvzone.FPS()



def empty(a):
    pass







while True:





    try:
        bts += stream.read(CAMERA_BUFFRER_SIZE)
        jpghead = bts.find(b'\xff\xd8')
        jpgend = bts.find(b'\xff\xd9')

        if jpghead > -1 and jpgend > -1:
            jpg = bts[jpghead:jpgend + 2]
            bts = bts[jpgend + 2:]
            img = cv.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv.IMREAD_UNCHANGED)
            # img=cv.flip(img,0) #>0:垂直翻轉, 0:水平翻轉, <0:垂直水平翻轉
            # h,w=img.shape[:2]
            # print('影像大小 高:' + str(h) + '寬：' + str(w))
            #img2 = img

            k = cv2.waitKey(25)

            if k == 27:
                break
            elif k == ord('w'):  # wait for 's' key to save and exit
                #cv2.imwrite('images/stereoLeft/imageL' + str(num) + '.png', img)
                cv2.imwrite('images/stereoRight/imageR' + str(num) + '.png', img)
                print("images saved!")
                num += 1


            fps = fpsReader.update()


            #cTime = time.time()
            #fps = 1 / (cTime - pTime)
            #pTime = cTime

            cv.putText(img, str(int(fps)), (70, 50), cv.FONT_HERSHEY_PLAIN, 3, (255, 0, 0), 3)

            imgRGB = cv.cvtColor(img, cv.COLOR_BGR2RGB)
            
            #print(results.pose_landmarks)

            cv.imshow("Image", img)




    except Exception as e:
        print("Error:" + str(e))
        bts = b''
        stream = urlopen(url)
        continue

    k = cv.waitKey(5)
    # 按a拍照存檔
    if k & 0xFF == ord('a'):
        cv.imwrite(str(i) + ".jpg", img)
        i = i + 1
    # 按q離開
    if k & 0xFF == ord('q'):
        break



cv.destroyAllWindows()