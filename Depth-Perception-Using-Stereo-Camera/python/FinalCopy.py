import cv2 as cv
import socket
import json
import numpy as np
from urllib.request import urlopen
import os
import datetime
import time
import sys

import mediapipe as mp

mpFaceDetection = mp.solutions.face_detection
mpDraw = mp.solutions.drawing_utils
faceDetection = \
    mpFaceDetection.FaceDetection()

# change to your ESP32-CAM ip
url = "http://192.168.4.1:81/stream"
CAMERA_BUFFRER_SIZE = 18432
stream = urlopen(url)
bts = b''
i = 0

def current_milli_time():
    return round(time.time() * 1000)




def empty(a):
    pass



l_h, l_s, l_v = 10, 75, 55
u_h, u_s, u_v = 40, 178, 215


HOST = "192.168.4.1"
PORT= 100

move = {"N":4,"D1":230,"D2":230}
text2 = {"N":1,"D1":0,"D2":0,"D3":2}
left = {"N":4,"D1":210,"D2":20}
right = {"N":4,"D1":20,"D2":210}



Testing=50
ggg = 0
Heartbeat_time = 0
Motor_Time = 0

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    Heartbeat = "{Heartbeat}"

    go = 0
    data = s.recv(1024)
    print(data)
    stop = False
    pTime=0


    while True:
        Turn = True
        print(current_milli_time() - ggg,"current time")



        try:
            bts += stream.read(CAMERA_BUFFRER_SIZE)
            jpghead = bts.find(b'\xff\xd8')
            jpgend = bts.find(b'\xff\xd9')
            if jpghead > -1 and jpgend > -1:
                jpg = bts[jpghead:jpgend + 2]
                bts = bts[jpgend + 2:]
                img = cv.imdecode(np.frombuffer(jpg, dtype=np.uint8), -1)
                # img=cv.flip(img,0) #>0:垂直翻轉, 0:水平翻轉, <0:垂直水平翻轉
                # h,w=img.shape[:2]
                # print('影像大小 高:' + str(h) + '寬：' + str(w))
                frame = cv.resize(img, (640, 480))

                imgRGB = cv.cvtColor(img, cv.COLOR_BGR2RGB)

                hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

                l_b = np.array([l_h, l_s, l_v])
                u_b = np.array([u_h, u_s, u_v])

                mask = cv.inRange(hsv, l_b, u_b)
                cnts, hierarchy_ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
                if cnts:
                    for c in cnts:
                        area = cv.contourArea(c)

                        if area > 1000:
                            cv.drawContours(frame, [c], -1, (255, 0, 0), 3)
                            M = cv.moments(c)
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m"
                                                  "00"])
                            print(cx, cy, "LOCATION ")
                            print(area,"-AREA")
                            cv.circle(frame, (cx, cy), 7, (255, 255, 255), -1)

                            if area <= 10000:

                                if cx <= 250 and go != 250:
                                    if (current_milli_time() - ggg > Testing):
                                        s.send(bytes(json.dumps(left), encoding="utf-8"))
                                        go = 250
                                        print(go, "speeeeeeeeed 250")
                                        ggg = current_milli_time()
                                if cx >= 390 and go != 390:
                                    if (current_milli_time() - ggg > Testing):
                                        s.send(bytes(json.dumps(right), encoding="utf-8"))
                                        go = 390
                                        print(go, "speeeeeeeeed 390")
                                        ggg = current_milli_time()
                                if cx > 250 and cx < 390 and go != 800:
                                    if (current_milli_time() - ggg > Testing):
                                        s.send(bytes(json.dumps(move), encoding="utf-8"))
                                        go = 800
                                        print(go, "speeeeeeeeed 800")

                                        ggg = current_milli_time()

                                # s.send(bytes(json.dumps(text1), encoding="utf-8"))

                                stop = True
                                Turn = False
                            elif stop == True:
                                if (current_milli_time() - ggg > Testing):
                                    s.send(bytes(json.dumps(text2), encoding="utf-8"))
                                    print("sent 00000000000000000")
                                    stop = False
                                    Turn = False
                                    ggg = current_milli_time()

                            break

                    if Turn == True and stop == True:
                        if (current_milli_time() - ggg > Testing):
                            stop = False
                            s.send(bytes(json.dumps(text2), encoding="utf-8"))
                            print("sent 00000000000000000")
                            go = 0
                            ggg = current_milli_time()
                else:
                    if Turn == True and stop == True:
                        if (current_milli_time() - ggg > Testing):
                            stop = False
                            s.send(bytes(json.dumps(text2), encoding="utf-8"))
                            print("sent 00000000000000000")
                            go = 0
                            ggg = current_milli_time()


                if (current_milli_time() - Heartbeat_time > 2500):
                    s.send(bytes(json.dumps(Heartbeat), encoding="utf-8"))
                    print("HEARTBEAT SENT")
                    Heartbeat_time = current_milli_time()

                res = cv.bitwise_and(frame, frame, mask=mask)
                cv.line(frame, (250, 0), (250, 480), (0, 255, 0), 1)
                cv.line(frame, (390, 0), (390, 480), (0, 255, 0), 1)
                BIGGER =cv.resize(frame,(1280,720))

                cTime = time.time()
                fps = 1 / (cTime - pTime)
                pTime = cTime
                cv.putText(BIGGER, f"fps:{int(fps)}", (40, 70), cv.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 3)

                results = faceDetection.process(imgRGB)

                print(results)
                if results.detections:
                    for id, detection in enumerate(results.detections):
                        mpDraw.draw_detection(img, detection)
                        print(id, detection)

                cv.imshow("FACE DETECTION", img)
                cv.imshow("live transmission", BIGGER)




        except Exception as e:
            print("Error:" + str(e))
            bts = b''
            stream = urlopen(url)
            continue

        k = cv.waitKey(1)
        # 按a拍照存檔
        if k & 0xFF == ord('a'):
            cv.imwrite(str(i) + ".jpg", frame)
            i = i + 1
        # 按q離開
        if k & 0xFF == ord('q'):
            break

cv.destroyAllWindows()