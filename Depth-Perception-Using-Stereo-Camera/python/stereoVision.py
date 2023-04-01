import cv2 as cv
import cv2
import numpy as np
from urllib.request import urlopen
import numpy as np
import time
import imutils
from matplotlib import pyplot as plt

# Function for stereo vision and depth estimation
import triangulation as tri
import calibration

# Mediapipe for face detection
import mediapipe as mp
import time

mp_facedetector = mp.solutions.face_detection
mp_draw = mp.solutions.drawing_utils

# Open both cameras


pTime = 0

# change to your ESP32-CAM ip
url = "http://192.168.50.159:81/stream"
CAMERA_BUFFRER_SIZE = 18432
stream = urlopen(url)
bts = b''
i = 0


num=0



pTime1 = 0

# change to your ESP32-CAM ip
url1 = "http://192.168.50.246:81/stream"
CAMERA_BUFFRER_SIZE = 18432
stream1 = urlopen(url1)
bts1 = b''
i1 = 0


num1=0




def empty(a):
    pass

print("heree")









# Stereo vision setup parameters
frame_rate = 120    #Camera frame rate (maximum at 120 fps)
B = 3             #Distance between the cameras [cm]
f = 5          #Camera lense's focal length [mm]
alpha = 60   #Camera field of view in the horisontal plane [degrees]

jpgend = -1
jpgend1 = -1
# Main program loop with face detector and depth estimation using stereo vision
with mp_facedetector.FaceDetection(min_detection_confidence=0.7) as face_detection:

    while(True):

        try:
            while(jpgend == -1):



                bts += stream.read(CAMERA_BUFFRER_SIZE)


                jpghead = bts.find(b'\xff\xd8')

                jpgend = bts.find(b'\xff\xd9')

                if jpghead > -1 and jpgend > -1:
                    jpg = bts[jpghead:jpgend + 2]
                    bts = bts[jpgend + 2:]

                    img = cv.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv.IMREAD_UNCHANGED)
                    # img=cv.flip(img,0) #>0:垂直翻轉, 0:水平翻轉, <0:垂直水平翻轉
                    succes_right=True
                    # h,w=img.shape[:2]
                    # print('影像大小 高:' + str(h) + '寬：' + str(w))

                    # img2 = img

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

        jpgend = -1
        try:
            while(jpgend1 == -1):
                bts1 += stream1.read(CAMERA_BUFFRER_SIZE)
                jpghead1 = bts1.find(b'\xff\xd8')
                jpgend1 = bts1.find(b'\xff\xd9')
                if jpghead1 > -1 and jpgend1 > -1:
                    jpg1 = bts1[jpghead1:jpgend1 + 2]
                    bts1 = bts1[jpgend1 + 2:]
                    img1 = cv.imdecode(np.frombuffer(jpg1, dtype=np.uint8), cv.IMREAD_UNCHANGED)
                    # img=cv.flip(img,0) #>0:垂直翻轉, 0:水平翻轉, <0:垂直水平翻轉
                    succes_left=True
                    # h,w=img.shape[:2]
                    # print('影像大小 高:' + str(h) + '寬：' + str(w))

                    # img2 = img

                    cv.imshow("Image1", img1)







        except Exception as e:
            print("Error:" + str(e))
            bts1 = b''
            stream1 = urlopen(url1)
            continue

        k = cv.waitKey(5)
        # 按a拍照存檔
        if k & 0xFF == ord('a'):
            cv.imwrite(str(i) + ".jpg", img1)
            i1 = i1 + 1
        # 按q離開
        if k & 0xFF == ord('q'):
            break

        jpgend1 = -1
        frame_right = img
        frame_left = img1

    ################## CALIBRATION #########################################################

        frame_right, frame_left = calibration.undistortRectify(frame_right, frame_left)
        cv.imshow("FR", frame_right)
        cv.imshow("FL", frame_left)

        ########################################################################################

        # If cannot catch any frame, break
        if not succes_right or not succes_left:                    
            break

        else:

            start = time.time()
            
            # Convert the BGR image to RGB
            frame_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2RGB)
            frame_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2RGB)

            # Process the image and find faces
            results_right = face_detection.process(frame_right)
            results_left = face_detection.process(frame_left)

            # Convert the RGB image to BGR
            frame_right = cv2.cvtColor(frame_right, cv2.COLOR_RGB2BGR)
            frame_left = cv2.cvtColor(frame_left, cv2.COLOR_RGB2BGR)


            ################## CALCULATING DEPTH #########################################################

            center_right = 0
            center_left = 0

            if results_right.detections:
                for id, detection in enumerate(results_right.detections):
                    mp_draw.draw_detection(frame_right, detection)

                    bBox = detection.location_data.relative_bounding_box

                    h, w, c = frame_right.shape

                    boundBox = int(bBox.xmin * w), int(bBox.ymin * h), int(bBox.width * w), int(bBox.height * h)

                    center_point_right = (boundBox[0] + boundBox[2] / 2, boundBox[1] + boundBox[3] / 2)

                    cv2.putText(frame_right, f'{int(detection.score[0]*100)}%', (boundBox[0], boundBox[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,255,0), 2)


            if results_left.detections:
                for id, detection in enumerate(results_left.detections):
                    mp_draw.draw_detection(frame_left, detection)

                    bBox = detection.location_data.relative_bounding_box

                    h, w, c = frame_left.shape

                    boundBox = int(bBox.xmin * w), int(bBox.ymin * h), int(bBox.width * w), int(bBox.height * h)

                    center_point_left = (boundBox[0] + boundBox[2] / 2, boundBox[1] + boundBox[3] / 2)

                    cv2.putText(frame_left, f'{int(detection.score[0]*100)}%', (boundBox[0], boundBox[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,255,0), 2)




            # If no ball can be caught in one camera show text "TRACKING LOST"
            if not results_right.detections or not results_left.detections:
                cv2.putText(frame_right, "TRACKING LOST", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),2)
                cv2.putText(frame_left, "TRACKING LOST", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),2)

            else:
                # Function to calculate depth of object. Outputs vector of all depths in case of several balls.
                # All formulas used to find depth is in video presentaion
                depth = tri.find_depth(center_point_right, center_point_left, frame_right, frame_left, B, f, alpha)

                cv2.putText(frame_right, "Distance: " + str(round(depth,1)), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,255,0),3)
                cv2.putText(frame_left, "Distance: " + str(round(depth,1)), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,255,0),3)
                # Multiply computer value with 205.8 to get real-life depth in [cm]. The factor was found manually.
                print("Depth: ", str(round(depth,1)))



            end = time.time()
            totalTime = end - start

            fps = 1 / totalTime
            #print("FPS: ", fps)

            cv2.putText(frame_right, f'FPS: {int(fps)}', (20,450), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,255,0), 2)
            cv2.putText(frame_left, f'FPS: {int(fps)}', (20,450), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,255,0), 2)                                   


            # Show the frames
            cv2.imshow("frame right", frame_right) 
            cv2.imshow("frame left", frame_left)


            # Hit "q" to close the window
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


# Release and destroy all windows before termination


cv2.destroyAllWindows()