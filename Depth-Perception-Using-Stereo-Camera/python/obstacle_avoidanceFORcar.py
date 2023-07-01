
import cv2
import socket
import json
import numpy as np
import urllib.request
import time

from urllib.request import urlopen








HOST = "192.168.50.113"
PORT= 100

move = {"N":4,"D1":35,"D2":35}
text2 = {"N":1,"D1":0,"D2":0,"D3":2}
# left = {"N":4,"D1":90,"D2":0}
# right = {"N":4,"D1":0,"D2":90}
left = {"N":102,"D1":3}
right = {"N":102,"D1":3}

Testing=300
ggg = 0
Heartbeat_time = 0
LastCommand = 0
go = 10

def current_milli_time():
    return round(time.time() * 1000)

def empty(a):
    pass












pTime = 0

btsR= b''
btsL = b''

urlLeft = "http://192.168.50.87/capture"
urlRight = "http://192.168.50.16/capture"
CAMERA_BUFFRER_SIZE = 1024

num=0
retL = False
retR = False
streamLeft = None
streamRight =None


imgR = None
imgL = None


def Esp32Frame(stream,bts,ret):
    jpghead =-1
    jpgend = -1


    while (jpghead < 0 or jpgend < 0):
        bts += stream.read()

        if jpghead < 0 :
            jpghead = bts.find(b'\xff\xd8')

        if jpgend < 0:


            jpgend = bts.find(b'\xff\xd9')


        if jpghead > -1 and jpgend > -1:
            jpg = bts[jpghead:jpgend + 2]
            bts = bts[jpgend + 2:]

            img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
            # img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)



            ret = True
        else:
            ret = False

    return bts,img,ret






# Reading the mapping values for stereo image rectification
cv_file = cv2.FileStorage("params_py.xml", cv2.FILE_STORAGE_READ)
Left_Stereo_Map_x = cv_file.getNode("Left_Stereo_Map_x").mat()
Left_Stereo_Map_y = cv_file.getNode("Left_Stereo_Map_y").mat()
Right_Stereo_Map_x = cv_file.getNode("Right_Stereo_Map_x").mat()
Right_Stereo_Map_y = cv_file.getNode("Right_Stereo_Map_y").mat()
cv_file.release()

disparity = None
depth_map = None

# These parameters can vary according to the setup
max_depth = 20 # maximum distance the setup can measure (in cm)
min_depth = 14# minimum distance the setup can measure (in cm)
depth_thresh = 16 # Threshold for SAFE distance (in cm)

# Reading the stored the StereoBM parameters
cv_file = cv2.FileStorage("../data/depth_estmation_params_py.xml", cv2.FILE_STORAGE_READ)
numDisparities = int(cv_file.getNode("numDisparities").real())
blockSize = int(cv_file.getNode("blockSize").real())
preFilterType = int(cv_file.getNode("preFilterType").real())
preFilterSize = int(cv_file.getNode("preFilterSize").real())
preFilterCap = int(cv_file.getNode("preFilterCap").real())
textureThreshold = int(cv_file.getNode("textureThreshold").real())
uniquenessRatio = int(cv_file.getNode("uniquenessRatio").real())
speckleRange = int(cv_file.getNode("speckleRange").real())
speckleWindowSize = int(cv_file.getNode("speckleWindowSize").real())
disp12MaxDiff = int(cv_file.getNode("disp12MaxDiff").real())
minDisparity = int(cv_file.getNode("minDisparity").real())
M = cv_file.getNode("M").real()
cv_file.release()

# mouse callback function
def mouse_click(event,x,y,flags,param):
	global Z
	if event == cv2.EVENT_LBUTTONDBLCLK:
		print("Distance = %.2f cm"%depth_map[y,x])	


cv2.namedWindow('disp',cv2.WINDOW_NORMAL)
cv2.resizeWindow('disp',600,600)
cv2.setMouseCallback('disp',mouse_click)

output_canvas = None

# Creating an object of StereoBM algorithm
stereo = cv2.StereoBM_create()

def obstacle_avoid(gg,goo,LC):

	# Mask to segment regions with depth less than threshold
	mask = cv2.inRange(depth_map,10,depth_thresh)

	# Check if a significantly large obstacle is present and filter out smaller noisy regions
	if np.sum(mask)/255.0 > 0.01*mask.shape[0]*mask.shape[1]:

		# Contour detection 


		contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		cnts = sorted(contours, key=cv2.contourArea, reverse=True)

		# Check if detected contour is significantly large (to avoid multiple tiny regions)
		if cv2.contourArea(cnts[0]) > 0.01*mask.shape[0]*mask.shape[1]:

			x,y,w,h = cv2.boundingRect(cnts[0])
			xCenter = round(x + (w / 2))
			yCenter = round(y+ (h/ 2))

			# finding average depth of region represented by the largest contour
			mask2 = np.zeros_like(mask)
			cv2.drawContours(mask2, cnts[0], 0, 255, -1)
			cv2.drawContours(output_canvas, cnts[0], -1, (0,150,0), 3)
			# Calculating the average depth of the object closer than the safe distance
			depth_mean, _ = cv2.meanStdDev(depth_map, mask=mask2)

			# Display warning text
			cv2.putText(output_canvas, "WARNING !", (x+5,y-40), 1, 2, (0,0,255), 2, 2)
			cv2.putText(output_canvas, "Object at", (x+5,y), 1, 2, (255,255,255), 2, 2)
			cv2.putText(output_canvas, "%.2f cm"%depth_mean, (x+5,y+40), 1, 2, (255,255,255), 2, 2)
			cv2.rectangle(output_canvas, (x, y), (x + w, y + h), (255, 0, 0), 4)
			cv2.circle(output_canvas, (xCenter, yCenter), radius=1, color=(0, 0, 255), thickness=1)

			if (goo < 4):
				goo = 4

			if (current_milli_time() - gg > Testing and goo >= 3 and goo < 7):
				LC = 1
				s.send(bytes(json.dumps(text2), encoding="utf-8"))
				print("STOP")
				goo += 1

				gg = current_milli_time()
			elif(current_milli_time() - gg > Testing and goo >= 7 ):
				if(xCenter < 320):
					s.send(bytes(json.dumps(right), encoding="utf-8"))
					goo = 4
				if(xCenter > 320):
					s.send(bytes(json.dumps(left), encoding="utf-8"))
					goo = 4






	else:
		cv2.putText(output_canvas, "SAFE!", (100,100),1,3,(0,255,0),2,3)

		if (goo > 0 ):
			goo -= 1

		if (current_milli_time() - gg > Testing and goo < 3):
			s.send(bytes(json.dumps(move), encoding="utf-8"))
			print("MOVE B****")

			gg = current_milli_time()
	cv2.imshow('output_canvas',output_canvas)
	cv2.imshow("mask",mask)

	return gg ,goo , LC







with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:


	s.connect((HOST, PORT))

	Heartbeat = "{Heartbeat}"
	print("connect")
	go = 0
	data = s.recv(1)
	print(data)
	print("after connect")

	pTime = 0






	while True:

		# print(current_milli_time() - ggg, "current time")


		# Capturing and storing left and right camera images
		streamLeft = urllib.request.urlopen(urlLeft)
		streamRight = urllib.request.urlopen(urlRight)

		btsL, imgL, retL = Esp32Frame(streamLeft, btsL, retL)
		btsR, imgR, retR = Esp32Frame(streamRight, btsR, retR)
		if (current_milli_time() - Heartbeat_time > 2000):
			print("HEARTBEAT before sent")
			s.send(bytes(json.dumps(Heartbeat), encoding="utf-8"))
			print("HEARTBEAT SENT")
			Heartbeat_time = current_milli_time()




		# cv2.imshow("leftREAL", imgL)
		# cv2.imshow("rightREAL", imgR)
		if retL and retR:


			imgR_gray = cv2.cvtColor(imgR,cv2.COLOR_BGR2GRAY)
			imgL_gray = cv2.cvtColor(imgL,cv2.COLOR_BGR2GRAY)

			# Applying stereo image rectification on the left image
			Left_nice= cv2.remap(imgL_gray,
								Left_Stereo_Map_x,
								Left_Stereo_Map_y,
								cv2.INTER_LANCZOS4,
								cv2.BORDER_CONSTANT,
								0)
			Left_nice = cv2.bilateralFilter(Left_nice, 5, 30, 30)		# Applying stereo image rectification on the right image
			output_canvas =  cv2.cvtColor(Left_nice, cv2.COLOR_GRAY2BGR)

			# Left_nice  = cv2.Canny(image=Left_nice, threshold1=100, threshold2=200)
			Right_nice= cv2.remap(imgR_gray,
								Right_Stereo_Map_x,
								Right_Stereo_Map_y,
								cv2.INTER_LANCZOS4,
								cv2.BORDER_CONSTANT,
								0)

			Right_nice = cv2.bilateralFilter(Right_nice, 5, 30, 30)
			# Right_nice = cv2.Canny(image=Right_nice, threshold1=1, threshold2=20)
			cv2.imshow("RIGHTNICE",Right_nice)
			# Setting the updated parameters before computing disparity map
			stereo.setNumDisparities(numDisparities)
			stereo.setBlockSize(blockSize)
			stereo.setPreFilterType(preFilterType)
			stereo.setPreFilterSize(preFilterSize)
			stereo.setPreFilterCap(preFilterCap)
			stereo.setTextureThreshold(textureThreshold)
			stereo.setUniquenessRatio(uniquenessRatio)
			stereo.setSpeckleRange(speckleRange)
			stereo.setSpeckleWindowSize(speckleWindowSize)
			stereo.setDisp12MaxDiff(disp12MaxDiff)
			stereo.setMinDisparity(minDisparity)

			# Calculating disparity using the StereoBM algorithm
			disparity = stereo.compute(Left_nice,Right_nice)
			# NOTE: compute returns a 16bit signed single channel image,
			# CV_16S containing a disparity map scaled by 16. Hence it
			# is essential to convert it to CV_16S and scale it down 16 times.

			# Converting to float32
			disparity = disparity.astype(np.float32)

			# Normalizing the disparity map
			disparity = (disparity/16.0 - minDisparity)/numDisparities

			depth_map = M/(disparity) # for depth in (cm)

			mask_temp = cv2.inRange(depth_map,min_depth,max_depth)
			depth_map = cv2.bitwise_and(depth_map,depth_map,mask=mask_temp)

			ggg , go , LastCommand = obstacle_avoid(ggg,go,LastCommand)

			# cv2.resizeWindow("disp",700,700)
			cv2.imshow("disp",disparity)

			if cv2.waitKey(1) == 27:
				break

		else:
			# Capturing and storing left and right camera images
			print("FAILED")
			retL = False
			retR = False

