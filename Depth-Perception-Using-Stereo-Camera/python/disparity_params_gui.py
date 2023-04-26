import cv2
import numpy as np
from urllib.request import urlopen

pTime = 0





btsR= b''
btsL = b''
# change to your ESP32-CAM ip
urlLeft = "http://192.168.137.231:81/stream"
urlRight = "http://192.168.137.35:81/stream"
CAMERA_BUFFRER_SIZE = 1024
streamLeft = urlopen(urlLeft)
streamRight = urlopen(urlRight)
num=0
retL = False
retR = False
ret = None

imgR = None
imgL = None
def Esp32Frame(stream,bts,ret):
	jpghead = -1
	jpgend = -1

	while (jpghead < 0 or jpgend < 0):
		bts += stream.read(CAMERA_BUFFRER_SIZE)

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
			img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
				# h,w=img.shape[:2]
				# print('影像大小 高:' + str(h) + '寬：' + str(w))
				# img2 = img

			k = cv2.waitKey(5)
			ret = True



		else:
			ret= False



	return bts , img,ret












# Reading the mapping values for stereo image rectification
cv_file = cv2.FileStorage("params_py.xml", cv2.FILE_STORAGE_READ)
Left_Stereo_Map_x = cv_file.getNode("Left_Stereo_Map_x").mat()
Left_Stereo_Map_y = cv_file.getNode("Left_Stereo_Map_y").mat()
Right_Stereo_Map_x = cv_file.getNode("Right_Stereo_Map_x").mat()
Right_Stereo_Map_y = cv_file.getNode("Right_Stereo_Map_y").mat()
cv_file.release()

def nothing(x):
    pass

cv2.namedWindow('disp',cv2.WINDOW_NORMAL)
cv2.resizeWindow('disp',600,600)

cv2.createTrackbar('numDisparities','disp',1,17,nothing)
cv2.createTrackbar('blockSize','disp',5,50,nothing)
cv2.createTrackbar('preFilterType','disp',1,1,nothing)
cv2.createTrackbar('preFilterSize','disp',2,25,nothing)
cv2.createTrackbar('preFilterCap','disp',5,62,nothing)
cv2.createTrackbar('textureThreshold','disp',10,100,nothing)
cv2.createTrackbar('uniquenessRatio','disp',15,100,nothing)
cv2.createTrackbar('speckleRange','disp',0,100,nothing)
cv2.createTrackbar('speckleWindowSize','disp',3,25,nothing)
cv2.createTrackbar('disp12MaxDiff','disp',5,25,nothing)
cv2.createTrackbar('minDisparity','disp',5,25,nothing)

# Creating an object of StereoBM algorithm
stereo = cv2.StereoSGBM_create()

while True:

	# Capturing and storing left and right camera images
	btsL,imgL,retL = Esp32Frame(streamLeft,btsL,retL)

	btsR,imgR,retR = Esp32Frame(streamRight,btsR,retR)


	# Proceed only if the frames have been captured
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
		Left_nice = cv2.bilateralFilter(Left_nice, 30, 25, 25)
		# Applying stereo image rectification on the right image
		Right_nice= cv2.remap(imgR_gray,
							Right_Stereo_Map_x,
							Right_Stereo_Map_y,
							cv2.INTER_LANCZOS4,
							cv2.BORDER_CONSTANT,
							0)
		Right_nice = cv2.bilateralFilter(Right_nice, 30, 25, 25)
		# Updating the parameters based on the trackbar positions
		numDisparities = cv2.getTrackbarPos('numDisparities','disp')*16
		blockSize = cv2.getTrackbarPos('blockSize','disp')*2 + 5
		preFilterType = cv2.getTrackbarPos('preFilterType','disp')
		preFilterSize = cv2.getTrackbarPos('preFilterSize','disp')*2 + 5
		preFilterCap = cv2.getTrackbarPos('preFilterCap','disp')
		textureThreshold = cv2.getTrackbarPos('textureThreshold','disp')
		uniquenessRatio = cv2.getTrackbarPos('uniquenessRatio','disp')
		speckleRange = cv2.getTrackbarPos('speckleRange','disp')
		speckleWindowSize = cv2.getTrackbarPos('speckleWindowSize','disp')*2
		disp12MaxDiff = cv2.getTrackbarPos('disp12MaxDiff','disp')
		minDisparity = cv2.getTrackbarPos('minDisparity','disp')

		# Setting the updated parameters before computing disparity map
		stereo.setNumDisparities(numDisparities)
		stereo.setBlockSize(blockSize)
		#stereo.setPreFilterType(preFilterType)
		#stereo.setPreFilterSize(preFilterSize)
		stereo.setPreFilterCap(preFilterCap)
		#stereo.setTextureThreshold(textureThreshold)
		stereo.setUniquenessRatio(uniquenessRatio)
		stereo.setSpeckleRange(speckleRange)
		stereo.setSpeckleWindowSize(speckleWindowSize)
		stereo.setDisp12MaxDiff(disp12MaxDiff)
		stereo.setMinDisparity(minDisparity)

		# Calculating disparity using the StereoBM algorithm
		disparity = stereo.compute(Left_nice,Right_nice)
		# NOTE: compute returns a 16bit signed single channel image,
		# CV_16S containing a disparity map scaled by 16. Hence it
		# is essential to convert it to CV_32F and scale it down 16 times.

		# Converting to float32
		disparity = disparity.astype(np.float32)

		# Scaling down the disparity values and normalizing them
		disparity = (disparity/16.0 - minDisparity)/numDisparities

		# Displaying the disparity map
		cv2.imshow("disparity",disparity)
		cv2.imshow("left", Left_nice)
		cv2.imshow("right", Right_nice)
		# Close window using esc key
		if cv2.waitKey(1) == 27:
			break

	else:
		btsL,imgL,retL = Esp32Frame(streamLeft, imgL,btsL,retL)
		btsR,imgR,retR = Esp32Frame(streamRight, imgR,btsR,retR)



print("Saving depth estimation paraeters ......")

cv_file = cv2.FileStorage("../data/depth_estmation_params_py.xml", cv2.FILE_STORAGE_WRITE)
cv_file.write("numDisparities",numDisparities)
cv_file.write("blockSize",blockSize)
cv_file.write("preFilterType",preFilterType)
cv_file.write("preFilterSize",preFilterSize)
cv_file.write("preFilterCap",preFilterCap)
cv_file.write("textureThreshold",textureThreshold)
cv_file.write("uniquenessRatio",uniquenessRatio)
cv_file.write("speckleRange",speckleRange)
cv_file.write("speckleWindowSize",speckleWindowSize)
cv_file.write("disp12MaxDiff",disp12MaxDiff)
cv_file.write("minDisparity",minDisparity)
cv_file.write("M",39.075)
cv_file.release()