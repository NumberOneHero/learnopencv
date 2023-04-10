import numpy as np 
import cv2
from tqdm import tqdm

# Set the path to the images captured by the left and right cameras
pathL = "./data/stereoL/"
pathR = "./data/stereoR/"

print("Extracting image coordinates of respective 3D pattern ....\n")

# Termination criteria for refining the detected corners
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

img_ptsL = []
img_ptsR = []
obj_pts = []

for i in tqdm(range(1,46)):
	imgL = cv2.imread(pathL+"img%d.png"%i)
	imgR = cv2.imread(pathR+"img%d.png"%i)
	imgL_gray = cv2.imread(pathL+"img%d.png"%i,0)
	imgR_gray = cv2.imread(pathR+"img%d.png"%i,0)

	outputL = imgL.copy()
	outputR = imgR.copy()

	retR, cornersR =  cv2.findChessboardCorners(outputR,(9,6),None)
	retL, cornersL = cv2.findChessboardCorners(outputL,(9,6),None)

	if retR and retL:
		obj_pts.append(objp)
		cv2.cornerSubPix(imgR_gray,cornersR,(11,11),(-1,-1),criteria)
		cv2.cornerSubPix(imgL_gray,cornersL,(11,11),(-1,-1),criteria)
		cv2.drawChessboardCorners(outputR,(9,6),cornersR,retR)
		cv2.drawChessboardCorners(outputL,(9,6),cornersL,retL)
		cv2.imshow('cornersR',outputR)
		cv2.imshow('cornersL',outputL)
		cv2.waitKey(0)

		img_ptsL.append(cornersL)
		img_ptsR.append(cornersR)


print("Calculating left camera parameters ... ")
# Calibrating left camera
retL, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(obj_pts,img_ptsL,imgL_gray.shape[::-1],None,None,flags=cv2.CALIB_RATIONAL_MODEL)
hL,wL= imgL_gray.shape[:2]

#change the 1 to a 0 , investigate what it does
new_mtxL, roiL= cv2.getOptimalNewCameraMatrix(mtxL,distL,(wL,hL),-1,(wL,hL))



















print("Calculating right camera parameters ... ")
# Calibrating right camera
retR, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(obj_pts,img_ptsR,imgR_gray.shape[::-1],None,None,flags=cv2.CALIB_RATIONAL_MODEL)
hR,wR= imgR_gray.shape[:2]

new_mtxR, roiR= cv2.getOptimalNewCameraMatrix(mtxR,distR,(wR,hR),-1,(wR,hR))




# undistort
#im3=cv2.imread(pathR+"img%d.png"%30)

#dst = cv2.undistort(im3, mtxR, distR, None, new_mtxR)
# crop the image
#x, y, wL, hR = roiR
#dst = dst[y:y+hR, x:x+wR]
#while True:
#	cv2.imshow("im4",im3)
#	cv2.imshow('im3', dst)
#	cv2.waitKey(0)












print("Stereo calibration .....")
flags = 0
flags |= cv2.CALIB_FIX_INTRINSIC
# Here we fix the intrinsic camara matrixes so that only Rot, Trns, Emat and Fmat are calculated.
# Hence intrinsic parameters are the same 

criteria_stereo= (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


# This step is performed to transformation between the two cameras and calculate Essential and Fundamenatl matrix
retS, new_mtxL, distL, new_mtxR, distR, Rot, Trns, Emat, Fmat = cv2.stereoCalibrate(obj_pts,
                                                          img_ptsL,
                                                          img_ptsR,
                                                          new_mtxL,
                                                          distL,
                                                          new_mtxR,
                                                          distR,
                                                          imgL_gray.shape[::-1],
                                                          criteria_stereo,
                                                          flags)

# Once we know the transformation between the two cameras we can perform stereo rectification
# StereoRectify function
rectify_scale= 0 # if 0 image croped, if 1 image not croped
rect_l, rect_r, proj_mat_l, proj_mat_r, Q, roiL, roiR= cv2.stereoRectify(new_mtxL, distL, new_mtxR, distR,
                                                 imgL_gray.shape[::-1], Rot, Trns,
                                                 rectify_scale,(0,0))




# Use the rotation matrixes for stereo rectification and camera intrinsics for undistorting the image
# Compute the rectification map (mapping between the original image pixels and 
# their transformed values after applying rectification and undistortion) for left and right camera frames
Left_Stereo_Map= cv2.initUndistortRectifyMap(new_mtxL, distL, rect_l, proj_mat_l,
                                             imgL_gray.shape[::-1], cv2.CV_16SC2)
Right_Stereo_Map= cv2.initUndistortRectifyMap(new_mtxR, distR, rect_r, proj_mat_r,
                                              imgR_gray.shape[::-1], cv2.CV_16SC2)

# Applying stereo image rectification on the left image
Left_nice = cv2.remap(imgL_gray,
					  Left_Stereo_Map[0],
					  Left_Stereo_Map[1],
					  cv2.INTER_LANCZOS4,
					  cv2.BORDER_CONSTANT,
					  0)

# Applying stereo image rectification on the right image
Right_nice = cv2.remap(imgR_gray,
					   Right_Stereo_Map[0],
					   Right_Stereo_Map[1],
					   cv2.INTER_LANCZOS4,
					   cv2.BORDER_CONSTANT,
					   0)

while True:
	cv2.imshow("imgGL", imgL_gray)
	cv2.imshow("imgGR", imgR_gray)
	cv2.imshow("leftNice",Left_nice)
	cv2.imshow("rightNice",Right_nice)
	cv2.waitKey(0)


print("Saving paraeters ......")
cv_file = cv2.FileStorage("data/params_py.xml", cv2.FILE_STORAGE_WRITE)
cv_file.write("Left_Stereo_Map_x",Left_Stereo_Map[0])
cv_file.write("Left_Stereo_Map_y",Left_Stereo_Map[1])
cv_file.write("Right_Stereo_Map_x",Right_Stereo_Map[0])
cv_file.write("Right_Stereo_Map_y",Right_Stereo_Map[1])
cv_file.release()