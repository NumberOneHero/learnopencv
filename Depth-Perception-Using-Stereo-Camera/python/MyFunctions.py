import cv2
import numpy as np
from urllib.request import urlopen

pTime = 0





btsR= b''
btsL = b''
# change to your ESP32-CAM ip
urlLeft = "http://192.168.137.170:81/stream"
urlRight = "http://192.168.137.66:81/stream"
CAMERA_BUFFRER_SIZE = 18432
streamLeft = urlopen(urlLeft)
streamRight = urlopen(urlRight)
num=0
retL = False
retR = False
ret = None

imgR = None
imgL = None
def Esp32Frame(stream,img,bts,ret):

	bts += stream.read(CAMERA_BUFFRER_SIZE)
	jpghead = bts.find(b'\xff\xd8')
	jpgend = bts.find(b'\xff\xd9')


	if jpghead > -1 and jpgend > -1:
		jpg = bts[jpghead:jpgend + 2]
		bts = bts[jpgend + 2:]

		img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
		img=cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
			# h,w=img.shape[:2]
			# print('影像大小 高:' + str(h) + '寬：' + str(w))
			# img2 = img

		k = cv2.waitKey(5)
		ret = True


	else:
		ret= False



	return bts , img,ret
