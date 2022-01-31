import numpy as np
import cv2
import sys

templates = ['X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-CHAMFER-2', 'X1-Y2-Z2-TWINFILLET', 
                 'X1-Y3-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y3-Z2-FILLET-2', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2', 
                 'X2-Y2-Z2-FILLET', 'X2-Y2-Z2-FILLET-1', 'X2-Y2-Z2-FILLET-2', 'X2-Y2-Z2-FILLET-3']

back = np.zeros((800,800,3), np.uint8)

for temp in templates:

	img = cv2.imread("templates/"+temp+".jpg")

	hh, ww, cc = img.shape
	print(img.shape)
	new = back.copy()
	new[100:hh+100, 100:ww+100] = img

	new = new[80:hh+120, 80:ww+120]

	cv2.imwrite("templates2/"+temp+".jpg", new)

	# cv2.imshow("back", back)
	# cv2.imshow("new", new)

	# cv2.waitKey(0)
	# cv2.destroyAllWindows()

	# sys.exit(0)