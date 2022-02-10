import cv2
import numpy as np
from scipy import stats
import sys

img = cv2.imread("/home/simone/image.jpg")

lower = np.array([140, 140, 140])
upper = np.array([170 ,170 ,170])

mask = cv2.inRange(img, lower, upper) # modify your thresholds
inv_mask = cv2.bitwise_not(mask)
img = cv2.bitwise_and(img, img, mask=inv_mask)

hh, ww, cc = img.shape

# convert to gray
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# threshold the grayscale image
ret, thresh = cv2.threshold(gray,0,255,0)

# find outer contour
cntrs = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cntrs = cntrs[0] if len(cntrs) == 2 else cntrs[1]

contours= sorted(cntrs, key=cv2.contourArea, reverse= True)
cnt= contours[0]

# get rotated rectangle from outer contour
rotrect = cv2.minAreaRect(max(cntrs, key = cv2.contourArea))
box = cv2.boxPoints(rotrect)
box = np.int0(box)

xMin = min(box[::-1, 0])
xMax = max(box[::-1, 0])
yMin = min(box[::-1, 1])
yMax = max(box[::-1, 1])
box = np.array([[xMin, yMax], [xMax, yMax], [xMax, yMin], [xMin, yMin]])
result = img.copy()
cv2.drawContours(result,[box],0,(0,0,255),2)

cv2.imshow("img", result)
cv2.waitKey(0)
cv2.destroyAllWindows()