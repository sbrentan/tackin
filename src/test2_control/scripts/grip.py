import cv2
import numpy as np
import sys
from scipy import stats


image = cv2.imread('/home/simone/tackin/brut.jpg')

# contours,h = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
# # contours = contours[0] if len(contours) == 2 else contours[1]

# final = np.zeros(img.shape,np.uint8)
# mask = np.zeros(img.shape,np.uint8)

# cv2.drawContours(mask, contours, -1, (255),1)
# inv_mask = cv2.bitwise_not(mask)
# final = cv2.bitwise_and(img, img, mask=inv_mask)

# m = 100
# final = np.where(final == 0, 0, final + m)





# print("Getting class of image")

# lower = np.array([140, 140, 140])
# upper = np.array([170 ,170 ,170])
    
# ret, thresh = cv2.threshold(image,0,255,0)
# thresh = cv2.cvtColor(thresh, cv2.COLOR_RGB2GRAY)
# contours,h = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)


# mask = cv2.inRange(image, lower, upper) # modify your thresholds


# contours= sorted(contours, key=cv2.contourArea, reverse= True)
# cnt= contours[0]

# cv2.drawContours(mask, cnt, -1, (255),1)
# inv_mask = cv2.bitwise_not(mask)
# image = cv2.bitwise_and(image, image, mask=inv_mask)

# image = np.where(image == 0, 130, image)

# cv2.imshow("final", image)

# cv2.waitKey(0)
# cv2.destroyAllWindows()

# sys.exit(0)














print("Getting class of image")

lower = np.array([140, 140, 140])
upper = np.array([180 ,180 ,180])

mask = cv2.inRange(image, lower, upper) # modify your thresholds
inv_mask = cv2.bitwise_not(mask)

ret, thresh = cv2.threshold(inv_mask,0,255,0)

# thresh = cv2.cvtColor(thresh, cv2.COLOR_RGB2GRAY)
contours,h = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

contours= sorted(contours, key=cv2.contourArea, reverse= True)
cnt= contours[0]

mask2 = np.zeros(image.shape, np.uint8)
mask2 = cv2.cvtColor(mask2, cv2.COLOR_RGB2GRAY)
cv2.fillPoly(mask2, pts =[cnt], color=(255))

# inv_mask = cv2.bitwise_not(mask)
image = cv2.bitwise_and(image, image, mask=mask2)
# grey = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)


# m = stats.mode(image[np.where(image != 0)])[0][0]
# mx = np.max(image)
# m = mx - m
# image = np.where(image == 0, 0, image+m)

image[np.all(image == (0, 0, 0), axis=-1)] = (130,130,130)
# image = np.where(image == 0, 130, image)
# cv2.imwrite("yoo.jpg", grey)

# image = cv2.bitwise_and(image, grey, mask=mask2)

# cv2.imshow("final", image)

# cv2.waitKey(0)
# cv2.destroyAllWindows()

# sys.exit(0)



















# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# threshold the grayscale image
ret, thresh = cv2.threshold(image,0,255,0)

# find outer contour
cntrs = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cntrs = cntrs[0] if len(cntrs) == 2 else cntrs[1]



# get rotated rectangle from outer contour
rotrect = cv2.minAreaRect(max(cntrs, key = cv2.contourArea))
box = cv2.boxPoints(rotrect)
box = np.int0(box)

xMin = min(box[::-1, 0])
xMax = max(box[::-1, 0])
yMin = min(box[::-1, 1])
yMax = max(box[::-1, 1])
# box = np.array([[xMin, yMax], [xMax, yMax], [xMax, yMin], [xMin, yMin]])

# print(image[yMin:yMax, xMin:xMax])

m = stats.mode(image[yMin:yMax, xMin:xMax])[0][0]
m = stats.mode(m)[0][0]
# print(m)
m = 200 - m



a = image[yMin:yMax, xMin:xMax]
image[yMin:yMax, xMin:xMax] = np.where(a == 0, a, a + m)



# cv2.imwrite("gg.jpg", image)













cv2.imshow("final", image)

cv2.waitKey(0)
cv2.destroyAllWindows()