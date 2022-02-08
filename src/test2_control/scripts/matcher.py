import cv2 as cv
import numpy as np
import sys
from scipy import stats

img = cv.imread('/home/simone/tackin/cool_camera_image.jpg')
# img2 = img.copy()

# lower = np.array([140, 140, 140])
# upper = np.array([170 ,170 ,170])

# mask = cv.inRange(img2, lower, upper) # modify your thresholds
# inv_mask = cv.bitwise_not(mask)
# img = cv.bitwise_and(img2, img2, mask=inv_mask)

# hh, ww, cc = img2.shape
# # convert to gray
# gray = cv.cvtColor(img2, cv.COLOR_BGR2GRAY)

# # threshold the grayscale image
# ret, img2 = cv.threshold(gray,0,255,0)

# img2 = cv.cvtColor(img2, cv.COLOR_RGB2GRAY)





# template = cv.imread('/home/simone/tackin/gg.jpg')

# template = cv.cvtColor(template, cv.COLOR_RGB2GRAY)

lower = np.array([140, 140, 140])
upper = np.array([170 ,170 ,170])

mask = cv.inRange(img, lower, upper) # modify your thresholds
inv_mask = cv.bitwise_not(mask)
img = cv.bitwise_and(img, img, mask=inv_mask)

hh, ww, cc = img.shape
# convert to gray
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

# threshold the grayscale image
ret, thresh = cv.threshold(gray,0,255,0)

# find outer contour
cntrs = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
cntrs = cntrs[0] if len(cntrs) == 2 else cntrs[1]




# get rotated rectangle from outer contour
rotrect = cv.minAreaRect(max(cntrs, key = cv.contourArea))
box = cv.boxPoints(rotrect)
box = np.int0(box)
xMin = min(box[::-1, 0])
xMax = max(box[::-1, 0])
yMin = min(box[::-1, 1])
yMax = max(box[::-1, 1])
box = np.array([[xMin, yMax], [xMax, yMax], [xMax, yMin], [xMin, yMin]])
# cv.drawContours(thresh,[box],0,(0,0,255),2)

box = np.array([[xMin, yMax], [xMax, yMax], [xMax, yMin], [xMin, yMin]])

m = stats.mode(img[yMin:yMax, xMin:xMax])[0][0]
m = stats.mode(m)[0][0]
# print(m)
m = 200 - m

a = img[yMin:yMax, xMin:xMax]
img[yMin:yMax, xMin:xMax] = np.where(a == 0, a, a + m)

# yMax = yMax - int(((0.0855 - 0.0565) / 0.0855) * (yMax - yMin))

marco = img[yMin+2:yMax-1, xMin+2:xMax-1]

# cv.imwrite("templates/test2.jpg", img)
# cv.imshow("template", marco)
cv.imwrite("templates2/ppo.jpg", marco)
# cv.waitKey(0)
# cv.destroyAllWindows()

# cv.imwrite("templates/X2-Y2-Z2-FILLET-3.jpg", img)

sys.exit(0)



# 245 pixel sta a 0.095 a z = -0.2 





# w, h = template.shape[:2]
# All the 6 methods for comparison in a list
methods = ['cv.TM_CCOEFF', 'cv.TM_CCOEFF_NORMED', 'cv.TM_CCORR']
            # 'cv.TM_CCORR_NORMED', 'cv.TM_SQDIFF', 'cv.TM_SQDIFF_NORMED']

templates = ['X1-Y1-Z2.jpg', 'X1-Y2-Z1.jpg', 'X1-Y2-Z2.jpg', 'X1-Y3-Z2-FILLET.jpg', 'X2-Y2-Z2.jpg', 'X2-Y2-Z2-FILLET.jpg']
for temp in templates:


    template = cv.imread('/home/simone/tackin/src/test2_control/scripts/templates'+temp, 0)
    # meth = 'cv.TM_CCOEFF_NORMED'

    for meth in methods:

        method = eval(meth)
        # Apply template Matching
        res = cv.matchTemplate(img,template,method)
        min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)

        print(temp + " " +meth + " " + str(max_val))