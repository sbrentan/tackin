import cv2
import numpy as np
from scipy import stats
import sys

def getPose(image):

    # load image as HSV and select saturation
    # pre_img = cv2.imread("/home/simone/tackin/cool_camera_image.jpg")
    pre_img = image


    lower = np.array([140, 140, 140])
    upper = np.array([170 ,170 ,170])

    mask = cv2.inRange(pre_img, lower, upper) # modify your thresholds
    inv_mask = cv2.bitwise_not(mask)
    img = cv2.bitwise_and(pre_img, pre_img, mask=inv_mask)


    hh, ww, cc = img.shape
    # print(hh, ww, cc)

    # convert to gray
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # threshold the grayscale image
    ret, thresh = cv2.threshold(gray,0,255,0)

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
    box = np.array([[xMin, yMax], [xMax, yMax], [xMax, yMin], [xMin, yMin]])
    result = img.copy()
    cv2.drawContours(result,[box],0,(0,0,255),2)


    # draw rotated rectangle on copy of img as result
    # result = img.copy()
    # cv2.drawContours(result,[box],0,(0,0,255),2)

    # get angle from rotated rectangle
    angle = rotrect[-1]

    # from https://www.pyimagesearch.com/2017/02/20/text-skew-correction-opencv-python/
    # the `cv2.minAreaRect` function returns values in the
    # range [-90, 0); as the rectangle rotates clockwise the
    # returned angle trends to 0 -- in this special case we
    # need to add 90 degrees to the angle
    if angle < -45:
        angle = -(90 + angle)
     
    # otherwise, just take the inverse of the angle to make
    # it positive
    else:
        angle = -angle

    angle = np.abs(angle)
    if(xMax - xMin > yMax - yMin and angle > 45):
        angle += 90
    elif(xMax - xMin < yMax - yMin and angle < 45):
        angle += 90



    xcenter = xMin + np.round((xMax - xMin)/2)
    ycenter = yMin + np.round((yMax - yMin)/2)

    # print(xcenter, ycenter, end=" asdf ")
    # print(xMax, xMin, yMax, yMin)

    # return np.abs(angle), [(xMax - xMin) - ww, (yMax - yMin) - hh]
    return angle, [np.round(ww/2) - xcenter, np.round(hh/2) - ycenter]

def getClass(image):
    print("Getting class of image")
    
    lower = np.array([140, 140, 140])
    upper = np.array([170 ,170 ,170])

    mask = cv2.inRange(image, lower, upper) # modify your thresholds
    inv_mask = cv2.bitwise_not(mask)
    image = cv2.bitwise_and(image, image, mask=inv_mask)

    image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

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
    print(m)
    m = 200 - m

    a = image[yMin:yMax, xMin:xMax]
    image[yMin:yMax, xMin:xMax] = np.where(a == 0, a, a + m)

    
    cv2.imwrite("gg.jpg", image)
    # cv2.imwrite("a.jpg", a)
    # sys.exit(0)


    methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR', 'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']
            # 'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']

    templates = ['X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-CHAMFER-2', 'X1-Y2-Z2-TWINFILLET', 
                 'X1-Y3-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y3-Z2-FILLET-2', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2', 
                 'X2-Y2-Z2-FILLET', 'X2-Y2-Z2-FILLET-1', 'X2-Y2-Z2-FILLET-2', 'X2-Y2-Z2-FILLET-3']

    results = {}
    for temp in templates:


        template = cv2.imread('/home/simone/tackin/src/test2_control/scripts/templates2/'+temp+'.jpg', 0)
        # meth = 'cv2.TM_CCOEFF_NORMED'

        results2 = []

        for meth in methods:

            method = eval(meth)
            # Apply template Matching
            res = cv2.matchTemplate(image,template,method)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

            # print(temp + " " +meth + " " + str(max_val))
            if(meth in ['cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']):
                results2.append(min_val)
            else:
                results2.append(max_val)

        results[temp] = results2

    return results