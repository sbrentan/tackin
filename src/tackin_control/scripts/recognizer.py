import cv2
import numpy as np
from scipy import stats
import sys


BRICKS = ['X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET', 
          'X1-Y3-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2', 'X2-Y2-Z2-FILLET']


def getPose(image):
    pre_img = image

    lower = np.array([140, 140, 140])
    upper = np.array([170 ,170 ,170])

    mask = cv2.inRange(pre_img, lower, upper) # modify your thresholds
    inv_mask = cv2.bitwise_not(mask)
    img = cv2.bitwise_and(pre_img, pre_img, mask=inv_mask)

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

    # get angle from rotated rectangle
    angle = rotrect[-1]

    if angle < -45:
        angle = -(90 + angle)
    else:
        angle = -angle

    angle = np.abs(angle)
    if(xMax - xMin > yMax - yMin and angle > 45):
        angle += 90
    elif(xMax - xMin < yMax - yMin and angle < 45):
        angle += 90

    M = cv2.moments(cnt)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])

    xcenter = xMin + np.round((xMax - xMin)/2)
    ycenter = yMin + np.round((yMax - yMin)/2)

    return angle, [np.round(ww/2) - xcenter, np.round(hh/2) - ycenter], [xMin, xMax, yMin, yMax], [cX, cY]

def filterImage(image):
    lower = np.array([140, 140, 140])
    upper = np.array([170 ,170 ,170])

    mask = cv2.inRange(image, lower, upper)
    inv_mask = cv2.bitwise_not(mask)
        
    ret, thresh = cv2.threshold(inv_mask,0,255,0)

    contours,h = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    contours= sorted(contours, key=cv2.contourArea, reverse= True)
    cnt= contours[0]

    mask2 = np.zeros(image.shape, np.uint8)
    mask2 = cv2.cvtColor(mask2, cv2.COLOR_RGB2GRAY)
    cv2.fillPoly(mask2, pts =[cnt], color=(255))

    image = cv2.bitwise_and(image, image, mask=mask2)

    
    return image


def getClass(image, model):
    # print("Getting class of image")
    
    image = filterImage(image)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    m = stats.mode(image[np.where(image != 0)])[0][0]
    mx = np.max(image)
    m = mx - m
    image = np.where(image == 0, 0, image+m)


    methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR', 'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']

    # templates = ['X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-CHAMFER-2', 'X1-Y2-Z2-TWINFILLET', 
    #              'X1-Y3-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y3-Z2-FILLET-2', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2', 
    #              'X2-Y2-Z2-FILLET', 'X2-Y2-Z2-FILLET-1', 'X2-Y2-Z2-FILLET-2', 'X2-Y2-Z2-FILLET-3']

    templates = []
    if("X1-Y2-Z2-CHAMFER" == BRICKS[model]):
        templates += ['X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-CHAMFER-2']
    if("X1-Y3-Z2-FILLET" == BRICKS[model]):
        templates += ['X1-Y3-Z2-FILLET', 'X1-Y3-Z2-FILLET-2']
    if("X2-Y2-Z2-FILLET" == BRICKS[model]):
        templates += ['X2-Y2-Z2-FILLET', 'X2-Y2-Z2-FILLET-1', 'X2-Y2-Z2-FILLET-2', 'X2-Y2-Z2-FILLET-3']

    # orientation = ['up2', 'down2', 'opp2']

    results = {}
    for temp in templates:


        template = cv2.imread('/home/simone/tackin/src/tackin_control/scripts/templates2/'+temp+'.jpg', 0)

        results2 = []

        for meth in methods:

            method = eval(meth)
            # Apply template Matching
            res = cv2.matchTemplate(image,template,method)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

            if(meth in ['cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']):
                results2.append(min_val)
            else:
                results2.append(max_val)

        results[temp] = results2

    return results, templates