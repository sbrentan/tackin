import cv2
import numpy as np

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
    print(hh, ww, cc)

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

    xcenter = xMin + np.round((xMax - xMin)/2)
    ycenter = yMin + np.round((yMax - yMin)/2)

    print(xcenter, ycenter, end=" asdf ")
    print(xMax, xMin, yMax, yMin)

    # return np.abs(angle), [(xMax - xMin) - ww, (yMax - yMin) - hh]
    return np.abs(angle), [np.round(ww/2) - xcenter, np.round(hh/2) - ycenter]

# print(angle,"deg")

# write result to disk
# cv2.imwrite("wing2_rotrect.png", result)

# cv2.imshow("THRESH", thresh)
# cv2.imshow("RESULT", result)
# cv2.waitKey(0)
# cv2.destroyAllWindows()