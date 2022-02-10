def create_dataset(nRepetitions,folder):
    rospy.wait_for_service("gazebo/delete_model")
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    os.chdir("/media/sf_bob/images/"+ folder +"/")

    x = 0.6
    y = 0.6
    z = 0.08
    nBlocks = len(blocks)
    fileName = 0

    compute_kinematik([2,x,y,-0.12])
    time.sleep(1)
    open_gripper()

    with tqdm(total=nBlocks*nRepetitions) as pbar:
        for i in range(nBlocks):
            time.sleep(1)
            for y in range(nRepetitions):
                
                os.system("rosrun test2_control spawner_dataset.py "+ str(i) + " " + str(0) + " 0.6 0.6")
                time.sleep(0.5)

                camera_image = CvBridge().imgmsg_to_cv2(last_image)
                
                if(createLabels(camera_image,i,str(fileName),folder)):
                    cv2.imwrite(str(fileName)+".jpg", camera_image)

                delete_model("brick")
                time.sleep(0.5)
                fileName += 1
                pbar.update(1)
    pbar.close()

def createLabels(image,blockId,fileName,folder):

    # load image as HSV and select saturation
    # pre_img = cv2.imread("/home/simone/tackin/cool_camera_image.jpg")
    pre_img = image


    lower = np.array([135, 135, 135])
    upper = np.array([175 ,175 ,175])

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

    xMin = min(map(lambda x : x[0][0], cntrs[0]))
    xMax = max(map(lambda x : x[0][0], cntrs[0]))
    yMin = min(map(lambda x : x[0][1], cntrs[0]))
    yMax = max(map(lambda x : x[0][1], cntrs[0]))

    # box = np.array([[xMin, yMax], [xMax, yMax], [xMax, yMin], [xMin, yMin]])
    # cv2.drawContours(img,[box],0,(0,0,255),2)
    # cv2.imwrite("X-"+fileName+".jpg", img)

    blockClass = blockId
    xcenter = (xMax + xMin) / 2 / 800
    ycenter = (yMax + yMin) / 2 / 800
    width = (xMax - xMin) / 800
    height = (yMax - yMin) / 800

    if(height < 0.03 or width < 0.03):
        return False

    f = open("/media/sf_bob/labels/"+folder+"/"+fileName+".txt","w")
    f.write(str(blockClass)+ " " +str(xcenter)+ " " +str(ycenter)+ " " +str(width)+ " " +str(height))
    f.close()
    return True
