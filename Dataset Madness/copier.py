import os,shutil,json

movdir = r"C:\Users\alexr\Desktop\robotica dataset\train"
basedir = r"C:\Users\alexr\Desktop\robotica dataset\datatrain"
size = 1024
# Walk through all files in the directory that contains the files to copy
i = 0
if not os.path.exists(basedir+"\\images"):
    os.makedirs(basedir+"\\images")
if not os.path.exists(basedir+"\\labels"):
    os.makedirs(basedir+"\\labels")

names = [ 'X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET', 'X1-Y2-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y3-Z2', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2-FILLET', 'X2-Y2-Z2' ] # class names
for root, dirs, files in os.walk(movdir):
    for directory in dirs:
        dirName= "".join([basedir,"\\",directory])
        for root2,dirs2, files2 in os.walk("".join([movdir,"\\",directory])):
            for filename in files2:
                # Separate base from extension
                base, extension = os.path.splitext(filename)
                if(len(base.split("_"))==1):
                    # I use absolute path, case you want to move several dirs.
                    old_name = "".join([movdir,"\\",directory,"\\", filename])

                    # Initial new name
                    if(extension!=".json"):
                        new_name = "".join([basedir,"\\images\\",str(i), extension])
                        shutil.copy(old_name, new_name)
                    else:
                        with open(old_name, 'r') as data_file:
                            data = json.load(data_file)
                            fo = open("".join([basedir,"\\labels\\",str(i),".txt"]),"w+")

                            for objs in data:
                                pos = data[objs]["3d_bbox_pixel_space"]
                                xmin = ymin = size
                                xmax = ymax = 0
                                for x in pos:
                                    if(x[0]>xmax):
                                        xmax = x[0]
                                    if(x[0]<xmin):
                                        xmin = x[0]
                                    if(x[1]>ymax):
                                        ymax = x[1]
                                    if(x[1]<ymin):
                                        ymin = x[1]
                                name = names.index(data[objs]["y"])
                                xcenter = (xmax+xmin)/2/size
                                ycenter = (ymax+ymin)/2/size
                                width =   (xmax-xmin)/size
                                height =  (ymax-ymin)/size
                                line = [str(name),str(xcenter),str(ycenter),str(width),str(height),"\n"]
                                fo.writelines(" ".join(line))
                            fo.close()
                        i+=1
print("done")