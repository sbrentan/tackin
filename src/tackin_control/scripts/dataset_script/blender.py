import bpy
import math, mathutils
import os, time
import json
import numpy as np
from mathutils.bvhtree import BVHTree

mode  = 'train' # FLAGS : train or val
objsPath = 'F:/Robotica/blends/'
imgPath = f'F:/Robotica/Dataset/images/{mode}/'
labelPath = f'F:/Robotica/Dataset/labels/{mode}/'


rotv = 3
picsNum = 900
# Number of objects in a scene

objsNum = 1
if objsNum > len(os.listdir(objsPath)):
    objsNum = len(os.listdir(objsPath)) 

cameraLens = 15
img_w=800
img_h=800

objNameList = []
materials = []

nondeletable = ["Camera","Light","light"]

def loadObjs():
    objsList = os.listdir(objsPath)
    for objName in objsList:
        file_path = os.path.join(objsPath, objName)
        objN = objName.split('.')[0]
        objNameList.append(objN)
        with bpy.data.libraries.load(file_path,link=False) as (data_from, data_to):
            data_to.objects = [name for name in data_from.objects if name.startswith(objN)]
        for i in range(100):
            MatName = "mat"+str(i)
            material = bpy.data.materials.new(MatName)
            material.use_nodes = True
            principled_bsdf = material.node_tree.nodes['Principled BSDF']
            if principled_bsdf is not None:
                rnadom = np.append(np.random.random((3,1)),(1))
                principled_bsdf.inputs[0].default_value = rnadom
            materials.append(material)


def changeObjs():
    for obj in bpy.context.collection.objects:
        if obj.name not in nondeletable:
            bpy.context.collection.objects.unlink(obj)
    nameList = []
    while len(nameList) < objsNum:
        #np.random.seed(time.process_time())
        obj = np.random.choice(bpy.data.objects)
        if not (obj.name in nameList) and obj.name not in nondeletable:
            bpy.context.collection.objects.link(obj)
            nameList.append(obj.name)


def loadCamera():
    light_data = bpy.data.lights.new(name="Light",type="POINT")
    light_data.energy = 27
    light_data.distance = 31
    light_object = bpy.data.objects.new('light', light_data)
    bpy.context.scene.collection.objects.link(light_object)

    camera_data = bpy.data.cameras.new(name='Camera')
    camera_data.lens = cameraLens
    camera_object = bpy.data.objects.new('Camera', camera_data)
    camera_object.rotation_euler[0] = math.pi/2
    bpy.context.scene.collection.objects.link(camera_object)
    for obj in bpy.data.objects:
        if obj.name not in nondeletable:
            obj.parent = bpy.data.objects['Camera'] 

def randomPos():
    bpy.context.scene.collection.objects[0].select_set(True)
    bpy.context.scene.collection.objects[0].location = (0,0,0)
    bpy.ops.object.randomize_transform(random_seed = np.random.randint(0,100), loc=(0.25,0,0.25))
    for obj in bpy.context.collection.objects:
        if obj.name not in nondeletable:
            obj.select_set(True)
            while True:              
                try:
                    scale = math.sqrt(max(obj.dimensions))*bpy.data.objects['Camera'].data.lens
                    obj.location = (0,0,-0.6)
                    break
                except:
                    continue              
            bpy.ops.object.randomize_transform(random_seed = np.random.randint(0,100), loc=(0.24,0.1,0), rot=(rotv,rotv,3.14159)) #loc=(0.24,0.1,0.05),
            obj.active_material = np.random.choice(materials)
        else:
            obj.rotation_euler[2] = 4*random.uniform(-0.7,0.7)
            
def snapIt(scene, idNum):
    for obj in bpy.context.collection.objects:
        if obj.name not in nondeletable:
            obj.select_set(False)
            
    imId = f'{imgPath}{idNum}.png'
    scene.render.filepath = (imId) 
    bpy.ops.render.render(write_still=True)
    
def clamp(x, minimum, maximum):
    return max(minimum, min(x, maximum))

def labelIt(idNum):
    # 2D Bbox
    scene = bpy.context.scene
    cam_ob = bpy.context.scene.camera
    imLabel2dPath = f'{labelPath}{idNum}.txt'
    with open(imLabel2dPath,'w',encoding='utf-8') as f2:  
        for obj in bpy.context.collection.objects:
            if obj.name == 'Camera':
                continue
            obj.select_set(True) 
            mat = cam_ob.matrix_world.normalized().inverted()
            depsgraph = bpy.context.evaluated_depsgraph_get()
            mesh_eval = obj.evaluated_get(depsgraph)
            me = mesh_eval.to_mesh()
            me.transform(obj.matrix_world)
            me.transform(mat)
            camera = cam_ob.data
            frame = [-v for v in camera.view_frame(scene=scene)[:3]]
            camera_persp = camera.type != 'ORTHO'
            lx = []
            ly = []
            for v in me.vertices:
                co_local = v.co
                z = -co_local.z

                if camera_persp:
                    if z == 0.0:
                        lx.append(0.5)
                        ly.append(0.5)
                    else:
                        frame = [(v / (v.z / z)) for v in frame]
                min_x, max_x = frame[1].x, frame[2].x
                min_y, max_y = frame[0].y, frame[1].y
                x = (co_local.x - min_x) / (max_x - min_x)
                y = (co_local.y - min_y) / (max_y - min_y)
                lx.append(x)
                ly.append(y)
            min_x = clamp(min(lx), 0.0, 1.0)
            max_x = clamp(max(lx), 0.0, 1.0)
            min_y = clamp(min(ly), 0.0, 1.0)
            max_y = clamp(max(ly), 0.0, 1.0)
            mesh_eval.to_mesh_clear()
            r = scene.render
            fac = r.resolution_percentage * 0.01
            dim_x = r.resolution_x * fac
            dim_y = r.resolution_y * fac
            if round((max_x - min_x) * dim_x) == 0 or round((max_y - min_y) * dim_y) == 0:
                return (0, 0, 0, 0)
            X = min_x * dim_x           # X
            Y = dim_y - max_y * dim_y    # Y
            Width = (max_x - min_x) * dim_x  # Width
            Height = (max_y - min_y) * dim_y   # Height            
            nX = (X + Width/2)/img_w
            nY = (Y + Height/2)/img_h
            nW = Width/img_w
            nH = Height/img_h
            
            # in radiants
            euX = obj.rotation_euler[0] 
            euY = obj.rotation_euler[1] 
            euZ = obj.rotation_euler[2] 
            euler_rotation = mathutils.Euler((euX, euY, euZ), 'XYZ')
            R = euler_rotation.to_matrix()
            
            if (R[2][2]) <= 0:
                r_y = -np.arctan(R[0][2]/R[2][2])
            elif (R[0][2]) <= 0:
                r_y = -np.pi-np.arctan(R[0][2]/R[2][2])
            else:
                r_y = np.pi-np.arctan(R[0][2]/R[2][2])
        
            locx = obj.location[0]
            locy = obj.location[1]
            locz = obj.location[2]
            
            dimx = obj.dimensions[0]
            dimy = obj.dimensions[1]
            dimz = obj.dimensions[2]
            beta = np.arctan(locx/locz) 
            alpha = (np.pi+beta)-(np.pi+r_y)
            objLabel2d = f'{objNameList.index(obj.name)} {nX} {nY} {nW} {nH}'
            obj.select_set(False)
            f2.write(objLabel2d+'\n')
        f2.close()
def calibCamera():
    cam = bpy.data.objects['Camera']
    camd = cam.data
    f_in_mm = camd.lens
    scene = bpy.context.scene
    resolution_x_in_px = scene.render.resolution_x
    resolution_y_in_px = scene.render.resolution_y
    scale = scene.render.resolution_percentage / 100
    sensor_width_in_mm = camd.sensor_width
    sensor_height_in_mm = camd.sensor_height
    pixel_aspect_ratio = scene.render.pixel_aspect_x / scene.render.pixel_aspect_y
    if (camd.sensor_fit == 'VERTICAL'):
        # the sensor height is fixed (sensor fit is horizontal), 
        # the sensor width is effectively changed with the pixel aspect ratio
        s_u = resolution_x_in_px * scale / sensor_width_in_mm / pixel_aspect_ratio 
        s_v = resolution_y_in_px * scale / sensor_height_in_mm
    else: # 'HORIZONTAL' and 'AUTO'
    # the sensor width is fixed (sensor fit is horizontal), 
        # the sensor height is effectively changed with the pixel aspect ratio
        pixel_aspect_ratio = scene.render.pixel_aspect_x / scene.render.pixel_aspect_y
        s_u = resolution_x_in_px * scale / sensor_width_in_mm
        s_v = resolution_y_in_px * scale * pixel_aspect_ratio / sensor_height_in_mm
  
        # Parameters of intrinsic calibration matrix K
        alpha_u = f_in_mm * s_u
        alpha_v = f_in_mm * s_v
        u_0 = resolution_x_in_px*scale / 2
        v_0 = resolution_y_in_px*scale / 2
        skew = 0 # only use rectangular pixels
        calList = [[f'P0: 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0\n'],
                   [f'P1: 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0\n'],
                   [f'P2: {alpha_u} {skew} {u_0} 0.0 0.0 {alpha_v} {v_0} 0.0 0.0 0.0 1.0 0.0\n'],
                   [f'P3: 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0\n'],
                   [f'R0_rect: 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0\n'],
                   [f'Tr_velo_to_cam: 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0\n'],
                   [f'Tr_imu_to_velo: 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0\n']]
        return calList
      
def clearAll():
    for obj in bpy.data.objects:
        bpy.data.objects.remove(obj)
    for img in bpy.data.images:
        bpy.data.images.remove(img)
    for ma in bpy.data.materials:
        bpy.data.materials.remove(ma)
    for me in bpy.data.meshes:
        bpy.data.meshes.remove(me)    
    for ng in bpy.data.node_groups:
        bpy.data.node_groups.remove(ng)
    for cd in bpy.data.cameras:
        bpy.data.cameras.remove(cd)

def checkOverlaps():
    bvh = []
    for obj in bpy.context.collection.objects:
        if obj.name not in nondeletable:
            vert = [obj.matrix_world @ v.co for v in obj.data.vertices] 
            poly = [p.vertices for p in obj.data.polygons]

            bvh.append(BVHTree.FromPolygons(vert, poly))
    for i in range(1,objsNum):
        for j in range(i+1,objsNum):
            if( bvh[i].name not in nondeletable and bvh[j].name not in nondeletable and bvh[i].overlap(bvh[j])):
                return True
    return False
                  
def main():
    clearAll()
    loadObjs()
    loadCamera()
    scene = bpy.context.scene
    scene.render.engine = 'BLENDER_EEVEE'
    scene.camera = scene.objects['Camera']
    scene.render.resolution_x = img_w
    scene.render.resolution_y = img_h
    bpy.data.worlds['World'].use_nodes = True
    bpy.data.worlds["World"].node_tree.nodes["Background"].inputs[0].default_value=[0.160,0.160,0.160,1]
    cont = 0
    for i in range(picsNum):
        over = True
        while over:
            changeObjs()
            randomPos()
            over = checkOverlaps()
            cont+=1
        cont-=1
        snapIt(scene, i)
        labelIt(i)
    print(cont)
if __name__ == '__main__':
    main()