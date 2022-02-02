#!/usr/bin/env python3

import time
import numpy as np

import json
import sys
import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel, SpawnModelRequest, SpawnModelResponse, GetModelState 
from geometry_msgs.msg import *

BRICKS = ['X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET', 
          'X1-Y3-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2', 'X2-Y2-Z2-FILLET']

# def sub_callback(state):
# 	model_sub.unregister()
# 	print(state)
# 	sys.exit(0)


# 	for i in range(nbricks):
	    
	

# 	waiting = False



def init(nbricks, names = []):

	rospy.init_node('supreme_spawner')
	rospy.wait_for_service("gazebo/spawn_sdf_model")
	spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

	rospy.wait_for_service("gazebo/delete_model")
	delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

	global model_srv, waiting

	rospy.wait_for_service('/gazebo/get_model_state')
	model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
	count = 1
	exists = True
	while(exists):
		model_name = "brick"+str(count)
		try:
			resp = model_srv(model_name, model_name)
			if(resp.success == True):
				delete_model("brick"+str(count))
			else:
				break
		except rospy.ServiceException:
			exists = False
			break

		count+=1


	positions = []
	for i in range(nbricks):

		while(True):
			ypos = np.random.uniform(0.1, 0.9)
			xpos = np.random.uniform(-0.9, 0.9)
			if(np.abs(xpos) <= 0.3 and ypos <= 0.3):
				continue
			found = False
			dist = 0.2
			for pos in positions:
				if(xpos >= pos[0]-dist and xpos <= pos[0]+dist and ypos >= pos[1]-dist and ypos <= pos[1]+dist):
					found = True
					break
			if(not found):
				break

		if(len(fixedpos) > 0):
			xpos = fixedpos[0]
			ypos = fixedpos[1]

		# print(xpos, ypos)
		# sys.exit(0)

		if(len(names) == 0):
			brick = BRICKS[np.random.randint(0,len(BRICKS))]
		else:
			brick = names[i]
		with open("/home/simone/tackin/src/test2_gazebo/models/bricks/"+brick+"/model.sdf", "r") as f: product_xml = f.read()

		color = str(round(np.random.uniform(0,1),2)) + " " + str(round(np.random.uniform(0,1),2)) + " " + str(round(np.random.uniform(0,1),2)) + " 1"
		product_xml = product_xml.replace("COLOR", color)


		req = SpawnModelRequest()
		req.model_name = "brick"+str(i+1)
		req.model_xml = product_xml
		req.initial_pose.position.x = xpos
		req.initial_pose.position.y = ypos
		req.initial_pose.position.z = 0

		angle = np.random.randint(0, 360)
		q2 = tf.transformations.quaternion_from_euler(0.0, 0.0, np.deg2rad(angle))
		req.initial_pose.orientation.x = q2[0]
		req.initial_pose.orientation.y = q2[1]
		req.initial_pose.orientation.z = q2[2]
		req.initial_pose.orientation.w = q2[3]
		spawn_model.call(req)

		positions.append([xpos, ypos])
		rospy.sleep(0.5)


if __name__ == '__main__':
	try:
		fixedpos = []
		if(len(sys.argv) > 1):
			if(sys.argv[1] == "n"):
				init(int(sys.argv[1]))

			elif(sys.argv[1] == "one" and len(sys.argv) > 3):
				fixedpos = [float(sys.argv[2]), float(sys.argv[3])]
				init(1)

			elif(sys.argv[1] == "file"):
				if(len(sys.argv) > 2):
					filename = sys.argv[2]
				else:
					filename = "/home/simone/tackin/src/test2_control/scripts/configuration.json"
				f = open(filename)

				names = []
				data = json.load(f)
				nbricks = len(data['bricks'])
				for brick in data['bricks']:
					names.append(brick['model'])
				f.close()
				print(nbricks, names)
				init(nbricks, names)

		else:
			init(10)

	except rospy.ROSInterruptException:
		pass
