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


def init():

	global spawn_model, delete_model, model_srv

	rospy.init_node('supreme_spawner')
	rospy.wait_for_service("gazebo/spawn_sdf_model")
	spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

	rospy.wait_for_service("gazebo/delete_model")
	delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

	rospy.wait_for_service('/gazebo/get_model_state')
	model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

def spawn_bricks(nbricks, names = []):
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

		# YANGLES = []
		YANGLES = [np.pi/2, 0]
		if(not("FILLET" in brick or "CHAMFER" in brick or brick == "X1-Y1-Z2")):
			YANGLES.append(np.pi)

		with open("/home/simone/tackin/src/test2_gazebo/models/bricks/"+brick+"/model.sdf", "r") as f: brick_xml = f.read()

		r1 = round(np.random.uniform(0,1),2)
		r2 = round(np.random.uniform(0,1),2)
		r3 = round(np.random.uniform(0,1),2)
		if(r1 < 170 and r1 > 130 and r2 < 170 and r2 > 130 and r3 < 170 and r3 > 130):
			r3 += 60
		color = str(r1) + " " + str(r2) + " " + str(r3) + " 1"
		brick_xml = brick_xml.replace("COLOR", color)
		brick_xml = brick_xml.replace("BITMASK", str(2**(i+1)))


		req = SpawnModelRequest()
		req.model_name = "brick"+str(i+1)
		req.model_xml = brick_xml
		req.initial_pose.position.x = xpos
		req.initial_pose.position.y = ypos
		req.initial_pose.position.z = 0.05

		angle = np.random.randint(0, 360)
		yangle = np.random.randint(0, len(YANGLES))

		q2 = tf.transformations.quaternion_from_euler(0.0, YANGLES[yangle], np.deg2rad(angle))
		req.initial_pose.orientation.x = q2[0]
		req.initial_pose.orientation.y = q2[1]
		req.initial_pose.orientation.z = q2[2]
		req.initial_pose.orientation.w = q2[3]
		spawn_model.call(req)

		positions.append([xpos, ypos])
		
		# rospy.sleep(0.5)

def get_xy_ground_pos(index):
	groundx = np.floor(index / 3) - 3
	groundy = (index % 3) - 3

	if(groundx >= -1):
		groundx = ((groundx+3) * 0.3)-0.07
	else:
		groundx = (groundx * 0.3)+0.05

	return groundx, (groundy * 0.3) + 0.05

def spawn_grounds():

	resp = model_srv("brick_ground1", "brick_ground1")
	if(resp.success == False):
		for i in range(len(BRICKS)):
			brick = BRICKS[i]
			with open("/home/simone/tackin/src/test2_gazebo/models/brick_grounds/"+brick+"_ground/model.sdf", "r") as f: ground_xml = f.read()
			req2 = SpawnModelRequest()
			req2.model_name = "brick_ground"+str(i+1)
			req2.model_xml = ground_xml

			req2.initial_pose.position.x, req2.initial_pose.position.y = get_xy_ground_pos(i)
			req2.initial_pose.position.z = 0
			spawn_model.call(req2)

def get_models(args):
	if(len(args) > 2):
		filename = args[2]
	else:
		filename = "/home/simone/tackin/src/test2_control/scripts/configuration.json"
	f = open(filename)

	names = []
	data = json.load(f)
	nbricks = len(data['bricks'])
	for brick in data['bricks']:
		names.append(brick['model'])
	f.close()
	return nbricks, names


if __name__ == '__main__':
	try:
		fixedpos = []
		init()
		spawn_grounds()
		if(len(sys.argv) > 1):

			if(sys.argv[1] == "n"):
				spawn_bricks(int(sys.argv[2]))

			elif(sys.argv[1] == "one" and len(sys.argv) > 3):
				fixedpos = [float(sys.argv[2]), float(sys.argv[3])]
				spawn_bricks(1)


			elif(sys.argv[1] == "file"):
				nbricks, names = get_models(sys.argv)
				spawn_bricks(nbricks, names)

		else:
			spawn_bricks(10)

	except rospy.ROSInterruptException:
		pass
