#!/usr/bin/env python
import rospy
import numpy as np
from irpos import *
from std_msgs.msg import *
import threading
import PyKDL

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
import math

y = 0
homogVector = []
pointInCheckerFrame = np.matrix('0.0;0.0;0.0;1.0')
newPoint = [0,0,0,0]
realHomogMatrix = np.zeros(shape=(4,4))
publisher = None
foundLock = threading.Lock()
homogLock = threading.Lock()

def readHomogVector(msg):
	homogLock.acquire()
	global y
	y = msg.data
	global realHomogMatrix
	realHomogMatrix = np.array([[1,0,0,0.9],[0,1,0,2.2+y/4],[0,0,1,0.9],[0,0,0,1]])
	homogLock.release()

def calculateNewPoint():
	global newPoint
	newPoint = realHomogMatrix * pointInCheckerFrame
	
def onInit():
	rospy.Subscriber('realHomogMatrix',Float32, readHomogVector)
	topic = 'realGrid'
	global publisher
	publisher = rospy.Publisher(topic, Marker,queue_size=10)
	rospy.init_node('realGrid')
	
def realGrid():
	pawn = Marker()
	pawn.header.frame_id = "/world"
	pawn.type = pawn.CUBE
	pawn.action = pawn.ADD
	
	global realHomogMatrix
	T = PyKDL.Frame(PyKDL.Rotation(realHomogMatrix[0][0], realHomogMatrix[0][1], realHomogMatrix[0][2],realHomogMatrix[1][0], realHomogMatrix[1][1], realHomogMatrix[1][2], realHomogMatrix[2][0], realHomogMatrix[2][1], realHomogMatrix[2][2]))
	print realHomogMatrix
    
	q = T.M.GetQuaternion()
	pawn.pose = Pose(Point(newPoint[0],newPoint[1],newPoint[2]), Quaternion(q[0],q[1],q[2],q[3]))
	pawn.color.a = 1.0
	pawn.color.r = 1.0
	pawn.color.g = 1.0
	pawn.color.b = 0.0
	pawn.scale.x = 0.2
	pawn.scale.y = 0.4
	pawn.scale.z = 0.01
	
	
	pawn.scale.x = 0.2
	pawn.scale.y = 0.4
	pawn.scale.z = 0.01
		
	publisher.publish(pawn)
	rospy.sleep(0.002)

if __name__ == '__main__':
	onInit()
	while not rospy.is_shutdown():
		calculateNewPoint()
		realGrid()
