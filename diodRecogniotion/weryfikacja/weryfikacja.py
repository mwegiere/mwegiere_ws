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

homogVector = []
homogVectorSzachownica = []
pointInCheckerFrame = np.matrix('0.0;0.0;0.0;1.0')
newPoint = [0,0,0,0]
newPointSzachownica = [0,0,0,0]
homogMatrix = np.zeros(shape=(4,4))
homogMatrixSzachownica = np.zeros(shape=(4,4))
publisher = None
homogLock = threading.Lock()
global odlegloscPozycji
global odlegloscOrientacji

def readHomogVector(msg):
	homogLock.acquire()
	global homogVector
	homogVector = msg.data
	global homogMatrix
	homogMatrix = np.array([[homogVector[0],homogVector[1],homogVector[2],homogVector[3]],[homogVector[4],homogVector[5],homogVector[6],homogVector[7]],[homogVector[8],homogVector[9],homogVector[10],homogVector[11]],[homogVector[12],homogVector[13],homogVector[14],homogVector[15]]])
	homogLock.release()

def readHomogVector(msg):
	homogLock.acquire()
	global homogVectorSzachownica
	homogVectorSzachownica = msg.data
	global homogMatrixSzachownica
	homogMatrixSzachownica = np.array([[homogVectorSzachownica[0],homogVectorSzachownica[1],homogVectorSzachownica[2],homogVectorSzachownica[3]],[homogVectorSzachownica[4],homogVectorSzachownica[5],homogVectorSzachownica[6],homogVectorSzachownica[7]],[homogVectorSzachownica[8],homogVectorSzachownica[9],homogVectorSzachownica[10],homogVectorSzachownica[11]],[homogVectorSzachownica[12],homogVectorSzachownica[13],homogVectorSzachownica[14],homogVectorSzachownica[15]]])
	homogLock.release()

def calculateNewPoint():
	global newPoint
	global newPointSzachownica
	newPoint = homogMatrix * pointInCheckerFrame
	newPointSzachownica = homogMatrixSzachownica * pointInCheckerFrame
	
def onInit():
	rospy.Subscriber('/homog_matrix',Float32MultiArray, readHomogVector)
	rospy.Subscriber('/homog_matrixSzachownica',Float32MultiArray, readHomogVectorSzachownica)
	
def liczenieOdleglosci():
	T = PyKDL.Frame(PyKDL.Rotation(homogMatrix[0][0], homogMatrix[0][1], homogMatrix[0][2],homogMatrix[1][0], homogMatrix[1][1], homogMatrix[1][2], homogMatrix[2][0], homogMatrix[2][1], homogMatrix[2][2]))
	q = T.M.GetQuaternion()

	TSzachownica = PyKDL.Frame(PyKDL.Rotation(homogMatrixSzachownica[0][0], homogMatrixSzachownica[0][1], homogMatrixSzachownica[0][2],homogMatrixSzachownica[1][0], homogMatrixSzachownica[1][1], homogMatrixSzachownica[1][2], homogMatrixSzachownica[2][0], homogMatrixSzachownica[2][1], homogMatrixSzachownica[2][2]))
	qSzachownica = TSzachownica.M.GetQuaternion()

	global odlegloscOrientacji
	odlegloscOrientacji = np.sqrt(2*(1-np.absoluste(q*qSzachownica)))

	global odlegloscPozycji
	odlegloscPozycji = np.sqrt(newPoint[0]*newPointSzachownica[0] + newPoint[1]*newPointSzachownica[2] + newPoint[1]*newPointSzachownica[2])

	print odlegloscOrientacji
	print odlegloscPozycji
	
	rospy.sleep(0.01)

if __name__ == '__main__':
	onInit()
	while not rospy.is_shutdown():
		calculateNewPoint()
		liczenieOdleglosci()
