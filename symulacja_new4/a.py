#!/usr/bin/env python
import rospy
import numpy as np
from irpos import *
from std_msgs.msg import *
from serwo.msg import SerwoInfo
import threading
import PyKDL
import matrixOperations

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
import math

#konwencja
#T_aB - pozycja ukladu a w ukladzie B
#w mojej notacji a stoi na dole, a B stoi na gorze
class Visualization:
  def __init__(self): 
    self.T_gC_vector = []
    self.pointInCheckerFrame = np.matrix('0.0;0.0;0.0;1.0')
    self.newPoint = [0,0,0,0]
    self.realHomogMatrix = np.zeros(shape=(4,4))
    self.T_bC_matrix = np.zeros(shape=(4,4))
    self.publisher = None
    self.msgLock = threading.Lock()
    self.rospy.Subscriber('realHomogMatrix',SerwoInfo, callback)
    self.topic = 'realGrid'	
    self.publisher = rospy.Publisher(topic, Marker,queue_size=10)
    self.rospy.init_node('realGrid')
    self.listener.waitForTransform('/p_c_optical_frame', '/world', rospy.Time(0), rospy.Duration(10))
  
  def callback(msg):
    self.homogLock.acquire()
    T_gC_vector = msg.matrix  
    self.homogLock.release()

  def calculateNewPoint():
    self.newPoint = self.realHomogMatrix * self.pointInCheckerFrame
 
  def realGrid():
    rate = rospy.Rate(500)
    pawn = Marker()
    pawn.header.frame_id = "/world"
    pawn.type = pawn.CUBE
    pawn.action = pawn.ADD


    #T_bC pozycja /wordl (B) w ukladzie /p_c_optical_frame (C)
    (trans,rot) = listener.lookupTransform('/p_c_optical_frame', '/world', rospy.Time(0))
    T_bC = matrixOperations.euler_matrix_from_quaternion(rot, trans)

    T_gC = matrixOperations.translation_from_vector(T_gC_vector)

    T_gB = np.dot(np.linalg.inv(T_bC) , T_gC)
    realHomogMatrix = T_gB

    T = PyKDL.Frame(PyKDL.Rotation(realHomogMatrix[0][0], realHomogMatrix[0][1], realHomogMatrix[0][2],realHomogMatrix[1][0], realHomogMatrix[1][1], realHomogMatrix[1][2], realHomogMatrix[2][0], realHomogMatrix[2][1], realHomogMatrix[2][2]))
    
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
    rate.sleep()
     
  def run(self):
     
def main():
  si = serwoInfo()
  si.run()

if __name__ == '__main__':
  main()

