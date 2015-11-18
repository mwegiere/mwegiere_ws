#!/usr/bin/env python
import rospy

#pakiety do subskrybcji tf
import roslib
roslib.load_manifest('serwo')
import rospy
import math
import tf
from tf import transformations
import numpy as np
import geometry_msgs.msg

from std_msgs.msg import Float32
from serwo.msg import SerwoInfo
from irpos import *
import matrixOperations

#konwencja
#T_aB - pozycja ukladu a w ukladzie B
#w mojej notacji a stoi na dole, a B stoi na gorze

class serwoInfo:

  def __init__(self): 

    self.irpos = IRPOS("maciek", "Irp6p", 6, "irp6p_manager")

    #rospy.init_node('listener', anonymous=True)
    self.listener = tf.TransformListener()
    self.y_gB = 0.0
    self.y_gC = 0
    self.reg = 0
    self.newVel = 0
    self.gain = 0.5

    self.frequency = 500

    self.T_gC_vector = []

  def callback(self, msg):
    #print self.T_gC_vector
    self.T_gC_vector = msg.matrix


  def run(self):
    rate = rospy.Rate(self.frequency) #czestotliwosc pracy wezla
    self.irpos.set_tool_physical_params(10.8, Vector3(0.004, 0.0, 0.156)) #ustawienia do sterowania predkoscia

    pub = rospy.Publisher('uchyb', Float32, queue_size=10) #publikacja uchybu do celu rysowania wykresy
    rospy.Subscriber("realHomogMatrix", SerwoInfo, self.callback) #subskrybcja T_gC_vector
 
    while not rospy.is_shutdown():
    	#print self.T_gC_vector
        if not not self.T_gC_vector:
    		self.T_gC = matrixOperations.translation_from_vector(self.T_gC_vector) #przeksztalcenie wektora na macierz
		self.y_gC = self.T_gC[0,3] #uchyb w osi y	
		self.reg = self.y_gC * self.gain
		self.newVel = self.reg
		print self.y_gC
		if self.newVel > 0.05:
			self.newVel = 0.05
		if self.newVel < -0.05:
			self.newVel = -0.05

		if self.y_gC > 0.2 or self.y_gC < 0.2:
	    		self.irpos.start_force_controller(Inertia(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), ReciprocalDamping(Vector3(0.0025, 0.0025, 0.0025), Vector3(0.0, 0.0, 0.0)), Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Twist(Vector3(0.0, self.newVel, 0.0), Vector3(0.0, 0.0, 0.0))) #sterowanie predkoscia

        rate.sleep()
        pub.publish(self.y_gC) #publikacja uchybu
    self.irpos.stop_force_controller()   
def main():
  si = serwoInfo()
  si.run()

if __name__ == '__main__':
  main()

