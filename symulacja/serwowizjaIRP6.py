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

from irpos import *

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
    self.newPos = 0
    self.newVel = 0
    self.gain = 10

    self.frequency = 50

    #definicjie macierzy
    self.T_gC = np.empty([4, 4])
    self.T_bC_koniec = np.empty([4, 4])
    self.T_gB_init = np.empty([4, 4])
    self.T_gC_koniec = np.empty([4, 4])

    #zmienne pomocnicze
    self.inicjalizacja = 0


  def callback(self, data):
    self.y_gB = 2.2 + data.data/4

  def predkosc(self, vel_y):
    self.irpos.set_tool_physical_params(10.8, Vector3(0.004, 0.0, 0.156))
    self.irpos.start_force_controller(Inertia(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), ReciprocalDamping(Vector3(0.0025, 0.0025, 0.0025), Vector3(0.0, 0.0, 0.0)), Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Twist(Vector3(0.0, vel_y, 0.0), Vector3(0.0, 0.0, 0.0)))

    time.sleep(1)
    self.irpos.stop_force_controller()

  def run(self):
    rospy.Subscriber("seeByIRP6HomogMatrix", Float32, self.callback) 
    self.irpos.set_tool_physical_params(10.8, Vector3(0.004, 0.0, 0.156))

    rate = rospy.Rate(self.frequency) 
    while not rospy.is_shutdown():
        self.listener.waitForTransform('/p_c_optical_frame', '/world', rospy.Time(0), rospy.Duration(1))
        #T_bC pozycja /wordl (B) w ukladzie /p_c_optical_frame (C)
    	(trans,rot) = self.listener.lookupTransform('/p_c_optical_frame', '/world', rospy.Time(0))

        self.y_gC = self.y_gB - trans[0]
	#REGULATOR	
	self.reg = self.y_gC * self.gain

	self.newVel = self.reg / self.frequency
	
	print self.newVel
    	self.irpos.start_force_controller(Inertia(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), ReciprocalDamping(Vector3(0.0025, 0.0025, 0.0025), Vector3(0.0, 0.0, 0.0)), Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Twist(Vector3(0.0, -self.newVel, 0.0), Vector3(0.0, 0.0, 0.0)))
        #print self.newVel

        rate.sleep()
    self.irpos.stop_force_controller()   
def main():
  si = serwoInfo()
  si.run()

if __name__ == '__main__':
  main()

