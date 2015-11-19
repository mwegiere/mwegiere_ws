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
    self.listener = tf.TransformListener()
    self.y_gB = 0.0
    self.y_gC = 0
    self.reg = 0
    self.newVel = 0
    self.gain = 0.5
    self.found = 0

    self.frequency = 500

    self.T_gC_vector = []
 
    self.error = [0,0,0,0,0,0] #uchyby dla 6 stopni swobody

    self.minError = [0.01,0.01,0.01,0.01,0.01,0.01] #minimalny uchyb przy ktorym zostanie wyliczona nowa predkosc

    self.newVel = [0,0,0,0,0,0] #wektro predkosci dla 6 stopni swobody
    self.newAcceleration = [0,0,0,0,0,0] #wektro przyspieszenia dla 6 stopni swobody

    self.p = [0,0.5,0,0,0,0] #wartosc p dla regulatora PID
    self.i = [0,0,0,0,0,0] #wartosc p dla regulatora PID
    self.d = [0,0,0,0,0,0] #wartosc p dla regulatora PID

    self.maxSpeed = [0.05,0.05,0.05,0,0,0]
    self.maxAcceleration = [25.0,25.0,25.0,0,0,0]

    #kartezajanskie ograniczenia pozycji jako trojki xyz
    #w ukladzie zwiazanym z baza robota
    #przetestowac ograniczenia na prawdziwym robocie

    self.x_min = 0.8
    self.x_max = 0.9
    self.y_min = -0.3
    self.y_max = 0.3
    self.z_min = 1.2
    self.z_max = 1.4

    self.w_vector = [[0 for x in range(3)] for x in range(8)] 
    self.w_vector[0] = [self.x_max, self.y_max, self.z_min]
    self.w_vector[1] = [self.x_min, self.y_max, self.z_min]
    self.w_vector[2] = [self.x_min, self.y_min, self.z_min]
    self.w_vector[3] = [self.x_max, self.y_min, self.z_min]
    self.w_vector[4] = [self.x_max, self.y_max, self.z_max]
    self.w_vector[5] = [self.x_min, self.y_max, self.z_max]
    self.w_vector[6] = [self.x_min, self.y_min, self.z_max]
    self.w_vector[7] = [self.x_max, self.y_min, self.z_max]

  def limits_test(self):
  #przejazd robota po ograniczeniach kartezjanskich
    irpos.move_to_joint_position([0.0, -1.57079632679, -0.0, -0.0, 4.71238898038, 1.57079632679], 10.0)
    for i in range(8):
      self.irpos.move_to_cartesian_pose(5.0,Pose(Point(self.w_vector[i][0], self.w_vector[i][1], self.w_vector[i][2]), Quaternion(1.0, 0.0, 0.0, 0.0)))

  def callback(self, msg):
    self.T_gC_vector = msg.matrix
    self.found = msg.found

  def setNewVel(self):
    if self.found == 1:
      for i in range(6):
        if self.error[i] > self.minError[i] or self.error[i] < -self.minError[i]:
          print self.error[i]
          self.newVel[i] = self.error[i] * self.p[i]
          if self.newVel[i] > self.maxSpeed[i]:
            self.newVel[i] = self.maxSpeed[i]
          if self.newVel[i] < -self.maxSpeed[i]:
	    self.newVel[i] = -self.maxSpeed[i]
        else:
	  self.newVel[i] = 0
    else: 
      for i in range(5):
        self.newVel[i] = 0

  def checkLimits(self):
    current_pos_cartesian = self.irpos.get_cartesian_pose()

    new_pos_cartesian_x = current_pos_cartesian.position.x + self.newVel[0]*(1/self.frequency)
    new_pos_cartesian_y = current_pos_cartesian.position.y + self.newVel[1]*(1/self.frequency)
    new_pos_cartesian_z = current_pos_cartesian.position.z + self.newVel[2]*(1/self.frequency)
    self.newAcceleration[0] = self.newVel[0]*self.frequency
    self.newAcceleration[1] = self.newVel[1]*self.frequency
    self.newAcceleration[2] = self.newVel[2]*self.frequency
    

    #if not (new_pos_cartesian_x > self.x_min and new_pos_cartesian_x < self.x_max and new_pos_cartesian_y > self.y_min and new_pos_cartesian_y < self.y_max and new_pos_cartesian_z > self.z_min and new_pos_cartesian_z < self.z_max):
    if not (new_pos_cartesian_y > self.y_min and new_pos_cartesian_y < self.y_max):
      print 'przekroczony limit pozycji'
      #print 'new_pos_cartesian_x'
      #print new_pos_cartesian_x
      print 'current_pos_cartesian.position.y'
      print current_pos_cartesian.position.y
      print 'new_pos_cartesian_y'
      print new_pos_cartesian_y
      #print 'new_pos_cartesian_z'
      #print new_pos_cartesian_z
      self.newVel[0] = 0
      self.newVel[1] = 0
      self.newVel[2] = 0
    if not (self.newAcceleration[1] > -self.maxAcceleration[1] and self.newAcceleration[1] < self.maxAcceleration[1]):
      print 'przekroczono limit przyspieszenia'
      print 'self.newAcceleration[1]'
      print self.newAcceleration[1]
      self.newVel[0] = 0
      self.newVel[1] = 0
      self.newVel[2] = 0


  def TestVel(self):
    rate = rospy.Rate(self.frequency) #czestotliwosc pracy wezla
    self.irpos.set_tool_physical_params(10.8, Vector3(0.004, 0.0, 0.156)) #ustawienia do sterowania predkoscia

    current_vel = 0.000001
    for x in range(6):
      current_vel = current_vel * 10
      self.newVel = [0,current_vel,0,0,0,0]
      print 'current_vel ='
      print current_vel
      self.checkLimits()
      self.irpos.start_force_controller(Inertia(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), ReciprocalDamping(Vector3(0.0025, 0.0025, 0.0025), Vector3(0.0, 0.0, 0.0)), Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Twist(Vector3(self.newVel[0], self.newVel[1], self.newVel[2]), Vector3(self.newVel[3], self.newVel[4], self.newVel[5]))) #sterowanie predkoscia
      rate.sleep()

    self.irpos.stop_force_controller()
      
  def run(self):
    rate = rospy.Rate(self.frequency) #czestotliwosc pracy wezla
    self.irpos.set_tool_physical_params(10.8, Vector3(0.004, 0.0, 0.156)) #ustawienia do sterowania predkoscia

    pub = rospy.Publisher('uchyb', Float32, queue_size=10) #publikacja uchybu do celu rysowania wykresy
    rospy.Subscriber("realHomogMatrix", SerwoInfo, self.callback) #subskrybcja T_gC_vector
 
    while not rospy.is_shutdown():
        if not not self.T_gC_vector:
    		self.T_gC = matrixOperations.translation_from_vector(self.T_gC_vector) #przeksztalcenie wektora na macierz
		#dekomozycja regulatora
		self.error[0] = self.T_gC[1,3] #uchyb w osi x kamery
		self.error[1] = self.T_gC[0,3] #uchyb w osi y kamery
                self.error[2] = self.T_gC[2,3] #uchyb w osi z kamery
		#TODO dekopozycja katow os-kat
		self.setNewVel()
		self.checkLimits()

		self.irpos.start_force_controller(Inertia(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), ReciprocalDamping(Vector3(0.0025, 0.0025, 0.0025), Vector3(0.0, 0.0, 0.0)), Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Twist(Vector3(self.newVel[0], self.newVel[1], self.newVel[2]), Vector3(self.newVel[3], self.newVel[4], self.newVel[5]))) #sterowanie predkoscia

        rate.sleep()
        pub.publish(self.error[1]) #publikacja uchybu
    self.irpos.stop_force_controller()   
def main():
  si = serwoInfo()
  #si.run()
  #si.limits_test()
  #si.checkLimits()
  si.TestVel()

if __name__ == '__main__':
  main()

