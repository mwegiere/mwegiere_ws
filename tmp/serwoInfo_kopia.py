#!/usr/bin/env python
import rospy
from serwo.msg import SerwoInfo

#pakiety do subskrybcji tf
import roslib
roslib.load_manifest('serwo')
import rospy
import math
import tf
from tf import transformations
import numpy as np
import geometry_msgs.msg

import matrixOperations

#konwencja
#T_aB - pozycja ukladu a w ukladzie B
#w mojej notacji a stoi na dole, a B stoi na gorze

class serwoInfo:

  def __init__(self): 

    rospy.init_node('listener', anonymous=True)
    self.listener = tf.TransformListener()

    self.matrix = []
    self.found = 0
    self.out_time_sec_pocz = 0
    self.out_time_nsec_pocz = 0
    self.out_time_sec_kon = 0
    self.out_time_nsec_kon = 0

    self.tmp_sec_kon = 2
    self.tmp_nsec_kon = 2

    #czas od zrobienia zdjecia do momentu wyslania info z SolvePnP
    self.sec = 0
    self.nsec = 0

    #definicjie macierzy
    self.T_gC = np.empty([4, 4])
    self.T_bC_koniec = np.empty([4, 4])
    self.T_gB_init = np.empty([4, 4])
    self.T_gC_koniec = np.empty([4, 4])

    #zmienne pomocnicze
    self.inicjalizacja = 0

    #regulator 
    self.przesuniecie

  def callback(self, data):
    self.matrix = data.matrix
    self.found = data.found 
    self.out_time_nsec_pocz = data.out_time_nsec_pocz
    self.out_time_sec_pocz = data.out_time_sec_pocz
    self.out_time_nsec_kon = data.out_time_nsec_kon
    self.out_time_sec_kon = data.out_time_sec_kon

  def run(self):
    rospy.Subscriber("serwo_info", SerwoInfo, self.callback) 
    
    while (1): 
      if (self.tmp_sec_kon != self.out_time_sec_kon & self.tmp_nsec_kon != self.out_time_nsec_kon):
	#ustwienie wartosci czasow potrzebnych tylko do warunku if
        self.tmp_sec_kon = self.out_time_sec_kon
        self.tmp_nsec_kon = self.out_time_sec_kon

        self.listener.waitForTransform('/p_c_optical_frame', '/pl_base', rospy.Time(0), rospy.Duration(10))
        #T_bC pozycja /wordl (B) w ukladzie /p_c_optical_frame (C)
    	(trans,rot) = self.listener.lookupTransform('/p_c_optical_frame', '/world', rospy.Time(0))
	self.T_bC_koniec = matrixOperations.euler_matrix_from_quaternion(rot, trans)

	#liczenie roznicy czasu
    	self.sec = self.out_time_sec_kon - self.out_time_sec_pocz
        self.nsec = self.out_time_nsec_kon - self.out_time_nsec_pocz
        print "czas sekundy i nanosekundy od zrobienia zdjecia do momentu wyslania info z SolvePnP"
        print self.sec
        print self.nsec

	#T_gC_koniec - pozycja ukladu g w ukladzie C
        self.T_gC = matrixOperations.translation_from_vector(self.matrix)

	if (self.inicjalizacja == 0):
          self.T_gB_init = self.T_gC * np.inv(self.T_bC_koniec)
          self.inicjalizacja = 1
        else:
          self.T_gC_koniec = self.T_bC_koniec * self.T_gB_init
          #dekompozycja regulatora
          print self.T_gC_koniec

          #liczenie T_gC przez regulator
          #self.przesuniecie = self.T_gC_koniec[3,2] * 0.01

	  #przeksztalcenie T_gC do T_cB
	  


          #przeksztalcenie na predkosc
          #ograniczenie predkoci
          #zadanie predkosci




    #na podstawie matrix czyli T_G^C oraz T_B^C z chwili obcnej z tf licze T_G^B_init


    #od drugiego obiegu petli licze T_G^C_koniec = T_G^B_init * T_B^C_koniec
    #publikuje T_G^C_koniec na topicu

def main():
  si = serwoInfo()
  si.run()

if __name__ == '__main__':
  main()

