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
import numpy
import geometry_msgs.msg

class serwoInfo:

  def __init__(self): 
    self.matrix = []
    rospy.init_node('listener', anonymous=True)
    self.listener = tf.TransformListener()
    #self.found = 0
    #self.out_time_sec_pocz = 0
    #self.out_time_nsec_pocz = 0

  def callback(self, data):
    self.matrix = data.matrix
    #self.found = data.found 
    #self.out_time_nsec_pocz = data.out_time_nsec_pocz
    #self.out_time_sec_pocz = data.out_time_sec_pocz
   

  def run(self):
    self.listener.waitForTransform('/p_c_optical_frame', '/world', rospy.Time(0), rospy.Duration(10))
    (trans,rot) = self.listener.lookupTransform('/p_c_optical_frame', '/world', rospy.Time(0))
    print trans

    #na podstawie matrix czyli T_G^C oraz T_B^C z chwili obcnej z tf licze T_G^B_init


    #od drugiego obiegu petli licze T_G^C_koniec = T_G^B_init * T_B^C_koniec
    #publikuje T_G^C_koniec na topicu

def main():
  si = serwoInfo()
  si.run()

if __name__ == '__main__':
  main()

