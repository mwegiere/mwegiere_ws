#!/usr/bin/env python

#robot porusza sie po zadanej trajektorii
#zatrzymuje sie po drodze podzial-razy
#za kazdym zatrzymaniem wykonuje zdjecie
#zdjecia sa zapisywane

from irpos import *

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import os.path
import time

#pakiety sluzace do dynamicznej rekonfiguracji parametrow kamery
PACKAGE = 'pointgrey_camera_driver'
import roslib;roslib.load_manifest(PACKAGE)
import rospy
import dynamic_reconfigure.client

class image_converter:

  def __init__(self): 
    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image_color", Image, self.makePhotoCallback) 
    self.irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")
    self.image = None
    self.client = dynamic_reconfigure.client.Client("/camera/camera_nodelet", timeout=1, config_callback=self.dynamicReconfigureCallback)
    
  def interpret_key(delf, key, key_up, key_down, axis, pos, vel):
    if key == key_up:
      pos[axis] = pos[axis] + vel[axis] * 0.2
    if key == key_down:
      pos[axis] = pos[axis] - vel[axis] * 0.2
      
    return pos
    
  def run(self):    
    #pozycja pionowa
    self.irpos.move_to_joint_position([0.0, -1.57079632679, -0.0, -0.0, 4.71238898038, 1.57079632679], 10.0)
    #ustawianie narzedzia
    self.irpos.set_tool_geometry_params(Pose(Point(0.0, 0.0, 0.5), Quaternion(0.0, 0.0, 0.0, 1.0)))

    for i in range(70):
       self.irpos.move_rel_to_cartesian_pose(1.0,Pose(Point(0.0, 0.0, 0.0), Quaternion(-0.00872654, 0.0, 0.0, 0.99996192)))
       for j in range(6):
          for k in range(3):
             print i
             gain = j*5
	     if k == 0:
		shutter_speed = 0.001
	     if k == 1:
		shutter_speed = 0.005
	     if k == 2:
		shutter_speed = 0.01
	     if k == 3:
		shutter_speed = 0.5
	     
             self.client.update_configuration({"gain":gain})
             self.client.update_configuration({"shutter_speed":shutter_speed})
    
       	     self.image = None
             while not self.image:
                time.sleep(1)
             name = str(j) +'_'+ str(k) +'_'+ str(i) + ".png"
             cv2.imwrite(name, self.bridge.imgmsg_to_cv2(self.image, "bgr8"))

  def makePhotoCallback(self, data):  
    try:  
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.image = data   
      cv2.imshow("Image window", cv_image)
      cv2.waitKey(3)
      
    except Exception, e:
      print e

  def dynamicReconfigureCallback(self,config):
    rospy.loginfo("Config set to {shutter_speed}".format(**config))

def main(args):
  ic = image_converter()
  ic.run()
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


