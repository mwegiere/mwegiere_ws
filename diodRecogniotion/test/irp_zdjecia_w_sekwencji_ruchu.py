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
    self.goodImages = []
    self.image = None

    self.client = dynamic_reconfigure.client.Client("/camera/camera_nodelet", timeout=1, config_callback=self.dynamicReconfigureCallback)
    
  def interpret_key(delf, key, key_up, key_down, axis, pos, vel):
    if key == key_up:
      pos[axis] = pos[axis] + vel[axis] * 0.2
    if key == key_down:
      pos[axis] = pos[axis] - vel[axis] * 0.2
      
    return pos
    
  def run(self):
    
    #pos_current = self.irpos.get_joint_position()

    # list of important poses
    #pos1 = [-0.10063291139240507, -1.5419428654532268, 0.019737556833721442, 1.1335183568246088, 3.658072916666667, -2.7381185214159984]
    #podzial = 10
    #pos2 = [-(0.1996774977436294 - (-0.10063291139240507))/podzial, -(-1.642122929252318 - (-1.5419428654532268))/podzial, -(-0.08032381762494378-(-1.1335183568246088))/podzial, -(1.0334922597384786-1.1335183568246088)/podzial, -(3.6579092920492924-3.658072916666667)/podzial, -(-2.7381185214159984-(-2.7381185214159984))/podzial]
    #pos2 = [0.1996774977436294, -1.642122929252318, -0.08032381762494378, 1.0334922597384786, 3.6579092920492924, -2.7381185214159984]
    #finalPose = (-(np.array(pos2) - np.array(pos1))/podzial)
    # maximum joint velocities
    #max_vel = [0.5, 0.5, 0.5, 0.5, 0.5, 0.25]
     
    #diff = np.array(pos_current) - np.array(pos1)
    #print diff
    #diff = np.absolute(diff)
    #print diff
    #diff = np.divide(diff, max_vel)
    #print diff
    #first_time = max(np.amax(diff), 1)
    #print first_time

    # move to initial position
    #self.irpos.move_to_joint_position(pos1, first_time)
   
    # move_rel_to_joint_position
    
    #pozycja pionowa
    self.irpos.move_to_joint_position([0.0, -1.57079632679, -0.0, -0.0, 4.71238898038, 1.57079632679], 10.0)
    print "Pozycja startowa ustawiona"
    print "Start ustawiania narzedzia"
    self.irpos.set_tool_geometry_params(Pose(Point(0.0, 0.0, 0.5), Quaternion(0.0, 0.0, 0.0, 1.0)))
    print "Koniec ustawiania narzedzia"

    for i in range(70):
        
       self.client.update_configuration({"shutter_speed":0.049})
       self.image = None
       while not self.image:
          print self.image
          time.sleep(5)
       self.goodImages.append(self.image)
       print "jasne"
       print len(self.goodImages)

       self.client.update_configuration({"shutter_speed":0.00001})
       self.image = None
       while not self.image:
          print self.image
          time.sleep(5)
       self.goodImages.append(self.image)
       print "ciemne"
       print len(self.goodImages)
       
       self.irpos.move_rel_to_cartesian_pose(1.0,Pose(Point(0.0, 0.0, 0.0), Quaternion(-0.00872654, 0.0, 0.0, 0.99996192)))
       print i

    # after the movement save all the photos
    print "Saving", len(self.goodImages), "images"
    for i in range(len(self.goodImages)):
      fname = str(self.goodImages[i].header.stamp.secs) + "_" + format(self.goodImages[i].header.stamp.nsecs, '09') + ".png"
      print "Saving", fname
      cv2.imwrite(fname, self.bridge.imgmsg_to_cv2(self.goodImages[i], "bgr8"))

    print "I'm done!"

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


