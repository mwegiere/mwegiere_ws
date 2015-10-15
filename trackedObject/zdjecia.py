#!/usr/bin/env python

#robot porusza sie po zadanej trajektorii
#zatrzymuje sie po drodze 70 razy
#za kazdym zatrzymaniem wykonuje zdjecie oraz zapisuje aktualne przeksztalcenie miedzy okladem kamery a swiata
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

#pakiety do subskrybcji tf
import roslib
roslib.load_manifest('serwo')
import rospy
import math
import tf
from tf import transformations
import numpy
import geometry_msgs.msg

class image_converter:

  def __init__(self): 
    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image_color", Image, self.makePhotoCallback) 
    self.irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")
    self.goodImages = []
    self.image = None
    self.client = dynamic_reconfigure.client.Client("/camera/camera_nodelet", timeout=1, config_callback=self.dynamicReconfigureCallback)

  def pozycjaPionowa(self):
     self.irpos.move_to_joint_position([0.0, -1.57079632679, -0.0, -0.0, 4.71238898038, 1.57079632679], 10.0)

  def ustawianieNarzedzia(self):
      self.irpos.set_tool_geometry_params(Pose(Point(0.0, 0.0, 0.5), Quaternion(0.0, 0.0, 0.0, 1.0)))
    
  def zdjecia(self):
    listener = tf.TransformListener()
    rate = rospy.Rate(1.0)
    tfCamWorld = open("/home/mwegiere/ws_irp6/mwegiere_ws/src/serwo/tfCamWorld", "wb")

    for i in range(70):
       #zapisanie aktualnego przeksztalcenia z /p_c_optical_frame do /world
       rate.sleep()
       (trans,rot) = listener.lookupTransform('/p_c_optical_frame', '/world', rospy.Time(0))
       camToWorld = self.fromTranslationRotation(trans, rot)
       tfCamWorld.write(str(trans)+"\n")
       tfCamWorld.write(str(rot)+"\n")
        
       self.client.update_configuration({"shutter_speed":0.4})
       self.client.update_configuration({"gain":20.0})
       self.image = None
       while not self.image:
          print self.image
          time.sleep(5)
       self.goodImages.append(self.image)
       print "jasne"
       print len(self.goodImages)

       self.client.update_configuration({"shutter_speed":0.001})
       self.client.update_configuration({"gain":0.0})
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
    tfCamWorld.close()

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
    rospy.loginfo("Config set to {gain}".format(**config))

  def run(self):
     pozycjaPionowa(self)
     ustawianie narzedzia(self)
     zdjecia(self)

def main(args):
  ic = image_converter()
  ic.run()
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


