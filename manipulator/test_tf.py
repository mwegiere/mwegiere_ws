#!/usr/bin/env python  
import roslib
roslib.load_manifest('serwo')
import rospy
import math
import tf
from tf import transformations
import numpy
import geometry_msgs.msg

def fromTranslationRotation(translation, rotation):
    return numpy.dot(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))


if __name__ == '__main__':
    rospy.init_node('test_tf')
    
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/p_c_optical_frame', '/world', rospy.Time(0))
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

	camToWorld = fromTranslationRotation(trans, rot)
        print camToWorld[1,2]
        rate.sleep()
