#!/usr/bin/env python

PACKAGE = 'pointgrey_camera_driver'
import roslib;roslib.load_manifest(PACKAGE)
import rospy

import dynamic_reconfigure.client

def callback(config):
    rospy.loginfo("Config set to {gain}".format(**config))

if __name__ == "__main__":
    rospy.init_node("dynamic_client")

    client = dynamic_reconfigure.client.Client("/camera/camera_nodelet", timeout=1, config_callback=callback)

    r = rospy.Rate(1)
    x = 0.1
    b = False
    while not rospy.is_shutdown():
        x = x+1
        if x>10:
            x=0.1
        b = not b
        client.update_configuration({"gain":x})
	print x
        r.sleep()
