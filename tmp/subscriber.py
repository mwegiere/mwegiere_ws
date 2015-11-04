#!/usr/bin/env python
import rospy
from serwo.msg import SerwoInfo

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %d", data.out_time_nsec_pocz)
    
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("serwo_info", SerwoInfo, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
