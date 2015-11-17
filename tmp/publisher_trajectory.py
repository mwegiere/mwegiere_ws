#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('frames', anonymous=True)

    #publikuje 40 ramek na sekunde i tyle ramek jestem w stanie przetworzyc
    rate = rospy.Rate(40) # 40hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % math.sin(rospy.get_time())
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
