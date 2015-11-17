#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import Float32

def talker():
    pub = rospy.Publisher('realHomogMatrix', Float32, queue_size=10)
    rospy.init_node('realHomogMatrix', anonymous=True)

    #w rzeczywistosci obiekt poruszasza sie z taka czestotliwoscia
    rate = rospy.Rate(500) 
    while not rospy.is_shutdown():
        y = math.sin(rospy.get_time()/5)
        pub.publish(y)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
