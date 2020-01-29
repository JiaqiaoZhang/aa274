#!/usr/bin/env python

import rospy
from aa274_s2.msg import MyMessage

def publisher():
    pub = rospy.Publisher('my_topic', MyMessage, queue_size=10)
    rospy.init_node('my_node', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
	my_message = MyMessage()
	my_message.text = "Hello Human!"
	rospy.loginfo("I published %s", my_message.text)
        pub.publish(my_message)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
