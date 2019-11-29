#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def call_back(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, call_back)

    rospy.spin()

if __name__ == "__main__":
    listener()