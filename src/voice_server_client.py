# -*- coding: utf-8 -*-
#! /usr/bin/env python

import rospy
import sys
sys.path.append('/home/robovie/catkin_ws/src')
from std_msgs.msg import String
from tos_voice_gestures_tools import gesture, pause

import zmq


if __name__ == "__main__":
    rospy.init_node('pre_experiment', anonymous=True)
    global pub_voice_gestures
    pub_voice_gestures = rospy.Publisher('tos_voice_gestures', String, latch=True, queue_size=10)
    rate = rospy.Rate(5)

    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind('tcp://*:5557')
    print('voice-server start up')

    while not rospy.is_shutdown():
        msg = socket.recv_string(0)
        msg = msg.encode('utf-8')
        print(msg)
        pub_voice_gestures.publish(
            # gesture("sds_wave", 'こんにちは！' + pause(1) + today + 'で'+ pause(1) + tomorrow + 'です')
            gesture("sds_wave", msg)
        )
        socket.send_string('success.')
        rate.sleep()
