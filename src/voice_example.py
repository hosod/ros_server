# -*- coding: utf-8 -*-
#! /usr/bin/env python
import roslib
roslib.load_manifest('ros_class5_hosoda')
import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped, Quaternion
from geometry_msgs.msg import Point, PointStamped
import sys
sys.path.append('/home/robovie/catkin_ws/src')
from layer2.msg import HTEntityList, HTEntity
from tf.transformations import quaternion_from_euler
import forecast
from std_msgs.msg import String
from tos_voice_gestures_tools import gesture, pause

def __lookup_ht():
    ht_msg = rospy.wait_for_message('/ht', HTEntityList)
    # print(type(ht_msg))
    # listener = tf.TransformListener()
    range_min = 2.0
    listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(4.0))
    for human in ht_msg.list:
        pt_human = PointStamped()
        pt_human.header.frame_id = human.header.frame_id
        pt_human.point.x = human.x
        pt_human.point.y = human.y

        try:
            now = rospy.Time.now()
            pt_human.header.stamp = now
            listener.waitForTransform('/map', '/base_link', now, rospy.Duration(3.0))
            pt_bl = listener.transformPoint('/base_link', pt_human)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print('hoge', e)
            continue

        range_human = math.sqrt(pt_bl.point.x**2 + pt_bl.point.y**2)
        print(range_human)
        if range_human < range_min:
            # print('found')
            return (True, human, pt_bl)
        else:
            # print('not found')
            continue
    return (False, None, None)

def lookup_ht():
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            flag, human, human_bl = __lookup_ht()
            break
        except:
            continue
        rate.sleep()
    return (flag, human, human_bl)

# goal_vel = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
# human: HTEntity
# human_bl: PointStamped human in base_link
def get_goal(human, human_bl):    
    x = human_bl.point.x
    y = human_bl.point.y
    angle = math.atan2(y, x)

    goal = PoseStamped()
    goal.header.frame_id = human_bl.header.frame_id
    goal.header.stamp = rospy.Time.now()
    q = quaternion_from_euler(0,0,angle)
    goal.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    rospy.loginfo(goal)
    goal_vel.publish(goal)
    
        

if __name__ == "__main__":
    global goal_vel
    goal_vel = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    global pub_voice_gestures
    pub_voice_gestures = rospy.Publisher("/tos_voice_gestures", String, latch=True, queue_size=10)

    rospy.init_node('lookup_ht', anonymous=True)
    global listener
    listener = tf.TransformListener()

    # kyoto = forecast.get_forecasts()
    today = '今日の天気は晴れ'
    tomorrow = '明日の天気は晴時々曇'
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        flag, human, human_bl =  ()
        if flag:
            get_goal(human, human_bl)
            print('こんにちは　'+today + tomorrow)
            pub_voice_gestures.publish(gesture("sds_wave", 'こんにちは！' + pause(1) + today + 'で'+ pause(1) + tomorrow + 'です'))
            rospy.sleep(6)
            break

        # print(flag)
        rate.sleep()
  