# -*- coding: utf-8 -*-
#! /usr/bin/env python
import zmq
import cv2
import rospy
import tf
import math
import sys
import threading
sys.path.append('/home/robovie/catkin_ws/src')
from layer2.msg import HTEntityList, HTEntity
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, PointStamped



# parse zmq input from moverio and return list of camera Pose from view of each markers
def parse_mtx(matrix_str):
    # print(str(matrix_str))
    matrix_str = matrix_str.split(';')
    matrix_list = map(str.split, matrix_str)
    
    length = len(matrix_list)
    pose_list = []
    IDS = []
    for mtx in matrix_list:
        mtx = map(float, mtx)
        if mtx[0] in id_list:
            pose = get_camera_pos(mtx)
            pose_list.append(pose)
            IDS.append(mtx[0])
            # print(IDS)
        # pose_list.add(get_camera_pos(mtx))
    
    return pose_list

def get_camera_pos(params):
    marker_id = str(int(params[0]))
    tvec = np.array(params[1:4])
    rvec = np.array(params[4:7])
    # quat = rod2quat(rvec)
    
    R,_=cv2.Rodrigues(rvec)
    camera_position = -1 * R.T.dot(tvec)

    Rtmp = np.identity(4)
    Rtmp[:3, :3] = R
    camera_orientation = tf.transformations.quaternion_from_matrix(Rtmp.T)
    # camera_orientation = tf.transformations.quaternion_inverse(quat)

    now = rospy.Time.now()
    camera_pose = PoseStamped()
    camera_pose.header.stamp = now
    camera_pose.header.frame_id = '/marker'+marker_id
    camera_pose.pose.position.x = camera_position[0]
    camera_pose.pose.position.y = camera_position[1]
    camera_pose.pose.position.z = camera_position[2]
    camera_pose.pose.orientation.x = camera_orientation[0]
    camera_pose.pose.orientation.y = camera_orientation[1]
    camera_pose.pose.orientation.z = camera_orientation[2]
    camera_pose.pose.orientation.w = camera_orientation[3]

    return camera_pose


def rod2quat(rod):
    norm = tf.transformations.vector_norm(rod)
    rot_vector = np.array([rod[0]/norm, rod[1]/norm, rod[2]/norm])
    quat = np.zeros(4)
    
    # quat[0] = rot_vector[2] * math.sin(norm/2)  # w
    # quat[1] = math.cos(norm/2)                  # x
    # quat[2] = rot_vector[0] * math.sin(norm/2)  # y
    # quat[3] = rot_vector[1] * math.sin(norm/2)  # z
    return quat

def cvtPose2World(pose_list):
    world_pose_list = []
    for pose_camera in pose_list:

        try:
            now = rospy.Time.now()
            # pose_camera.header.stamp = now
            listener.waitForTransform('world', pose_camera.header.frame_id, now, rospy.Duration(3.0))
            pose_camera_world = listener.transformPose('world', pose_camera)
            world_pose_list.append(pose_camera_world)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print('hoge', e)
            continue
    return world_pose_list

def cvtPose2Map(pose_list):
    map_pose_list = []
    for camera_pose in pose_list:
        try:
            now = rospy.Time.now()
            listener.waitForTransform('map', camera_pose.header.frame_id, now, rospy.Duration(3.0))
            pose_camera_map = listener.transformPose('map', camera_pose)
            map_pose_list.append(pose_camera_map)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print('hoge', e)
            continue
    return map_pose_list

# obj_list: list of PointStamped that mean Object Coordinates
def __cvtObj2Camera(now):
    cam_obj_list = []
    id_list = []

    ht_msg = rospy.wait_for_message('/ht', HTEntityList)
    print('fuga')
    for human in ht_msg.list:
        point_human = PointStamped()
        point_human.header.frame_id = human.header.frame_id
        point_human.point.x = human.x
        point_human.point.y = human.y
        point_human.point.z = 1.4

        try:
            point_human.header.stamp = now
            listener.waitForTransform('camera', 'map', now, rospy.Duration(3.0))
            point_human_camera = listener.transformPoint('camera', point_human)
            cam_obj_list.append(point_human_camera)
            id_list.append(human.id)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)
            continue

    return cam_obj_list, id_list

def points2str(point_list):
    return ''

def cvt2camera(x, y, z, tnow):
    point = PointStamped()
    point.header.frame_id = 'world'
    point.point.x = x
    point.point.y = y
    point.point.z = z
    
    point.header.stamp = tnow

    try:
        # now = rospy.Time.now()
        # point.header.stamp = now
        listener.waitForTransform('camera', 'map', tnow, rospy.Duration(3.0))
        cam_point = listener.transformPoint('camera', point)
    except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print(e)
        return 0,0,0
    return cam_point.point.x, cam_point.point.y, cam_point.point.z
    

def devide_n(n, target):
    return target / n
    

if __name__ == "__main__":
    rospy.init_node('estimate_camera_pose', anonymous=True)
    global listener
    listener = tf.TransformListener()

    global id_list
    id_list = [0., 3., 4., 9., 10.]

# low-path filter
# x = a*x + (1-a)*z
    init_flag = True
    alpha = 0.5
    camera_position = np.zeros(3)
    camera_orientation = np.zeros(4)

    br = tf.TransformBroadcaster()
    x = 1.2
    y = 0.
    z = 1.5


    context = zmq.Context()
    socket = context.socket(zmq.REP)
    # socket.bind('tcp://*:5556')
    socket.bind('tcp://*:5558')

    print('server startup.')

    pub = rospy.Publisher("human", PointStamped ,queue_size=10)

    while not rospy.is_shutdown():
        msg = socket.recv_string(0)
        msg = msg.encode('utf-8')
        # print(msg)
        
        pose_list = parse_mtx(msg)
        # pose_list = cvtPose2World(pose_list)
        # print(pose_list)
        pose_list = cvtPose2Map(pose_list)
        # print(pose_list)
        sum_translation = np.zeros(3)
        sum_rotation = np.zeros(4)
        cnt = 0
        if len(pose_list) is not 0:
            for pose in pose_list:

                # pub.publish(pose)
                translation = pose.pose.position
                translation = np.array([translation.x,translation.y,translation.z])
                rotation = pose.pose.orientation
                rotation = np.array([rotation.x,rotation.y,rotation.z,rotation.w])

                if not init_flag:
                    if np.linalg.norm(camera_position-translation)>1.0:
                        continue

                sum_translation = sum_translation + translation
                sum_rotation = sum_rotation + rotation
                cnt+=1

            if cnt==0:
                socket.send_string('404 '+str(x)+' '+str(y)+' '+str(z))
                continue
            # print(sum_translation)
            # print(len(pose_list))
            translation = sum_translation/cnt
            norm = np.linalg.norm(sum_rotation, 2)
            rotation = sum_rotation/norm
            if init_flag:
                camera_position = translation
                camera_orientation = rotation
                init_flag = False
            else:
                if np.linalg.norm(camera_position-translation)>0.6:
                    # noise
                    socket.send_string('404 '+str(x)+' '+str(y)+' '+str(z))
                    continue
                else:
                    camera_position = alpha*camera_position + (1-alpha)*translation
                    camera_orientation = alpha*camera_orientation + (1-alpha)*rotation
                    camera_orientation = camera_orientation / np.linalg.norm(camera_orientation)
                    # print(camera_position)
                    now = rospy.Time.now()
                    br.sendTransform(tuple(camera_position),
                                    tuple(camera_orientation),
                                    now,
                                    "camera",
                                    "map") 

                    # x, y, z = cvt2camera(2.5, 2.0, 1.55, now)
                    print('hoge')
                    cam_obj_list, human_id_list =  __cvtObj2Camera(now)
                    point_human = cam_obj_list[0]
                    pub.publish(point_human)
                    x = point_human.point.x
                    y = point_human.point.y
                    z = point_human.point.z-0.3

                    # print(cam_obj_/list)
                    # x, y, z = cvt2camera(1.7, 0., 1.5, now)
            print(x,y,z, 'hoge')
            socket.send_string('0 '+str(x)+' '+str(y)+' '+str(z))
        else:
            socket.send_string('404 '+str(x)+' '+str(y)+' '+str(z))
        


    socket.close()
    