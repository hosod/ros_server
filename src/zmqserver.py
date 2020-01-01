# -*- coding: utf-8 -*-
#! /usr/bin/env python
import zmq
import cv2
import json
import rospy
import tf
import math
import sys
import threading
sys.path.append('/home/robovie/catkin_ws/src')
from layer2.msg import HTEntityList, HTEntity
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, PointStamped

strategy_code = 0
item_dic = {'icon':0,'lower':70000,'higher':100000}
keywords = [{'word':'Windows','state':0}, {'word':'Mac','state':1}, {'word':'Office', 'state':1}]
position = {'x':0.,'y':0.,'z':1.0}

sales_info = {
    'strategy_code':strategy_code,
    'item':item_dic,
    'keywords':keywords,
    'position':position
}


robot_info = {
    'statusCode':0,
    'statusDisc':'Working with A',
    'position': {'x':0.,'y':0.,'z':1.0}
}

json_dic = {
    'robot': [robot_info],
    'customer':[]
}

# # parse zmq input from moverio and return list of camera Pose from view of each markers
# # 
# def parse_mtx(matrix_str):
#     # print(str(matrix_str))
#     matrix_str = matrix_str.split(';')
#     matrix_list = map(str.split, matrix_str)
    
#     length = len(matrix_list)
#     pose_list = []
#     IDS = []
#     for mtx in matrix_list:
#         mtx = map(float, mtx)
#         if mtx[0] in id_list:
#             pose = get_camera_pos(mtx)
#             pose_list.append(pose)
#             IDS.append(mtx[0])
#             # print(IDS)
#         # pose_list.add(get_camera_pos(mtx))
    
#     return pose_list

def parse_msg(msg):
    msg_list = msg.split(';')
    msg_list = map(str.split, msg_list)

    msg_code = msg_list[0][0]
    msg_body = msg_list[1:]

    return msg_code, msg_body
    # if msg_code is '101': # camera localization
    #     msg_list = msg_list[1:]
    #     localize(msg_list)
    # elif msg_code is '102': # update robot info
    #     pass


# estimate camera pose from pose of single marker captured by camera
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

def create_camera_pose_list(mtx_list):
    pose_list = []
    Ids = []
    for mtx in mtx_list:
        mtx = map(float, mtx)
        if mtx[0] in id_list:
            pose = get_camera_pos(mtx)
            pose_list.append(pose)
            Ids.append(mtx[0])
    return pose_list

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

def integrate_camera_pos(pose_list):
    if len(pose_list) is not 0:
        print('pose list is not empty')
        sum_translation = np.zeros(3)
        sum_rotation = np.zeros(4)
        cnt = 0
        for pose in pose_list:
            tmp_translation = pose.pose.position
            tmp_translation = np.array([tmp_translation.x,tmp_translation.y,tmp_translation.z])
            tmp_rotation = pose.pose.orientation
            tmp_rotation = np.array([tmp_rotation.x,tmp_rotation.y,tmp_rotation.z,tmp_rotation.w])

            print('init flag',init_flag)
            if not init_flag:
                if np.linalg.norm(camera_position-tmp_translation)>1.0:
                    print('noise')
                    continue
            
            sum_translation = sum_translation + tmp_translation
            sum_rotation = sum_rotation + tmp_rotation
            cnt+=1
        
        if cnt==0:
            return False,np.zeros(3),np.zeros(4)
        else:
            translation = sum_translation/cnt
            norm = np.linalg.norm(sum_rotation, 2)
            rotation = sum_rotation/norm

            return True, translation, rotation
    else:
        print('pose list is empty')
        return False,np.zeros(3),np.zeros(4)

def low_pass_filter():
    alpha = 0.5
    camera_position = alpha*camera_position + (1-alpha)*translation
    camera_orientation = alpha*camera_orientation + (1-alpha)*rotation
    camera_orientation = camera_orientation / np.linalg.norm(camera_orientation)

# obj_list: list of PointStamped that mean Object Coordinates
# translate map-frame position of object to camera-frame
def __cvtObj2Camera(now):
    cam_obj_list = []
    ht_id_list = []

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
            ht_id_list.append(human.id)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)
            continue

    return cam_obj_list, ht_id_list

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


def localize(mtx_list, init_flag):

    # print('----localize-----')
    # list of camera pose estimated from each aruco marker
    pose_list = create_camera_pose_list(mtx_list)
    # translate pose from each aruco-marker-frame to map-frame
    pose_list = cvtPose2Map(pose_list)
    # wanna get camera position in map-frame
    suc_flag,trans,rot = integrate_camera_pos(pose_list)
    # print(suc_flag,trans,rot)

    if suc_flag:
        if init_flag:
            # print('init')
            camera_position = trans.copy()
            camera_orientation = rot.copy()
            init_flag = False
            tf_flag = False
            return init_flag,tf_flag,rospy.Time.now() 
        else:

            if np.linalg.norm(camera_position-trans)>0.6:
                # print('noise')
                tf_flag = False
                return init_flag,tf_flag, rospy.Time.now()
                # ---------alert---------
            else:
                low_pass_filter()
                now = rospy.Time.now()
                br.sendTransform(tuple(camera_position),
                                tuple(camera_orientation),
                                now,
                                "camera",
                                "map") 
                # print('hoge')
                tf_flag = True
                return init_flag,tf_flag, now
    else:
        # print('fail')
        tf_flag = False
        return init_flag,tf_flag, rospy.Time.now()




if __name__ == "__main__":
    rospy.init_node('estimate_camera_pose', anonymous=True)
    global listener
    listener = tf.TransformListener()
    global br
    br = tf.TransformBroadcaster()

    global id_list
    id_list = [0., 3., 4., 9., 10.]

    global init_flag
    init_flag = True

    global camera_position
    camera_position = np.zeros(3)
    global camera_orientation
    camera_orientation = np.zeros(4)

# low-path filter
# x = a*x + (1-a)*z
    alpha = 0.5

    
    x = 1.2
    y = 0.
    z = 1.5

    context = zmq.Context()
    socket = context.socket(zmq.REP)
    # socket.bind('tcp://*:5556')
    socket.bind('tcp://*:5556')

    print('server startup.')

    pub = rospy.Publisher("human", PointStamped ,queue_size=10)

    while not rospy.is_shutdown():
        print(camera_position)
        msg = socket.recv_string(0)
        msg = msg.encode('utf-8')
        # print(msg)
        # print(msg)
        msg_code, msg_body = parse_msg(msg)
        # print(msg_code,msg_body)
        if msg_code == '101': # camera localization
            flag, tf_flag, now = localize(msg_body, init_flag)
            # print(tf_flag)
            init_flag = flag
            if tf_flag:
                cam_human_list, human_id_list = __cvtObj2Camera(now)
                human = cam_human_list[0]
                pub.publish(human)
                x = human.point.x
                y = human.point.y
                z = human.point.z
                robot_info['position']['x'] = x
                robot_info['position']['y'] = y
                robot_info['position']['z'] = z
                socket.send_string(json.dumps(json_dic))
                print(x,y,z)
            else:

                socket.send_string('404 no object is in field')
                    
        elif msg_code == '102': # update robot info
                socket.send_string('404 not found')
        else:
            err_msg = '404 your message is not acceptable'
            print(err_msg)
            socket.send_string(err_msg)
            

        
# # ------------------localize-------------
#         pose_list = parse_mtx(msg)
       
#         pose_list = cvtPose2Map(pose_list)
#         # print(pose_list)
#         sum_translation = np.zeros(3)
#         sum_rotation = np.zeros(4)
#         cnt = 0
#         if len(pose_list) is not 0:
#             for pose in pose_list:

#                 # pub.publish(pose)
#                 translation = pose.pose.position
#                 translation = np.array([translation.x,translation.y,translation.z])
#                 rotation = pose.pose.orientation
#                 rotation = np.array([rotation.x,rotation.y,rotation.z,rotation.w])

#                 if not init_flag:
#                     if np.linalg.norm(camera_position-translation)>1.0:
#                         continue

#                 sum_translation = sum_translation + translation
#                 sum_rotation = sum_rotation + rotation
#                 cnt+=1

#             if cnt==0:
#                 socket.send_string('404 '+str(x)+' '+str(y)+' '+str(z))
#                 continue
#             else:
#                 # print(sum_translation)
#                 # print(len(pose_list))
#                 translation = sum_translation/cnt
#                 norm = np.linalg.norm(sum_rotation, 2)
#                 rotation = sum_rotation/norm

#             if init_flag:
#                 camera_position = translation
#                 camera_orientation = rotation
#                 init_flag = False
#             else:
#                 if np.linalg.norm(camera_position-translation)>0.6:
#                     # noise
#                     socket.send_string('404 '+str(x)+' '+str(y)+' '+str(z))
#                     continue
#                 else:
#                     camera_position = alpha*camera_position + (1-alpha)*translation
#                     camera_orientation = alpha*camera_orientation + (1-alpha)*rotation
#                     camera_orientation = camera_orientation / np.linalg.norm(camera_orientation)
#                     # print(camera_position)
#                     now = rospy.Time.now()
#                     br.sendTransform(tuple(camera_position),
#                                     tuple(camera_orientation),
#                                     now,
#                                     "camera",
#                                     "map") 
# # ---------------localize--------------
#                     # x, y, z = cvt2camera(2.5, 2.0, 1.55, now)
#                     print('hoge')
#                     cam_obj_list, human_id_list =  __cvtObj2Camera(now)
#                     point_human = cam_obj_list[0]
#                     pub.publish(point_human)
#                     x = point_human.point.x
#                     y = point_human.point.y
#                     z = point_human.point.z-0.3

#                     # print(cam_obj_/list)
#                     # x, y, z = cvt2camera(1.7, 0., 1.5, now)
#             print(x,y,z, 'hoge')
#             socket.send_string('0 '+str(x)+' '+str(y)+' '+str(z))
#         else:
#             socket.send_string('404 '+str(x)+' '+str(y)+' '+str(z))
        


#     socket.close()
    