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


class zmq_server_object():
    def __init__(self,):
        # self.strategy_code = 0
        # self.item_dic = {'icon':0,'lower':70000,'higher':100000}
        # self.keywords = [{'word':'Windows','positive':0}, {'word':'Mac','positive':1}, {'word':'Office', 'positive':1}]
        # self.position = {'x':0.,'y':0.,'z':1.0}
        # self.sales_info = {
        #     'strategy_code':self.strategy_code,
        #     'item':self.item_dic,
        #     'keywords':self.keywords,
        #     'position':self.position
        # }

        self.sales_info = {
            'state':1,
            'item':{'id':0,'name':'sugoi computer','price':98000},
            'keywords':['notePC', 'Windows','office'],
            'position':{'x':0.,'y':0.,'z':1.0}
        }
        self.robot_info = {
            'statusCode':0,
            'statusDisc':'Working with A',
            'position': {'x':0.,'y':0.,'z':1.0}
        }
        self.json_dic = {
            'robot': [self.robot_info],
            'customer':[]
        }
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.id_list = [0.,3.,4.,9.,10.]
        self.init_flag = True
        self.camera_position = np.zeros(3)
        self.camera_orientation = np.zeros(4)
        self.alpha = 0.5
        self.prev_time = -1

    def parse_msg(self, msg):
        msg_list = msg.split(';')
        msg_list = map(str.split, msg_list)
        
        msg_code = msg_list[0][0]
        
        msg_body = msg_list[1:]

        return msg_code, msg_body
    
    def calc_camera_pose(self, params):
        marker_id = str(int(params[0]))
        tvec = np.array(params[1:4])
        rvec = np.array(params[4:7])

        R,_ = cv2.Rodrigues(rvec)
        translation = -1 * R.T.dot(tvec)

        Rtmp = np.identity(4)
        Rtmp[:3,:3] = R
        rotation = tf.transformations.quaternion_from_matrix(Rtmp.T)

        now = rospy.Time.now()
        camera_pose_stamped = PoseStamped()
        camera_pose_stamped.header.stamp = now
        camera_pose_stamped.header.frame_id = '/marker'+marker_id
        camera_pose_stamped.pose.position.x = translation[0]
        camera_pose_stamped.pose.position.y = translation[1]
        camera_pose_stamped.pose.position.z = translation[2]
        camera_pose_stamped.pose.orientation.x = rotation[0]
        camera_pose_stamped.pose.orientation.y = rotation[1]
        camera_pose_stamped.pose.orientation.z = rotation[2]
        camera_pose_stamped.pose.orientation.w = rotation[3]

        return camera_pose_stamped

    def create_camera_pose_list(self, mtx_list):
        pose_list = []
        Ids = []
        for mtx in mtx_list:
            mtx = map(float, mtx)
            if mtx[0] in self.id_list:
                pose = self.calc_camera_pose(mtx)
                pose_list.append(pose)
                Ids.append(mtx[0])
        return pose_list

    def cvt_pose2map(self, pose_list):
        map_pose_list = []
        for pose in pose_list:
            try:
                now = rospy.Time.now()
                self.listener.waitForTransform('map',pose.header.frame_id,now,rospy.Duration(3.0))
                pose_map = self.listener.transformPose('map', pose)
                map_pose_list.append(pose_map)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print('hoge', e)
                continue
        return map_pose_list

    def integrate_camera_pose(self, pose_list):
        if len(pose_list) is not 0:
            sum_translation = np.zeros(3)
            sum_rotation = np.zeros(4)
            cnt = 0
            for pose in pose_list:
                tmp_translation = pose.pose.position
                tmp_translation = np.array([tmp_translation.x,tmp_translation.y,tmp_translation.z])
                tmp_rotation = pose.pose.orientation
                tmp_rotation = np.array([tmp_rotation.x,tmp_rotation.y,tmp_rotation.z,tmp_rotation.w])

                if not self.init_flag:
                    if np.linalg.norm(self.camera_position-tmp_translation)>1.0:
                        print('noise')
                        continue
                sum_translation = sum_translation + tmp_translation
                sum_rotation = sum_rotation + tmp_rotation

                cnt+=1
            if cnt==0:
                return False, np.zeros(3),np.zeros(4)
            else:
                print(cnt)
                translation = sum_translation / cnt
                norm = np.linalg.norm(sum_rotation, 2)
                rotation = sum_rotation / norm
                return True, translation, rotation
        else:
            return False, np.zeros(3),np.zeros(4)

    def low_pass_filter(self, trans, rot):
        alpha = self.alpha
        self.camera_position = alpha*self.camera_position + (1-alpha)*trans
        self.camera_orientation = rot
        tmp = tf.transformations.euler_from_quaternion(self.camera_orientation)
        print(tmp[0]*180.0/math.pi, tmp[1]*180.0/math.pi, tmp[2]*180.0/math.pi)




    def localize(self, msg_body):
        # list of camera pose estimated from each aruco marker
        pose_list = self.create_camera_pose_list(msg_body)
        # translate pose from each aruco-marker-frame to map-frame
        pose_list = self.cvt_pose2map(pose_list)
        # wanna get camera position in map-frame
        suc_flag,trans,rot = self.integrate_camera_pose(pose_list)
        

        if suc_flag:
            # tmp = tf.transformations.euler_from_quaternion(rot)
            # print(tmp[0]*180.0/math.pi, tmp[1]*180.0/math.pi, tmp[2]*180.0/math.pi)

            if self.init_flag:
                self.camera_position = trans
                self.camera_orientation = rot
                self.init_flag = False
                tf_flag = False
                return tf_flag, rospy.Time.now()
            else:
                if np.linalg.norm(self.camera_position-trans)>0.6:
                    tf_flag = False
                    return tf_flag, rospy.Time.now()
                else:
                    self.low_pass_filter(trans,rot)
                    now = rospy.Time.now()
                    self.br.sendTransform(
                        tuple(self.camera_position),
                        tuple(self.camera_orientation),
                        now,
                        'camera',
                        'map'
                    )
                    tf_flag = True
                    return tf_flag, now
        else:
            tf_flag = False
            return tf_flag, rospy.Time.now()

    def cvt_object2camera(self, now):
        cam_object_list = []
        ht_id_list = []
        ht_msg = rospy.wait_for_message('/ht', HTEntityList)

        point_human = PointStamped()
        point_human.header.frame_id = 'map'
        point_human.point.x = 3.1
        point_human.point.y = 0.13
        point_human.point.z = 1.3
        point_human.header.stamp = now
        try:
            self.listener.waitForTransform('camera','map',now,rospy.Duration(3.0))
            point_human_camera = self.listener.transformPoint('camera',point_human)
            cam_object_list.append(point_human_camera)
            # ht_id_list.append(human.id)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)
        
        # for human in ht_msg.list:
        #     point_human = PointStamped()
        #     point_human.header.frame_id = human.header.frame_id
        #     # point_human.point.x = human.x
        #     # point_human.point.y = human.y
        #     # point_human.point.z = human.z + 0.5
        #     point_human.point.x = 3.1
        #     point_human.point.y = 0.13
        #     point_human.point.z = 1.3
        #     point_human.header.stamp = now
        #     try:
        #         self.listener.waitForTransform('camera','map',now,rospy.Duration(3.0))
        #         point_human_camera = self.listener.transformPoint('camera',point_human)
        #         cam_object_list.append(point_human_camera)
        #         ht_id_list.append(human.id)
        #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        #         print(e)
        #         continue
        
        return cam_object_list, ht_id_list
    
    def update_sale_info(self,params):
        self.sales_info['state'] = int(params[0])
        self.sales_info['item']['id'] = int(params[1])
        self.sales_info['item']['name'] = params[2]
        self.sales_info['item']['price'] = int(params[3])
        param_kw = params[4:]
        kw = []
        for i in range(len(param_kw)):
            kw.append(param_kw[i])
        self.sales_info['keywords'] = kw
        # self.item_dic['icon'] = int(params[1])
        # self.item_dic['lower'] = int(params[2])
        # self.item_dic['higher'] = int(params[3])
        # param_keywords = params[4:]
        # keywords = []
        # for i in range(len(param_keywords)/2):
        #     word = param_keywords[i*2]
        #     positive = int(param_keywords[i*2+1])
        #     keywords.append({'word':word,'positive':positive})
        # self.sales_info['keywords'] = keywords

    def update_position(self,x,y,z):
        self.sales_info['position']['x'] = x
        self.sales_info['position']['y'] = y
        self.sales_info['position']['z'] = z

    def onGyroChanged(self,params):
        x,y,z = params[0:3]
        x = float(x)
        y = -float(y)
        z = -float(z)
        time = float(params[3])
        t_delta = 0
        if self.prev_time>0:
            t_delta = time-self.prev_time /1000000000
        else:
            self.prev_time = time
            return '404'
        g_rot = tf.transformations.quaternion_from_euler(
            x*t_delta,
            y*t_delta,
            z*t_delta)
        rot_tmp = tf.transformations.quaternion_multiply(self.camera_orientation,g_rot)
        # self.camera_orientation = self.alpha*self.camera_orientation + (1-self.alpha)*rot_tmp
        # self.camera_orientation = self.camera_orientation / np.linalg.norm(self.camera_orientation)
        now = rospy.Time.now()
        self.br.sendTransform(
            tuple(self.camera_position),
            tuple(self.camera_orientation),
            now,
            'camera',
            'map'
        )
        cam_human_list,_ = server.cvt_object2camera(now)
        human = cam_human_list[0]     
        x = human.point.x
        y = human.point.y
        z = human.point.z
        return '{} {} {}'.format(x,y,z)
        
        

    

if __name__ == "__main__":
    rospy.init_node('estimate_camera_pose', anonymous=True)
    server = zmq_server_object()
    pub = rospy.Publisher('human', PointStamped, queue_size=10)

    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind('tcp://*:5556')
    print('server startup.')

    while not rospy.is_shutdown():
        msg = socket.recv_string(0)
        msg = msg.encode('utf-8')
        # print(msg)

        msg_code, msg_body = server.parse_msg(msg)
        if msg_code=='101':#camera localization
            tf_flag,now = server.localize(msg_body)
            if tf_flag:
                cam_human_list,human_id_list = server.cvt_object2camera(now)
                human = cam_human_list[0]
                pub.publish(human)
                x = human.point.x
                y = human.point.y
                z = human.point.z
                # server.robot_info['position']['x'] = x
                # server.robot_info['position']['y'] = y
                # server.robot_info['position']['z'] = z
                server.update_position(x,y,z)
                socket.send_string(json.dumps(server.sales_info))
            else:
                socket.send_string('404 no object is in field')

        elif msg_code=='102':#update sales info
            # print(msg_body[0])
            server.update_sale_info(msg_body[0])
            socket.send_string(json.dumps(server.sales_info))
            print(json.dumps(server.sales_info))
        elif msg_code=='103':
            if server.init_flag:
                print('hoge')
                socket.send_string('404')
            else:
                msg = server.onGyroChanged(msg_body[0])
                socket.send_string(msg)
                print(msg)
        else:
            err_msg = '404 your message is not acceptable'
            print(err_msg)
            socket.send_string(err_msg)
