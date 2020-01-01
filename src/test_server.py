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


class 