#! /usr/bin/env python

import rospy
import tf.transformations as tft

import numpy as np

import kinova_msgs.msg
import kinova_msgs.srv
import std_msgs.msg
import std_srvs.srv
import geometry_msgs.msg
import sensor_msgs.msg
import actionlib_msgs.msg
import cv2
from cv_bridge import CvBridge, CvBridgeError
import msgs.msg
import msgs.srv


from helpers.gripper_action_client import set_finger_positions
from helpers.position_action_client import position_client, move_to_position
from helpers.joints_action_client import joint_angle_client
from helpers.covariance import generate_cartesian_covariance
#import kinova_angle_home

import os
import time
import random
import pickle
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

import inspect

KINOVA_HOME_ANGLE = [4.543, 3.370, -0.264, 0.580, 2.705, 4.350, 6.425, 0, 0,0 ]
KINOVA_HOME_XYZ = (0.09, -0.446, 0.375)
KINOVA_HOME_ORIENTATION = (0.708, -0.019, 0.037, 0.705)

X_HIGH = 0.15
X_LOW = -0.15
Y_HIGH = -0.4
Y_LOW = -0.75
Z_HIGH = 0.45
Z_LOW = 0.35


class AgentKinova():
    VELOCITY_CONTROL = 1  # when 0, position control; when 1, velocity control

    #target_position = (0, -0.5, 0.4)
    bridge = CvBridge()
    MOVING = True  # when false, this program is to be shut down

    temp_angles = []
    home_angles = []
    pose = []

    def __init__(self,
                 control_rate = 10, # control rate
                 hand_low=(X_LOW, Y_LOW, Z_LOW),
                 hand_high=(X_HIGH, Y_HIGH, Z_HIGH),

                 ):

        self.control_rate = 10
        self.Kv = 0.02  # coefficient of velocity, that is, the process quantity for unit action.
        self.x_v = 0
        self.y_v = 0
        self.z_v = 0


        # setting the workspace wrt ee
        self.ee_X_upperLimit = hand_high[0]
        self.ee_X_lowerLimit = hand_low[0]
        self.ee_Y_upperLimit = hand_high[1]
        self.ee_Y_lowerLimit = hand_low[1]
        self.ee_Z_upperLimit = hand_high[2]
        self.ee_Z_lowerLimit = hand_low[2]


        rospy.init_node('kinova_control_node')
        self._init_pubs_and_subs()
        self.r = rospy.Rate(100)
        self.r.sleep()
        self.move_home_init()
        self.r.sleep()
        self.home_angles = self.temp_angles
        print('init finished')

    def _init_pubs_and_subs(self):
        # rospy.Subscriber('/j2s7s300_driver/out/tool_wrench', geometry_msgs.msg.WrenchStamped, self.robot_wrench_callback,
        #                  queue_size=1)
        rospy.Subscriber('/j2s7s300_driver/out/tool_pose', geometry_msgs.msg.PoseStamped, self.pose_callback, queue_size=1)
        # rospy.Subscriber('/j2s7s300_driver/out/joint_state', sensor_msgs.msg.JointState, self.joint_callback, queue_size=1)
        rospy.Subscriber('/j2s7s300_driver/out/joint_angles', kinova_msgs.msg.JointAngles, self.joint_angle_callback,
                          queue_size=1)
        # rospy.Subscriber('/kinova_agent/position_feed', Float32MultiArray, self.move_callback_velocity_control, queue_size=1)
        # if VELOCITY_CONTROL == 1:
        rospy.Subscriber('/kinova_agent/position_feed', msgs.msg.ActionCommand, self.move_callback_velocity_control, queue_size=1)
        # else:
        #     rospy.Subscriber('/target_goal', Float32MultiArray, move_callback_position_control, queue_size=1)
        self.velo_pub = rospy.Publisher('/j2s7s300_driver/in/cartesian_velocity', kinova_msgs.msg.PoseVelocity, queue_size=1)
        CURRENT_VELOCITY = [0, 0, 0, 0, 0, 0]
        self.velo_pub.publish(kinova_msgs.msg.PoseVelocity(*CURRENT_VELOCITY))
        self.r = rospy.Rate(100)
        self.home_srv = rospy.Service('/kinova_agent/srv/home', msgs.srv.Home, self.move_home)

    def pose_callback(self, pose_data):
        self.pose = [pose_data.pose.position.x,
                             pose_data.pose.position.y,
                             pose_data.pose.position.z
                             ]

    def joint_angle_callback(self, data):
        self.temp_angles = data

    def robot_wrench_callback(self, msg):
        if abs(msg.wrench.force.x) > 5.0 or abs(msg.wrench.force.y) > 5.0 or abs(msg.wrench.force.z) > 7.0:
            rospy.logerr('Force Detected. Stopping.')
            self.MOVING = False
            print(msg.wrench)
            print(self.temp_angles)
            if self.temp_angles:
                joint_angle_client([self.temp_angles.joint1, self.temp_angles.joint2, self.temp_angles.joint3,
                                    self.temp_angles.joint4, self.temp_angles.joint5, self.temp_angles.joint6,
                                    self.temp_angles.joint7])
                rospy.ServiceProxy('/j2s7s300_driver/in/start_force_control', kinova_msgs.srv.Start)

    def move_home_init(self):
        # move_to_position([x_r,y_r,z_r], [0.072, 0.6902, -0.7172, 0.064])

        move_to_position(KINOVA_HOME_XYZ, KINOVA_HOME_ORIENTATION)
        time.sleep(0.5)
        self.MOVING = True
        return 1

    def move_home(self,req):
        if req.home:
            self.MOVING = False
            self.x_v = 0
            self.y_v = 0
            self.z_v = 0
            joint_angle_client([self.home_angles.joint1, self.home_angles.joint2, self.home_angles.joint3,
                                self.home_angles.joint4, self.home_angles.joint5, self.home_angles.joint6,
                                self.home_angles.joint7])
            time.sleep(0.5)
            print('home!')
            self.MOVING = True
        return 1

    def move_callback_velocity_control(self, data):

        x_v = data.x * self.control_rate * self.Kv
        y_v = data.y * self.control_rate * self.Kv
        z_v = data.z * self.control_rate * self.Kv


        if self.pose[0] > self.ee_X_upperLimit :
            x_v = -abs(x_v)
        elif self.pose[0] < self.ee_X_lowerLimit :
            x_v = abs(x_v)
        if self.pose[1] > self.ee_Y_upperLimit :
            y_v = -abs(y_v)
        elif self.pose[1] <self.ee_Y_lowerLimit:
            y_v = abs(y_v)
        if self.pose[2] > self.ee_Z_upperLimit:
            z_v = -abs(z_v)
        elif self.pose[2] < self.ee_Z_lowerLimit:
            z_v = abs(z_v)
        self.x_v = x_v
        self.y_v = y_v
        self.z_v = z_v
        # move_to_position(disp,[0.072, 0.6902, -0.7172, 0.064])

    def __main__(self):
        while not rospy.is_shutdown():
            if self.MOVING:
                CURRENT_VELOCITY = [self.x_v, self.y_v, self.z_v, 0, 0, 0]
                self.velo_pub.publish(kinova_msgs.msg.PoseVelocity(*CURRENT_VELOCITY))
                self.r.sleep()


if __name__ == '__main__':


    # rospy.Timer(rospy.Duration(0.1),timer_callback)
    # from helpers.transforms import current_robot_pose, publish_tf_quaterion_as_transform, convert_pose, publish_pose_as_transform
    #
    # start_force_srv = rospy.ServiceProxy('/j2s7s300_driver/in/start_force_control', kinova_msgs.srv.Start)
    # stop_force_srv = rospy.ServiceProxy('/j2s7s300_driver/in/stop_force_control', kinova_msgs.srv.Stop)
    # time.sleep(1)
    # stop_srv = rospy.ServiceProxy('/j2s7s300_driver/in/start', kinova_msgs.srv.Start)
    # home_srv = rospy.ServiceProxy('/j2s7s300_driver/in/home_arm', kinova_msgs.srv.HomeArm)
    # home_srv()
    # time.sleep(1)

    kinova = AgentKinova()
    kinova.__main__()
    # data = dataIO.read_pickle()
    # img = cv2.imread(str(data[0][0]))
    # cv2.imshow('1', img)
    # time.sleep(10)