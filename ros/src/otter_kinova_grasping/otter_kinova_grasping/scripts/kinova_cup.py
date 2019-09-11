#! /usr/bin/env python
import os
import inspect
# currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
# #print ("current_dir=" + currentdir)
# os.sys.path.insert(0,currentdir)
# import rospy
# import tf.transformations as tft

import numpy as np
import abc

import kinova_msgs.msg
import kinova_msgs.srv
import std_msgs.msg
import std_srvs.srv
import geometry_msgs.msg
import sensor_msgs.msg
import actionlib_msgs.msg
# import otter_kinova_grasping.ActionCommand.msg
import msgs.msg
import msgs.srv

from .kinova_agent_base import *

# from otter_kinova_grasping.otter_kinova_grasping.scripts.helpers.gripper_action_client import set_finger_positions
# from otter_kinova_grasping.otter_kinova_grasping.scripts.helpers.position_action_client import position_client, move_to_position
# from otter_kinova_grasping.otter_kinova_grasping.scripts.helpers.joints_action_client import joint_angle_client
# from otter_kinova_grasping.otter_kinova_grasping.scripts.helpers.covariance import generate_cartesian_covariance
# import kinova_angle_home


import time
import random
import pickle
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import collections



__all__ = [ 'ROSImageKinovaCupPusherEnv']

DEFAULT_TARGET_POS = (-0.15, -0.75, 0.25)
CPU_DEFALUT_POSITION = [0., -0.55, 0.25]

class ROSKinovaCupPusher(AgentROSbase,metaclass=abc.ABCMeta,):
    def __init__(self,
                 random_init_cup_position=True,
                 random_target_position=False,
                 default_target_position=DEFAULT_TARGET_POS,
                 isImageObservation = True,
                 **kwargs
                 ):

        self._isImageObservation = isImageObservation
        self._default_target_positon = list(default_target_position)
        self._random_target_position = random_target_position

        self._random_init_cup_position = random_init_cup_position
        AgentROSbase.__init__(self, **kwargs)


    def set_target_pos(self):
        if self._random_target_position:
            goal_point = np.array([self.np_random.uniform(-0.25, 0.25), self.np_random.uniform(-0.8, -0.6), 0.25])
        else:
            goal_point = np.array(self._default_target_positon)

        self.target_pos = goal_point


    def get_cup_pos(self):
        # TODO
        cup_pos =  [0,0,0]
        return  cup_pos

    def set_init_cup_pos(self):
        if self._random_init_cup_position:
            init_pos = np.array([self.np_random.uniform(-0.25, 0.25), self.np_random.uniform(-0.46, -0.55), 0.25])
        else:

            init_pos = CPU_DEFALUT_POSITION

        return init_pos

    def reward(self, obs, action):
        cupPos = self.get_cup_pos()

        # TODO
        dist = np.array(cupPos) - self.target_pos
        reward_dist = -np.square(dist).sum()

        return reward_dist

    def get_state_dim(self):
        #TODO
        return  3*128*128

    def get_action_dim(self):
        return  3

    def _observe(self):
       if  self._isImageObservation:
           obs = self.rollout_temp.image.flatten()
       else:
           obs = np.concatenate(
                (self.rollout_temp.joint_angle, \
                 self.rollout_temp.joint_velocity))
           
       return obs

    def reset(self):
        obs = super().reset()
        self.set_target_pos()
        return obs



class ROSImageKinovaCupPusherEnv(ROSKinovaCupPusher):
    environment_name = 'ROSImageKinovaCupPusherEnv-v0'
     
    
    def __init__(self, **kwargs):
        config = {
            'control_rate': kwargs.pop('control_rate', 10.0),
            
            #'hand_low': kwargs.pop('hand_low', (X_LOW, Y_LOW, Z_LOW)),
            #'hand_high': kwargs.pop('hand_high', (X_HIGH, Y_HIGH, Z_HIGH)),
            'isImageObservation': kwargs.pop('isImageObservation', True),
            'random_init_arm_angle': kwargs.pop('random_init_arm_angle', False),  #TODO

            # task
            'random_init_cup_position': kwargs.pop('random_init_cup_position', True),
            'random_target_position': kwargs.pop('random_target_position', False),
            'default_target_position': kwargs.pop('default_target_position', DEFAULT_TARGET_POS),

            # new parameters
            'image': kwargs.pop('image', True),
            'image_dim': kwargs.pop('image_dim', 128),
            'sliding_window': kwargs.pop('sliding_window', 0),

        }
        if config['image'] is True:
            config['isImageObservation'] = True
            config['image_height'] = config['image_width'] = config['image_dim']
        super(ROSImageKinovaCupPusherEnv, self).__init__(**config)
        
    def torque_matrix(self):
        return 2 * np.eye(self.get_action_dim())

    def make_summary(self, observations, name):
        if self.image:
            pass
            observations = T.reshape(observations, [-1] + self.image_size())
            T.core.summary.image(name, observations)

    def is_image(self):
        return self.image

    def image_size(self):
        if self.image:
            return [self.image_dim, self.image_dim, 3]
        return None

    #def cost_fn(self, s, a):
    #    return np.linalg.norm(s[:,-3:], axis=-1) + np.sum(np.square(a), axis=-1)

     

#if __name__ == '__main__':
def _test():
    cupAgent = CupAgentROS()
    print('set')
    DIR1 = grandgrandparentdir
    DIR2 = str(rospy.get_time())
    DIR = DIR1+'/data/'+DIR2
    os.makedirs(DIR, mode=0o777)
    dataIO = IO(DIR + '/cup_data.pkl')
    dataIO.to_pickle(cupAgent.rollouts(3, 30, cupAgent.policy))
    cupAgent.rollouts(3, 30, cupAgent.policy)

