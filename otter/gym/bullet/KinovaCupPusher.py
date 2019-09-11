import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
#print ("current_dir=" + currentdir)
os.sys.path.insert(0,currentdir)

import abc
import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import time
import pybullet as p
from . import kinova
import random
import pybullet_data
from pkg_resources import parse_version
import os
# from deepx import *
# import cv2


from .base import  KinovaXYZ
from ..gym_wrapper import GymWrapper



DEFAULT_TARGET_POS = (-0.15, -0.75, 0.25)
CPU_DEFALUT_POSITION = [0., -0.55, 0.25]

X_HIGH = 0.3
X_LOW = -0.3
Y_HIGH = -0.3
Y_LOW = -0.85
Z_HIGH = 0.6
Z_LOW = 0.2

__all__ = [ 'ImageKinovaCupPusherEnv']

class ImageKinovaCupPusher(KinovaXYZ):
    def __init__(self,
                 random_init_cup_position= True,
                 random_target_position = False,
                 default_target_position = DEFAULT_TARGET_POS,
                 **kwargs):

        self._default_target_positon = list(default_target_position)
        self._random_target_position = random_target_position


        self._random_init_cup_position = random_init_cup_position

        KinovaXYZ.__init__(self,  **kwargs)

    def build_env(self):
        cup_init_position = self.set_init_cup_pos()
        orn = p.getQuaternionFromEuler([math.pi/2, 0, 0])

        self.teacupUid = p.loadURDF(os.path.join(self._robot_urdfRoot, "urdf/coffee_cup.urdf"),
                                    cup_init_position,
                                    [orn[0], orn[1], orn[2], orn[3]],
                                    useFixedBase=False)


        self.targetUid = p.loadURDF(os.path.join(self._robot_urdfRoot, "urdf/ball.urdf"),
                                    self._default_target_positon,
                                    [0,0,0,1],
                                    useFixedBase=True)

        tea_dyn = p.getDynamicsInfo(self.teacupUid, -1)
        print('fiction of tea cup:', tea_dyn)

    def reset(self):
        if not self._hard_reset:
            cup_init_position = self.set_init_cup_pos()
            orn = p.getQuaternionFromEuler([math.pi / 2, 0, 0])
            self._p.resetBasePositionAndOrientation(self.teacupUid, cup_init_position, orn)
            # print('reset goal point:', self.goal_point)
        obs = super().reset()

        self.set_target_pos()

        return  obs


    def set_target_pos(self):
        if self._random_target_position:
            goal_point = np.array([self.np_random.uniform(-0.25, 0.25), self.np_random.uniform(-0.8, -0.6), 0.25])
        else:
            goal_point = np.array(self._default_target_positon)

        self.target_pos = goal_point

        self._p.resetBasePositionAndOrientation(self.targetUid, self.target_pos, [0,0,0,1])

    def get_cup_pos(self):
        cup_pos, _ = self._p.getBasePositionAndOrientation(self.teacupUid)
        return  cup_pos

    def set_init_cup_pos(self):
        if self._random_init_cup_position:
            init_pos = np.array([self.np_random.uniform(-0.25, 0.25), self.np_random.uniform(-0.46, -0.55), 0.25])
        else:

            init_pos = CPU_DEFALUT_POSITION

        return init_pos

    def _reward(self, obs, action):

        # if contact_with_table or self.n_contacts >= N_CONTACTS_BEFORE_TERMINATION \
        #         or self.n_steps_outside >= N_STEPS_OUTSIDE_SAFETY_SPHERE:
        #     self.terminated = True
        kinovaEEPos, _ = self.kinova.GetEndEffectorObersavations()
        cupPos = self.get_cup_pos()

        #TODO
        dist = np.array(cupPos) - self.target_pos
        reward_dist = -np.square(dist).sum()

        #reward_ctrl = -np.linalg.norm(action) * 10
        #dist = -obs
        return reward_dist

class ImageKinovaCupPusherEnv(GymWrapper):
    environment_name = 'ImageKinovaCupPusherEnv-v0'
    entry_point = "otter.gym.bullet.KinovaCupPusher:ImageKinovaCupPusher"
    max_episode_steps = 1000
    reward_threshold = -3.75

    def __init__(self, **kwargs):
        config = {
            'isDiscrete': kwargs.pop('isDiscrete', False),
            'isAbsolute_control':kwargs.pop('isAbsolute_control', False),
            'timeStep': kwargs.pop('timeStep', 0.01),
            'actionRepeat': kwargs.pop('actionRepeat', 10),
            'isEnableSelfCollision': kwargs.pop('isEnableSelfCollision', True),
            'urdfRoot': kwargs.pop('urdfRoot', pybullet_data.getDataPath()),
            'isRender': kwargs.pop('isRender', False),
            'maxSteps': kwargs.pop('maxSteps', 1000),
            'debug': kwargs.pop('debug', False),
            'multi_view': kwargs.pop('multi_view', False),
            'hard_reset': kwargs.pop('hard_reset', False),

            'state_vis': kwargs.pop('state_vis', False),
            'robot_info_debug': kwargs.pop('robot_info_debug', False),
            'hand_low': kwargs.pop('hand_low', (X_LOW, Y_LOW, Z_LOW)),
            'hand_high': kwargs.pop('hand_high', (X_HIGH, Y_HIGH, Z_HIGH)),

            'isImageObservation': kwargs.pop('isImageObservation', True),
            'random_init_arm_angle': kwargs.pop('random_init_arm_angle', False),


            # task
            'random_init_cup_position': kwargs.pop('random_init_cup_position', True),
            'random_target_position': kwargs.pop('random_target_position', False),
            'default_target_position': kwargs.pop('default_target_position', DEFAULT_TARGET_POS),

            # new parameters
            'image': kwargs.pop('image', True),
            'image_dim': kwargs.pop('image_dim', 48),
            'sliding_window': kwargs.pop('sliding_window', 0),

        }
        if config['image'] is True:
            config['isImageObservation'] = True
            config['image_height'] = config['image_width'] = config['image_dim']
        super(ImageKinovaCupPusherEnv, self).__init__(config)#TODO check param transfer

    def torque_matrix(self):
        return 2 * np.eye(self.get_action_dim())

    def make_summary(self, observations, name):
        if self.image:
            pass
            # observations = T.reshape(observations, [-1] + self.image_size())
            # T.core.summary.image(name, observations)

    def is_image(self):
        return self.image

    def image_size(self):
        if self.image:
            return [self.image_dim, self.image_dim, 3]
        return None

    def cost_fn(self, s, a):
        return np.linalg.norm(s[:,-3:], axis=-1) + np.sum(np.square(a), axis=-1)