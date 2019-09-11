import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
os.sys.path.insert(0,currentdir)

import numpy as np
import pybullet_data
import pybullet as p


from .base_vel import  KinovaJointControl
from ..gym_wrapper import GymWrapper

__all__ = [ 'ImageKinovaReacherJointEnv']

X_HIGH = 0.3
X_LOW = -0.3
Y_HIGH = -0.3
Y_LOW = -0.85
Z_HIGH = 0.6
Z_LOW = 0.2

class ImageKinovaReacherJoint(KinovaJointControl):
    def __init__(self,
                 random_target=False,
                 default_goal=(0.2, -0.8, 0.25),
                 target_upper_space=(0.2, -0.75, 0.25),
                 target_lower_space=(-0.2, -0.85, 0.25),
                 **kwargs):

        self._random_target = random_target
        self._default_target_positon = default_goal
        self._target_upper_space = target_upper_space
        self._target_lower_space = target_lower_space
        self.OutrangeCost = -20
        KinovaJointControl.__init__(self, **kwargs)

    def build_env(self):
        self.targetUid = p.loadURDF(os.path.join(self._robot_urdfRoot, "urdf/ball.urdf"),
                                    self._default_target_positon,
                                    [0, 0, 0, 1],
                                    useFixedBase=True)
    def reset(self):
        obs = super().reset()
        self.goal_point = self.get_target_pos()
        self._p.resetBasePositionAndOrientation(self.targetUid, self.goal_point, [0, 0, 0, 1])
        return obs

    def get_target_pos(self):
        if self._random_target:
            goal_point = np.array([self.np_random.uniform(self._target_lower_space[0], self._target_upper_space[0]),
                                   self.np_random.uniform(self._target_lower_space[1], self._target_upper_space[1]),
                                   self.np_random.uniform(self._target_lower_space[2], self._target_upper_space[2])])
        else:
            goal_point = np.array(self._default_goal)
        return  goal_point


    def _reward(self, obs, action):

        torque = self.kinova.GetMotorTorques()
        kinovaEEPos, _ = self.kinova.GetEndEffectorObersavations()
        dist = np.array(kinovaEEPos) - self.goal_point

        reward_dist = -np.square(dist).sum()
        reward_ctrl = -np.linalg.norm(torque) * 0.1

        reward = reward_dist + reward_ctrl
        return reward

    def _termination(self):
        if self.terminated or self._envStepCounter > self._maxSteps:
            self._observation = self._get_obs()
            return True, 0
        if self.kinova.CheckEEpos():
            return True, self.OutrangeCost
        return False, 0

class ImageKinovaReacherJointEnv(GymWrapper):
    environment_name = 'ImageKinovaReacherJointEnv-v0'
    entry_point = "otter.gym.bullet.KinovaReacherJoint:ImageKinovaReacherJoint"
    max_episode_steps = 1000
    reward_threshold = 5000

    def __init__(self, **kwargs):
        config = {
            'isVelocity': kwargs.pop('isVelocity', True),

            'timeStep': kwargs.pop('timeStep', 0.01),
            'actionRepeat': kwargs.pop('actionRepeat', 10),
            'isEnableSelfCollision': kwargs.pop('isEnableSelfCollision', True),
            'urdfRoot': kwargs.pop('urdfRoot', pybullet_data.getDataPath()),
            'isRender': kwargs.pop('isRender', True),
            'maxSteps': kwargs.pop('maxSteps', 1000),
            'debug': kwargs.pop('debug', False),
            'multi_view': kwargs.pop('multi_view', False),
            'hard_reset': kwargs.pop('hard_reset', False),
            'state_vis': kwargs.pop('state_vis', False),

            'isImageObservation': kwargs.pop('isImageObservation', True),
            'random_target': kwargs.pop('random_target', False),
            'random_init_arm_angle': kwargs.pop('random_init_arm_angle', False),
            'default_goal': kwargs.pop('default_goal', [0.5, 0, 0.5]),

            # new parameters
            'image': kwargs.pop('image', True),
            'image_dim': kwargs.pop('image_dim', 480),
            'sliding_window': kwargs.pop('sliding_window', 0),

        }
        if config['image'] is True:
            #config['isImageObservation'] = True
            config['image_height'] = config['image_width'] = config['image_dim']

        super(ImageKinovaReacherJointEnv, self).__init__(config)

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