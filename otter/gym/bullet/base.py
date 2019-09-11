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
import cv2

largeValObservation = 100

RENDER_HEIGHT = 480
RENDER_WIDTH = 480

N_DISCRETE_ACTIONS = 7 # depends on tasks
DELTA_V = 0.01
NOISE_STD_DISCRETE = 0.002  # Add noise to actions, so the env is not fully deterministic  0.01

DELTA_V_CONTINUOUS = 0.002  # velocity per physics step (for continuous relative control).
NOISE_STD_CONTINUOUS = 0.000 # noise standard deviation (for continuous relative control)
NOISE_STD_ABSOLUTE_ACTION = 0.0005 # noise standard deviation(for absolute control )


dv = DELTA_V  # velocity per physics step.

KINOVA_HOME_ANGLE = [4.543, 3.370, -0.264, 0.580, 2.705, 4.350, 6.425, 0, 0,0 ]
KINOVA_HOME_EE_POS = [0.092, -0.443, 0.369]
KINOVA_HOME_EE_ORN = [0.727, -0.01, 0.029, 0.686]

# the first camera paremeters
CAMERA_TARGET_POS = (0.0, -0.735, -0.105)
CAM_DIS = 1.158
CAM_YAW  = 174.7
CAM_PITCH  = -64.4
CAM_ROLL = 0

# the second camera paremeters, only if multi-view is True
CAMERA_TARGET_POS_2RD = (0.0, -0.735, -0.105)
CAM_DIS_2RD = 1.158
CAM_YAW_2RD  = 174.7
CAM_PITCH_2RD  = -64.4
CAM_ROLL_2RD = 0


X_HIGH = 0.3
X_LOW = -0.3
Y_HIGH = -0.3
Y_LOW = -0.85
Z_HIGH = 0.6
Z_LOW = 0.2

from ..gym_wrapper import GymWrapper

__all__ = ['KinovaXYZ']


class KinovaXYZ(gym.Env, metaclass=abc.ABCMeta,):

    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 50
    }

    def __init__(self,
                 isDiscrete= False,
                 isAbsolute_control = False,
                 timeStep = 0.01,
                 urdfRoot = pybullet_data.getDataPath(),
                 actionRepeat = 10,
                 isEnableSelfCollision = True,
                 isRender = False,
                 maxSteps = 1000,
                 debug = False,
                 multi_view = False,
                 hard_reset = True,
                 isImageObservation = True,

                 default_goal = (0,0,0),
                 random_init_arm_angle = False,
                 state_vis = False,
                 robot_info_debug = False,

                 image_height = RENDER_HEIGHT,
                 image_width=RENDER_WIDTH,

                 hand_low=(X_LOW, Y_LOW, Z_LOW),
                 hand_high=(X_HIGH, Y_HIGH, Z_HIGH),
                 *args, **kwargs):
        """

        :param isDiscrete:  whether to use discrete mode to control the ee
        :param isAbsolute_control: wheter to use absolute positon of ee to control (just for debug)
        :param timeStep: the physics step time, e.g. 0.01
        :param urdfRoot: the path of robot urdf
        :param actionRepeat: the step times of each action, which can adjust the control rate
        :param isEnableSelfCollision: whether to avoid collison
        :param isRender: whether to render during simulation
        :param maxSteps: the maximum of physics steps for each episode
        :param debug: True could adjust some parameters, e.g:camera or sth.
        :param multi_view: open the second camera
        :param hard_reset: if true, the urdf of robot and env will be loaded again after reset
        :param isImageObservation: if true, observation is raw iamges
        :param random_target: whether to random the target positon
        :param default_goal: the default positon of the target
        :param random_init_arm_angle: whether to random the initial angle of the robot
        :param image_height:
        :param image_width:
        :param args:
        :param kwargs:
        """
        self.__dict__.update(kwargs)
        self._isDiscrete = isDiscrete
        self._isAbsolute_control = isAbsolute_control
        self._timeStep = timeStep
        self._urdfRoot =  urdfRoot
        self._actionRepeat =  actionRepeat
        self._isEnableSelfCollision =  isEnableSelfCollision


        print('action repeat :', self._actionRepeat)
        self._renders = isRender
        self._maxSteps = maxSteps
        self._debug = debug
        self._multi_view = multi_view

        self._isImageObservation = isImageObservation
      #  self._random_target = random_target
        self._default_goal = default_goal
        self._random_init_arm_angle = random_init_arm_angle

        self._state_vis = state_vis
        self._robot_info_debug =  robot_info_debug

        self.terminated = 0
        self._observation = []
        self._envStepCounter = 0
        self.hard_reset = hard_reset

        self.camera_target_pos = CAMERA_TARGET_POS
        self._cam_dist = CAM_DIS
        self._cam_yaw = CAM_YAW
        self._cam_pitch = CAM_PITCH
        self._cam_roll = CAM_ROLL
        self.renderer = p.ER_TINY_RENDERER  #p.ER_BULLET_HARDWARE_OPENGL  #

        self.hand_high = hand_high
        self.hand_low = hand_low

        self._image_height = image_height
        self._image_width = image_width
        GUI_FLAG = False
        self._p = p

        print('numpy:', self._p.isNumpyEnabled())
        if self._renders:
            if not GUI_FLAG:
                cid = p.connect(p.SHARED_MEMORY)
                if (cid < 0):
                    cid = p.connect(p.GUI)
                else:
                    p.connect(p.DIRECT)
                p.resetDebugVisualizerCamera(1.5, 180, -41, [0.32, 0.1, -0.00])
            else:
                p.connect(p.DIRECT)

            if self._debug:
                # Debug sliders for moving the camera
                self.x_slider = p.addUserDebugParameter("x_slider", -10, 10, self.camera_target_pos[0])
                self.y_slider = p.addUserDebugParameter("y_slider", -10, 10, self.camera_target_pos[1])
                self.z_slider = p.addUserDebugParameter("z_slider", -10, 10, self.camera_target_pos[2])
                self.dist_slider = p.addUserDebugParameter("cam_dist", 0, 10, self._cam_dist)
                self.yaw_slider = p.addUserDebugParameter("cam_yaw", -200, 200, self._cam_yaw)
                self.pitch_slider = p.addUserDebugParameter("cam_pitch", -180, 180, self._cam_pitch)

            self.debug_ino_step = self._p.addUserDebugText('step: %d' % self._envStepCounter,
                                                                         [-0.8, 0, 0.0], textColorRGB=[0, 0, 0],
                                                                         textSize=1.5 )
        else:
            p.connect(p.DIRECT)

        self._hard_reset = True
        self._robot_urdfRoot = os.path.join(currentdir, 'assets')

        # timinglog = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "kukaTimings.json")
        self._seed()
        self.reset()

        # define Action Space
        if self._isDiscrete:
            self.action_space = spaces.Discrete(N_DISCRETE_ACTIONS)
        else:
            if self._isAbsolute_control:
                # 3 position for end-effector wrt the Cartesian Space
                action_dim = 3
                self._action_bound = 1

                # TODO action need to be scaled to the limitation of EE.
            else:
                # 3 direction for the arm movement, from -1 to 1
                action_dim = 3
                self._action_bound = 1
            action_high = np.array([self._action_bound] * action_dim)
            self.action_space = spaces.Box(-action_high, action_high )

        # define Obsevation Space
        observationDim = len(self._get_obs())
        observation_high = np.array([largeValObservation] * observationDim)
        self.observation_space = spaces.Box(-observation_high, observation_high)
        self.viewer = None

        self._hard_reset = self.hard_reset  # This assignment need to be after reset()
    def build_env(self):
        pass

    def reset(self):
        if self._hard_reset:
            p.resetSimulation()
            p.setPhysicsEngineParameter(numSolverIterations=150)
            p.setTimeStep(self._timeStep)

            # build gym env
            self.plane = p.loadURDF(os.path.join(self._urdfRoot, "plane.urdf"), [0, 0, -1])
            p.changeDynamics(self.plane, -1, contactStiffness = 1. , contactDamping = 1)
            self.tableUid = p.loadURDF(os.path.join(self._robot_urdfRoot, "table/table.urdf"),
                                       [0.0000000, -0.650000, -0.380000],
                                       [0.000000, 0.000000, 0.0, 1.0],
                                      useFixedBase=True)

            p.changeDynamics(self.tableUid, 0, lateralFriction = 1.0, spinningFriction = 1.0, rollingFriction= 1.0)
            print('fiction of table :', p.getDynamicsInfo(self.tableUid, -1))

            self.build_env()
            p.setGravity(0, 0, -9.81)

            # camera configuration
            self.view_matrix1 = p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=self.camera_target_pos,
                distance=self._cam_dist,
                yaw=self._cam_yaw,
                pitch=self._cam_pitch,
                roll=self._cam_roll,
                upAxisIndex=2)
            self.proj_matrix1 = p.computeProjectionMatrixFOV(
                fov=60, aspect=float(self._image_width) / self._image_height,
                nearVal=0.1, farVal=100.0)

            if self._multi_view:
                # adding a second camera on the other side of the robot
                self.view_matrix2 = p.computeViewMatrixFromYawPitchRoll(
                    cameraTargetPosition= CAMERA_TARGET_POS_2RD,
                    distance= CAM_DIS_2RD,
                    yaw= CAM_YAW_2RD,
                    pitch= CAM_PITCH_2RD,
                    roll=CAM_ROLL_2RD,
                    upAxisIndex=2)
                self.proj_matrix2 = p.computeProjectionMatrixFOV(
                    fov=60, aspect=float(self._image_width) / self._image_height,
                    nearVal=0.1, farVal=100.0)

            self.robot_init_pos = self.get_init_joint_angle()


            # load the kinova
            self.kinova = kinova.Kinova(p,  robot_type='j2s7s300',
                                        urdfRootPath=self._robot_urdfRoot,
                                        timeStep=self._timeStep,
                                        building_env=False,  # use gym env
                                        useInverseKinematics=True,  # IMPORTANCE! It determines the mode of the motion.
                                        torque_control_enabled=False,
                                        is_fixed=True,
                                        init_configuration = self.robot_init_pos,
                                        state_vis = self._state_vis,
                                        robot_info_debug = self._robot_info_debug,
                                        hand_high= self.hand_high,
                                        hand_low= self.hand_low)
        else:

            self.kinova.reset(reload_urdf=False)

        self.terminated = 0
        self._envStepCounter = 0

        self.kinova.useInverseKinematics = False
        INIT_RESET_STEPS = 100
        for i in range(INIT_RESET_STEPS):
            action = self.robot_init_pos
            self.kinova.ApplyAction(action)
            p.stepSimulation()  #TODO
        self.kinova.useInverseKinematics = True

        self._observation = self._get_obs()
        return np.array(self._observation)
    def get_init_joint_angle(self):
        if self._random_init_arm_angle:
            #TODO write a random function for start positon
            #start_pos = np.random.uniform(low=-np.pi, high=np.pi, size=self.model.nq)
            raise print('random function is not completed!')
        else:
            start_pos = KINOVA_HOME_ANGLE

        return start_pos


    def __del__(self):
        p.disconnect()

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _get_obs(self):
        if self._isImageObservation:
            img = self.render("rgb_array")

            self._observation = (cv2.resize(img, (self._image_width, self._image_height),
                                            interpolation= cv2.INTER_LINEAR) / 255).flatten()
        else:
            kinovaState = self.kinova.GetObservation()
            self._observation = \
                np.concatenate([np.array(kinovaState),
                                 ]).flatten()
        return self._observation

    def step(self, action):
        t1 = time.time()
        finger_angle = [0.0]  # Close the gripper
        orn = KINOVA_HOME_EE_ORN

        if (self._isDiscrete):
            if self._isAbsolute_control:
                raise Exception('Error. Discrete mode is not used in Absolute Control.')
            dv = DELTA_V  # velocity per physics step.
            # Add noise to action
            dv += self.np_random.normal(0.0, scale=NOISE_STD_DISCRETE)
            dx = [0,-dv, dv, 0, 0, 0, 0][action]
            dy = [0, 0, 0, -dv, dv, 0, 0][action]
            dz = [0, 0, 0, 0, 0, -dv, dv][action]

            realAction = np.concatenate(([dx, dy, dz], orn, finger_angle))
        else:
            if self._isAbsolute_control:
                action += self.np_random.normal(0.0, scale=NOISE_STD_ABSOLUTE_ACTION)
                realAction = np.concatenate((action, orn, finger_angle))
            else:
                dv = DELTA_V_CONTINUOUS
                # Add noise to action
                dv += self.np_random.normal(0.0, scale=NOISE_STD_CONTINUOUS)

                dx = action[0] * dv
                dy = action[1] * dv
                dz = action[2] * dv

                realAction = np.concatenate(([dx, dy, dz], orn, finger_angle))
        t2 = time.time()
        #print('step time :', t2-t1)
        return self.step2(realAction)

    def step2(self, action):
        t1 = time.time()
        for i in range(self._actionRepeat):
            if self._isAbsolute_control:
                self.kinova.ApplyAction_EndEffectorPose(action)
            else:
                self.kinova.ApplyAction(action)

            p.stepSimulation()
            self._envStepCounter += 1
            # if self._termination():
            #     break

        if self._renders:
            time.sleep(0.001)
            if self._renders is True and self._envStepCounter % 5 == 0:
                self.debug_ino_step = self._p.addUserDebugText('step: %d' % self._envStepCounter, [-0.8, 0, 0.0],
                                                           textColorRGB=[1, 0, 0], textSize=1.5, replaceItemUniqueId=self.debug_ino_step)

        self._observation = self._get_obs()

        #t2 = time.time()
        #print('step time :', t2 - t1)
        done = self._termination()
        # npaction = np.array(
        #     [action[3]])  # only penalize rotation until learning works well [action[0],action[1],action[3]])

        #end_pos = self._observation[0:3]
        #pos_delta = end_pos - self.goal_point

        reward = self._reward(self._observation, action)



        return np.array(self._observation), reward, done, {}

    def render(self, mode="human",  close=False):
        if mode != "rgb_array":
            return np.array([])

        t1 = time.time()
        if self._debug:
            self._cam_dist = p.readUserDebugParameter(self.dist_slider)
            self._cam_yaw = p.readUserDebugParameter(self.yaw_slider)
            self._cam_pitch = p.readUserDebugParameter(self.pitch_slider)
            x = p.readUserDebugParameter(self.x_slider)
            y = p.readUserDebugParameter(self.y_slider)
            z = p.readUserDebugParameter(self.z_slider)
            camera_target_pos = (x, y, z)
            #self._cam_roll = p.readUserDebugParameter(self.roll_slider)

            # TODO: recompute view_matrix and proj_matrix only in debug mode
            self.view_matrix1 = p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=camera_target_pos,
                distance=self._cam_dist,
                yaw=self._cam_yaw,
                pitch=self._cam_pitch,
                roll=self._cam_roll,
                upAxisIndex=2)
            self.proj_matrix1 = p.computeProjectionMatrixFOV(
                fov=60, aspect=float(self._image_width) / self._image_height,
                nearVal=0.1, farVal=100.0)

        (_, _, px1, _, _) = p.getCameraImage(width=self._image_width, height=self._image_height, viewMatrix=self.view_matrix1,
                                             projectionMatrix=self.proj_matrix1, renderer=self.renderer ,flags=p.ER_NO_SEGMENTATION_MASK,)
        t2 = time.time()
        rgb_array1 = np.array(px1, dtype=np.uint8)

        if self._multi_view:
            # adding a second camera on the other side of the robot
            (_, _, px2, _, _) = p.getCameraImage(width=self._image_width, height=self._image_height, viewMatrix=self.view_matrix2,
                                                    projectionMatrix=self.proj_matrix2, renderer=self.renderer)
            rgb_array2 = np.array(px2)
            rgb_array_res = np.concatenate((rgb_array1[:, :, :3], rgb_array2[:, :, :3]), axis=2)
        else:
            rgb_array_res = rgb_array1[:, :, :3]


        #print('img time :', t2 - t1)
        return rgb_array_res

    def _termination(self):
        if self.terminated or self._envStepCounter > self._maxSteps:
            self._observation = self._get_obs()
            return True
        return False

    def _reward(self, obs, action):
        reward_dist = -np.square(obs).sum()
        reward_ctrl = -np.linalg.norm(action) * 10
        dist = -obs
        return 0