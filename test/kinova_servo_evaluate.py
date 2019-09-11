
from otter.gym.bullet.kinova import Kinova
import visdom

import pybullet as p
import numpy as np
import time
import pybullet_data
import matplotlib.pyplot as plt

KINOVA_HOME_ANGLE = [4.543, 3.370, -0.264, 0.580, 2.705, 4.350, 6.425, 0, 0,0 ]
timeStep = 0.01


numSimSteps = 1000


def test_kinova_velocity_control():
    kinova = env_build(p)
    q = []
    q_dot = []
    for simStep in range(numSimSteps):
        commands = np.ones(10)*0.15
        kinova.ApplyAction_Velocity(commands)
        q.append(kinova.GetMotorAngles())
        q_dot.append(kinova.GetMotorVelocities())

        p.stepSimulation()
        time.sleep(0.01)
    q = np.array(q)
    q_dot = np.array(q_dot)
    plot_results(q, q_dot, toque=None)

def test_kinova_position_control():
    kinova = env_build(p)
    q = []
    q_dot = []
    for simStep in range(numSimSteps):
        commands = np.ones(10)

        if simStep >= 300:
            commands[7:] = np.array([0.5, 0.5, 0.5])

        if simStep >= 700:
            commands[7:] = np.array([1.5, 1.5, 1.5])

        kinova.ApplyAction_Velocity(commands)
        q.append(kinova.GetMotorAngles())
        q_dot.append(kinova.GetMotorVelocities())

        p.stepSimulation()
        time.sleep(0.01)

    q = np.array(q)
    q_dot = np.array(q_dot)
    plot_results(q, q_dot, toque=None)


def test_kinova_Cartesian_Control():
    pass

def test_kinova_torque_Control():
    pass

def test_kinova_finger_control():
    kinova = env_build(p)
    q = []
    q_dot = []
    for simStep in range(numSimSteps):
        commands = np.ones(10)
        kinova.ApplyAction(commands)
        q.append(kinova.GetMotorAngles())
        q_dot.append(kinova.GetMotorVelocities())

        p.stepSimulation()
        time.sleep(0.01)
    q = np.array(q)
    q_dot = np.array(q_dot)
    plot_results(q, q_dot, toque=None)


def env_build(p):
    p.connect(p.GUI)
    p.resetSimulation()
    p.setPhysicsEngineParameter(numSolverIterations=150)
    p.setTimeStep(timeStep)

    kinova = Kinova(p, robot_type='j2s7s300',
                    urdfRootPath='/home/drl/PycharmProjects/DeployedProjects/pararl/otter/gym/bullet/assets',
                    timeStep=timeStep,
                    building_env=False,  # use gym env
                    useInverseKinematics=False,  # IMPORTANCE! It determines the mode of the motion.
                    torque_control_enabled=False,
                    is_fixed=True,
                    init_configuration=KINOVA_HOME_ANGLE,
                    state_vis=False,
                    robot_info_debug=False,
                    )
    p.setGravity(0, 0, -9.81)
    return kinova

def plot_results(q, q_dot, toque = None):
    plt.figure(figsize=(15, 15))
    t = np.arange(0, numSimSteps * 0.01, 0.01)
    for i in range(7):
        plt.subplot(3, 3, i + 1)
        plt.plot(t, q[:, i], 'r--')
        plt.title('Joint {} Position'.format(i + 1))

    plt.show()
    plt.figure(figsize=(15, 15))
    for i in range(7):
        plt.subplot(3, 3, i + 1)
        plt.plot(t, q_dot[:, i], 'r--')
        plt.title('Joint {} Velocity'.format(i + 1))
    plt.show()



#test_kinova_position_control()
#test_kinova_velocity_control()