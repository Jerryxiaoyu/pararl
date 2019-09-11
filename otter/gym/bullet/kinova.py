import os,  inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)
import visdom
import pybullet as p
import numpy as np
import copy
import math
import pybullet_data
import matplotlib.pyplot as plt


## [0] coresponding to position noise stv, [1] -- velocity noise stv, [2] -- torque noise stv
SENSOR_NOISE_STDDEV = (0.0, 0.0, 0.0)
#pybullet_data.getDataPath()
INIT_ENDEFFORTPOSITION =   [0.09, -0.4, 0.4]
INIT_ENDEFFORTANGLE = 0

KINOVA_HOME_EE_POS = (0.092, -0.443, 0.369)
KINOVA_HOME_EE_ORN = (0.727, -0.01, 0.029, 0.686)


MAX_VELOCITY = 0.35

INIT_CONFIGURATION =  [math.pi/2, math.pi, math.pi, math.pi/6, 0, math.pi/2, 0, 1, 1, 1]

X_HIGH = 0.3
X_LOW = -0.3
Y_HIGH = -0.3
Y_LOW = -0.85
Z_HIGH = 0.6
Z_LOW = 0.2

class Kinova:
  #TODO urdf path
  def __init__(self,
               pybullet_client,
               robot_type = 'j2s7s300',
               urdfRootPath=os.path.abspath('../model'),
               timeStep=0.01,
               building_env = True,
               useInverseKinematics = True,
               torque_control_enabled = False,
               is_fixed = True,
               init_configuration = INIT_CONFIGURATION,
               verbose = False,
               state_vis= False,
               robot_info_debug= False,

               hand_low= (X_LOW, Y_LOW, Z_LOW),
               hand_high = (X_HIGH, Y_HIGH, Z_HIGH),
               basePosition = ( -0.000000, 0.000000, 0.000000 ),
               baseOrientationEuler = (0.000000, 0.000000, 0)
               ):

    self.robot_type = robot_type
    self._pybullet_client = pybullet_client
    self.urdfRootPath = urdfRootPath
    self.timeStep = timeStep


    # the initial position of robot wrt world space. I suggest the positon be set to zero so that the local space is equvelent to the world space. It's convienent for inverse kinematics.
    self._basePosition = basePosition
    self._baseOrientation = self._pybullet_client.getQuaternionFromEuler(baseOrientationEuler)
    self._init_jointPositions = init_configuration


    self._torque_control_enabled = torque_control_enabled
    self._observation_noise_stdev = SENSOR_NOISE_STDDEV


    self.useInverseKinematics = useInverseKinematics # 0
    self.useSimulation = 1
    self.useNullSpace = 1      # set 1, otherwise cause robot unstable
    self.useOrientation = 1   # set 1, otherwise cause robot unstable

    self.EndEffectorLinkName = '{}_joint_end_effector'.format(self.robot_type)
    self.numFingers = int(self.robot_type[5])# to test
    self.OnlyEndEffectorObervations = useInverseKinematics
    self._is_fixed = is_fixed
    self.verbose  = verbose
    self._robot_info_debug = robot_info_debug

   # kinova parameters
    self.maxForce = 30.
    self.max_velocity = MAX_VELOCITY

    self.fingerAForce = 2
    self.fingerBForce = 2.5
    self.fingerTipForce = 2

    # reset the kinova
    if building_env:
      self.build_env()
    self.reset()

    # ik paramters
    # lower limits for null space
    self.ll = self.jointLowerLimit[:self.numMotors ]
    # upper limits for null space
    self.ul = self.jointUpperLimit[:self.numMotors ]
    # joint ranges for null space
    self.jr = [3.5, 4, 6.8, 4.8, 5.8, 4.5, 7, 0.01, 0.01, 0.01]
    # restposes for null space
    self.rp = [1.1, 2.8, -3.2, 0.6, -0.505, 1.9, 0.12, 0.01, 0.01, 0.01]
    #joint damping coefficents
    self.jd = [5, 5, 5, 5, 5, 5, 5, 0.01, 0.01,   0.01]

    # setting the workspace wrt ee
    self.ee_X_upperLimit = hand_high[0]
    self.ee_X_lowerLimit = hand_low[0]
    self.ee_Y_upperLimit = hand_high[1]
    self.ee_Y_lowerLimit = hand_low[1]
    self.ee_Z_upperLimit = hand_high[2]
    self.ee_Z_lowerLimit = hand_low[2]


    if self.useInverseKinematics:
       ee_res = self.GetEndEffectorObersavations()
       self.endEffectorPos = list(KINOVA_HOME_EE_POS) #[0.09, -0.4, 0.4]#ee_res[0]
       self.endEffectorOrn = list(KINOVA_HOME_EE_ORN)

       self.endEffectorAngle  = self.GetTrueMotorAngles()[6]

       #self.endEffectorOrn_euler = list(p.getEulerFromQuaternion(self.endEffectorOrn ))
       #print('init ee angle :', self.GetTrueMotorAngles(), '  ,', self.endEffectorAngle )

    # if use visdom to visualize states
    self.state_vis = state_vis
    if self.state_vis:
      self.t = 0
      self.vis = visdom.Visdom(env='kinova')

  def build_env(self):
    # build env
    self.plane = self._pybullet_client.loadURDF("%s/plane.urdf" % pybullet_data.getDataPath())
    self._pybullet_client.changeVisualShape(self.plane, -1, rgbaColor=[1, 1, 1, 0.9])
    self._pybullet_client.configureDebugVisualizer(self._pybullet_client.COV_ENABLE_PLANAR_REFLECTION, 0)
    self._pybullet_client.setGravity(0, 0, -9.81)

  def reset(self, reload_urdf=True):
    if reload_urdf:
      self.kinovaUid = self._pybullet_client.loadURDF(
                          os.path.join(self.urdfRootPath,"urdf/j2s7s300.urdf"),
                          self._basePosition,
                          self._baseOrientation,
                          useFixedBase=self._is_fixed,
                          flags=self._pybullet_client.URDF_USE_SELF_COLLISION)

      self._BuildJointNameToIdDict()
      self._GetJointInfo()
      self._ResetJointState()

      # # reset joint angle
      # for i in range(self.numMotors):
      #   self._SetDesiredMotorAngleById(self.motorIndices[i], self._init_jointPositions[i], max_velocity= 10)
    else:
      self._pybullet_client.resetBasePositionAndOrientation(self.kinovaUid, self._basePosition, self._baseOrientation)
      self._pybullet_client.resetBaseVelocity(self.kinovaUid, [0, 0, 0], [0, 0, 0])

      self._ResetJointState()

    self.endEffectorPos = list(KINOVA_HOME_EE_POS)  # [0.09, -0.4, 0.4]#ee_res[0]
    self.endEffectorOrn = list(KINOVA_HOME_EE_ORN)
      # # reset joint ange
      # for i in range(self.numMotors):
      #   self._SetDesiredMotorAngleById(self.motorIndices[i], self._init_jointPositions[i], max_velocity=10)

    if self.verbose:
      print('reset joint angle: ', self.GetTrueMotorAngles())
      print('reset end-effortor: ', self.GetEndEffectorObersavations())


    if self._robot_info_debug:

      self.Xpos_info = self._pybullet_client.addUserDebugText('x:', [-0.8, 0, 0.6], textColorRGB=[1, 0, 0], textSize=1.5 )
      self.Ypos_info = self._pybullet_client.addUserDebugText('y:', [-0.8, 0, 0.5], textColorRGB=[1, 0, 0], textSize=1.5)
      self.Zpos_info = self._pybullet_client.addUserDebugText('z:', [-0.8, 0, 0.4], textColorRGB=[1, 0, 0], textSize=1.5)


  def _BuildJointNameToIdDict(self):
    num_joints = self._pybullet_client.getNumJoints(self.kinovaUid)
    self._joint_name_to_id = {}
    for i in range(num_joints):
      joint_info = self._pybullet_client.getJointInfo(self.kinovaUid, i)
      self._joint_name_to_id[joint_info[1].decode("UTF-8")] = joint_info[0]
    print('joint id :', self._joint_name_to_id)
  def _GetJointInfo(self):
    self.actuator_joint = []
    self.motorNames = []
    self.motorIndices = []
    self.joint_q_index = []
    self.joint_u_index = []
    self.jointLowerLimit = []
    self.jointUpperLimit = []
    self.jointMaxForce = []
    self.jointMaxVelocity = []
    self.paramIds = []
    self.paramNames = []
    for i in range(self._pybullet_client.getNumJoints(self.kinovaUid)):
      joint_info = self._pybullet_client.getJointInfo(self.kinovaUid, i)
      qIndex = joint_info[3]
      if qIndex > - 1:  # JOINT_FIXED
        self.motorNames.append(joint_info[1].decode("UTF-8"))
        self.motorIndices.append(i)
        self.joint_q_index.append(joint_info[3])
        self.joint_u_index.append(joint_info[4])
        self.jointLowerLimit.append(joint_info[8])
        self.jointUpperLimit.append(joint_info[9])
        self.jointMaxForce.append(joint_info[10])
        self.jointMaxVelocity.append(joint_info[11])


        jointName = joint_info[1]
        jointType = joint_info[2]
        if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
          self.paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, 0))
          self.paramNames.append(jointName.decode("utf-8"))

    self.numMotors = len(self.motorNames)

    self.EndEffectorIndex = self._joint_name_to_id[self.EndEffectorLinkName]

  def _ResetJointState(self):

    for i in range(self.numMotors):
      self._pybullet_client.resetJointState(
        self.kinovaUid,
        self._joint_name_to_id[self.motorNames[i]],
        self._init_jointPositions[i],
        targetVelocity=0)

  def DebugJointControl(self):
    for i in range(self.numMotors):
      c = self.paramIds[i]
      targetPos = p.readUserDebugParameter(c)

      motor_id = self.motorIndices[i]
      self._SetDesiredMotorAngleById(motor_id, targetPos)



  def _SetMotorTorqueById(self, motor_id, torque):
    self._pybullet_client.setJointMotorControl2(
        bodyIndex=self.kinovaUid,
        jointIndex=motor_id,
        controlMode=self._pybullet_client.TORQUE_CONTROL,
        force=torque)

  def _SetDesiredMotorAngleById(self, motor_id, desired_angle, desired_vel=None, max_velocity = None):
    #if max_velocity is None:
    #    max_velocity = self.jointMaxVelocity[motor_id]
    if desired_vel is None:
      self._pybullet_client.setJointMotorControl2(
          bodyIndex=self.kinovaUid,
          jointIndex=motor_id,
          controlMode=self._pybullet_client.POSITION_CONTROL,
          targetPosition=desired_angle,
          positionGain= 1,
          velocityGain= 0.1,
          maxVelocity = 1,#self.max_velocity,
          force=self.maxForce)
    else:
      self._pybullet_client.setJointMotorControl2(
        bodyIndex=self.kinovaUid,
        jointIndex=motor_id,
        controlMode=self._pybullet_client.POSITION_CONTROL,
        targetPosition=desired_angle,
        targetVelocity = desired_vel,
        positionGain=0.5,
        velocityGain=0.1,
        maxVelocity=1,  # self.max_velocity,
        force=self.maxForce)

  def _SetDesiredMotorVelocityById(self, motor_id, desired_vel, max_velocity = None):
    #if max_velocity is None:
    #    max_velocity = self.jointMaxVelocity[motor_id]
    self._pybullet_client.setJointMotorControl2(
        bodyIndex=self.kinovaUid,
        jointIndex=motor_id,
        controlMode=self._pybullet_client.VELOCITY_CONTROL,
        targetVelocity = desired_vel,

        maxVelocity = self.max_velocity, #self.max_velocity,
        force=self.maxForce)

  def _SetDesiredMotorTorqueById(self, motor_id, desired_torque, max_velocity=None):
    # if max_velocity is None:
    #    max_velocity = self.jointMaxVelocity[motor_id]
    self._pybullet_client.setJointMotorControl2(
      bodyIndex=self.kinovaUid,
      jointIndex=motor_id,
      controlMode=self._pybullet_client.TORQUE_CONTROL,
      force=desired_torque)

  def GetActionDimension(self):
    if (self.useInverseKinematics):
      return 5  #position x,y,z angle and finger angle
    return len(self.motorIndices)

  def GetObservationDimension(self):
    return len(self.GetObservation())

  def _AddSensorNoise(self, sensor_values, noise_stdev):
    if noise_stdev <= 0:
      return sensor_values
    observation = sensor_values + np.random.normal( scale=noise_stdev, size=sensor_values.shape)
    return observation

  def GetTrueMotorAngles(self):
    """Gets the joints angles at the current moment.
    """
    motor_angles = [
        self._pybullet_client.getJointState(self.kinovaUid, motor_id)[0]
        for motor_id in self.motorIndices]
    return motor_angles

  def GetMotorAngles(self):
    """Gets the actual joint angles with noise.
    This function mimicks the noisy sensor reading and adds latency. The motor
    angles that are delayed, noise polluted, and mapped to [-pi, pi].
    Returns:
      Motor angles polluted by noise and latency, mapped to [-pi, pi].
    """
    motor_angles = self._AddSensorNoise(np.array(self.GetTrueMotorAngles()[0:self.numMotors]),
        self._observation_noise_stdev[0])
    return motor_angles  # delete maping

  def GetTrueMotorVelocities(self):
    """Get the velocity of all joints.
    Returns:
      Velocities of all joints.
    """
    motor_velocities = [
        self._pybullet_client.getJointState(self.kinovaUid, motor_id)[1]
        for motor_id in self.motorIndices]

    return motor_velocities

  def GetMotorVelocities(self):
    """Get the velocity of all eight motors.

    This function mimicks the noisy sensor reading and adds latency.
    Returns:
      Velocities of all eight motors polluted by noise and latency.
    """
    return self._AddSensorNoise(
        np.array( self.GetTrueMotorVelocities()[0:self.numMotors]),
        self._observation_noise_stdev[1])

  def GetTrueMotorTorques(self):
    """Get the amount of torque the motors are exerting.

    Returns:
      Motor torques of all eight motors.
    """
    #TODO if considering motor dynamics, need to add a coversion function of motor torques.
    motor_torques = [
          self._pybullet_client.getJointState(self.kinovaUid, motor_id)[3]
          for motor_id in self.motorIndices ]

    return motor_torques
  def GetMotorTorques(self):
    """Get the amount of torque the motors are exerting.
    This function mimicks the noisy sensor reading and adds latency.
    Returns:
      Motor torques of all eight motors polluted by noise and latency.
    """
    return self._AddSensorNoise( np.array( self.GetTrueMotorTorques()[0: self.numMotors]),
                                 self._observation_noise_stdev[2])

  def GetEndEffectorObersavations(self):
    """Get the end effector of kinova

    Returns:
      Position of the end effecotr:[x, y, z] wrt world Cartesian space
      Oreintation of the end effector, in quaternion form.
    """
    state = self._pybullet_client.getLinkState(self.kinovaUid,
                                               self.EndEffectorIndex, computeForwardKinematics=1)
    ee_pos = state[4]
    ee_orn = state[5]

    return np.array(ee_pos), np.array(ee_orn)
  def GetObservation(self):
    """Get the observations of kinova.

    It includes the angles, velocities, torques and the orientation of the base.

    Returns:
      The observation list. observation[0:8] are motor angles. observation[8:16]
      are motor velocities, observation[16:24] are motor torques.
      observation[24:28] is the orientation of the base, in quaternion form.
    """
    observation = []

    if self.OnlyEndEffectorObervations:
      ee_pos, ee_orn = self.GetEndEffectorObersavations()
      ee_euler = self._pybullet_client.getEulerFromQuaternion(ee_orn)

      observation.extend(ee_pos.tolist())
      observation.extend(ee_orn.tolist())
    else:
      observation.extend(self.GetMotorAngles().tolist())
      observation.extend(self.GetMotorVelocities().tolist())
      observation.extend(self.GetMotorTorques().tolist())

    return observation



  def GetObservationUpperBound(self):
    """Get the upper bound of the observation.

    Returns:
      The upper bound of an observation. See GetObservation() for the details
        of each element of an observation.
    """
    if self.OnlyEndEffectorObervations:
      raise print('need to developed!')
    else:
      upper_bound = np.array([0.0] * self.GetObservationDimension())
      upper_bound[0:self.numMotors] = self.jointUpperLimit  # Joint angle.
      upper_bound[self.numMotors:2 * self.numMotors] = self.jointMaxVelocity  #    Joint velocity.
      upper_bound[2 * self.numMotors:3 * self.numMotors] = self.jointMaxForce # Joint torque.

    return upper_bound

  def GetObservationLowerBound(self):
    """Get the lower bound of the observation."""
    if self.OnlyEndEffectorObervations:
      raise print('need to developed!')
    else:
      lower_bound = np.array([0.0] * self.GetObservationDimension())
      lower_bound[0:self.numMotors] = self.jointLowerLimit  # Joint angle.
      lower_bound[self.numMotors:2 * self.numMotors] = self.jointMaxVelocity*(-1)  # Joint velocity.
      lower_bound[2 * self.numMotors:3 * self.numMotors] = self.jointMaxForce*(-1)  # Joint torque.

    return lower_bound

  def ApplyAction(self, commands):
    """

    :param commands:

    1. control end-effector in Cartesian Space. useInverseKinematics is True.
    np.array o list with size 9
    [dx, dy, dz, orn, fingerAngle]
    dx,dy,dz : the relative value of end-effector in Cartesian Space. Unit: meter
    orn : the quaterion of the end-effector
    fingerAngle : angle of the fingers

    2. control joint angle derectly in joint configuration space. useInverseKinematics is False.
    np.array o list with size 10
    [j1,j2,...,j7, f1, f2, f3] Unit: rad

    """
    if (self.useInverseKinematics):
      if np.array(commands).size != 8:
        raise Exception("Command size is not matched, require a command with the size of 8 but got ",
                        np.array(commands).size)

      dx = commands[0]
      dy = commands[1]
      dz = commands[2]

      orn = commands[3:7]

      fingerAngle = commands[7]

      self.endEffectorPos[0] += dx
      self.endEffectorPos[0] = np.clip(self.endEffectorPos[0], self.ee_X_lowerLimit, self.ee_X_upperLimit)

      self.endEffectorPos[1] += dy
      self.endEffectorPos[1] = np.clip(self.endEffectorPos[1], self.ee_Y_lowerLimit, self.ee_Y_upperLimit)

      self.endEffectorPos[2] += dz
      self.endEffectorPos[2] = np.clip(self.endEffectorPos[2], self.ee_Z_lowerLimit, self.ee_Z_upperLimit)

      pos = self.endEffectorPos

      ee_pos, ee_orn = self.GetEndEffectorObersavations()
      if self.verbose:

        print('end-effecter position: ',ee_pos)
        print('end-effecter orentation ', ee_orn)

      if self.state_vis:
        ee_pos, ee_orn = self.GetEndEffectorObersavations()
        self.vis.line(X=np.array([self.t]),
                      Y=np.column_stack((np.array([pos[0]]),np.array([ee_pos[0]]))),
                      opts=dict(showlegend=True, title = 'X position'), win='X position',   update='append', )
        self.vis.line(X=np.array([self.t]),
                      Y=np.column_stack((np.array([pos[1]]), np.array([ee_pos[1]]))),
                      opts=dict(showlegend=True, title='Y position'), win='Y position', update='append', )
        self.vis.line(X=np.array([self.t]),
                      Y=np.column_stack((np.array([pos[2]]), np.array([ee_pos[2]]))),
                      opts=dict(showlegend=True, title='Z position'), win='Z position', update='append', )
        self.t += 0.01

        #plt.plot(pos[0])
        #self.vis.matplot(plt)
        # self.vis.line(X = self.t, Y= np.array(pos[0]), win='x position', update='append' ,
        #                opts={'dash': np.array(['solid', 'dash', 'dashdot']),
        #                     'linecolor': np.array([
        #                       [0, 191, 255],
        #                       [0, 191, 255],
        #                       [255, 0, 0],
        #                       ]),
        #                     'title': 'Different line dash types'
        #             } )f


      if self._robot_info_debug:
        x_pos = "x: %.3f" %ee_pos[0]
        y_pos = "y: %.3f" % ee_pos[1]
        z_pos = "z: %.3f" % ee_pos[2]

        self._pybullet_client.addUserDebugText(x_pos,  [-0.8, 0, 0.6], textColorRGB=[1, 0, 0], textSize=1.5 ,  replaceItemUniqueId =self.Xpos_info)
        self._pybullet_client.addUserDebugText(y_pos, [-0.8, 0, 0.5], textColorRGB=[1, 0, 0], textSize=1.5 , replaceItemUniqueId=self.Ypos_info)
        self._pybullet_client.addUserDebugText(z_pos, [-0.8, 0, 0.4], textColorRGB=[1, 0, 0], textSize=1.5 ,replaceItemUniqueId=self.Zpos_info)

      # ik
      if (self.useNullSpace==1):
        if (self.useOrientation==1):
          jointPoses = self._pybullet_client.calculateInverseKinematics(self.kinovaUid,self.EndEffectorIndex,pos,orn,
                                                                        lowerLimits=self.ll, upperLimits=self.ul,
                                                                        jointRanges=self.jr, restPoses=self.rp,
                                                                        residualThreshold=0.001, jointDamping=self.jd )
        else:
          jointPoses = self._pybullet_client.calculateInverseKinematics(self.kinovaUid,self.EndEffectorIndex,pos,
                                                                        lowerLimits=self.ll, upperLimits=self.ul,
                                                                        jointRanges=self.jr, restPoses=self.rp,
                                                                        residualThreshold=0.001, jointDamping=self.jd)
      else: #TODO test
        if (self.useOrientation==1):
          jointPoses = self._pybullet_client.calculateInverseKinematics(self.kinovaUid,self.EndEffectorIndex,pos,orn,jointDamping=self.jd)
        else:
          jointPoses = self._pybullet_client.calculateInverseKinematics(self.kinovaUid,self.EndEffectorIndex,pos)

      if self.verbose:
        currentPosition = self.GetTrueMotorAngles()
        print('ik joint angle results:', jointPoses)
        print('current joint angle ', currentPosition )

      if (self.useSimulation):
        for i in range(self.numMotors - self.numFingers):
          motor_id = self.motorIndices[i]
          self._SetDesiredMotorAngleById(motor_id, jointPoses[i])

        # end-effector angle
        #self._SetDesiredMotorAngleById(self.motorIndices[6], self.endEffectorAngle)

      else:
        #TODO test  #reset the joint state (ignoring all dynamics, not recommended to use during simulation)
        for i in range (self.numJoints):
          self._pybullet_client.resetJointState(self.kukaUid,i,jointPoses[i])

      #fingers
      for i in range(self.numFingers):
        finger_id = self.motorIndices[self.numMotors - self.numFingers + i]
        self._pybullet_client.setJointMotorControl2(self.kinovaUid,finger_id,self._pybullet_client.POSITION_CONTROL,
                                                    targetPosition= fingerAngle,  force=self.fingerTipForce)

    else:# TODO test
      if np.array(commands).size != self.numMotors:  # 10 for j2n7s300
        raise Exception("Command size is not matched, require a command with the size of ", self.numMotors, " but got ", np.array(commands).size)
      for i in range (self.numMotors):
        motor_id = self.motorIndices[i]
        self._SetDesiredMotorAngleById(motor_id, commands[i])

  def CheckEEpos(self):
    ee_pos, ee_orn = self.GetEndEffectorObersavations()

    if ee_pos[0]  >  self.ee_X_upperLimit or ee_pos[0] < self.ee_X_lowerLimit:
      return True
    if ee_pos[1]  >  self.ee_Y_upperLimit or ee_pos[1] < self.ee_Y_lowerLimit:
      return True
    if ee_pos[2]  >  self.ee_Z_upperLimit or ee_pos[2] < self.ee_Z_lowerLimit:
      return True

    return False


  def ApplyAction_Velocity(self, commands):
    """
    :param commands:

    """
    if np.array(commands).size != 10:
      raise Exception("Command size is not matched, require a command with the size of 8 but got ",
                      np.array(commands).size)

    for i in range (self.numMotors - self.numFingers):
      motor_id = self.motorIndices[i]
      self._SetDesiredMotorVelocityById(motor_id, commands[i])

    #fingers
    for i in range(self.numFingers):
      fingerAngle = commands[-self.numFingers +i]
      finger_id = self.motorIndices[self.numMotors - self.numFingers + i]
      self._pybullet_client.setJointMotorControl2(self.kinovaUid,finger_id,
                                                  self._pybullet_client.POSITION_CONTROL,
                                                  targetPosition= fingerAngle,
                                                  force=self.fingerTipForce)


  def Enable_Torque_mode(self):
    for i in range(self.numMotors - self.numFingers):
      motor_id = self.motorIndices[i]
      maxForce = 0
      mode = p.VELOCITY_CONTROL
      p.setJointMotorControl2(self.kinovaUid, motor_id,
                              controlMode=mode, force=maxForce)

  def ApplyAction_Torque(self, commands):
    """
    :param commands:


    """
    if np.array(commands).size != 10:
      raise Exception("Command size is not matched, require a command with the size of 10 but got ",
                      np.array(commands).size)

    for i in range (self.numMotors - self.numFingers):
      motor_id = self.motorIndices[i]
      self._SetDesiredMotorTorqueById(motor_id, commands[i])

    #fingers
    for i in range(self.numFingers):
      fingerAngle = commands[-self.numFingers +i]
      finger_id = self.motorIndices[self.numMotors - self.numFingers + i]
      self._pybullet_client.setJointMotorControl2(self.kinovaUid,finger_id,
                                                  self._pybullet_client.POSITION_CONTROL,
                                                  targetPosition= fingerAngle,
                                                  force=self.fingerTipForce)
  def ApplyAction_EndEffectorPose(self, commands):
    if (self.useInverseKinematics):
      if np.array(commands).size != 8:
        raise Exception("Command size is not matched, require a command with the size of 8 but got ",
                        np.array(commands).size)
      x = commands[0]
      y = commands[1]
      z = commands[2]
      orn = commands[3:7]
      fingerAngle = commands[7]


      self.endEffectorPos[0] = x
      self.endEffectorPos[0] = np.clip(self.endEffectorPos[0], self.ee_X_lowerLimit, self.ee_X_upperLimit)

      self.endEffectorPos[1] = y
      self.endEffectorPos[1] = np.clip(self.endEffectorPos[1], self.ee_Y_lowerLimit, self.ee_Y_upperLimit)

      self.endEffectorPos[2] = z
      self.endEffectorPos[2] = np.clip(self.endEffectorPos[2], self.ee_Z_lowerLimit, self.ee_Z_upperLimit)

      pos = self.endEffectorPos

      if self.verbose:
        ee_pos, ee_orn = self.GetEndEffectorObersavations()
        print('end-effecter position: ', ee_pos)
        print('end-effecter orentation ', ee_orn)

      if self.state_vis:
        ee_pos, ee_orn = self.GetEndEffectorObersavations()
        self.vis.line(X=np.array([self.t]),
                      Y=np.column_stack((np.array([pos[0]]), np.array([ee_pos[0]]))),
                      opts=dict(showlegend=True, title='X position'), win='X position', update='append', )
        self.vis.line(X=np.array([self.t]),
                      Y=np.column_stack((np.array([pos[1]]), np.array([ee_pos[1]]))),
                      opts=dict(showlegend=True, title='Y position'), win='Y position', update='append', )
        self.vis.line(X=np.array([self.t]),
                      Y=np.column_stack((np.array([pos[2]]), np.array([ee_pos[2]]))),
                      opts=dict(showlegend=True, title='Z position'), win='Z position', update='append', )
        self.t += 0.01

      # ik
      if (self.useNullSpace == 1):
        if (self.useOrientation == 1):
          jointPoses = self._pybullet_client.calculateInverseKinematics(self.kinovaUid, self.EndEffectorIndex, pos, orn,
                                                                        lowerLimits=self.ll, upperLimits=self.ul,
                                                                        jointRanges=self.jr, restPoses=self.rp,
                                                                        residualThreshold=0.0001, jointDamping=self.jd)
        else:
          jointPoses = self._pybullet_client.calculateInverseKinematics(self.kinovaUid, self.EndEffectorIndex, pos,
                                                                        lowerLimits=self.ll, upperLimits=self.ul,
                                                                        jointRanges=self.jr, restPoses=self.rp,
                                                                        residualThreshold=0.0001, jointDamping=self.jd)
        if (self.useSimulation):
          for i in range(self.numMotors - self.numFingers):
            motor_id = self.motorIndices[i]
            self._SetDesiredMotorAngleById(motor_id, jointPoses[i])
        else:
          # TODO test  #reset the joint state (ignoring all dynamics, not recommended to use during simulation)
          for i in range(self.numJoints):
            self._pybullet_client.resetJointState(self.kukaUid, i, jointPoses[i])
      # fingers
      for i in range(self.numFingers):
        finger_id = self.motorIndices[self.numMotors - self.numFingers + i]
        self._pybullet_client.setJointMotorControl2(self.kinovaUid, finger_id, self._pybullet_client.POSITION_CONTROL,
                                                    targetPosition=fingerAngle, force=self.fingerTipForce)

    else:
      raise Exception("You must set \'useInverseKinematics==True\' in the ApplyAction_EndEffectorPose function.")


