import gym
import pybullet
import numpy as np


from base_env import EnvBase

class Nao():
    joints = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch',  'LAnklePitch', 'LAnkleRoll',
    'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch',  'RAnklePitch', 'RAnkleRoll']
    obj_joints = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch',  'LAnklePitch', 'LAnkleRoll',
    'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch',  'RAnklePitch', 'RAnkleRoll', 'HeadYaw', 'HeadPitch']
    positions = []
    velocity = []
    def __init__(self, simulation_manager, robot, client):
        self.manager = simulation_manager
        self.robot = robot
        # self.client = client
        self.action_joints = {}
        self.obs_joints = {}
        self.get_joint_parameters()
        high, low = self.get_joint_limits()
        print(len(high), len(low))
        self.action_space = gym.spaces.Box(low, high, dtype = np.float32)
        high, low = self.get_joint_limits(joint_type='obs')
        self.observation_space = gym.spaces.Box(low, high, dtype=np.float32)
        self.get_base()
        # self.setup_foot()

    def get_joint_position(self):
        """ get position from robot for our observation space
        """
        joint_names = list(self.obs_joints.keys())
        position = self.robot.getAnglePosition(joint_names)
        return position

    def get_joint_velocity(self):
        """ get  joint velocity for observation space
        """
        joint_names = list(self.obj_joints.keys())
        velocity =  self.robot.getAngleVelocity(joint_names)
        return velocity


    def get_joint_parameters(self):
        """ get joint parameter for action space and observation space
        """
        print("Here")
        for name, joint in self.robot.joint_dict.items():
            print(name)
            print('$'*30)
            # action joints
            if name in self.joints:
                self.action_joints[name] = {}
                self.action_joints[name]['limits'] = [joint.getLowerLimit(),
                                                      joint.getUpperLimit()]
                print(self.action_joints[name]['limits'])
                self.action_joints[name]['position'] = self.robot.getAnglesPosition(name)

            # obs joints
            if name in self.obs_joints:
                self.obs_joints[name] = {}
                self.obj_joints[name]['limits'] = [joint.getLowerLimit(),
                                                      joint.getUpperLimit()]
                self.obs_joints[name]['position'] = self.robot.getAnglesPosition(name)

    def get_joint_limits(self, joint_type = 'action'):
        high, low = [], []
        if joint_type == 'action':
            for name in self.action_joints.keys():
                high.append(self.action_joints[name]['limits'][1])
                low.append(self.action_joints[name]['limits'][0])
        else:
            for name in self.obs_joints.keys():
                high.append(self.obs_joints[name]['limits'][1])
                low.append(self.obs_joints[name]['limits'][0])
        high, low = np.array(high), np.array(low)
        return high, low


    def set_actions(self, actions, per = 0.5):
        joint_names = list(self.action_joints.keys())
        percentage = [per] * len(joint_names)
        self.robot.setAngles(joint_names, actions.tolist(), percentage)


    def get_base(self):
        print(self.robot.getPosition())
        _, _, _, self.base_xyz= self.robot.getPosition()











class NaoEnv(EnvBase):

    def __init__(self, render = False):
        super(NaoEnv, self).__init__(Nao, render = True)
        self.observation_space = self.robotclass.observation_space
        self.action_space = self.robotclass.action_space


    def reset(self):
        # reset using simulation manager. Goesback to the original.
        self.simulation_manager.reset()


    def step(self, position):
        # TODO: create a PD torque controller for Nao env
        self.robotclass.set_actions(position)



