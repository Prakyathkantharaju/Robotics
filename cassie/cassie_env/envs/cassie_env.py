import gym
from gym import spaces
import pybullet as p
import numpy as np
import matplotlib as plt
import time

class cassie_env(gym.Env):
    metadata = {'render.modes' : ['human']}

    def __init__(self, path = 'None', render = 'human'):
        super(cassie_env, self).__init__()
        if render == metadata['render.modes']:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)
        if path == None:
            p.loadURDF('urdf/plane.URDF')
            self.cassie = p.loadURDF('urdf/cassie_collide.urdf', [0,0,0.8],
                                     useFixedBased = False)
            self._gravity = p.addUserDebugParameter("gravity", -10,10,-10)
            self._JointIds = []
            self._activeJoint = []
        p.setPhysicsEngineParameter(numSolverInteration = 1000)
        p.changeDynamics(self.cassie, -1, linearDamping = 0, angularDampling = 0)


        # setting damping to zero for each joint and angle
        for j in range (p.getNumJoints(self.cassie)):
            p.changeDynamics(self.cassie,j,linearDamping=0, angularDamping=0)
            info = p.getJointInfo(self.cassie,j)
            print('jointname:',info[1])
            print('Linkname:', info[12])
            print('Global position:', info[14])
            jointName = info[1]
            jointType = info[2]
            if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
                self._jointIds.append(j)
                self._paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"),
                                                        -4,4,jointAngles[activeJoint]))
                p.resetJointState(humanoid, j, jointAngles[activeJoint])
                activeJoint+=1

        info = p.getJointInfo(self.cassie, 0)
        self.global_x = info[14][0]
        self.global_y = info[14][1]
        self.global_z = info[14][2]







    def step(self, action)
        pass


    def reset(self):
        pass

    def render(self):
        pass
