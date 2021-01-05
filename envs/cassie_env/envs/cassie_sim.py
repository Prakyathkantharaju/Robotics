import numpy as np
import matplotlib.pyplot as plt
import pybullet as p

# state
from cassie_state import CassieState

class CassieSim(object):
    """
    Cassie sim
    """
    def __init__(self, path = None, render ='human' ):
        if path is None and render == 'human':
            p.connect(p.GUI)
            p.loadURDF("urdf/plane.urdf")
            self.cassie_ID = p.loadURDF("urdf/cassie_collide.urdf", [0, 0, 0.8],
                                        useFixedBase = False)
        else:
            p.connect(p.DIRECT)
            p.loadURDF("urdf/plane.urdf")
            self.cassie_ID = p.loadURDF(path, [0, 0, 0.8], useFixedBase = False)
        p.setGravity(0,0,-10)
        # self.gravID = p.addUserDebugParameter("gravity", -10, 10, -10)
        self.actuator = {'prismatic':[],
                         'revolute':[]}
        self.state = CassieState(self.cassie_ID)
        jointIDs, parmsIDs = self._initialize()
        self.jointIDs = jointIDs
        self.parmsIDs = parmsIDs
        print(self.jointIDs)
        # remove 5 and 7 for now.
        print(len(self.jointIDs) , len(self.parmsIDs))

    def _initialize(self):
        jointIDs = []
        parmsIDs = []
        p.getCameraImage(320,200)
        p.setPhysicsEngineParameter(numSolverIterations=100)
        p.changeDynamics(self.cassie_ID, -1, linearDamping=0,
                         angularDamping=0)
        jointAngles = [0, 0, 1.0204, -1.97, -0.084, 2.06, -1.9, 0, 0, 1.0204,
                        -1.97, -0.084, 2.06, -1.9, 0]
        activeJoint = 0

        for j in range (p.getNumJoints(self.cassie_ID)):
            p.changeDynamics(self.cassie_ID,j,linearDamping=0, angularDamping=0)
            info = p.getJointInfo(self.cassie_ID,j)
            #print(info)
            jointName = info[1].decode('utf8')
            jointType = info[2]
            if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
                jointIDs.append(j)
                parmsIDs.append(p.addUserDebugParameter(jointName, -4, 4,
                                                        jointAngles[activeJoint]))
                p.resetJointState(self.cassie_ID, j, jointAngles[activeJoint])
                # p.enableJointForceTorqueSensor(self.cassie_ID, j, 1)
                print('joint:', jointAngles[activeJoint])
                activeJoint += 1
                if jointType == p.JOINT_PRISMATIC:
                    self.actuator['prismatic'].append(j)
                elif jointType == p.JOINT_REVOLUTE:
                    self.actuator['revolute'].append(j)
        return jointIDs, parmsIDs

    def add_state(self):
        """add state to the state class
        """
        self.state.add_state()

    def step(self, action):
        """Step function
        """
        self._apply_action(action)
        self.add_state()
        p.stepSimulation()


    def _apply_action(self, action):
        p.setJointMotorControlArray(self.cassie_ID, self.jointIDs, p.POSITION_CONTROL,  action)


if __name__ == '__main__':
    CassieSim().add_state()


