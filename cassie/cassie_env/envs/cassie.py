import pybullet as p
from cassie_state import CassieState
import time

p.connect(p.GUI)
p.loadURDF("urdf/plane.urdf")
humanoid = p.loadURDF("urdf/cassie_collide.urdf",[0,0,0.8], useFixedBase=False)
gravId = p.addUserDebugParameter("gravity",-10,10,-10)
jointIds=[]
paramIds=[]

p.setPhysicsEngineParameter(numSolverIterations=100)
p.changeDynamics(humanoid,-1,linearDamping=0, angularDamping=0)

jointAngles=[0,0,1.0204,-1.97,-0.084,2.06,-1.9,0,0,1.0204,-1.97,-0.084,2.06,-1.9,0]
activeJoint=0
for j in range (p.getNumJoints(humanoid)):
    p.changeDynamics(humanoid,j,linearDamping=0, angularDamping=0)
    info = p.getJointInfo(humanoid,j)
    #print(info)
    jointName = info[1]
    jointType = info[2]
    if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
        jointIds.append(j)
        paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"),-4,4,jointAngles[activeJoint]))
        p.resetJointState(humanoid, j, jointAngles[activeJoint])
        print(info[12])
        print(info[-4])
        activeJoint+=1
state = CassieState(humanoid)

p.setRealTimeSimulation(1)
while(1):
    break
    p.getCameraImage(320,200)
    p.setGravity(0,0,p.readUserDebugParameter(gravId))
    for i in range(len(paramIds)):
        c = paramIds[i]
        targetPos = p.readUserDebugParameter(c)
        print(targetPos, i)
        p.setJointMotorControl2(humanoid,jointIds[i],p.TORQUE_CONTROL,force=targetPos * 100)
        time.sleep(0.01)
