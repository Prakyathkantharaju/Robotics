import time
import numpy as np
import matplotlib.pyplot as plt

import pybullet as p


from cassie_sim import CassieSim

from trajectory.trajectory import CassieTrajectory

class CassieTest(object):
    """ test class to test the trajectory for cassie
    """
    def __init__(self):
        self.sim = CassieSim()
        self.traj = CassieTrajectory('trajectory/stepdata.bin')
        self.angle = [0, 0, 1.0204, -1.97, -0.084, 2.06, -1.9, 0, 0, 1.0204,
                      -1.97, -0.084, 2.06, -1.9]


    def simulate(self):
        len_t = len(self.traj.time)
        p.setRealTimeSimulation(1)
        time_ = 0
        while 1:
            trq = []
            time_ += 1
            action = self.traj.action(time_)
            pos = action[0]
            counter = 0
            for i in range(len(self.sim.parmsIDs)):
                if (i == 4 or i == 7 or i == 13 or i == 10 ) and False:
                    trq.append(0)
                else:
                    c = self.sim.parmsIDs[i]
                    temp = p.readUserDebugParameter(c)
                    # targetPos = pos[counter]
                    #if i < 1:
                    targetPos = temp + np.random.uniform(-0.001, 0.001)
                    print(targetPos, i, temp)
                    time.sleep(0.1)
                    trq.append(targetPos)
                    counter += 1
            print(trq, len(self.sim.parmsIDs))
            self.sim.step(trq)

        position = np.array(self.sim.state.store)
        plt.plot(position)
        plt.show()



if __name__ == '__main__':
    CassieTest().simulate()





