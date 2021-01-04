import time
import numpy as np
import matplotlib.pyplot as plt

from cassie_sim import CassieSim

from trajectory.trajectory import CassieTrajectory

class CassieTest(object):
    """ test class to test the trajectory for cassie
    """
    def __init__(self):
        self.sim = CassieSim()
        self.traj = CassieTrajectory('trajectory/stepdata.bin')

    def simulate(self):
        tmax = self.traj.time[-1]
        for i in range(0,len(self.traj.time)):
            time.sleep(0.1)
            print('iter:', i)
            action = self.traj.action(i)
            print(len(action[0]))
            trq = [c for c in action[0]]
            self.sim.step(trq)

        position = np.array(self.sim.state.store)
        plt.plot(position)
        plt.show()



if __name__ == '__main__':
    CassieTest().simulate()





