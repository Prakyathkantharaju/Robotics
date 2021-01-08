import time
import numpy as np
import matplotlib.pyplot as plt
import gym
import mocca_envs

class CassieTest(object):
    """Test class to test and plot the cassie env with random steps
       (this is just for debugging and should not be used in any controls
    """
    def __init__(self):
        self.env = gym.make("CassieEnv-v0")
        self.env.reset()

    def run(self):
        count = 1
        while 1:
            obs, reward, done, info = self.env.step(self.env.action_space.sample())
            if count > 1000:
                break




if __name__ == '__main__':
    CassieTest().run()
