import numpy as np
import pybullet as p
import gym
import time


from nao_env import NaoEnv


nao = NaoEnv()

while 1:
    nao.step(nao.action_space.sample() * 0.1)
    time.sleep(0.1)
