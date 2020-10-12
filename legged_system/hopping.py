

# general import
import json
from enum import Enum
import logging as log
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp, odeint
import sklearn.mixture as mixture


class Hopper():
    def __init__(self, length = 0):
        self.l = length
        self._initialize_parameter()





    def _initialize_parameter(self):
        '''
            Intialize the internal parameter
        '''
        # flight x and y distance
        self.x_f, self.y = 0, 0
        # mass of the pendulum.
        self.m = 80
        # gracity
        self.g = 9.81
        # stiffness
        self.k = 22000
        # initalize the distance in x and y direction.
        self.x_0 = 0
        self.y_0 = np.sqrt(self.l**2 - self.x_0**2)
        # initialize the velocity
        self.v_0 = 5
        #





