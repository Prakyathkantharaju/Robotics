import numpy as np
from typing import List
# Kinematics parameter file


class kinematic_info(object):
    def __init__(self,N:int):
        # number of links
        self.N = N
        # Length of the links
        self.length = [0] * N

        # initial angle
        self.theta = [0] * N

        # Constrain
        self.const = None

        # reference
        # in cartesian co-ordinates
        self.ref = [0]*2

    def update_const(self,value:List):
        assert self.N == len(value)
        self.const = np.copy(value)

    def update_length(self, length:List):
        assert self.N == len(length)
        self.length = np.copy(length)

    def update_theta(self, theta:List):
        assert self.N == len(theta)
        self.theta = np.copy(theta)

    def update_ref(self,ref: List):
        self.ref = np.copy(ref)

    def update_path(self, path: List):
        # trajectory for animation etc
        self.path = path

