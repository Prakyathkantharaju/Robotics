

# general import
import json
from enum import Enum
import logging as log
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp, odeint
import sklearn.mixture as mixture


class Hopper():
    def __init__(self, length: int = 0):
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
        # x velocity
        self.dx_0 = np.copy(self.v_0)
        # y velocity
        self.dy_0 = 0
        # reference velocity
        self.v0_ref = 5
        # initial velocity
        self.vint = 0
        # Energy of the system
        self.Esys = self.m*self.g*self.y_0 + 1/2 * self.m * self.v_0**2
        # initial height
        self.yinit = (self.Esys - 1/2 * self.m *self.vint**2)/self.m/self.g

        # landing condition
        self.LANDING_ANGLE = 69
        self.ALPHA_0 = self.LANDING_ANGLE*np.pi/180
        self.DXFOOT = 1*np.cos(self.ALPHA_0)
        self.Y_LAND = self.l * np.sin(self.ALPHA_0)

        # storing lists
        self.actual_acc = []
        self.acc_predict_store = []
        self.store_mass_predict = []

        self.temp_state = 0

    def _stance(self):
        '''
            Stance phase
        '''
        x, y = np.copy(self.x), np.copy(self.y)
        # removing
        x = x - self.x_f
        y = y - self.y_f
        # log.debug(f'actual_x: {x} actual_y {y}')
        l_temp = np.sqrt(np.abs(x)**2 + y**2)
        k = self.k*(self.l/l_temp - 1)
        # log.debug(f'k {k} length : {l_temp} sqrt: {np.sqrt(x**2 + y**2)}')
        tfx = k*(x/l_temp)
        tfy = k*(y/l_temp)
        self.x_s = x
        self.y_s = y
        return np.array([tfx,tfy])



    def _flight(self):
        x,y = np.copy(self.x),np.copy(self.y)
        if self.Flight_state:
            tfx = x + self.DXFOOT
            tfy = y - self.Y_LAND
        else:
            tfx = self.x_f
            tfy = self.y_f
        self.x_f,self.y_f = np.copy(tfx),np.copy(tfy)
        return np.array([tfx,tfy])

    def _state_detection(self):
        x,y = self.x,self.y
        cond_1 =  np.sqrt(x**2 + y**2) > self.l # take off condition
        cond_2 =  y - self.Y_LAND < 0 # landing condition
        # Conditions for the states
        if cond_1 and not cond_2 or self.Stance_state and cond_1:
            self.Flight_state = True
            self.Stance_state = False
        if not cond_1 and cond_2  or self.Flight_state and cond_2:
            self.Stance_state = True
            self.Flight_state = False

    def _force_calculator(self):
        F_f = self._flight()
        F_s = self._stance()
        self._state_detection()
        # log.debug(f'x:{self.x} y:{self.y}')
        if self.Stance_state:
            # log.debug(f'state: stance')
            self.F = F_s
        elif self.Flight_state:
            # log.debug(f'state: flight')
            self.F = F_s

    def _intergrate(self,t,x):
        # accept [x,y,xdot,ydot]
        # return [xdot,ydot,xacc,yacc]
        self.store_x.append(x[0])
        # log.debug(f'time: t{t}')
        xdot = np.copy(x)
        self.x,self.y = x[0],x[1]
        self._force_calculator()
        xdot[0],xdot[1] = x[2],x[3]
        xdot[2] = self.F[0]/self.m
        xdot[3] = self.F[1]/self.m - self.g
        # print('intergrate')
        # log.debug(f'y_acc:{xdot[3]} x_acc: {xdot[2]} force {self.F}')
        self.y_acc = xdot[3]
        return xdot





