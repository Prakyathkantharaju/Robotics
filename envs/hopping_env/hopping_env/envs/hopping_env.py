# general import
from typing import Dict, List, Any
import numpy as np
import matplotlib.pyplot as plt

# scipy modules
from scipy.integrate import solve_ivp, odeint
import sklearn.mixture as mixture
import random

# gym modules
import gym
from gym import error, spaces, utils
from gym.utils import seeding


class Hopper(gym.Env):
    metadata = {'render.modes': ['human']}
    def __init__(self, length: int = 0, k_range: tuple = (7000, 30000),
                 alpha_range: tuple = (20, 90), max_distance = 1) -> None:
        self.l = length
        self.action_range = (k_range, alpha_range)
        self._initialize_parameter(self.action_range[0], self.action_range[1],
                                   max_distance)

    def _initialize_parameter(self, k_range: tuple, alpha_range: tuple,
                              max_distance: int) -> None:
        '''
            Intialize the internal parameter
        '''
        # flight x and y distance
        self.x_f, self.y = 0, 0
        # mass of the pendulum.
        self.m = 80
        # gravity
        self.g = 9.81
        # stiffness
        self.k = 22000

        # initalize the distance in x and y direction.
        self.x_0 = 0.25
        self.l = 1
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
        self.esys = self.m*self.g*self.y_0 + 1/2 * self.m * self.v_0**2
        # initial height
        self.yinit = (self.esys - 1/2 * self.m *self.vint**2)/self.m/self.g

        # landing condition Constant
        self.landing_angle = 69
        self.alpha_0 = self.landing_angle * np.pi/180
        self.DXFOOT = 1*np.cos(self.alpha_0)
        self.Y_LAND = self.l * np.sin(self.alpha_0)

        # storing lists
        self.actual_acc = []
        self.acc_predict_store = []
        self.store_mass_predict = []
        self.store_x = []

        self.temp_state = 0

        # stance distance
        self.x_s, self.y_s = 0, 0

        # flight distance
        self.x_f, self.y_f = 0, 0

        # states
        self.stance_state = True
        self.flight_state = False

        # Force
        self.F = [0]

        # action range
        # k range
        self.k_min = 7000
        self.k_max = 300000
        # angle range
        self.alpha_min = 30
        self.alpha_max = 80

        # state initialize
        self.state = {}

        # max_distance
        self.max_distance = max_distance

        self.i = 0

    def _stance(self) -> Any:
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
        return np.array([tfx, tfy])



    def _flight(self) -> Any:
        """
            Flight dynamics
        """
        x, y = np.copy(self.x), np.copy(self.y)
        if self.flight_state:
            tfx = x + self.DXFOOT
            tfy = y - self.Y_LAND
        else:
            tfx = self.x_f
            tfy = self.y_f
        self.x_f, self.y_f = np.copy(tfx), np.copy(tfy)
        return np.array([tfx, tfy])

    def _state_detection(self) -> None:
        """
            state detection
        """
        x, y = self.x, self.y
        cond_1 = np.sqrt(x**2 + y**2) > self.l # take off condition
        cond_2 = y - self.Y_LAND < 0 # landing condition
        # Conditions for the states
        if cond_1 and not cond_2 or self.stance_state and cond_1:
            self.flight_state = True
            self.stance_state = False
        if not cond_1 and cond_2  or self.flight_state and cond_2:
            self.stance_state = True
            self.flight_state = False

    def _force_calculator(self) -> None:
        """ Force and acceleration calculator
        """
        f_f = self._flight()
        f_s = self._stance()
        self._state_detection()
        # log.debug(f'x:{self.x} y:{self.y}')
        if self.stance_state:
            # log.debug(f'state: stance')
            self.F = f_s
        elif self.flight_state:
            # log.debug(f'state: flight')
            self.F = f_f

    def _intergrate(self, t: int, x: List) -> List:
        """ main integration function
            args:
                t (int): Time
                x (list): state list
            return:
                xdot (List): derivate of the states
        """
        # accept [x,y,xdot,ydot]
        # return [xdot,ydot,xacc,yacc]
        self.store_x.append(x[0])
        xdot = np.copy(x)
        self.x, self.y = x[0], x[1]
        self._force_calculator()
        xdot[0], xdot[1] = x[2], x[3]
        xdot[2] = self.F[0]/self.m
        xdot[3] = self.F[1]/self.m - self.g
        self.y_acc = xdot[3]
        return xdot


    def step(self, action: List) -> Dict:
        """ step function will intergrate for one step
            args:
                action: (list) [stiffness of spring: int, landing angle in deg (alpha) : int ]
            return:
            ob, reward, episode_over, info: (tuple)

            ob: (tuple) observation of the system.
                x, y, xdot, ydot

            reward: (bool)
                boolean suggesting the trajectory has reached desired apex point.
                and if the system is stable.

            episode_over: (bool)
                total time is over 10 or system is unstable.

            info: (dict)
                mass, action, time, observation etc.

        """
        self.action = action
        t_eval, y_0 = self._get_state()
        # updating the actions.
        # check if the actions in the required space.
        self.k = np.copy(action[0])
        self.Y_LAND = self.l * np.sin(action[1] * np.pi / 180)

        sol = solve_ivp(self._intergrate, [t_eval,t_eval + 0.01],
                        y0=y_0, dense_output=True)

        output = self._update_state(sol)
        self.i += 1
        return output

    def _get_state(self):
        if len(self.state) == 0:
            # time eval
            t_eval = np.arange(0, 4, 0.01)
            # initial state
            y_0 = [self.x_0, self.y_0, self.dx_0, self.dy_0]
            return 0, y_0
        else:
            return self.state['teval'], self.state['ob']

    def reset(self, length: int = 0) -> None:
        """ reset the env
            args:
                length (int): length of the hopping leg
            return:
        """
        self.l = length
        self._initialize_parameter(self.action_range[0], self.action_range[1])



    def _update_state(self, sol: solve_ivp) -> dict:
        """ Update state information after simulation

            Args:
            sol (solve_ivp object): the solution object contains all the
                information about the solution.

            return:
            output (dic): keys teval, observation, action, state
        """
        self.state['teval'] = sol.t[-1] + 0.01
        self.state['ob'] = sol.y[:,-1]
        self.state['action'] = self.action
        self.state['state'] = [0 if self.flight_state else 1]
        self.state['reward'] = self._reward()
        return np.copy(self.state)

    def _reward(self, mode: str = 'distance') -> int:
        """custom reward function:
            option 1: stability,  mass is above the leg.
            option 2: distance, can hopper cover 10 m.
        """
        if mode == 'stability':
            if self.y < self.l / 4:
                reward = 1
            else:
                reward = 0
        elif mode == 'distance':
            if self.x > self.max_distance:
                reward = 1
            else:
                reward =  0
        else:
            reward = 0
        return reward

    def render(self, mode:str = 'human') -> None:
        pass


    def solver(self):
        '''
            full time solver and intergration function
        '''
        iter_ = 0
        result = np.zeros((1, 4))
        # time eval
        t_eval = np.arange(0, 4, 0.01)
        # initial state
        y_0 = [self.x_0, self.y_0, self.dx_0, self.dy_0]

        print(self.alpha_min, self.alpha_max)
        self.k = random.randrange(self.k_min, self.k_max)
        self.alpha_0 = random.randrange(self.alpha_min, self.alpha_max)
        self.Y_LAND =  self.l * np.sin(self.alpha_0 * np.pi / 180)

        print(self.k, self.Y_LAND)
        # main for loop
        for i in range(1, len(t_eval) - 1):
            sol = solve_ivp(self._intergrate, [t_eval[i-1], t_eval[i]],
                            y0=y_0, dense_output=True)
            y_0 = sol.y[:,-1]

            result = np.append(result, np.copy(sol.y[:, -1].reshape(1, 4)), axis=0)
            iter_ += 1
            # print(sol.y)
        print(self.x)
        return result

def show_plot(result):
    '''
        plot the the y axis and the x axis result
    '''
    plt.plot(result[:, 1], label='y-axis')
    plt.legend()
    plt.show()
    plt.figure()
    plt.plot(result[1:, 1], result[1:, 3])
    plt.legend()
    plt.show()


if __name__ == '__main__':
    hop = Hopper()
    result = hop.solver()
    show_plot(result)





