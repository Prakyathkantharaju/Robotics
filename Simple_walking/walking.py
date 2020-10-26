import numpy as np
import matplotlib.pyplot as plt




class walking_parms(object):
    def __init__(self):
        '''
            This class is the class to store all the parmeters for the rimless walking.
        '''
        self._initalize()

    def _initalize(self):
        self.com_mass = 1.0 # COM mass
        self.leg_mass = 0.5 # Mass of the leg
        self.leg_interia = 0.02 # Leg Inertia
        self.leg_length = 1.0 # Leg length
        self.gravity = 9.81 # gravity
        self.gam = 0.01
        self.center = 0.5


class walking(walking_parms):
    def __init__(self):
        '''
            Main Rimless walking model
        '''
        # loading the parameters
        super().__init__()

    def _single_stance(self, t, x):
        '''
            stance intergration function
            - Input is time and x [theta1,omega1,theta2,omega2]
        '''
        theta1, omega1, theta2, omega2 = x[0], x[1], x[2], x[3]
        A = np.zeros((2,2))
        A[0, 0] = 2 * self.leg_interia + 2 * self.com_mass * self.leg_length ** 2 +\
                 2 * self.center**2 * self.leg_mass + 2*self.leg_length**2*self.leg_mass -\
                 2 * self.center * self.leg_length * self.leg_mass - 2 * self.center \
                 * self.leg_length * self.leg_mass * np.cos(theta2)

        A[0, 1] = self.leg_interia + self.center**2 * self.leg_mass  - \
                self.center * self.leg_length * self.leg_mass * np.cos(theta2)
        A[1, 0] = A[0, 1]
        A[1, 1] = self.leg_interia + self.center**2 * self.leg_mass

        B = np.zeros((2, 1))
        B[0, 0] = self.center * self.gravity * self.leg_mass * np.sin(self.gam - theta1) - \
                  self.com_mass * self.gravity * self.leg_length * np.sin(self.gam - theta1) -\
                  self.center * self.gravity * self.leg_mass * np.sin(theta1 - self.gam + theta2) -\
                  2*self.gravity*self.leg_length*self.leg_mass*np.sin(self.gam - theta1) -\
                  self.center * self.leg_length * self.leg_mass * omega2**2 * np.sin(theta2) -\
                  2 * self.center * self.leg_length * omega1 * omega2 * np.sin(theta2)


        B[1, 0] = - self.center * self.leg_mass * (self.gravity * np.sin(theta1 -\
                    self.gam + theta2) -\
                    self.leg_length * omega1**2 * np.sin(theta2))

        acceleration = np.matmul(np.linalg.pinv(A),B)

        return [omega1 , acceleration[0], omega2, acceleration[1]]

    def _collision(self, x):
        '''
            collision detection and the single stance to double stance
        '''
        theta1, theta2 = x[0], x[3]
        terminal = 0
        gstop = theta2 + theta1 * 2
        if theta1 > -0.05:
            terminal = 0
        else:
            terminal = 1
        if gstop >  -0.001 and gstop < 0.001:
            terminal = 1
        return terminal

    def _double_stance(self, x)



