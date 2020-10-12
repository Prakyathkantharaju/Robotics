import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp,odeint
import sklearn.mixture as mixture
from enum import Enum
import logging as log
import json


class Hopper(object):
    def __init__(self, l=1):
        self.l = l
        self._initialize_parameters()
        # most important parameters
        self.store_x = []
        self.store_flight_x = []
        self.store_state = []
        self.x = self.x_0
        self.y = self.y_0
        self.flight_state, self.stance_state = False, True
        self.F = None
        self.store_F_y = []
        self.store_F_x = []
        self.store_mass = []


    def _initialize_parameters(self):
        # Initialitation of the parameters required
        self.x_f = 0
        self.y_f = 0
        self.first = False
        self.m = 80
        self.control_mass = None
        self.change_mass = self.m
        self.g = 9.81
        self.k = 22000
        self.change_k = self.k
        self.x_0 = 0
        self.y_0 = np.sqrt(self.l**2 - self.x_0**2)
        self.v_0 = 5
        self.dx_0 = self.v_0
        self.dy_0 = 0
        self.v0_ref = 5
        self.y0_re = 1
        self.vint = 0
        self.actual_acc = []
        self.acc_predict_store = []
        self.store_mass_predict = []
        self.Esys = self.m*self.g*self.y_0 + 1/2 * self.m * self.v_0**2
        self.yinit = (self.Esys - 1/2 * self.m *self.vint**2)/self.m/self.g
        # self.y_0 = 1
        self.ALPHA_0 = 69*np.pi/180
        self.DXFOOT = 1*np.cos(self.ALPHA_0)
        self.Y_LAND = self.l * np.sin(self.ALPHA_0)
        self.temp_state = 0

    def _stance(self):
        x, y = np.copy(self.x), np.copy(self.y)
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
        if cond_1 == True and cond_2 == False or self.Stance_state and cond_1:
            self.Flight_state = True
            self.Stance_state = False
        if cond_1 == False and cond_2 == True or self.Flight_state and cond_2:
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

    def _apex_detection(self,pos,vel):
        if pos > self.Y_LAND and -0.005< vel < 0.005:
            return 1
        else:
            return 0

    def _apex_store(self,pos,vel):
        self.store_apex = [self._apex_detection(i[0],i[1]) for i in zip(pos,vel)]

    def _bottom_detection(self,pos,vel):
        if pos < self.Y_LAND and -0.01< vel < 0.01:
            return 1
        else:
            return 0

    def _bottom_store(self,pos,vel):
        self.store_bottom = [self._bottom_detection(i[0],i[1]) for i in zip(pos,vel)]

    def solver(self,predict = True):
        self.count = 0
        self.result = np.zeros((1,4))
        t_eval = np.arange(0,4,0.001)
        y_0 = [self.x_0,self.y_0,self.dx_0,self.dy_0]
        self.flag_apex = False
        self.apex_data = np.array([])
        self.bottom_data = np.array([])
        self.x_s_store = []
        self.y_s_store = []
        self.y_acc_store = []
        self.mass_predict_store = []
        self.predict_mass = self.m
        self.predict = 0.88
        self.store_predict = []
        for i in range(1,len(t_eval)-1):
            if i > 1000:
                self.m = self.change_mass
            self.sol = solve_ivp(self._intergrate,[t_eval[i-1],t_eval[i]],y0=y_0,dense_output = True)
            y_0 = self.sol.y[:,-1]
            self.result = np.append(self.result,np.copy(self.sol.y[:,-1]).reshape(1,4),axis = 0)
            # storing for the rendering
            ###
            self.y_acc_store.append(self.y_acc)
            self.x_s_store.append(self.x_s)
            self.store_state.append(1 if self.Stance_state == True else 0)
            self.y_s_store.append(self.y_s)
            ###
            # for prediction
            if predict:
                # self._apex_prediction()
                self._bottom_prediction()
                if self._bottom_detection(self.result[-1,1],self.result[-1,3]):
                    log.debug(i)
                    # plt.plot(self.result[i-50:i,1],self.result[i-50:i,3])
                    # plt.show()
                    log.debug(f'####### actual prediction,{self.result[-1,1]} acc = {self.y_acc}')
                    if self.predict_mass < 90:
                        self.control_mass = 80
                    elif self.predict_mass > 90:
                        self.control_mass = 100
                self._controller()
                self.store_predict.append(self.predict)
                self.mass_predict_store.append(self.predict_mass)

    def _apex_prediction(self):
        train_data = self.apex_data
        y_0 = np.copy(self.sol.y[:,-1])
        if y_0[1] > self.Y_LAND and y_0[3] > 0:
            if len(train_data) == 0:
                train_data = np.append(np.copy(self.sol.y[1,-1]),np.copy(self.sol.y[3,-1])).reshape(1,2)
            else:
                train_data = np.append(train_data,np.append(np.copy(self.sol.y[1,-1]),np.copy(self.sol.y[3,-1])).reshape(1,2),axis=0)
            if len(train_data) > 25:
                self.self_tune = 0.5e-6
                self._real_time_gmm(train_data)
                # print('apex predict', self.predict)
                # print('mass',self.Esys / (1/2 * 5**2  + self.g * self.predict))
        else:
            train_data = np.array([])
        self.apex_data = train_data

    def _bottom_prediction(self):
        train_data = self.bottom_data
        y_0 = np.copy(self.sol.y[:,-1])
        if y_0[1] < self.Y_LAND and y_0[3] < 0:
            if len(train_data) == 0:
                train_data = np.append(np.copy(self.sol.y[1,-1]),np.copy(self.sol.y[3,-1])).reshape(1,2)
            else:
                train_data = np.append(train_data,np.append(np.copy(self.sol.y[1,-1]) ,np.copy(self.sol.y[3,-1])).reshape(1,2),axis=0)
            if len(train_data) > 45:
                self.self_tune = 7e-8
                y = train_data[-35:-1,1]
                t = np.arange(1,len(y)+1)
                coeff = np.polynomial.polynomial.polyfit(t,y,5)
                ffit = np.polynomial.polynomial.Polynomial(coeff)
                y_predict = ffit(np.arange(1,150))
                try:
                    loc =np.where(y_predict > 0)[0][0]
                except:
                    loc = 150
                acc = (y_predict[loc-5] - y_predict[loc-6])/0.001
                resdual= np.sqrt(self.x_s**2 + self.y_s**2)
                log.debug(abs((1 - abs(y[3]))))
                self.self_diag_tune = self.self_tune * 1e10
                self._real_time_gmm(train_data[-30:-1,:], control_diag = True)
                mass = (((1/round(self.predict,2) - 1) * self.k)) / (self.g + acc)
                # mass = mass - mass*(1 - abs(1+y[3]))/self.g
                # mass = mass - mass*(1 - abs((1 - abs(y[3]))))/self.g
                # mass = mass - mass/self.g
                self.predict_mass = np.copy(mass)
                log.debug(f'bottom predict {self.predict} mass{mass} acc {acc} ')
                self.actual_acc.append(self.y_acc)
                self.acc_predict_store.append(acc)
                self.store_mass_predict.append(mass)
        else:
            train_data = np.array([])
        self.bottom_data = train_data

    def start(self,predict = True):
        self.solver(predict = predict)
        self._apex_store(self.result[1:,1],self.result[1:,3])
        self._bottom_store(self.result[1:,1],self.result[1:,3])

    def _real_time_gmm(self,train_data,control_diag = False):
        ik = len(train_data)
        gmm = mixture.BayesianGaussianMixture(n_components=5, max_iter = 100,
            weight_concentration_prior=1e0,weight_concentration_prior_type='dirichlet_process',
            mean_precision_prior=1e-1) #, covariance_prior=1e-1 * np.eye(2))
        gmm.fit(train_data)
        predict = gmm.predict(train_data)
        gmm.fit(train_data)
        predict = gmm.predict(train_data)
        # log.debug('prediction number',predict.size)
        unique = np.unique(predict)
        mean = gmm.means_[unique]
        cov = gmm.covariances_[unique]
        if np.any(cov[:,0,0] > self.self_tune*abs(ik - 1)):
            cov[cov[:,0,0] > self.self_tune*abs(ik - 1),0,0] = self.self_tune*abs(ik-1)
        if control_diag == True:
            if np.any(cov[:,0,1] > self.self_diag_tune*abs(ik - 1)):
                cov[cov[:,0,1] > self.self_diag_tune*abs(ik - 1),0,1] = self.self_tune*abs(ik-1)
        # log.debug(f'means {mean}')
        A = np.multiply(cov[:,0,1],np.reciprocal(cov[:,0,0]))
        B = np.multiply(A,mean[:,0]) - mean[:,1]
        self.predict = abs(sum(B)/sum(A))

    def show_plot(self):
        plt.plot(self.result[:,1],label = 'y-axis')
        plt.show()
        plt.figure()
        plt.plot(self.result[1:,1],self.result[1:,3])
        plt.legend()
        plt.show()

    def gmm(self, train_data):
        # log.debug(f'##### Start GMM ######')
        solution = []
        for ik in range(5,len(train_data)):
            self.self_tune = 10e-3
            gmm = mixture.BayesianGaussianMixture(n_components=5, max_iter = 100,
                weight_concentration_prior=1e0,weight_concentration_prior_type='dirichlet_process',
                mean_precision_prior=1e-1) #, covariance_prior=1e-1 * np.eye(2))
            gmm.fit(train_data[:ik])
            predict = gmm.predict(train_data[:ik])
            gmm.fit(train_data[:ik])
            predict = gmm.predict(train_data[:ik])
            # log.debug('prediction number',predict.size)
            unique = np.unique(predict)
            mean = gmm.means_[unique]
            cov = gmm.covariances_[unique]
            if np.any(cov[:,0,0] > self.self_tune*abs(ik - 1)):
                cov[cov[:,0,0] > self.self_tune*abs(ik - 1),0,0] = self.self_tune*abs(ik-1)
            # log.debug(f'cov{cov[:,0,0]}, {cov[:,0,1]}, limit: {self.self_tune*abs(ik-1)}')
            # log.debug(f'means {mean}')
            A = np.multiply(cov[:,0,1],np.reciprocal(cov[:,0,0]))
            B = np.multiply(A,mean[:,0]) - mean[:,1]
            solution.append(sum(B)/sum(A))
            # log.debug(f'solution {solution[-1]}')
            # log.debug(f'sum,{sum(A)},{sum(B)}')
        self.solution = solution

    def _controller(self):
        with open('superstable_1.json') as f:
            data = json.load(f)
        if self.control_mass != None:
            if self.control_mass < 90:
                self.k = data['k'][data['m'].index(85)]
            if self.control_mass > 90:
                self.k = data['k'][data['m'].index(100)]





if __name__ == "__main__":
    hopper = Hopper()
    hopper.start(predict=False)
    hopper.show_plot()
