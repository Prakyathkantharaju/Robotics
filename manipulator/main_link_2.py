import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize as opt
from typing import List

# relative imports
from kinematic_info import kinematic_info

def animation(kin_info:kinematic_info) -> None:
    # Doing some trignometry from angle to display the links
    links = np.copy(kin_info.length)
    theta = np.copy(kin_info.theta)
    theta = [0,0,0]
    # global origin
    L0 = [0,0]
    L1 = [links[0] * np.cos(theta[0]), links[1] * np.sin(theta[0])]
    L2 = [L1[0] + links[1] * np.cos(theta[1] + theta[0]),
        L1[1] + links[1] * np.sin(theta[1] + theta[0])]
    L3 = [L2[0] + links[2] * np.cos(theta[1] + theta[0] + theta[2]),
        L2[1] + links[2] * np.sin(theta[1] + theta[0] + theta[2])]
    fig,ax = plt.subplots()
    ax.plot([L0[0],L1[0]],[L0[1],L1[1]],'r')
    ax.plot([L1[0],L2[0]],[L1[1],L2[1]],'g')
    ax.plot([L2[0],L3[0]],[L2[1],L3[1]],'b')
    # plt.show()

    traj = np.copy(kin_info.path)

    traj_length = len(traj)

    print('here')
    print(traj_length)
    output_drawn_x = []
    output_drawn_y = []
    # main animation loop
    for i in range(traj_length):
        ax.clear()
        theta[0] = traj[i][0]
        theta[1] = traj[i][1]
        theta[2] = traj[i][2]
        L0 = [0,0]
        L1 = [links[0] * np.cos(theta[0]), links[0] * np.sin(theta[0])]
        L2 = [L1[0] + links[1] * np.cos(theta[1] + theta[0]),
            L1[1] + links[1] * np.sin(theta[1] + theta[0])]
        L3 = [L2[0] + links[2] * np.cos(theta[1] + theta[0] + theta[2]),
            L2[1] + links[2] * np.sin(theta[1] + theta[0] + theta[2])]
        ax.plot([L0[0],L1[0]],[L0[1],L1[1]],'r')
        ax.plot([L1[0],L2[0]],[L1[1],L2[1]],'g')
        ax.plot([L2[0],L3[0]],[L2[1],L3[1]],'b')
        output_drawn_x.append(L3[0])
        output_drawn_y.append(L3[1])
        ax.plot(output_drawn_x,output_drawn_y)
        print(f'frame {i}')
        plt.pause(20/traj_length)
    return None



def transform_matrix(theta: float, length: float) -> np.ndarray:
    T = np.array([[np.cos(theta), -np.sin(theta), length],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]])
    return T

def get_link_points(X:List) -> List:

    theta_0 = X[0]
    theta_1 = X[1]
    theta_2 = X[2]

    # getting references
    X_ref = kin_info.ref[0]
    Y_ref = kin_info.ref[1]

    # getting the link length
    l1 = kin_info.length[0]
    l2 = kin_info.length[1]
    l3 = kin_info.length[2]

    # Getting the transformation matrix
    T_0 = transform_matrix(theta_0, 0)
    T_1 = transform_matrix(theta_1, l1)
    T_2 = transform_matrix(theta_2, l2)
    P_3 = np.array([[l3],[0],[1]])

    # getting points in the child frames
    P_2 = np.matmul(T_2,P_3)
    P_1 = np.matmul(T_1,P_2)

    # getting point in global frame
    P_0 = np.matmul(T_0,P_1)

    return [P_0,P_1,P_2,P_3]


def dynamics_2_link(X: List, kin_info: kinematic_info) -> np.array:


    # Getting the position of links
    Points = get_link_points(X)
    # Get only the last point in global frame
    P_0 = Points[0]
    # Getting the references
    X_ref = kin_info.ref[0]
    Y_ref = kin_info.ref[1]

    Cost = np.array([P_0[0] - X_ref,P_0[1] - Y_ref])
    Cost = Cost[:,0]

    return Cost




# creating a curve to follows
t = np.linspace(0,2 * np.pi, 51)
x_center = 1
y_center = 0.5
a = 0.5
x_ref = x_center + a * np.cos(t)**3
y_ref = y_center + a * np.sin(t)**3
plt.plot(x_ref,y_ref)
plt.show()


# bounds for the angle
theta0_min = -np.pi/2
theta0_max = np.pi/2
theta1_min = 0
theta1_max = np.pi
theta2_min = 0
theta2_max = np.pi/2

upper_bounds = (theta0_min,theta1_min,theta2_min)
lower_bounds = (theta0_max,theta1_max,theta2_max)

# kinematic info
kin_info = kinematic_info(3)
kin_info.update_ref([x_ref[0],y_ref[0]])
kin_info.update_length([1,0.5,0.5])
kin_info.update_const([[theta0_min,theta0_max],
                       [theta1_min,theta1_max],
                       [theta2_min,theta2_max]])

# initial guess for the starting angle
theta_0  = [-0.5,0,0]

# Store_theta
store_theta = []

for i in range(50):
    # update the reference information
    kin_info.update_ref([x_ref[i],y_ref[i]])
    # optimize the to find the theta
    out = opt.least_squares(dynamics_2_link,theta_0,bounds=(upper_bounds,lower_bounds),
                        args = {kin_info})
    # update the inital theta
    theta0 = out.x.tolist()

    store_theta.append(out.x.tolist())
    print(out.success, out.status)
#import pudb; pu.db


kin_info.update_path(store_theta)
animation(kin_info)



