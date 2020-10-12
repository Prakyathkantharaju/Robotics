function F = fn_end_effector_position(theta,param)

l1 = param(1); 
l2 = param(2);
l3 = param(3);
x_ref = param(4);
y_ref = param(5);

% Get individual theta's from input
theta1 = theta(1);
theta2 = theta(2);
theta3 = theta(3);
T_1 = [cos(theta1) -sin(theta1) 0;
     sin(theta1)  cos(theta1) 0;
     0 0 1];
T_2 = [cos(theta2) -sin(theta2) l1;
     sin(theta2)  cos(theta2) 0;
     0 0 1];
T_3 = [cos(theta3) -sin(theta3) l2;
     sin(theta3)  cos(theta3) 0;
     0 0 1];
% Location of end of link 3 wrt frame 0
temp = T_1 * T_2 * T_3 * [l3;0;1];
x_R = temp(1);
y_R = temp(2);

%x,y of end of link3, F = 0 is the output
F = [x_R-x_ref; y_R-y_ref;0]; 
% F = [x_R;y_R];