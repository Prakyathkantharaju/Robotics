clc
clear all
% close all

%%% This example plots the workspace of the manipulator given the joint limits
% <<two_link.png>>

%%% Specify link lengths
l1 = 1; l2 = 0.5; l3 = 0.5;

%define the limits
theta1_min = -pi/2; theta1_max = pi/2;
theta2_min = 0; theta2_max = pi;
theta3_min = 0; theta3_max = pi/2;

theta1_vec = linspace(theta1_min,theta1_max); %create vectors
theta2_vec = linspace(theta2_min,theta2_max);
theta3_vec = linspace(theta3_min, theta3_max);
[theta1_all,theta2_all,theta3_all] = meshgrid(theta1_vec,theta2_vec, theta3_vec); %create matrix for all combinations of theta's
%%

[m,n,p] = size(theta1_all);

for i=1:m
    for j=1:n
        for k = 1:p
            theta1 = theta1_all(i,j,k);
            theta2 = theta2_all(i,j,k);
            theta3 = theta3_all(i,j,k);
            %%%%%%%% origin  in world frame  %%%%%%
            x_O0 = 0; y_O0 = 0; 

            %%%%% end of link1 in world frame %%%%
            x_P = l1*cos(theta1); 
            y_P = l1*sin(theta1);  

            %%%% end of link 2 in world frame  %%%%%%%
            x_Q = x_P + l2*cos(theta1+theta2); 
            y_Q = y_P + l2*sin(theta1+theta2);
            
            %%%% end of link 3 in the work frame %%%%%
            x_R = x_Q + l3*cos(theta1+theta2 + theta3); 
            y_R = y_Q + l3*sin(theta1+theta2 + theta3);

            %save end-effector position in world frame
            x(i,j) = x_R; %save x position
            y(i,j) = y_R; %save y position
        end
    end
end

figure(1)
plot(x,y,'ro');
xlabel('x');
ylabel('y');
grid on;
title('workspace for the two-link manipulator');