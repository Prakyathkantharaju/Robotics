clc
clear all
close all
warning('off')

%% parameters
% movie parameters
movieWrite = 1; 
movieName = 'manipulator_drawing.avi';

% link parameters %%%%%%
l1 = 1; l2 = 0.5; l3  = 0.5;

delay = 0.05; %increase the value to slow down animation

% parameters of the circle
x_center = 0.58; y_center = 0.72; a = 0.93;

% t is the parameter used to describe the astroid
t = linspace(0,2 * pi,51);
n = length(t);
x_ref_all = x_center+a*cos(t).^3; 
y_ref_all = y_center+a*sin(t).^3;
x_ref_old = 1+0.5*cos(t).^3; 
y_ref_old = 0.5+0.5*sin(t).^3;

plot(x_ref_all,y_ref_all, 'DisplayName', 'new', 'LineWidth', 5);
hold on
plot(x_ref_old,y_ref_old, 'DisplayName', 'old');
legend;
hold off;
%%
% keyboard;
%%


%%%% uppper and lower bound %%%%%
theta1_min = -pi/2; theta1_max = pi/2;
theta2_min = 0; theta2_max = pi;
theta3_min = 0; theta3_max = pi/2;
ub = [theta1_max; theta2_max; theta3_max];
lb = [theta1_min; theta2_min; theta3_min];

%%%% initial guess for fsolve %%%
theta10 =  -0.5 ; 
theta20 =  0 ; 
theta30 =  0; 

resnorm_store = zeros(n,2);
exit_store = zeros(n,1);
%%% Solve for the values of theta that give the required end-effector pose.
%fsolve solves for the roots for the equation F = [x-x_ref; y-y_ref];
for i=1:n
    x_ref = x_ref_all(i);
    y_ref = y_ref_all(i);
    param = [l1 l2 l3 x_ref y_ref];
    
    options = optimoptions('fsolve','Display','off','MaxIter',200);
%     [X,FVAL,EXITFLAG] = fsolve('fn_end_effector_position',[theta10,theta20, theta30],options,param);
    [X,resnorm,residual,EXITFLAG,output,lambda,jacobian] = lsqnonlin('fn_end_effector_position',[theta10; theta20; theta30],lb,ub,options,param);
    %%%%%%%%%% save the solution %%%%%%%%%%
    theta1(i) = X(1);
    theta2(i) = X(2);
    theta3(i) = X(3);
    
    theta10 = X(1);
    theta20 = X(2);
    theta30 = X(3);
%      disp(['Exitflag after running fsolve = ', num2str(EXITFLAG) ]) %Tells if fsolve converged or not
                   %1 means converged else not converged
                   %Type help fsolve to know more about what different 
                   %EXITFLAG mean.
    if EXITFLAG ~= 1
        i
        keyboard;
    end
    resnorm_store(i,:) = residual(1:2);
    exit_store(i) = EXITFLAG;
end               
% resnorm_store = resnorm_store(
figure(1)
plot(theta1,'r'); hold on
plot(theta2,'b-');
plot(theta3,'m-.');
ylabel('position');
legend('theta1','theta2','theta3','Location','Best');

figure(2)
   
x_drawn = [];
y_drawn = [];
if (movieWrite)
    mov = VideoWriter(movieName); 
    open(mov);
end

for i=1:n
 figure(2)
%%%% origin of the world/fixed frame %%%
x_O0 = 0; y_O0 = 0;

%%%%%%% end of link1 in world frame %%%
x_P = l1*cos(theta1(i)); 
y_P = l1*sin(theta1(i));

%%%% end of link2 in world frame %%%
x_Q = x_P  +  l2*cos(theta2(i) + theta1(i));
y_Q = y_P + l2*sin(theta2(i) + theta1(i));

%%%% end of link3 in world frame %%%
x_R = x_Q  +  l2*cos(theta3(i) + theta2(i) + theta1(i));
y_R = y_Q + l2*sin(theta3(i) + theta2(i) + theta1(i));

x_drawn = [x_drawn; x_R];
y_drawn = [y_drawn; y_R];

figure(2)
%Plot the point where we want the end-effector
plot(x_drawn(1:i),y_drawn(1:i),'k'); 


%%%%%%%% draw line to generate the manipulator
line([x_O0 x_P],[y_O0 y_P],'Linewidth',5,'Color','r');
line([x_P x_Q],[y_P y_Q],'Linewidth',5,'Color','g');
line([x_Q x_R],[y_Q y_R],'Linewidth',5,'Color','b');
xlabel('x'); 
ylabel('y');
grid on; %if you want the grid to show up.

%These set the x and y limits for the axis (will need adjustment)
xlim([-2 2]); 
ylim([-2 2]);
figure(2)
pause(delay);

if (movieWrite)
    axis off %does not show axis
    figure(2)
    set(gcf,'Color',[1,1,1]) %set background to white
    writeVideo(mov,getframe);
end

       
end

if (movieWrite)
    close(mov);
end
%% results
%ii
fprintf('i. x_0: %f, y_0 = %f, a = %f \n ',x_center, y_center, a );
%iii
figure(1234)
subplot(3,1,1)
title('1st theta')
plot(theta1,'r--','DisplayName','theta1');
line([0,n],[theta1_min,theta1_min],'color','k','DisplayName','min');
line([0,n],[theta1_max,theta1_max],'color','k','DisplayName','max');
axis([-1 50 -5 5])
legend;
title('link 1')
%
figure(1234)
subplot(3,1,2)
title('2nd theta')
plot(theta2,'g--','DisplayName','theta2');
line([0,n],[theta2_min,theta2_min],'color','k','DisplayName','min');
line([0,n],[theta2_max,theta2_max],'color','k','DisplayName','max');
legend;
axis([-1 50 -5 5])
title('link2')
%
figure(1234)
subplot(3,1,3)
title('3rd theta')
plot(theta3,'b--','DisplayName','theta3');
line([0,n],[theta3_min,theta3_min],'color','k','DisplayName','min');
line([0,n],[theta3_max,theta3_max],'color','k','DisplayName','max');
axis([-1 50 -5 5])
title('link3')
% iv
figure(1244)
plot(1:n,exit_store,'DisplayName','exit flag')
legend
% v
figure(1254)
plot(1:n,resnorm_store(:,1),'r','DisplayName','residual x');
hold on
plot(1:n,resnorm_store(:,2),'b','DisplayName','residual y');
legend


