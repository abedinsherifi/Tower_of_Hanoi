% Abedin Sherifi
% RBE 500
% Project 2
% 12/14/2019

close all
clear all
clc

%% Manipulator Details
La=0.300;                                           %Link A in meters
Lb=0.300;                                           %Link B in meters
Lc=0.600;                                           %Link C in meters
Ma=6;                                               %Mass A in Kg
Mb=6;                                               %Mass B in Kg
Mc=5;                                               %Mass C in Kg
manip_det=[La Lb Lc Ma Mb Mc];

%% Initial setup and known variables
th_initial=[1.3077 1.4535 -0.13];                  %Initial positions
th_goal=[0.51341 1.6248 -0.13];                    %Goal positions
initial=[0 0 th_initial 0 0 0 0 0];                %Initial state values

%% Time duration
t0 = 0.0;
tspan = t0:0.01:5;

%% PID parameters for theta 1 and 2 based on manual pid tunning
Kp1=1;
Kd1=20;
Ki1=10;
Kp2=1;
Kd2=15;
Ki2=10;
PIDs=[Kp1 Kd1 Ki1 Kp2 Kd2 Ki2];

%% ODE45 implemented in solving the ordinary differential equations
[T,X] = ode45(@(t,x) manipulator_3dof_scara_project2(t,x,th_goal,manip_det,PIDs),tspan,initial);

%% Output joint positions and velocities
th1=X(:,3);                     %Theta1 position
th2=X(:,4);                     %Theta2 position
d3=X(:,5);                      %Theta1 position
th1dot=X(:,6);                  %Theta1 velocity
th2dot=X(:,7);                  %Theta2 velocity
d3dot=X(:,8); 

%% Torque equations while taking into consideration the 15 N-m stall torque
trq1=diff(X(:,9))./diff(T);
trq2=diff(X(:,10))./diff(T);
ttime=0:(T(end)/(length(trq1)-1)):T(end);

%% Error plot for Theta1
figure
subplot(4,3,1)
plot(T,th_goal(1)-th1)
grid
title('Theta1 Error Vs. Time')
ylabel('Theta1 Error (rad)')
xlabel('Time (sec)')

%% Error plot for Theta2
subplot(4,3,2)
plot(T,th_goal(2)-th2)
grid
title('Theta2 Error Vs. Time')
ylabel('Theta2 Error (rad)')
xlabel('Time (sec)')

%% Error plot for D3
subplot(4,3,3)
plot(T,th_goal(3)-d3)
grid
title('D3 Error Vs. Time')
ylabel('D3 Error (m)')
xlabel('Time (sec)')

%% Plot for Theta1 Position
subplot(4,3,4)
plot(T,th1)
grid
title('Theta1 Vs. Time')
ylabel('Theta1 (rad)')
xlabel('Time (sec)')

%% Plot for Theta2 Position
subplot(4,3,5)
plot(T,th2)
grid
title('Theta2 Vs. Time')
ylabel('Theta2 (rad)')
xlabel('Time (sec)')

%% Plot for D3 Position
subplot(4,3,6)
plot(T,d3)
grid
title('D3 Vs. Time')
ylabel('D3 (m)')
xlabel('Time (sec)')

%% Plot for Theta1 Velocity
subplot(4,3,7)
plot(T,th1dot)
grid
title('Theta1 Velocity Vs. Time')
ylabel('Theta1dot (rad/sec)')
xlabel('Time (sec)')

%% Plot for Theta2 Velocity
subplot(4,3,8)
plot(T,th2dot)
grid
title('Theta2 Velocity Vs. Time')
ylabel('Theta2dot (rad/sec)')
xlabel('Time (sec)')

%% Plot for D3 Velocity
subplot(4,3,9)
plot(T,d3dot)
grid
title('D3 Velocity Vs. Time')
ylabel('D3dot (m/sec)')
xlabel('Time (sec)')

%% Plot for Torque 1
subplot(4,3,10)
plot(ttime,trq1)
grid
title('Theta1 TRQ Vs. Time')
ylabel('Theta1 Torque (N-m)')
xlabel('Time (sec)')

%% Plot for Torque 2
subplot(4,3,11)
plot(ttime,trq2)
grid
title('Theta2 TRQ Vs. Time')
ylabel('Theta2 Torque (N-m)')
xlabel('Time (sec)')