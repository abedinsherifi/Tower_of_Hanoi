% Abedin Sherifi
% RBE 500
% Project 2
% 12/14/2019

function output=manipulator_3dof_scara_project2(t,x,th_goal,manip_det,PIDs)
output=zeros(10,1);

%% Goal positions
th1_f=th_goal(1);
th2_f=th_goal(2);
d3_f=th_goal(3);

%% Manipulator Details
Ma=manip_det(4);
Mb=manip_det(5);
Mc=manip_det(6);
La=manip_det(1);
Lb=manip_det(2);
Lc=manip_det(3);
la = La/2;
lb = Lb/2;
lc = Lc/2;
g=9.8;                  

%% Jacobian Values
J1 = 2.000*10^(-4); 
J2 = 5.000*10^(-5);
J3 = 10.000*10^(-4);

%% Inertia Matrix
d11=J1 + J2 + Ma*la^2+Mb*La^2+Mc*la^2+Mc*Lb^2+2*Mb*La*lb*cos(th2_f)+Mc*lc^2*(sin(d3_f))^2+2*Mc*La*lc*sin(th2_f)*sin(d3_f)+2*Mc*La*Lb*cos(th2_f);
d12=J2 + Mb*lb^2+Mc*Lb^2+Mb*La*lb*cos(th2_f)+Mc*La*lc*sin(th2_f)*sin(d3_f)+Mc*La*Lb*cos(th2_f)+Mc*lc^2*(sin(d3_f))^2;
d13=-Mc*Lb*lc*cos(d3_f)-Mc*La*lc*cos(th2_f)*cos(d3_f);
d21 = d12;
d22 = J2 + Mb*lb^2+Mc*Lb^2+ Mc*lc^2*(sin(d3_f))^2;
d23 = -Mc*Lb*lc*cos(d3_f);
d31 = d13;
d32 = d23;
d33 = J3 + Mc*lc^2;
Dq=[d11 d12 d13;d21 d22 d23;d31 d32 d33];

%% Coriolis Matrix
th1d = x(5); % Theta1 dot
th2d = x(6); % Theta2 dot
d3d = x(7);  % D3 dot

c11 = -2*th2d*Mb*La*lb*sin(th2_f)+d3d*Mc*lc^2*sin(2*th2_f)+2*th2d*Mc*La*lc*cos(th2_f)*sin(d3_f)+...
    2*d3d*Mc*La*lc*sin(th2_f)*cos(d3_f)-2*th2d*Mc*La*Lb*sin(th2_f);
c12 = -th2d*Mb*La*lb*sin(th2_f)+d3d*Mc*lc^2*sin(2*d3_f)+2*d3d*Mc*La*lc*sin(th2_f)*cos(d3_f)+...
    th2d*Mc*La*lc*cos(th2_f)*sin(d3_f)-th2d*Mc*La*Lb*sin(th2_f);
c13 = d3d*Mc*lc*Lb*sin(d3_f)+d3d*Mc*La*lc*cos(th2_f)*sin(d3_f);
c21 = -3*th2d*Mb*La*lb*sin(th2_f)+d3d*Mc*lc^2*sin(2*d3_f)-2*th1d*Mb*La*lb*sin(th2_f)-...
    th1d*Mc*La*lc*sin(d3_f)*cos(th2_f)+th1d*Mc*La*Lb*sin(th2_f);
c23 = d3d*Mc*Lb*lc*sin(d3_f);
c22 = d3d*Mc*lc^2*sin(2*d3_f);
c31 = d3d*Mc*lc*La*cos(th2_f)*cos(d3_f)+0.5*th1d*Mc*lc^2*sin(2*d3_f)+th2d*Mc*lc^2*sin(2*th2_f)-...
    th1d*Mc*lc*La*sin(d3_f)*cos(th2_f)-th1d*Mc*lc*La*sin(th2_f)*sin(d3_f);
c32  =0;
c33 =0;
Cq=[c11 c12 c13;c21 c22 c23;c31 c32 c33];

%% Gravity Matrix
g1=0;
g2=0;
g3 = -Mc*g*lc*sin(d3_f);
Gq=[g1;g2;g3];

%% PID parameters for theta 1 and 2
Kp1=PIDs(1);
Kd1=PIDs(2);
Ki1=PIDs(3);
Kp2=PIDs(4);
Kd2=PIDs(5);
Ki2=PIDs(6);

%% Control input
trq1=Kp1*(th1_f-x(3))-Kd1*x(6)+Ki1*(x(1));
trq2=Kp2*(th2_f-x(4))-Kd2*x(7)+Ki2*(x(2));
trq=[trq1;trq2;0];
trqf=Dq*trq;

%% System states
output(1)=(th1_f-x(3)); 
output(2)=(th2_f-x(4));

%% Theta 1, Theta 2, and D3 positions
output(3)=x(5); % Theta1
output(4)=x(6); % Theta2
output(5)=x(7); % D3

%% Dynamic equation and joint variable velocities
q2dot=inv(Dq)*(-Cq-Gq+trqf);
output(6)=q2dot(1); %Theta1 dot
output(7)=q2dot(2); %Theta2 dot
output(8)=q2dot(3); %D3 dot

%% Torque 1 and 2 Returns
output(9)=trqf(1);
output(10)=trqf(2);
end