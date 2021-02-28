% Abedin Sherifi
% RBE 500
% Project 2
% 12/14/2019

function Tower_of_Hanoi()
cp_data = [0, 0, -.25, 0];

%% Move #1
%Grasp disc 4 from Peg 1 and ungrasp on Peg 3 in disc position 1
[th1_goal, th2_goal, d3_goal, cp_data] = grasp_disc(0.0, 0.0, -0.25, 1.0, 4.0, cp_data);
[th1_goal, th2_goal, d3_goal, cp_data] = ungrasp_disc(th1_goal, th2_goal, d3_goal, 3.0, 1.0, cp_data);

%% Move #2
%Grasp disc 3 from Peg 1 and ungrasp on Peg 2 in disc position 1
[th1_goal, th2_goal, d3_goal, cp_data] = grasp_disc(th1_goal, th2_goal, d3_goal, 1.0, 3.0, cp_data);
[th1_goal, th2_goal, d3_goal, cp_data] = ungrasp_disc(th1_goal, th2_goal, d3_goal, 2.0, 1.0, cp_data);

%% Move #3
%Grasp disc 1 from Peg 3, and ungrasp on Peg 2, in disc postion 2
[th1_goal, th2_goal, d3_goal, cp_data] = grasp_disc(th1_goal, th2_goal, d3_goal, 3.0, 1.0, cp_data);
[th1_goal, th2_goal, d3_goal, cp_data] = ungrasp_disc(th1_goal, th2_goal, d3_goal, 2.0, 2.0, cp_data);

%% Move #4
%Grasp disc 2 from Peg 1, and ungrasp on Peg 3, in disc postion 1
[th1_goal, th2_goal, d3_goal, cp_data] = grasp_disc(th1_goal, th2_goal, d3_goal, 1.0, 2.0, cp_data);
[th1_goal, th2_goal, d3_goal, cp_data] = ungrasp_disc(th1_goal, th2_goal, d3_goal, 3.0, 1.0, cp_data);

%% Move #5
%Grasp disc 2 from Peg 2, and ungrasp on Peg 1, in disc postion 2
[th1_goal, th2_goal, d3_goal, cp_data] = grasp_disc(th1_goal, th2_goal, d3_goal, 2.0, 2.0, cp_data);
[th1_goal, th2_goal, d3_goal, cp_data] = ungrasp_disc(th1_goal, th2_goal, d3_goal, 1.0, 2.0, cp_data);

%% Move #6
%Grasp disc 1 from Peg 2, and ungrasp on Peg 3, in disc postion 2
[th1_goal, th2_goal, d3_goal, cp_data] = grasp_disc(th1_goal, th2_goal, d3_goal, 2.0, 1.0, cp_data);
[th1_goal, th2_goal, d3_goal, cp_data] = ungrasp_disc(th1_goal, th2_goal, d3_goal, 3.0, 2.0, cp_data);

%% Move #7
%Grasp disc 2 from Peg 1, and ungrasp on Peg 3, in disc postion 3
[th1_goal, th2_goal, d3_goal, cp_data] = grasp_disc(th1_goal, th2_goal, d3_goal, 1.0, 2.0, cp_data);
[th1_goal, th2_goal, d3_goal, cp_data] = ungrasp_disc(th1_goal, th2_goal, d3_goal, 3.0, 3.0, cp_data);

%% Move #8
%Grasp disc 1 from Peg 1, and ungrasp on Peg 2, in disc postion 1
[th1_goal, th2_goal, d3_goal, cp_data] = grasp_disc(th1_goal, th2_goal, d3_goal, 1.0, 1.0, cp_data);
[th1_goal, th2_goal, d3_goal, cp_data] = ungrasp_disc(th1_goal, th2_goal, d3_goal, 2.0, 1.0, cp_data);

%% Move #9
%Grasp disc 3 from Peg 3, and ungrasp on Peg 2, in disc postion 2
[th1_goal, th2_goal, d3_goal, cp_data] = grasp_disc(th1_goal, th2_goal, d3_goal, 3.0, 3.0, cp_data);
[th1_goal, th2_goal, d3_goal, cp_data] = ungrasp_disc(th1_goal, th2_goal, d3_goal, 2.0, 2.0, cp_data);

%% Move #10
%Grasp disc 2 from Peg 3, and ungrasp on Peg 1, in disc postion 1
[th1_goal, th2_goal, d3_goal, cp_data] = grasp_disc(th1_goal, th2_goal, d3_goal, 3.0, 2.0, cp_data);
[th1_goal, th2_goal, d3_goal, cp_data] = ungrasp_disc(th1_goal, th2_goal, d3_goal, 1.0, 1.0, cp_data);

%% Move #11
%Grasp disc 2 from Peg 2, and ungrasp on Peg 1, in disc postion 2
[th1_goal, th2_goal, d3_goal, cp_data] = grasp_disc(th1_goal, th2_goal, d3_goal, 2.0, 2.0, cp_data);
[th1_goal, th2_goal, d3_goal, cp_data] = ungrasp_disc(th1_goal, th2_goal, d3_goal, 1.0, 2.0, cp_data);

%% Move #12
%Grasp disc 1 from Peg 3, and ungrasp on Peg 2, in disc postion 2
[th1_goal, th2_goal, d3_goal, cp_data] = grasp_disc(th1_goal, th2_goal, d3_goal, 3.0, 1.0, cp_data);
[th1_goal, th2_goal, d3_goal, cp_data] = ungrasp_disc(th1_goal, th2_goal, d3_goal, 2.0, 2.0, cp_data);

%% Move #13
%Grasp disc 2 from Peg 1, and ungrasp on Peg 3, in disc postion 1
[th1_goal, th2_goal, d3_goal, cp_data] = grasp_disc(th1_goal, th2_goal, d3_goal, 1.0, 2.0, cp_data);
[th1_goal, th2_goal, d3_goal, cp_data] = ungrasp_disc(th1_goal, th2_goal, d3_goal, 3.0, 1.0, cp_data);

%% Move #14
%Grasp disc 1 from Peg 1, and ungrasp on Peg 2, in disc postion 3
[th1_goal, th2_goal, d3_goal, cp_data] = grasp_disc(th1_goal, th2_goal, d3_goal, 1.0, 1.0, cp_data);
[th1_goal, th2_goal, d3_goal, cp_data] = ungrasp_disc(th1_goal, th2_goal, d3_goal, 2.0, 3.0, cp_data);

%% Move #15
%Grasp disc 1 from Peg 3, and ungrasp on Peg 2, in disc postion 4
[th1_goal, th2_goal, d3_goal, cp_data] = grasp_disc(th1_goal, th2_goal, d3_goal, 3.0, 1.0, cp_data);
[th1_goal, th2_goal, d3_goal, cp_data] = ungrasp_disc(th1_goal, th2_goal, d3_goal, 2.0, 4.0, cp_data);
  
% Write data to text file
dlmwrite("sim_data.txt", cp_data, " ");
end