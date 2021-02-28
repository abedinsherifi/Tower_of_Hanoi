% Abedin Sherifi
% RBE 500
% Project 2
% 12/14/2019

function [th1_goal, th2_goal, d3_goal, cp_data] = grasp_disc(th1_init, th2_init, d3_init, peg, disc_pos, cp_data)
%% Peg location, Peg Specifications, Disk Specifications, Link Specifications
pegs = [-0.2, 0.4, 0.0; 0.1, 0.4, 0.0; 0.4, 0.4, 0.0];
peg_Height = 0.2;
z0_offset = 0.5;
z1_offset = 0.02;
Link_d = 0.3;
Link3_Length = 0.6;
Disk_Height = 0.03;

%% Inverse Kinematics
ikine_th2 = @(x, y, l1, l2) (atan2(sqrt(1-(x^2 + y^2 - l1^2 - l2^2) / (2 * l1 * l2)), (x^2 + y^2 - l1^2 - l2^2) / (2 * l1 * l2)));
ikine_th1 = @(x, y, l1, l2, th2) (atan2(y, x) - atan2(l2 * sin(th2), l1 + l2 * cos(th2)));

% number of steps for each path segment
num_steps = 20;
  
%% Define first segment
th2_start = th2_init;
th2_goal = ikine_th2(pegs(peg,1), pegs(peg,2), Link_d, Link_d);
    
th1_start = th1_init;
th1_goal = ikine_th1(pegs(peg,1), pegs(peg,2), Link_d, Link_d, th2_goal);

%% set end point above peg:
above_peg = -0.05 + z0_offset + z1_offset - peg_Height - Link3_Length * .5;
above_disk = z0_offset + z1_offset - disc_pos * Disk_Height - Link3_Length * 0.5 - .02;

d3_start = d3_init;
d3_goal = above_peg;
  
%% Generate data
th1 = th1_start:(th1_goal - th1_start)/num_steps:th1_goal;
th2 = th2_start:(th2_goal - th2_start)/num_steps:th2_goal;
d3 = d3_start:(d3_goal - d3_start)/num_steps:d3_goal;
grip = ones(1, num_steps+1);

cp_data = [cp_data; th1', th2', d3', grip'];

%% Define second segment
d3_start = d3_goal;
d3_goal = above_disk;
     
th1 = th1_goal * ones(1, num_steps+1);
th2 = th2_goal * ones(1, num_steps+1);
d3 = d3_start:(d3_goal - d3_start)/num_steps:d3_goal;
grip = ones(1, num_steps+1);


cp_data = [cp_data; th1', th2', d3', grip'];
     
%% Define third segment
d3_start = d3_goal;
d3_goal = above_peg - 0.1;
     
d3 = d3_start:(d3_goal - d3_start)/num_steps:d3_goal;
     
cp_data = [cp_data; th1', th2', d3' grip'];
end