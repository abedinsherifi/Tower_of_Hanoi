% Abedin Sherifi

%Animation of the manipulator
function anim(fname)

fdata = dlmread(fname);         % Read sim_data file

d = 5;                          % Step Size
T = size(fdata,1);              % Time size
j = 1:d:T;                      % j is a vector with values starting with 1 and with end value T incrementing by d steps

% Link specifications
La=0.300;                          
Lb=0.300;                         

%%
    % Assign size of fdata to idx in order to get th1, th2, and d3
    % numbers for the whole trajectory. 
    for idx = 1:size(fdata,1)

        % set robot states from fdata
        th1 = fdata(idx,1);                     % Theta1 total trajectory values
        th2 = fdata(idx,2);                     % Theta2 total trajectory values
        
        y1(idx)=La.*sin(th1);                   % Y1 position for whole traj
        x1(idx)=La.*cos(th1);                   % X1 position for whole traj
        y2(idx)=La.*sin(th1)+Lb.*sin(th1+th2);  % Y2 position for whole traj
        x2(idx)=La.*cos(th1)+Lb.*cos(th1+th2);  % X2 position for whole traj
    end
    
%%
% 2D animation generation
figure

for i=1:length(j)-1
hold off
plot([x1(j(i)) x2(j(i))],[y1(j(i)) y2(j(i))],'g--o',[0 x1(j(i))],[0 y1(j(i))],'r',[x1(j(i)) x2(j(i))],[y1(j(i)) y2(j(i))],'r');
title('Motion of 3DOF SCARA Manipulator')
xlabel('X Positions')
ylabel('Y Positions')
axis([-1 1 -1 1]);
grid
hold on
animation(i)=getframe(gcf);
end
drawnow;
end
