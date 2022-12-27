% Abedin Sherifi

% Full trajectory generation plots of x,y, and z positions
function traj_gen(fname)
  
        % Read Data File
        fdata = dlmread(fname);
        
        % La and Lb specifications
        La=0.300;                                                           % Link A in meters
        Lb=0.300;                                                           % Link B in meters


    % Assign size of fdata to idx in order to get th1, th2, and d3
    % numbers for the whole trajectory. 
    for idx = 1:size(fdata,1)

        th1 = fdata(idx,1);                                                 % Theta1 total trajectory values
        th2 = fdata(idx,2);                                                 % Theta2 total trajectory values
        d3 = fdata(idx,3);                                                  % D3 total trajectory values
       
        % Calculating x,y, and z values for the total iterations of
        % th1,th2, and d3 throughout the whole trajectory.
        x(idx) = La*cos(th1)+Lb*cos(th1)*cos(th2)-Lb*sin(th1)*sin(th2);     % X total position trajectory
        y(idx) = La*sin(th1)+Lb*cos(th1)*sin(th2)-Lb*cos(th2)*sin(th1);     % Y total position trajectory
        z(idx) = -d3;                                                       % Z total position trajectory
    end
    
    % Plot
    figure
    plot3(x,y,z)
    grid
    title('3D Plot of Total X, Y, and Z Trajectories')
    ylabel('Y Trajectory')
    xlabel('X Trajectory')
    zlabel('Y Trajectory')
end
