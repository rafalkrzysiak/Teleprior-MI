function [xs, speed, tr, tf, tloc, file_id] = RobotExperimentDataSet(exp_id, condition)
% -- NEW:
% -- This function will serve to pull trajectory data captured by the 
% -- overhead tracking system developed for the Omron Lab
% addpath(strcat('data/FILTERED/', exp_id, '/'));
id=exp_id;
cond=num2str(condition);
traj_loc = ['..', filesep, 'data', filesep, 'FILTERED', filesep, id,...
    filesep, 'EKFtraj_condition_', cond, '.csv'];

traj = csvread(traj_loc); % -- pull data from csv file
tf = 3000; % -- size(traj,2)
xs = zeros(5, tf);

% -- sometimes the robot starts too close to the initial wall for condition 1
% -- Because of this, it will leave the domain, ending the simulation
% -- to mitigate this issue, shift the robot by 0.5m to the right (more into the domain).
% -- this will allow the collision avoidance to steer the robot away from the wall
if traj(1,1) < 0.5
 traj(1,1) = traj(1,1) + 0.5;
end

xs(1:3,1) = traj(1:3,1); % -- store only the (x,y,theta) data for the first timestep
tloc = [mean(traj(4,:)); mean(traj(5,:))]; % -- extract the target location data
xs(4:5,:) = tloc .* ones(2, tf); % -- store the target location data for all time steps

% get speed data 
speed_loc = ['..', filesep, 'data', filesep, 'FILTERED', filesep, id,...
    filesep, 'EKFVel_condition_', cond, '.csv'];

speed = csvread(speed_loc); % -- pull data from csv file

% get turn_rate data 
tr_loc = ['..', filesep, 'data', filesep, 'FILTERED', filesep, id,...
    filesep, 'EKFom_condition_', cond, '.csv'];

tr = csvread(tr_loc); % -- pull data from csv file

% output the an identifier for each file as for example below
file_id=[id, '_', cond];
end