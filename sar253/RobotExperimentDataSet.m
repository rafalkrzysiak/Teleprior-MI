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
xs = traj(1:3,:); % -- store only the (x,y,theta) data
tf = size(xs,2);
tloc = [mean(traj(4,:)) mean(traj(5,:))];

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