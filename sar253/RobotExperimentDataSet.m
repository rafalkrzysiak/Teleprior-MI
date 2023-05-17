function [xs, tf, tloc, file_id] = RobotExperimentDataSet()
% -- NEW:
% -- This function will serve to pull trajectory data captured by the 
% -- overhead tracking system developed for the Omron Lab
addpath('data/FILTERED/1181/');
id='1181';
cond='1';
loc = ['data', filesep, 'FILTERED', filesep, id,...
    filesep, 'EKFtraj_condition_', cond, '.csv'];

traj = csvread(loc); % -- pull data from csv file
xs = traj(1:3,:); % -- store only the (x,y,theta) data
tf = size(xs,2);
tloc = [mean(traj(4,:)) mean(traj(5,:))];

% output the an identifier for each file as for example below
file_id=[id, '_', cond];
end