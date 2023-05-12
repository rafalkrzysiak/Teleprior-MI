function p = RobotExperimentDataSet()
% -- NEW:
% -- This function will serve to pull trajectory data captured by the 
% -- overhead tracking system developed for the Omron Lab
addpath('data/FILTERED/1181/');
loc = 'data/FILTERED/1811/EKFtraj_condition_1.csv';

traj = readtable(loc); % -- pull data from csv file
p = traj(1:3,:); % -- store only the (x,y,theta) data

end