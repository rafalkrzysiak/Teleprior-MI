function [xs, tf, tloc] = RobotExperimentDataSet()
% -- NEW:
% -- This function will serve to pull trajectory data captured by the 
% -- overhead tracking system developed for the Omron Lab
addpath('data/FILTERED/1181/');
loc = "data\FILTERED\1181\EKFtraj_condition_1.csv";

traj = readmatrix(loc); % -- pull data from csv file
xs = traj(1:3,:); % -- store only the (x,y,theta) data
tf = size(xs,2);
tloc = [mean(traj(4,:)) mean(traj(5,:))];

end