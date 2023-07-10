function [param, maps, folder, bias, share_info, target_locations, agents, file_id] = ParamConfig(exp_id, exp_cond)

% -- This function will serve to contain all parameters to operate
% -- the mutual information based control Search and rescue simulation

param.dt = 1; % -- timestep (s)
[~, param.T, tloc, file_id] = RobotExperimentDataSet(exp_id, exp_cond); %2500/param.dt; % -- max time (s)
param.N = 1200; % -- number of particles in the simulation
param.nsim = 1; % -- number of simulations per combination
param.closedloop = 1; % -- flag for enabling/disabling MI
param.debug = 0; % -- flag for enabling/disabling debug plots
agents = 3; % -- number of agents in the simulation including the human robot
param.r_visible = 1.5*ones(1,agents); % -- visible range of the robots (m), human robot has slightly higher range % 4 -> 2
param.r_FOV = 2*pi*ones(1,agents);
param.r_FOV(1) = 70*pi/18; % -- human robot 70*pi/180 has FOV of 70 degrees, autonomous robots have 360 degrees
param.eta = 0.1*ones(1,agents); % -- sensor noise, human robot will have half of auto robot, try a possible higher eta values ^^^^^
param.r_robot = .15; % --  the radius of the robot (m)
% maps = ["plain_map", "u_map", "complex_map", "OmronLab"]; % -- image file names
maps = ["OmronLab"]; % -- image file names
folder = "maps/"; % -- folder name where maps are held
param.loc_sigma = 1; % -- spread of the location particles
param.kc = 1.5;
bias = [0]; % -- flag for enabling/disabling bias of the human operated robot
param.norm = 1;
share_info = 1;
param.alphaBegin = 0.5; 
param.tau = 15;
param.share = 1;
param.bias = 0;

[~, param.L]=robot_position(maps(1), zeros(1,2));

% -- locations where the target can be located
% R = param.L(2)*sqrt(2)/2; %L/4*(sqrt(2)/2+3);

% target_locations = [R*cosd(45-30), R*sind(45-30); 
%                     R*cosd(45), R*sind(45); 
%                     R*cosd(45+30), R*sind(45+30)];
% target_locations = [R*cosd(45+30), R*sind(45+30)];                

%target_locations = [2 2];
target_locations = [tloc(1) tloc(2)];

% -- the disturbance in the particles jusitification:
% -- 1. the robot itself can successfully localize itself great
% -- 2. the target has a small amount of jitter, it might move ever so slightly
% -- 3. no robot knows the speed of the human robot, it is 50% of the max
% --    possible velocity speed of the creates platform
param.w = diag([.1, .1, .01, .05, .05, .4, .4]); % -- disturbance

% -- Robot and Target dynamics
param.v=linspace(0,.833,3);
param.om=linspace(-.25,.25,3);
param.omega0 = randn*.15;
param.vel0 =.1+rand*.5;
end