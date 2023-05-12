function wrapper_script_MI
% -- This wrapper script will allow to test many different combinations
% -- of the pf MI code
% -- Calling the Matlab script infoseek2Db_multi_robot.m
% -- Written by: Rafal Krzysiak


% -- setup the variables that will be passed into the pf MI script
rng(2);
param.dt = 1; % -- timestep (s)
param.T = 2500/param.dt; % -- max time (s)
param.N = 1200; % -- number of particles in the simulation
param.nsim = 1; % -- number of simulations per combination
param.closedloop = 1; % -- flag for enabling/disabling MI
param.debug = 0; % -- flag for enabling/disabling debug plots
agents = 3; % -- number of agents in the simulation including the human robot
param.r_visible = 4*ones(1,agents); % -- visible range of the robots (m), human robot has slightly higher range
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

[~, param.L]=robot_position(maps(1), zeros(1,2));

% -- locations where the target can be located
% R = param.L(2)*sqrt(2)/2; %L/4*(sqrt(2)/2+3);

% target_locations = [R*cosd(45-30), R*sind(45-30); 
%                     R*cosd(45), R*sind(45); 
%                     R*cosd(45+30), R*sind(45+30)];
% target_locations = [R*cosd(45+30), R*sind(45+30)];                

target_locations = [2 8];

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
               
% -- initialize the iteration/combination number
% -- this will be used as a representation of the total number of 
% -- combinations done for the simulation
iter_num = 1;

tic
for env = 1:size(maps,2) % -- looping through every environment
    path = convertStringsToChars(strcat(folder, maps(env), ".jpg")); % -- strcat the folder and image name
    img = imread(path); % -- read the image into Matlab
    img = imbinarize(img); % -- binarize the image
    
    % -- create a folder that correspond to the environment
    map_folder = strcat("data/",maps(env),"/");
    mkdir(map_folder);
    
    for SI = 1:size(share_info,2) % -- loop through the flags for sharing info in pf update
        
        % -- store the flag that determines whether or not the info can be shared between robots
        param.share = share_info(SI);
        
        for robot = 1:size(agents,2) % -- loop through the number of agents

            % -- create a folder for the number of robots
            robot_folder = strcat(map_folder, num2str(agents(robot)), "robots/");
            mkdir(robot_folder);
            
            % -- store the number of agents to be simulated in the params struct
            param.agents = agents(robot);

            % -- Special Clause:
            % -- if we are only simulating one robot in the environment
            % -- Remember: Robot 1 is the human robot
            % -- we do not want to loop through the alpha values!
            % -- alpha only affects the autonomous robots
            if agents(robot) == 1
                alpha = 1; % -- set the alpha to a 1D value only for 1 Robot
            else
                alpha = 0; % -- set realtime next -- SB 
%                 alpha = 0:.25:1;
            end

            for a = 1:size(alpha,2)

                % -- create a folder for the alpha value within the map/robots folder
                alpha_folder = strcat(robot_folder, "alpha_", num2str(alpha(a)), "/");
                mkdir(alpha_folder);
                
                % -- store the alpha value in the params struct
                param.alpha = alpha(a);

                for b = 1:size(bias,2) % -- loop through different target locations in the map

                    % -- bias folder
                    bias_folder = strcat(alpha_folder, "bias_", num2str(bias(b)), "/");
                    mkdir(bias_folder);
                    
                    % -- store the bias in the params struct
                    param.bias = bias(b);

                    % -- initialize the counter for the target position number
                    target_position = 1;

                    for t_pos = 1:size(target_locations,1)

                        % -- display the combination of the simulation
                        fprintf('iter:%d', iter_num);
                        disp(maps(env)); fprintf('SI:%d, robot:%d, alpha:%.1f, bias:%d, target_position:%d', ...
                            share_info(SI), agents(robot), alpha(a), bias(b), target_position);

                        % -- create a new directory to hold all information about the simulation
                        saveIn = strcat(bias_folder, "target_position", num2str(target_position), "/");
                        saveFrames = strcat(saveIn, "frames/");
                        directory = mkdir(saveIn);
                        frames = mkdir(saveFrames);

                        % -- call the mutual inforation pf simulation matlab script
                        infoseek2Db_multi_robot(param, img, saveIn, maps(env), ...
                                                saveFrames, target_locations(t_pos,:));

                        % -- step the iteration/combination number to the next one
                        % -- as well as the target position number
                        target_position = target_position + 1;
                        iter_num = iter_num + 1; 
                    end
                end
            end
        end
    end
end
toc
end

