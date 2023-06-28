function wrapper_script_MI
% -- This wrapper script will allow to test many different combinations
% -- of the pf MI code
% -- Calling the Matlab script infoseek2Db_multi_robot.m
% -- Written by: Rafal Krzysiak

% -- setup the variables that will be passed into the pf MI script
rng(2);

% -- get the list of participants
ids = ls("data/FILTERED"); % -- get all in the folder
ids = ids(3:end-1,1:4); % -- reformat to only have the numbers

% -- initialize the parameters of the simulation
% [param, maps, folder, bias, share_info, target_locations, agents, file_id] = ...
%     ParamConfig();
               
% -- initialize the iteration/combination number
% -- this will be used as a representation of the total number of 
% -- combinations done for the simulation
iter_num = 1;

maps = "OmronLab";
folder = "maps/";
share_info = 0;
agents = 3;
bias = 0;
target_locations = [0,0];
conds = [1, 4];

tic % -- start timer to determine computational time
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
%             param.agents = agents(robot);

            % -- Special Clause:
            % -- if we are only simulating one robot in the environment
            % -- Remember: Robot 1 is the human robot
            % -- we do not want to loop through the alpha values!
            % -- alpha only affects the autonomous robots
            
            % I think we won't need alpha or bias for this setup so both
            % can go. We can instead use the file id of the experiment that
            % I have passed from RobotExperimentDataSet.m to name our
            % output files. That way we will be able to match experiment to
            % simulation
            if agents(robot) == 1
                alpha = 1; % -- set the alpha to a 1D value only for 1 Robot
            else
                alpha = 0; % -- set realtime next -- SB 
%                 alpha = 0:.25:1;
            end

            for a = 1:size(alpha,2)

                % -- create a folder for the alpha value within the map/robots folder
                %alpha_folder = strcat(robot_folder, "alpha_", num2str(alpha(a)), "/");
                %mkdir(alpha_folder);
                
                % -- store the alpha value in the params struct
                param.alpha = alpha(a);

                for b = 1:size(bias,2) % -- loop through different target locations in the map

                    % -- bias folder
                    %bias_folder = strcat(alpha_folder, "bias_", num2str(bias(b)), "/");
                    %mkdir(bias_folder);
                    
                    % -- store the bias in the params struct
                    param.bias = bias(b);

                    % -- initialize the counter for the target position number
                    target_position = 1;

                    for t_pos = 1:size(target_locations,1)

                        for participant = 1:size(ids, 1)
                            for exp_cond = size(conds, 1)

                                % -- initialize the parameters of the simulation
                                [param, maps, folder, bias, share_info, target_locations, agents, file_id] = ...
                                    ParamConfig(ids(participant, :), conds(exp_cond));

                                % -- store the number of agents to be simulated in the params struct
                                param.agents = agents(robot);

                                % -- display information about the simulation
                                fprintf('Map:%s \nParticipant:%s, condition:%d\n', ...
                                                         maps(env), ids(participant,:), conds(exp_cond));
        
                                % -- create a new directory to hold all information about the simulation
                                saveIn = strcat(robot_folder, ids(participant,:), "/condition_", num2str(exp_cond), "/");
                                saveFrames = strcat(saveIn, "frames/");
                                directory = mkdir(saveIn);
                                frames = mkdir(saveFrames);
        
                                % -- call the mutual inforation pf simulation matlab script
                                infoseek2Db_multi_robot(param, img, saveIn, maps(env), ...
                                                        saveFrames, target_locations(t_pos,:),...
                                                        ids(participant,:), exp_cond);
                            end
                        end

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

