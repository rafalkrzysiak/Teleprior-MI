function wrapper_script_SimExp
% -- This wrapper script will allow to test many different combinations
% -- of the pf MI code utilizing the Experiment data captured
% -- Calling the Matlab script infoseek2Db_multi_robot.m
% -- Written by: Rafal Krzysiak

% -- setup the variables that will be passed into the pf MI script
% rng(2); Seed only if debugging

% -- get the list of participants
% ids = ls("../data/FILTERED"); % -- get all in the folder
% ids = ids(3:end-1,1:4); % -- reformat to only have the numbers

addpath ../src
ID_Data=csvread('../src/IDList_Completed.csv',1,0);

dataFolder="../../simdata/"; % this can sit outside the github

% -- defining a variable to be a flag for us to easily switch
% -- between conditions wanting to test under for updating alpha of the
% -- autonomous robots
% -- Options so far:
% -- "alpha_t/TotalDist"
% -- "alpha_t/FreezeTime"
% -- "alpha_0"
% -- "alpha_1"
% -- "RandomWalk"
ConfigChoice={"alpha_t/TotalDist", "alpha_t/FreezeTime", "alpha_0", ...
    "alpha_1",  "RandomWalk"};


nsims = 1; %number of simulations

% say ii=3 for e.g. to run only for alpha_0

for ii = 1:size(ConfigChoice,2)
    
    ConfigSetup = ConfigChoice{ii};
    
    % -- initialize the parameters of the simulation
    % [param, maps, folder, bias, share_info, target_locations, agents, file_id] = ...
    %     ParamConfig();
    
    conds = [1, 2, 3, 4];
    
    tic % -- start timer to determine computational time
    
    
    % -- define the dt for the tracking system as well as the
    % -- teleoperation system used for the in-lab experiments
    dtTrack = 1/2;
    dtCommand = 1/10;
    
    % get the number of simulations already done 
    % if there is a bad batch, simply remove the folder and rerun
    existing_files=dir(strcat(dataFolder, ConfigSetup, "/"));
    % remove all hidden files
    existing_files=existing_files(~ismember({existing_files.name},...
        {'.','..', '.DS_Store'}));
    
    restart_num=size(existing_files,1);
    
    for test = restart_num+1:restart_num+nsims % -- looping through every environment
        
        % here we create the test/train scenario
        % use 26 randomly selected ids to create pdistr and xdistr and run it on 3
        % remaining ones only
        numSubjects=size(ID_Data,1);
        idsall=randperm(numSubjects);
        
        num_train=floor(0.9*numSubjects);
        
        ids_train=ID_Data(idsall(1:num_train),:);
        ids_test=ID_Data(idsall(num_train+1:numSubjects),:);
        
        % tau should be param.tau but for that paramconfig should be called outside
        % the for loop--can we do that?
        % set tau 15 seconds for distance and 30 seconds for freezing -- SB
        if ConfigSetup=="alpha_t/TotalDist"
            % -- getting the values for the distance
            tau=15; % in seconds, from classifypriorknowledge
            data=extract_dist_data(tau, ids_train, ...
                dtTrack, dtCommand); % distance
            [pdstr, xdstr]=calc_pdf(data);
        elseif ConfigSetup=="alpha_t/FreezeTime"
            % -- getting the values for the freeze time
            tau=30; % in seconds, from classifypriorknowledge
            data=extract_freezing_data(tau, ids_train, ...
                dtTrack, dtCommand); % distance
            [pdstr, xdstr]=calc_pdf(data);
        else
            tau=-1; % to prevent it from running elsewhere
            pdstr=[];
            xdstr=[];
        end
        
        
        % -- NOTE: For simplicity to not continuously comment/uncomment the
        % -- main script for the Sim Exp, save the p_... and x_... under the
        % -- same variable. However, understand that the variable definitions
        % -- for each of the calc_pdf functions have different meanings
        %     pdstr = p_frz;
        %     xdstr = x_frz;
        
        %path = convertStringsToChars(strcat(folder, maps(env), ".jpg")); % -- strcat the folder and image name
        img = imread('maps/OmronLab.jpg'); % -- read the image into Matlab
        img = imbinarize(img); % -- binarize the image
        
        % -- create a folder that correspond to the environment
        test_folder = strcat(dataFolder, ConfigSetup, "/",sprintf('%05d',test),"/");
        mkdir(test_folder);
        
        for participant = 1:size(ids_test, 1)
            for exp_cond = 1:size(conds, 2)
                
                % -- initialize the parameters of the simulation
                [param, ~, ~, ~, ~, target_locations, ~, ~] = ...
                    ParamConfig(num2str(ids_test(participant, 1)), conds(exp_cond));
                
                % reset tau here
                param.tau=tau;
                
                % -- display information about the simulation
                fprintf('Config:%s, Test:%05d \nParticipant:%s, condition:%d\n', ...
                    ConfigSetup, test, num2str(ids_test(participant,1)), conds(exp_cond));
                
                % -- create a new directory to hold all information about the simulation
                saveIn = strcat(test_folder, num2str(ids_test(participant,1)), ...
                    "/condition_", num2str(conds(exp_cond)), "/");
                saveFrames = strcat(saveIn, "frames/");
                directory = mkdir(saveIn);
                frames = mkdir(saveFrames);
                
                % -- call the mutual inforation pf simulation matlab script
                infoseek2Db_multi_robot(param, img, saveIn, "OmronLab", ...
                    saveFrames, target_locations,...
                    num2str(ids_test(participant,1)), exp_cond,...
                    pdstr, xdstr, ConfigSetup);
            end
        end
            
    end
    toc
end

end

