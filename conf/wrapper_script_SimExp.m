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

dataFolder="data/"; % this can sit outside the github
% https://www.dropbox.com/scl/fo/5shjyymaunngx1qepwhl3/h?rlkey=02dsr4ej3gxaswcc7fa5h6mji&dl=0

% -- Defining a variable to be a flag for us to easily switch
% -- between conditions wanting to test under 
% -- Options so far:
% -- "xMxT" -> No Map No Target
% -- "xMyT" -> No Map Yes Target
% -- "yMxT" -> Yes Map No Target
% -- "yMyT" -> Yes Map Yes Target
ConfigChoice={"xMxT", "xMyT",...
              "yMxT", "yMyT"};
conds = [1, 2, 3, 4];


nsims = 1; %number of simulations

for ii = 2:3%size(ConfigChoice,2)
    
    ConfigSetup = ConfigChoice{ii};
    
    % -- initialize the parameters of the simulation
    % [param, maps, folder, bias, share_info, target_locations, agents, file_id] = ...
    %     ParamConfig();
    
    cond = conds(ii);
    
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
    numSubjects=size(ID_Data,1);
    
    for subject = 1:numSubjects % -- looping through every environment
        % -- here we pull the ID of the participant we are testing with
        partID = ID_Data(subject,1);
        
        %path = convertStringsToChars(strcat(folder, maps(env), ".jpg")); % -- strcat the folder and image name
        img = imread('maps/OmronLab.jpg'); % -- read the image into Matlab
        img = imbinarize(img); % -- binarize the image
        
        % -- create a folder that correspond to the environment
        saveIn = strcat(dataFolder, ConfigSetup, "/", sprintf('%04d',partID),"/");
        mkdir(saveIn);
                
        % -- initialize the parameters of the simulation
        [param, ~, ~, ~, ~, target_locations, ~, ~] = ...
            ParamConfig(num2str(partID), cond);
        
        % -- set the map configuration to be tested
        param.config = ConfigSetup;
        param.ID = partID;
        
        % -- display information about the simulation
        fprintf('Config:%s, Participant:%04d, Config completion status:%0.1f%%\n', ...
            ConfigSetup, partID, (subject/numSubjects)*100.0);
        
        % -- create a new directory to hold all information about the simulation
        saveFrames = strcat(saveIn, "frames/");
        directory = mkdir(saveIn);
        frames = mkdir(saveFrames);
        
        % -- call the mutual inforation pf simulation matlab script
        infoseek2Db_multi_robot(param, img, saveIn, "OmronLab", ...
            saveFrames, target_locations,...
            num2str(partID), cond, ConfigSetup);
            
    end
    toc
end

end

