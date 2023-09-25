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

% -- initialize the parameters of the simulation
% [param, maps, folder, bias, share_info, target_locations, agents, file_id] = ...
%     ParamConfig();

conds = [1, 2, 3, 4];

tic % -- start timer to determine computational time
test_size = 200;

for test = 1:test_size % -- looping through every environment

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
    tau=20;
    [pdstr, xdstr]=calc_pdf_feature(tau, ids_train);

    %path = convertStringsToChars(strcat(folder, maps(env), ".jpg")); % -- strcat the folder and image name
    img = imread("maps/OmronLab.jpg"); % -- read the image into Matlab
    img = imbinarize(img); % -- binarize the image
    
    % -- create a folder that correspond to the environment
    test_folder = strcat("data/test/",sprintf('%05d',test),"/");
    mkdir(test_folder);

    for participant = 1:size(ids_test, 1)
        for exp_cond = 1:size(conds, 2)
            
            % -- we will set a fifth condition, which will statnd for randomwalk (openloop) 
            %if exp_cond == 5
                %param.closedloop = 0;
            %end

            % -- initialize the parameters of the simulation
            [param, maps, ~, ~, ~, target_locations, ~, ~] = ...
                ParamConfig(num2str(ids_test(participant, 1)), conds(exp_cond));

            % -- display information about the simulation
            fprintf('Test:%05d \nParticipant:%s, condition:%d\n', ...
                     test, num2str(ids_test(participant,1)), conds(exp_cond));

            % -- create a new directory to hold all information about the simulation
            saveIn = strcat(test_folder, num2str(ids_test(participant,1)), "/condition_", num2str(conds(exp_cond)), "/");
            saveFrames = strcat(saveIn, "frames/");
            directory = mkdir(saveIn);
            frames = mkdir(saveFrames);

            % -- call the mutual inforation pf simulation matlab script
            infoseek2Db_multi_robot(param, img, saveIn, "OmronLab", ...
                                    saveFrames, target_locations,...
                                    num2str(ids_test(participant,1)), exp_cond,...
                                    pdstr, xdstr);
        end
    end

end
toc
end

