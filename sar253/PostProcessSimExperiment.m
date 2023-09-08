function PostProcessSimExperiment()

% -- This function will serve as a mediator of other functions to
% -- display data saved from the simulation infoseek2Db_multi_robot.m which 
% -- imports experimental data captured using the overhead tracking system 
% -- from the NIU Omron Lab.
% -- Written by: Rafal Krzysiak

% -- clear verything prior to running this script
clear all variables

% -- import the data saved
%SimExp = ImportAllData();

% -- plot which robot found the missing target 
%PlotWhoFoundTarget(SimExp);
% PlotWhoFoundTarget_testSet();

% -- plot all trajectories 
%PlotAllTraj(SimExp);
% PlotTrajTestDataSet();
% PlotRWControls();
ControlInputMapped();

% -- plot all the timeseries alpha values
% PlotTransientAlpha(SimExp);
% GetAverageVelocityExp();
end

function SimExp = ImportAllData()

% -- Define the mat file name and the conditions wanting to observe
% -- these values will be stored as strings for ease
Conditions = ["/condition_1", "/condition_4"];
MatFile = "/OmronLab_p=1200_nsim=1_agents=3.mat";

% -- get the list of participants
loc = "data/OmronLab/3robots/";
ids = ls(loc); % -- get all in the folder
ids = ids(3:end-1,1:4); % -- reformat to only have the numbers

% -- begin looping through all participants and conditions ran
for i = 1:size(ids, 1)
    for j = 1:size(Conditions, 2)
        % -- load and save the data in a single structure
        SimExp(i, j) = load(loc + ids(i,:) + Conditions(j) + MatFile);
    end
end

end

function PlotTrajTestDataSet()

% -- define the conditions tested
conditions = ["condition_1",...
              "condition_2",...
              "condition_3",...
              "condition_4"];

% -- define the mat file name used for all participants
Matfile = "OmronLab_p=1200_nsim=1_agents=3.mat";

% -- pull list of test folders within test folder
parentDir = "data/test";
parentDir_RW = "data/RandomWalk";
files = dir(parentDir);
RW_files = dir(parentDir_RW);

i = 1;

figure(1); clf; gcf;

% -- define the domain size
L= [18 9];

% -- begin looping through each test folder
% for test = 4:4%1:size(files, 1)
% 
%     % -- make sure that we capture a number not '.' or '..'
%     if (files(test).name ~= "." && files(test).name ~= "..")
% 
%         % -- within the test number folder, get the participant numbers
%         subDir = strcat(parentDir, "/", files(test).name);
%         participant_folders = dir(subDir);
% 
%         % -- begin looping through all the participant folders
%         for participant = 1:size(participant_folders, 1)
%             figure(participant); gcf;
%             % -- make sure that we capture a number not '.' or '..'
%             if (participant_folders(participant).name ~= "." && participant_folders(participant).name ~= "..")
% 
%                 % -- create participant directory
%                 partDir = strcat(subDir,"/",participant_folders(participant).name);
% 
%                 % -- begin looping through all conditions tested
%                 for cond = 1:size(conditions, 2)
% 
%                     % -- create condition directory
%                     condDir = strcat(partDir, "/", conditions(cond));
%                     condFile = strcat(condDir, "/", Matfile);
% 
%                     % -- read the data captured for the test
%                     TestData = load(condFile);
% 
%                     % -- get the end time
%                     tf = TestData.simdata.time;
% 
%                     % -- plot everything nicely
%                     subplot(2, 4, cond);
%                     % -- display the Omron Lab map
%                     hold on; imagesc([0 L(1)],[0 L(2)], TestData.img); 
%                     set(gca,'ydir','reverse');
%                     axis image;
% 
%                     % -- plot the target location
%                     hold on; plot(TestData.simdata.tloc(1, 1), ...
%                                   TestData.simdata.tloc(1, 2), ...
%                                   'mx', 'LineWidth', 3, 'MarkerSize', 12); % -- Target location 
% 
%                     % -- plot the beginning and end points of the robots
%                     hold on; plot(TestData.simdata.Xs(1, 1, 1, 1), ...
%                                   TestData.simdata.Xs(2, 1, 1, 1), ...
%                                   'ks', 'LineWidth', 3, 'MarkerSize', 12); % -- experiment robot (human controlled robot start)
%                     hold on; plot(TestData.simdata.Xs(1, 1, 1, 2), ...
%                                   TestData.simdata.Xs(2, 1, 1, 2), ...
%                                   'rs', 'LineWidth', 3, 'MarkerSize', 12); % -- autonomous robot 1 start
%                     hold on; plot(TestData.simdata.Xs(1, 1, 1, 3), ...
%                                   TestData.simdata.Xs(2, 1, 1, 3), ...
%                                   'gs', 'LineWidth', 3, 'MarkerSize', 12); % -- autonomous robot 2 start
% 
%                     hold on; plot(TestData.simdata.Xs(1, tf, 1, 1), ...
%                                   TestData.simdata.Xs(2, tf, 1, 1), ...
%                                   'kx', 'LineWidth', 3, 'MarkerSize', 12); % -- experiment robot (human controlled robot end)
%                     hold on; plot(TestData.simdata.Xs(1, tf, 1, 2), ...
%                                   TestData.simdata.Xs(2, tf, 1, 2), ...
%                                   'rx', 'LineWidth', 3, 'MarkerSize', 12); % -- autonomous robot 1 end
%                     hold on; plot(TestData.simdata.Xs(1, tf, 1, 3), ...
%                                   TestData.simdata.Xs(2, tf, 1, 3), ...
%                                   'gx', 'LineWidth', 3, 'MarkerSize', 12); % -- autonomous robot 2 end
% 
%                     % -- On top of the domain map, plot all trajectories of the robots
%                     hold on; plot(TestData.simdata.Xs(1, 1:tf, 1, 1), ...
%                                   TestData.simdata.Xs(2, 1:tf, 1, 1), ...
%                                   'k-', 'LineWidth', 2); % -- experiment robot (human controlled robot trajectory)
%                     hold on; plot(TestData.simdata.Xs(1, 1:tf, 1, 2), ...
%                                   TestData.simdata.Xs(2, 1:tf, 1, 2), ...
%                                   'r-', 'LineWidth', 2); % -- autonomous robot 1 trajectory
%                     hold on; plot(TestData.simdata.Xs(1, 1:tf, 1, 3), ...
%                                   TestData.simdata.Xs(2, 1:tf, 1, 3), ...
%                                   'g-', 'LineWidth', 2); % -- autonomous robot 2 trajectory
%                 end
%                  i = i + 1;
%             end
%         end
%     end
% end

i = 1; 

% -- begin looping through each test folder
for RWtest = 4:4%1:size(RW_files, 1)

    % -- make sure that we capture a number not '.' or '..'
    if (RW_files(RWtest).name ~= "." && RW_files(RWtest).name ~= ".." && RW_files(RWtest).name ~= ".DS_Store")
        
        % -- within the test number folder, get the participant numbers
        RWsubDir = strcat(parentDir_RW, "/", RW_files(RWtest).name);
        RWparticipant_folders = dir(RWsubDir);
        
        % -- begin looping through all the participant folders
        for RWparticipant = 1:size(RWparticipant_folders, 1)
            figure(RWparticipant); gcf;
            
            % -- make sure that we capture a number not '.' or '..'
            if (RWparticipant_folders(RWparticipant).name ~= "." && RWparticipant_folders(RWparticipant).name ~= ".." && RWparticipant_folders(RWparticipant).name ~= ".DS_Store")
            
                % -- create participant directory
                RWpartDir = strcat(RWsubDir,"/",RWparticipant_folders(RWparticipant).name);
    
                % -- begin looping through all conditions tested
                for cond = 1:size(conditions, 2)
    
                    % -- create condition directory
                    RWcondDir = strcat(RWpartDir, "/", conditions(cond));
                    RWcondFile = strcat(RWcondDir, "/", Matfile);
    
                    % -- read the data captured for the test
                    RWTestData = load(RWcondFile);
                    % -- get the end time
                    tf = RWTestData.simdata.time;
                    
                    subplot(2, 2, cond);
                    % -- display the Omron Lab map
                    
                    hold on; imagesc([0 L(1)],[0 L(2)], RWTestData.img); 
                    set(gca,'ydir','reverse');
                    axis image;
            
                    % -- plot the target location
                    hold on; plot(RWTestData.simdata.tloc(1, 1), ...
                                  RWTestData.simdata.tloc(1, 2), ...
                                  'mx', 'LineWidth', 3, 'MarkerSize', 12); % -- Target location 
                
                    % -- plot the beginning and end points of the robots
                    hold on; plot(RWTestData.simdata.Xs(1, 1, 1, 1), ...
                                  RWTestData.simdata.Xs(2, 1, 1, 1), ...
                                  'ks', 'LineWidth', 3, 'MarkerSize', 12); % -- experiment robot (human controlled robot start)
                    hold on; plot(RWTestData.simdata.Xs(1, 1, 1, 2), ...
                                  RWTestData.simdata.Xs(2, 1, 1, 2), ...
                                  'rs', 'LineWidth', 3, 'MarkerSize', 12); % -- autonomous robot 1 start
                    hold on; plot(RWTestData.simdata.Xs(1, 1, 1, 3), ...
                                  RWTestData.simdata.Xs(2, 1, 1, 3), ...
                                  'gs', 'LineWidth', 3, 'MarkerSize', 12); % -- autonomous robot 2 start
                
                    hold on; plot(RWTestData.simdata.Xs(1, tf, 1, 1), ...
                                  RWTestData.simdata.Xs(2, tf, 1, 1), ...
                                  'kx', 'LineWidth', 3, 'MarkerSize', 12); % -- experiment robot (human controlled robot end)
                    hold on; plot(RWTestData.simdata.Xs(1, tf, 1, 2), ...
                                  RWTestData.simdata.Xs(2, tf, 1, 2), ...
                                  'rx', 'LineWidth', 3, 'MarkerSize', 12); % -- autonomous robot 1 end
                    hold on; plot(RWTestData.simdata.Xs(1, tf, 1, 3), ...
                                  RWTestData.simdata.Xs(2, tf, 1, 3), ...
                                  'gx', 'LineWidth', 3, 'MarkerSize', 12); % -- autonomous robot 2 end
                
                    % -- On top of the domain map, plot all trajectories of the robots
                    hold on; plot(RWTestData.simdata.Xs(1, 1:tf, 1, 1), ...
                                  RWTestData.simdata.Xs(2, 1:tf, 1, 1), ...
                                  'k-', 'LineWidth', 2); % -- experiment robot (human controlled robot trajectory)
                    hold on; plot(RWTestData.simdata.Xs(1, 1:tf, 1, 2), ...
                                  RWTestData.simdata.Xs(2, 1:tf, 1, 2), ...
                                  'r-', 'LineWidth', 2); % -- autonomous robot 1 trajectory
                    hold on; plot(RWTestData.simdata.Xs(1, 1:tf, 1, 3), ...
                                  RWTestData.simdata.Xs(2, 1:tf, 1, 3), ...
                                  'g-', 'LineWidth', 2); % -- autonomous robot 2 trajectory
                    title(sprintf("Random Walk scenario\n Condition: %d", cond))
    
                end
                 i = i + 1;
            end
        end
    end
end
end

function PlotRWControls()

% -- define the conditions tested
conditions = ["condition_1",...
              "condition_2",...
              "condition_3",...
              "condition_4"];

% -- define the mat file name used for all participants
Matfile = "OmronLab_p=1200_nsim=1_agents=3.mat";

% -- pull list of test folders within test folder
parentDir = "data/test";
parentDir_RW = "data/RandomWalk";
files = dir(parentDir);
RW_files = dir(parentDir_RW);

figure(1); clf; gcf;

% -- define the domain size
L= [18 9];

i = 1; 

% -- begin looping through each test folder
for RWtest = 4:4%1:size(RW_files, 1)

    % -- make sure that we capture a number not '.' or '..'
    if (RW_files(RWtest).name ~= "." && RW_files(RWtest).name ~= ".." && RW_files(RWtest).name ~= ".DS_Store")
        
        % -- within the test number folder, get the participant numbers
        RWsubDir = strcat(parentDir_RW, "/", RW_files(RWtest).name);
        RWparticipant_folders = dir(RWsubDir);
        
        % -- begin looping through all the participant folders
        for RWparticipant = 1:size(RWparticipant_folders, 1)
            figure(RWparticipant); gcf;
            
            % -- make sure that we capture a number not '.' or '..'
            if (RWparticipant_folders(RWparticipant).name ~= "." && RWparticipant_folders(RWparticipant).name ~= ".." && RWparticipant_folders(RWparticipant).name ~= ".DS_Store")
            
                % -- create participant directory
                RWpartDir = strcat(RWsubDir,"/",RWparticipant_folders(RWparticipant).name);
    
                % -- begin looping through all conditions tested
                for cond = 1:size(conditions, 2)
    
                    % -- create condition directory
                    RWcondDir = strcat(RWpartDir, "/", conditions(cond));
                    RWcondFile = strcat(RWcondDir, "/", Matfile);
    
                    % -- read the data captured for the test
                    RWTestData = load(RWcondFile);
                    % -- get the end time
                    tf = RWTestData.simdata.time;

                    % -- plot everything nicely
                    subplot(2, 2, cond);
                    % -- plot the turn rate and velocity data
                    plot(1:tf, RWTestData.simdata.omega(1,1:tf,1,2), 'r-', 'LineWidth', 2);
                    hold on; plot(1:tf, RWTestData.simdata.omega(1,1:tf,1,3), 'g-', 'LineWidth', 2);
                    hold on; plot(1:tf, RWTestData.simdata.vel(1,1:tf,1,2), 'm-', 'LineWidth', 2);
                    hold on; plot(1:tf, RWTestData.simdata.vel(1,1:tf,1,3), 'b-', 'LineWidth', 2);

                    legend('Auto1 omega','Auto2 omega','Auto1 vel','Auto1 vel');

                    title(sprintf("Random Walk scenario\n Condition: %d, Controls", cond))
                    
    
                end
                 i = i + 1;
            end
        end
    end
end
end

function PlotWhoFoundTarget(SimExp)

% -- get the size of the struct where all data is store for ease of use
[Np, Nc] = size(SimExp);

% -- create figure
figure(1); clf; gcf;

% -- loop through number of conditions
for cond = 1:Nc
    % -- create subplots for each condition
    subplot(1,Nc, cond);

    % -- collect the which_robot data
    for i = 1:Np
        WhichRobot(i) = SimExp(i, cond).simdata.Which_robot;
    end
    
    % -- store the data in a temperary and clean variables
    X = [1, 2, 3];
    Y = [sum(WhichRobot == 0), sum(WhichRobot == 2), sum(WhichRobot == 3)];

    % -- only for condition 4 (just for labeling purposes of title)
    if cond == 2
        cond = 4;
    end

    % -- plot bar graph and make it look nice
    bar(X, Y);
    text(1:length(X),Y',num2str(Y'),'vert','bottom','horiz','center','FontSize',16);
    xlabel("Robot");
    ylabel("Number of times target was found by robot");
    title(sprintf("Condition: %d", cond));

    % -- set figure parameters
    ax = gca; 
    ax.FontSize = 16;
end

end

function PlotWhoFoundTarget_testSet()

% -- define the conditions tested
conditions = ["condition_1",...
              "condition_2",...
              "condition_3",...
              "condition_4"];

% -- define the mat file name used for all participants
Matfile = "OmronLab_p=1200_nsim=1_agents=3.mat";

% -- pull list of test folders within test folder
parentDir = "data/test";
parentDir_RW = "data/RandomWalk";
files = dir(parentDir);
RW_files = dir(parentDir_RW);

% -- initialize the counter
i = 1; 

% -- begin looping through each test folder
for test = 1:size(files, 1)

    % -- make sure that we capture a number not '.' or '..'
    if (files(test).name ~= "." && files(test).name ~= "..")
        
        % -- within the test number folder, get the participant numbers
        subDir = strcat(parentDir, "/", files(test).name);
        participant_folders = dir(subDir);
        
        % -- begin looping through all the participant folders
        for participant = 1:size(participant_folders, 1)
            % -- make sure that we capture a number not '.' or '..'
            if (participant_folders(participant).name ~= "." && participant_folders(participant).name ~= "..")
            
                % -- create participant directory
                partDir = strcat(subDir,"/",participant_folders(participant).name);
    
                % -- begin looping through all conditions tested
                for cond = 1:size(conditions, 2)
    
                    % -- create condition directory
                    condDir = strcat(partDir, "/", conditions(cond));
                    condFile = strcat(condDir, "/", Matfile);
    
                    % -- read the data captured for the test
                    TestData = load(condFile);
                    WhichRobot(cond, i) = TestData.simdata.Which_robot;
    
                end
                i = i + 1;
            end
        end
    end
end

i = 1;
% -- begin looping through each test folder
for RWtest = 1:size(RW_files, 1)

    % -- make sure that we capture a number not '.' or '..'
    if (RW_files(RWtest).name ~= "." && RW_files(RWtest).name ~= ".." && RW_files(RWtest).name ~= ".DS_Store")
        
        % -- within the test number folder, get the participant numbers
        RWsubDir = strcat(parentDir_RW, "/", RW_files(RWtest).name);
        RWparticipant_folders = dir(RWsubDir);
        
        % -- begin looping through all the participant folders
        for RWparticipant = 1:size(RWparticipant_folders, 1)
            % -- make sure that we capture a number not '.' or '..'
            if (RWparticipant_folders(RWparticipant).name ~= "." && RWparticipant_folders(RWparticipant).name ~= ".." && RWparticipant_folders(RWparticipant).name ~= ".DS_Store")
            
                % -- create participant directory
                RWpartDir = strcat(RWsubDir,"/",RWparticipant_folders(RWparticipant).name);
    
                % -- begin looping through all conditions tested
                for cond = 1:size(conditions, 2)
    
                    % -- create condition directory
                    RWcondDir = strcat(RWpartDir, "/", conditions(cond));
                    RWcondFile = strcat(RWcondDir, "/", Matfile);
    
                    % -- read the data captured for the test
                    RWTestData = load(RWcondFile);
                    RWWhichRobot(cond, i) = RWTestData.simdata.Which_robot;
    
                end
                i = i + 1;
            end
        end
    end
end


% -- store the data in a temperary and clean variables
X = [1, 2];
Y1 = [sum(WhichRobot(1,:) == 0), sum(WhichRobot(1,:) == 2) + sum(WhichRobot(1,:) == 3)];
Y2 = [sum(WhichRobot(2,:) == 0), sum(WhichRobot(2,:) == 2) + sum(WhichRobot(2,:) == 3)];
Y3 = [sum(WhichRobot(3,:) == 0), sum(WhichRobot(3,:) == 2) + sum(WhichRobot(3,:) == 3)];
Y4 = [sum(WhichRobot(4,:) == 0), sum(WhichRobot(4,:) == 2) + sum(WhichRobot(4,:) == 3)];

RWY1 = [sum(RWWhichRobot(1,:) == 0), sum(RWWhichRobot(1,:) == 2) + sum(RWWhichRobot(1,:) == 3)];
RWY2 = [sum(RWWhichRobot(2,:) == 0), sum(RWWhichRobot(2,:) == 2) + sum(RWWhichRobot(2,:) == 3)];
RWY3 = [sum(RWWhichRobot(3,:) == 0), sum(RWWhichRobot(3,:) == 2) + sum(RWWhichRobot(3,:) == 3)];
RWY4 = [sum(RWWhichRobot(4,:) == 0), sum(RWWhichRobot(4,:) == 2) + sum(RWWhichRobot(4,:) == 3)];

% -- create figure
figure(1); clf; gcf;

% -- plot bar graph and make it look nice
subplot(2,2,1); bar(X, Y1);
text(1:length(X),Y1',num2str(Y1'),'vert','bottom','horiz','center','FontSize',16);
xlabel("Robot");
ylabel({"Number of times target";"was found by robot"});
title("Condition: 1");

% -- set figure parameters
ax = gca; 
ax.FontSize = 16;
xticklabels({'Ref robot','auto robot'});

subplot(2,2,2); bar(X, Y2);
text(1:length(X),Y2',num2str(Y2'),'vert','bottom','horiz','center','FontSize',16);
xlabel("Robot");
ylabel({"Number of times target";"was found by robot"});
title("Condition: 2");

% -- set figure parameters
ax = gca; 
ax.FontSize = 16;
xticklabels({'Ref robot','auto robot'});


subplot(2,2,3); bar(X, Y3);
text(1:length(X),Y3',num2str(Y3'),'vert','bottom','horiz','center','FontSize',16);
xlabel("Robot");
ylabel({"Number of times target";"was found by robot"});
title("Condition: 3");

% -- set figure parameters
ax = gca; 
ax.FontSize = 16;
xticklabels({'Ref robot','auto robot'});

subplot(2,2,4); bar(X, Y4);
text(1:length(X),Y4',num2str(Y4'),'vert','bottom','horiz','center','FontSize',16);
xlabel("Robot");
ylabel({"Number of times target";"was found by robot"});
title("Condition: 4");

% -- set figure parameters
ax = gca; 
ax.FontSize = 16;
xticklabels({'Ref robot','auto robot'});

% -- create figure
figure(2); clf; gcf;

% -- plot bar graph and make it look nice
subplot(2,2,1); bar(X, RWY1);
text(1:length(X),RWY1',num2str(RWY1'),'vert','bottom','horiz','center','FontSize',16);
xlabel("Robot");
ylabel({"Number of times target";"was found by robot"});
title("Condition: 1");

% -- set figure parameters
ax = gca; 
ax.FontSize = 16;
xticklabels({'Ref robot','auto robot'});

subplot(2,2,2); bar(X, RWY2);
text(1:length(X),RWY2',num2str(RWY2'),'vert','bottom','horiz','center','FontSize',16);
xlabel("Robot");
ylabel({"Number of times target";"was found by robot"});
title("Condition: 2");

% -- set figure parameters
ax = gca; 
ax.FontSize = 16;
xticklabels({'Ref robot','auto robot'});


subplot(2,2,3); bar(X, RWY3);
text(1:length(X),RWY3',num2str(RWY3'),'vert','bottom','horiz','center','FontSize',16);
xlabel("Robot");
ylabel({"Number of times target";"was found by robot"});
title("Condition: 3");

% -- set figure parameters
ax = gca; 
ax.FontSize = 16;
xticklabels({'Ref robot','auto robot'});

subplot(2,2,4); bar(X, RWY4);
text(1:length(X),RWY4',num2str(RWY4'),'vert','bottom','horiz','center','FontSize',16);
xlabel("Robot");
ylabel({"Number of times target";"was found by robot"});
title("Condition: 4");

% -- set figure parameters
ax = gca; 
ax.FontSize = 16;
xticklabels({'Ref robot','auto robot'});

SingleFigBarPlot(Y1, RWY1, Y2, RWY2, Y3, RWY3, Y4, RWY4);

end

function SingleFigBarPlot(Y1, RWY1, Y2, RWY2, Y3, RWY3, Y4, RWY4)

X = [1, 2, 3, 4];

% -- create figure
figure(3); clf; gcf;
subplot(2,2,1); bar(X, [Y1,RWY1]);
text(1:length(X),[Y1,RWY1]',num2str([Y1,RWY1]'),'vert','bottom','horiz','center','FontSize',16);
ylabel({"Number of times target";"was found by robot"});
title("Condition: 1");

% -- set figure parameters
ax = gca; 
ax.FontSize = 16;
xticklabels({'Ref robot: adaptive a','Auto robot: adaptive a',...
             'Ref robot: RW', 'Auto robot: RW'});

subplot(2,2,2); bar(X, [Y2,RWY2]);
text(1:length(X),[Y2,RWY2]',num2str([Y2,RWY2]'),'vert','bottom','horiz','center','FontSize',16);
ylabel({"Number of times target";"was found by robot"});
title("Condition: 2");

% -- set figure parameters
ax = gca; 
ax.FontSize = 16;
xticklabels({'Ref robot: adaptive a','Auto robot: adaptive a',...
             'Ref robot: RW', 'Auto robot: RW'});

subplot(2,2,3); bar(X, [Y3,RWY3]);
text(1:length(X),[Y3,RWY3]',num2str([Y3,RWY3]'),'vert','bottom','horiz','center','FontSize',16);
ylabel({"Number of times target";"was found by robot"});
title("Condition: 3");

% -- set figure parameters
ax = gca; 
ax.FontSize = 16;
xticklabels({'Ref robot: adaptive a','Auto robot: adaptive a',...
             'Ref robot: HRW', 'Auto robot: RW'});

subplot(2,2,4); bar(X, [Y4,RWY4]);
text(1:length(X),[Y4,RWY4]',num2str([Y4,RWY4]'),'vert','bottom','horiz','center','FontSize',16);
ylabel({"Number of times target";"was found by robot"});
title("Condition: 4");

% -- set figure parameters
ax = gca; 
ax.FontSize = 16;
xticklabels({'Ref robot: adaptive a','Auto robot: adaptive a',...
             'Ref robot: RW', 'Auto robot: RW'});

end

function PlotAllTraj(SimExp)

% -- get the list of participants
loc = "data/OmronLab/3robots/";
ids = ls(loc); % -- get all in the folder
ids = ids(3:end-1,1:4); % -- reformat to only have the numbers

% -- get the size of the struct where all data is store for ease of use
[Np, Nc] = size(SimExp);

% -- define the domain size
L= [18 9];

% -- loop through all the simulations and plot the trajectories
for i = 1:Np
    % -- create individual participant figures
    figure(9+i); clf; gcf;

    for j = 1:Nc

        % -- get the end time
        tf = SimExp(i,j).simdata.time;
    
        % -- create subplots for each condition for each participant
        subplot(1,Nc,j);
    
        % -- display the Omron Lab map
        hold on; imagesc([0 L(1)],[0 L(2)], SimExp(i).img); 
        set(gca,'ydir','reverse');
        axis image;

        % -- plot the target location
        hold on; plot(SimExp(i,j).simdata.tloc(1, 1), ...
                      SimExp(i,j).simdata.tloc(1, 2), ...
                      'mx', 'LineWidth', 3, 'MarkerSize', 12); % -- Target location 
    
        % -- plot the beginning and end points of the robots
        hold on; plot(SimExp(i,j).simdata.Xs(1, 1, 1, 1), ...
                      SimExp(i,j).simdata.Xs(2, 1, 1, 1), ...
                      'ks', 'LineWidth', 3, 'MarkerSize', 12); % -- experiment robot (human controlled robot start)
        hold on; plot(SimExp(i,j).simdata.Xs(1, 1, 1, 2), ...
                      SimExp(i,j).simdata.Xs(2, 1, 1, 2), ...
                      'rs', 'LineWidth', 3, 'MarkerSize', 12); % -- autonomous robot 1 start
        hold on; plot(SimExp(i,j).simdata.Xs(1, 1, 1, 3), ...
                      SimExp(i,j).simdata.Xs(2, 1, 1, 3), ...
                      'gs', 'LineWidth', 3, 'MarkerSize', 12); % -- autonomous robot 2 start
    
        hold on; plot(SimExp(i,j).simdata.Xs(1, tf, 1, 1), ...
                      SimExp(i,j).simdata.Xs(2, tf, 1, 1), ...
                      'kx', 'LineWidth', 3, 'MarkerSize', 12); % -- experiment robot (human controlled robot end)
        hold on; plot(SimExp(i,j).simdata.Xs(1, tf, 1, 2), ...
                      SimExp(i,j).simdata.Xs(2, tf, 1, 2), ...
                      'rx', 'LineWidth', 3, 'MarkerSize', 12); % -- autonomous robot 1 end
        hold on; plot(SimExp(i,j).simdata.Xs(1, tf, 1, 3), ...
                      SimExp(i,j).simdata.Xs(2, tf, 1, 3), ...
                      'gx', 'LineWidth', 3, 'MarkerSize', 12); % -- autonomous robot 2 end
    
        % -- On top of the domain map, plot all trajectories of the robots
        hold on; plot(SimExp(i,j).simdata.Xs(1, 1:tf, 1, 1), ...
                      SimExp(i,j).simdata.Xs(2, 1:tf, 1, 1), ...
                      'k-', 'LineWidth', 2); % -- experiment robot (human controlled robot trajectory)
        hold on; plot(SimExp(i,j).simdata.Xs(1, 1:tf, 1, 2), ...
                      SimExp(i,j).simdata.Xs(2, 1:tf, 1, 2), ...
                      'r-', 'LineWidth', 2); % -- autonomous robot 1 trajectory
        hold on; plot(SimExp(i,j).simdata.Xs(1, 1:tf, 1, 3), ...
                      SimExp(i,j).simdata.Xs(2, 1:tf, 1, 3), ...
                      'g-', 'LineWidth', 2); % -- autonomous robot 2 trajectory
    
        % -- make the figure look nice
        xlabel("X (m)");
        ylabel("Y (m)");
        if j == 2
            j = 4;
        end
        title(sprintf("Participant ID: %s, Condition: %d", ids(i, :), j));
        legend('Target','','','','','','','Human robot', 'Autonomous robot 1', 'Autonomous robot 2');
        ax = gca; 
        ax.FontSize = 12;
    end

end

end

function PlotTransientAlpha(SimExp)

% -- get the list of participants
loc = "data/OmronLab/3robots/";
ids = ls(loc); % -- get all in the folder
ids = ids(3:end-1,1:4); % -- reformat to only have the numbers

% -- get the size of the struct where all data is store for ease of use
[Np, Nc] = size(SimExp);

% -- loop through all the simulations and plot the trajectories
for i = 1:Np
    % -- create individual participant figures
    figure(29+i); clf; gcf;

    for j = 1:Nc
    
        % -- create subplots for each condition for each participant
        subplot(2,2,j);
        
        % -- plot the transient alpha values
        hold on; plot(SimExp(i,j).simdata.alpha(:,1), ...
                      'k-', 'LineWidth', 2); % -- transient alpha value
    
        % -- make the figure look nice
        xlabel("Timestep");
        ylabel("Alpha");
        title(sprintf("Participant ID: %s, Condition: %d", ids(i, :), j));
        ylim([0 1]);
        axis square;
        ax = gca; 
        ax.FontSize = 12;
    end
end

end

function ControlInputMapped()

% -- define the conditions tested
conditions = ["condition_1",...
              "condition_2",...
              "condition_3",...
              "condition_4"];

% -- define the mat file name used for all participants
Matfile = "OmronLab_p=1200_nsim=1_agents=3.mat";

% -- pull list of test folders within test folder
parentDir_RW = "data/RandomWalk";
RW_files = dir(parentDir_RW);

% -- define the domain size
L= [18 9];


% -- begin looping through each test folder
for RWtest = 1:size(RW_files, 1)

    % -- make sure that we capture a number not '.' or '..'
    if (RW_files(RWtest).name ~= "." && RW_files(RWtest).name ~= "..")
        
        % -- within the test number folder, get the participant numbers
        RWsubDir = strcat(parentDir_RW, "/", RW_files(RWtest).name);
        RWparticipant_folders = dir(RWsubDir);
        
        % -- begin looping through all the participant folders
        for RWparticipant = 1:size(RWparticipant_folders, 1)
            % -- make sure that we capture a number not '.' or '..'
            if (RWparticipant_folders(RWparticipant).name ~= "." && RWparticipant_folders(RWparticipant).name ~= "..")
            
                % -- create participant directory
                RWpartDir = strcat(RWsubDir,"/",RWparticipant_folders(RWparticipant).name);
    
                % -- begin looping through all conditions tested
                for cond = 1:size(conditions, 2)
    
                    % -- create condition directory
                    RWcondDir = strcat(RWpartDir, "/", conditions(cond));
                    RWcondFile = strcat(RWcondDir, "/", Matfile);
    
                    % -- read the data captured for the test
                    RWTestData = load(RWcondFile);
                    
                    % -- create the figure
                    figure(1); clf; gcf;
                    

                    % -- loop through the simulation
                    for k = 1:RWTestData.simdata.time
                        % -- display the Omron Lab map
                        subplot(2,2,[1,2]);
                        hold on; imagesc([0 L(1)],[0 L(2)], RWTestData.img); 
                        set(gca,'ydir','reverse');
                        axis image;

                        % -- plot the position of the robots in the domain
                        hold on; plot(RWTestData.simdata.Xs(1,k,1,1), RWTestData.simdata.Xs(2,k,1,1), ...
                                      'k.', MarkerSize=36); % -- ref robot position
                        hold on; plot(RWTestData.simdata.Xs(1,k,1,2), RWTestData.simdata.Xs(2,k,1,2), ...
                                      'r.', MarkerSize=36); % -- Auto 1 robot position
                        hold on; plot(RWTestData.simdata.Xs(1,k,1,3), RWTestData.simdata.Xs(2,k,1,3), ...
                                      'g.', MarkerSize=36); % -- Auto 2 robot position

                        % -- In the next subplot, plot the velocities of the robots over time
                        subplot(2,2,3);
                        hold on; plot(RWTestData.simdata.vel(1,1:k,1,1), ...
                                      'k-', LineWidth=2); % -- ref robot velocity
                        hold on; plot(RWTestData.simdata.vel(1,1:k,1,2), ...
                                      'r-', LineWidth=2); % -- Auto 1 robot velocity
                        hold on; plot(RWTestData.simdata.vel(1,1:k,1,3), ...
                                      'g-', LineWidth=2); % -- Auto 2 robot velocity
                        xlabel('Time step');
                        ylabel('Velocity (m/s)');

                        % -- In the next subplot, plot the turn rates of the robots over time
                        subplot(2,2,4);
                        hold on; plot(RWTestData.simdata.omega(1,1:k,1,1), ...
                                      'k-', LineWidth=2); % -- ref robot turn rate
                        hold on; plot(RWTestData.simdata.omega(1,1:k,1,2), ...
                                      'r-', LineWidth=2); % -- Auto 1 robot turn rate
                        hold on; plot(RWTestData.simdata.omega(1,1:k,1,3), ...
                                      'g-', LineWidth=2); % -- Auto 2 robot turn rate

                        xlabel('Time step');
                        ylabel('Turn rate (rad/s)');
                        pause(0.1);
                    end
                
                end
                i = i + 1;
            end
        end
    end
end
end

function GetAverageVelocityExp()

% -- Define folder locations where data is kept
loc = "data/FILTERED/";

% -- get the ID list
ID_Data=csvread('../src/IDList_Completed.csv',1,0);

% -- define the Velocity file name for each condition
condfiles = ["/EKFVel_condition_1.csv",...
             "/EKFVel_condition_2.csv",...
             "/EKFVel_condition_3.csv",...
             "/EKFVel_condition_4.csv"];

% -- create 1D array to contain all velocity data for all participants
% -- and conditions ran during the experiment
Velocity = zeros(1, size(condfiles, 2)*size(ID_Data,1));

i = 1; % -- counter

% -- loop through all participants
for participant = 1:size(ID_Data,1)
    % -- loop through each condition
    for cond = 1:size(condfiles, 2)
        % -- create str that points to the csv file where data is stored
        VelFile = strcat(loc, num2str(ID_Data(participant,1)), condfiles(cond));

        % -- pull the data
        VelData = csvread(VelFile);

        % -- collect the averaged out veloctiy data
        Velocity(i) = mean(VelData);

        i = i + 1;
    end
end

% -- get the total average velocity data
TotalMeanVel = mean(Velocity);

% -- plot the average teleoperated velocity distributions
%figure(1); clf; gcf;

%scatter(Velocity);


end


