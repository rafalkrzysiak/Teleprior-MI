function post_sim_analysis()

% -- This function will serve as a mediator of other functions to
% -- display data saved from the simulation infoseek2Db_multi_robot.m which
% -- imports experimental data captured using the overhead tracking system
% -- from the NIU Omron Lab.
% -- Written by: Rafal Krzysiak

% -- clear verything prior to running this script
clear variables

% run import and the performance comparison

% -- import the data saved
% ImportAllData();


% -- plot which robot found the missing target
plot_performance_comparison('performance_data_agents=3.csv');
% plot_success_traj('performance_data_agents=3.csv');

% -- plot all trajectories
% *deprecated*
% PlotAllTraj(SimExp);
% PlotWhoFoundTarget(SimExp);
% PlotWhoFoundTarget_testSet();
% TimeGained();
% PlotRWControls();
% ControlInputMapped();
% PlotTrajTestDataSet();

% -- plot all the timeseries alpha values
% PlotTransientAlpha(SimExp);
% GetAverageVelocityExp();
end

function ImportAllData()

% -- Define the mat file name and the conditions wanting to observe
% -- these values will be stored as strings for ease
% -- define the conditions tested
conditions = {'condition_1','condition_2','condition_3','condition_4'};

% -- define the mat file name used for all participants
% change for agents and then on line 67 for location
Matfile = "OmronLab_p=1200_nsim=1_agents=2.mat";
[~, filename]=fileparts(Matfile);

suffix=split(filename,'_');
suffix=suffix{4};

% strategies
strategies={'alpha_t/TotalDist/', 'alpha_t/FreezeTime/', ...
    'alpha_t/DistAndFreeze/', 'alpha_0/', ...
    'alpha_1/',  'RandomWalk/'};


% -- get the list of participants
time2find = readtable('../data/TimeToFind.csv');

% expTime = expTime(:,1:4);
% no need to scale because we are simply comparing
% strategies
% expTime(:,1)=expTime(:,1)/1.135; % scaling to compensate for length and obstacles

% IDs = csvread('../src/IDList_Completed.csv', 1);
% IDs = IDs(:,1);
IDs=time2find.id;

% -- adjusting for humans entering code into computer
expTime = table2array(time2find(:,2:5));
expTime = expTime-5;

% simulated data
loc = "../../simdata-nr=2/";
fps=2;
csvdata=[];
% -- begin looping through all participants and conditions ran
for ss = 1:size(strategies,2)
    runs = dir(loc+strategies{ss});
    runs=runs(~ismember({runs.name},{'.','..', '.DS_Store'}));
    
    for rr=1:size(runs,1)
        part=dir(loc+strategies{ss}+runs(rr).name+'/');
        part=part(~ismember({part.name},{'.','..', '.DS_Store'}));
        for pp=1:size(part,1)
            pidx=find(str2double(part(pp).name)==IDs);
            for cc=1:size(conditions,2)
                matfilename=[loc+strategies{ss}+runs(rr).name+'/'+...
                    part(pp).name+'/'+conditions{cc}+'/'+Matfile];
                data = load(matfilename);
                csvdata=[csvdata; ...
                    ss, rr, str2double(part(pp).name), cc, ...
                    data.simdata.Which_robot, data.simdata.time/fps,...
                    data.simdata.success, expTime(pidx,cc), ...
                    time2find.c4flag(pidx), matfilename];
                
            end
        end
    end
end
T = array2table(csvdata);
T.Properties.VariableNames = {'strategy','run','pid', ...
    'condition', 'whichrobot', ...
    'timesec', 'success', 'expTime', 'c4flag', 'matfile'};
writetable(T,sprintf('performance_data_%s.csv', suffix));

%csvwrite('performance_data.csv', csvdata)
end

function PlotTrajTestDataSet()

% -- define the conditions tested
conditions = ["condition_1",...
    "condition_2",...
    "condition_3",...
    "condition_4"];

cond_names=["xMxT","xMyT","yMxT","yMyT"];
cond_desc=["No Map, No Target",...
    "No Map, Yes Target",...
    "Yes Map, No Target",...
    "Yes Map, Yes Target"];

% -- define the mat file name used for all participants
Matfile = "OmronLab_p=1200_nsim=1_agents=2.mat";

% -- data to plot
parentDir = "../../simdata-nr=2/alpha_t/TotalDist";

% ids to plot
ids_to_plot=25;% 9> 10

files = dir(parentDir);
files=files(~ismember({files.name},{'.','..', '.DS_Store'}));

i = 1;

% -- define the domain size
L= [18 9];




% -- begin looping through each test folder
for test = ids_to_plot%1:size(files, 1)
    
    
    % -- within the test number folder, get the participant numbers
    subDir = strcat(parentDir, "/", files(test).name);
    participant_folders = dir(subDir);
    participant_folders=participant_folders(~ismember({participant_folders.name},{'.','..', '.DS_Store'}));
    
    % -- begin looping through all the participant folders
    for participant = 1:size(participant_folders, 1)
        % -- make sure that we capture a number not '.' or '..'
        figure(participant); gcf;
        
        % -- create participant directory
        partDir = strcat(subDir,"/",participant_folders(participant).name);
        
        % -- begin looping through all conditions tested
        for cond = 1:size(conditions, 2)
            
            % -- create condition directory
            condDir = strcat(partDir, "/", conditions(cond));
            condFile = strcat(condDir, "/", Matfile);
            
            % -- read the data captured for the test
            TestData = load(condFile);
            
            % -- get the end time
            tf = TestData.simdata.time;
            nr = size(TestData.simdata.Xs,4);
            
            % -- plot everything nicely
            subplot(2, 4, 2*cond-1);
            cla;
            % -- display the Omron Lab map
            hold on; 
            imagesc([0 L(1)],[0 L(2)], TestData.img);
            set(gca,'ydir','reverse');
            set(gca, 'fontsize', 16);
            axis image;
            
            % -- plot the target location
            plot(TestData.simdata.tloc(1, 1), ...
                TestData.simdata.tloc(1, 2),'s', ...
                'color', [0,77,64]/255, ...
                 'LineWidth', 3, 'MarkerSize', 12); % -- Target location
            
            % -- plot the beginning and end points of the robots
            plot(TestData.simdata.Xs(1, 1, 1, 1), ...
                TestData.simdata.Xs(2, 1, 1, 1), 'o', ...
                'color', [30, 136, 229]/255, ...
                'LineWidth', 3, 'MarkerSize', 12); % -- experiment robot (human controlled robot start)
            for rr=1:nr-1
            plot(TestData.simdata.Xs(1, 1, 1, rr+1), ...
                TestData.simdata.Xs(2, 1, 1, rr+1),'go', ...
                'LineWidth', 3, 'MarkerSize', 12); % -- autonomous robot 1 start
            end
            
            plot(TestData.simdata.Xs(1, tf, 1, 1), ...
                TestData.simdata.Xs(2, tf, 1, 1), 'x', ...
                'color', [30, 136, 229]/255, ...
                'LineWidth', 3, 'MarkerSize', 12); % -- experiment robot (human controlled robot end)
            for rr=1:nr-1
            plot(TestData.simdata.Xs(1, tf, 1, rr+1), ...
                TestData.simdata.Xs(2, tf, 1, rr+1), 'gx',...
                'LineWidth', 3, 'MarkerSize', 12); % -- autonomous robot 1 end
            end
            
            % -- On top of the domain map, plot all trajectories of the robots
            plot(TestData.simdata.Xs(1, 1:tf, 1, 1), ...
                TestData.simdata.Xs(2, 1:tf, 1, 1), ...
                'k-', 'LineWidth', 2); % -- experiment robot (human controlled robot trajectory)
            for rr=1:nr-1
            plot(TestData.simdata.Xs(1, 1:tf, 1, rr+1), ...
                TestData.simdata.Xs(2, 1:tf, 1, rr+1), ...
                'g-', 'LineWidth', 2); % -- autonomous robot 1 trajectory
            end
            %                     title(sprintf("\\alpha_k (%s)", cond_names(cond)))
            title(cond_desc(cond), 'fontweight', 'normal')
        end
        i = i + 1;
    end
end

% -- Alpha plotting
i = 1;


% -- begin looping through each test folder
for test = ids_to_plot%1:size(files, 1)
    
    % -- within the test number folder, get the participant numbers
    subDir = strcat(parentDir, "/", files(test).name);
    participant_folders = dir(subDir);
    participant_folders=participant_folders(~ismember({participant_folders.name},{'.','..', '.DS_Store'}));
    
    % -- begin looping through all the participant folders
    for participant = 1:size(participant_folders, 1)
        % -- make sure that we capture a number not '.' or '..'
        figure(participant); gcf;
        
        % -- create participant directory
        partDir = strcat(subDir,"/",participant_folders(participant).name);
        
        % -- begin looping through all conditions tested
        for cond = 1:size(conditions, 2)
            
            % -- create condition directory
            condDir = strcat(partDir, "/", conditions(cond));
            condFile = strcat(condDir, "/", Matfile);
            
            % -- read the data captured for the test
            TestData = load(condFile);
            
            % -- get the end time
            tf = TestData.simdata.time;
            
            % -- plot everything nicely
            subplot(2, 4, 2*cond);
            cla;
            
            % -- plot the target location
            hold on; plot(1:tf, ...
                TestData.simdata.alpha(1:tf), ...
                'k-', 'LineWidth', 3); % -- alpha
            grid on;
            set(gca, 'ylim', [0,1]);
            set(gca, 'fontsize', 16);
            
            %                     title(cond_names(cond))
            ylabel('$\alpha_k=p(\mathrm{xMxT}|f_{\tau})$', ...
                'interpreter', 'latex')
            xlabel('time (s)');
            
        end
        i = i + 1;
    end
    
end

end


% function PlotRWControls()
% 
% % -- define the conditions tested
% conditions = ["condition_1",...
%     "condition_2",...
%     "condition_3",...
%     "condition_4"];
% 
% % -- define the mat file name used for all participants
% Matfile = "OmronLab_p=1200_nsim=1_agents=3.mat";
% 
% % -- pull list of test folders within test folder
% parentDir = "data/test";
% parentDir_RW = "data/RandomWalk";
% files = dir(parentDir);
% RW_files = dir(parentDir_RW);
% 
% figure(1); clf; gcf;
% 
% % -- define the domain size
% L= [18 9];
% 
% i = 1;
% 
% % -- begin looping through each test folder
% for RWtest = 4:4%1:size(RW_files, 1)
%     
%     % -- make sure that we capture a number not '.' or '..'
%     if (RW_files(RWtest).name ~= "." && RW_files(RWtest).name ~= ".." && RW_files(RWtest).name ~= ".DS_Store")
%         
%         % -- within the test number folder, get the participant numbers
%         RWsubDir = strcat(parentDir_RW, "/", RW_files(RWtest).name);
%         RWparticipant_folders = dir(RWsubDir);
%         
%         % -- begin looping through all the participant folders
%         for RWparticipant = 1:size(RWparticipant_folders, 1)
%             figure(RWparticipant); gcf;
%             
%             % -- make sure that we capture a number not '.' or '..'
%             if (RWparticipant_folders(RWparticipant).name ~= "." && RWparticipant_folders(RWparticipant).name ~= ".." && RWparticipant_folders(RWparticipant).name ~= ".DS_Store")
%                 
%                 % -- create participant directory
%                 RWpartDir = strcat(RWsubDir,"/",RWparticipant_folders(RWparticipant).name);
%                 
%                 % -- begin looping through all conditions tested
%                 for cond = 1:size(conditions, 2)
%                     
%                     % -- create condition directory
%                     RWcondDir = strcat(RWpartDir, "/", conditions(cond));
%                     RWcondFile = strcat(RWcondDir, "/", Matfile);
%                     
%                     % -- read the data captured for the test
%                     RWTestData = load(RWcondFile);
%                     % -- get the end time
%                     tf = RWTestData.simdata.time;
%                     
%                     % -- plot everything nicely
%                     subplot(2, 2, cond);
%                     % -- plot the turn rate and velocity data
%                     plot(1:tf, RWTestData.simdata.omega(1,1:tf,1,2), 'r-', 'LineWidth', 2);
%                     hold on; plot(1:tf, RWTestData.simdata.omega(1,1:tf,1,3), 'g-', 'LineWidth', 2);
%                     hold on; plot(1:tf, RWTestData.simdata.vel(1,1:tf,1,2), 'm-', 'LineWidth', 2);
%                     hold on; plot(1:tf, RWTestData.simdata.vel(1,1:tf,1,3), 'b-', 'LineWidth', 2);
%                     
%                     legend('Auto1 omega','Auto2 omega','Auto1 vel','Auto1 vel');
%                     
%                     title(sprintf("Random Walk scenario\n Condition: %d, Controls", cond))
%                     
%                     
%                 end
%                 i = i + 1;
%             end
%         end
%     end
% end
% end

% function PlotWhoFoundTarget(SimExp)
% 
% % -- get the size of the struct where all data is store for ease of use
% [Np, Nc] = size(SimExp);
% 
% % -- create figure
% figure(1); clf; gcf;
% 
% % -- loop through number of conditions
% for cond = 1:Nc
%     % -- create subplots for each condition
%     subplot(1,Nc, cond);
%     
%     % -- collect the which_robot data
%     for i = 1:Np
%         WhichRobot(i) = SimExp(i, cond).simdata.Which_robot;
%     end
%     
%     % -- store the data in a temperary and clean variables
%     X = [1, 2, 3];
%     Y = [sum(WhichRobot == 0), sum(WhichRobot == 2), sum(WhichRobot == 3)];
%     
%     % -- only for condition 4 (just for labeling purposes of title)
%     if cond == 2
%         cond = 4;
%     end
%     
%     % -- plot bar graph and make it look nice
%     bar(X, Y);
%     text(1:length(X),Y',num2str(Y'),'vert','bottom','horiz','center','FontSize',16);
%     xlabel("Robot");
%     ylabel("Number of times target was found by robot");
%     title(sprintf("Condition: %d", cond));
%     
%     % -- set figure parameters
%     ax = gca;
%     ax.FontSize = 16;
% end
% 
% end

% function TimeGained()
% 
% % -- define the conditions tested
% conditions = ["condition_1",...
%     "condition_2",...
%     "condition_3",...
%     "condition_4"];
% 
% % -- define the mat file name used for all participants
% Matfile = "OmronLab_p=1200_nsim=1_agents=3.mat";
% 
% % -- pull list of test folders within test folder
% parentDir_FT = "../../simdata/alpha_t/FreezeTime";
% parentDir_TD = "../../simdata/alpha_t/TotalDist";%TotalDist,FreezeTime
% parentDir_RW = "../../simdata/RandomWalk";
% parentDir0 = "../../simdata/alpha_0";
% parentDir1 = "../../simdata/alpha_1";
% 
% 
% timeArray = csvread('../data/TimeToFind.csv');
% timeArray(:,1)=timeArray(:,1)/2.27; % scaling to compensate for length and obstacles
% timeArray(:,1:4) = timeArray(:,1:4)-5.0; % -- adjusting for humans entering code into computer
% IDs = csvread('../src/IDList_Completed.csv', 1);
% 
% [testTime_FT, meanTimeArrayComp_FT, stdTimeArrayComp_FT]=...
%     collate_time_data(parentDir_FT, timeArray, IDs, conditions, Matfile);
% 
% [testTime_TD, meanTimeArrayComp_TD, stdTimeArrayComp_TD]=...
%     collate_time_data(parentDir_TD, timeArray, IDs, conditions, Matfile);
% 
% [RWtestTime, RWmeanTimeArrayComp, RWstdTimeArrayComp]=...
%     collate_time_data(parentDir_RW, timeArray, IDs, conditions, Matfile);
% 
% [testTime0, meanTimeArrayComp0, stdTimeArrayComp0]=...
%     collate_time_data(parentDir0, timeArray, IDs, conditions, Matfile);
% 
% [testTime1, meanTimeArrayComp1, stdTimeArrayComp1]=...
%     collate_time_data(parentDir1, timeArray, IDs, conditions, Matfile);
% 
% % save workspace and then just select and run the rest of the script to
% % save time
% 
% save(['timegained_workspace', '.mat'])
% end


% function [testTime, meanTimeArrayComp, stdTimeArrayComp]=...
%     collate_time_data(parentDir, timeArray, IDs, ...
%     conditions, Matfile)
% 
% % -- initialize the counter
% i = 1;
% 
% condCount = ones(1,4);
% 
% timecount = 0;
% 
% fps=2; % frame rate for simdata --SB
% 
% files = dir(parentDir);
% files=files(~ismember({files.name},...
%     {'.','..', '.DS_Store'}));
% 
% num_sims=size(files, 1);
% 
% testTime = zeros(num_sims, 4);
% for test = 1:num_sims
%     % can collapse all these into a single function for efficiency --SB
%     
%     
%     % -- within the test number folder, get the participant numbers
%     subDir = strcat(parentDir, "/", files(test).name);
%     participant_folders = dir(subDir);
%     participant_folders=participant_folders(~ismember({participant_folders.name},...
%         {'.','..', '.DS_Store'}));
%     
%     % -- begin looping through all the participant folders
%     for participant = 1:size(participant_folders, 1)
%         
%         % -- create participant directory
%         partDir = strcat(subDir,"/",participant_folders(participant).name);
%         
%         % -- begin looping through all conditions tested
%         for cond = 1:size(conditions, 2)
%             
%             % -- create condition directory
%             condDir = strcat(partDir, "/", conditions(cond));
%             condFile = strcat(condDir, "/", Matfile);
%             
%             % -- read the data captured for the test
%             TestData = load(condFile);
%             
%             % -- check if the autonomous robot found the target
%             if TestData.simdata.Which_robot ~= 0
%                 % -- compare the ID looking at with the list
%                 % -- to get the original time ran
%                 for id = 1:size(IDs, 1)
%                     if num2str(IDs(id)) == participant_folders(participant).name
%                         % -- get the time difference and store it
%                         %                                 if cond ~= 4
%                         % counting total time... -- SB
%                         % only count if the difference is positive
%                         if timeArray(id, cond) - TestData.simdata.time/fps > 0
%                             testTime(condCount(1,cond), cond) = ...
%                                 timeArray(id, cond) - TestData.simdata.time/fps;
%                         end
%                         
%                         % Commenting this out because we want to
%                         % ignore deception - SB
%                         %                                 else
%                         %                                     testTime(condCount(1,cond), cond) = ...
%                         %                                         abs(timeArray(id, cond) + timeArray(id, cond+1) - TestData.simdata.time);
%                         %                                 end
%                         timecount = timecount + 1;
%                         break;
%                     end
%                 end
%             end
%             condCount(1,cond) = condCount(1,cond) + 1;
%         end
%         i = i + 1;
%     end
% end
% 
% meanTimeArrayComp = [mean(testTime(testTime(:,1) ~= 0, 1)), ...
%     mean(testTime(testTime(:,2) ~= 0, 2)), ...
%     mean(testTime(testTime(:,3) ~= 0, 3)), ...
%     mean(testTime(testTime(:,4) ~= 0, 4))];
% stdTimeArrayComp = [std(testTime(testTime(:,1) ~= 0, 1)), ...
%     std(testTime(testTime(:,2) ~= 0, 2)), ...
%     std(testTime(testTime(:,3) ~= 0, 3)), ...
%     std(testTime(testTime(:,4) ~= 0, 4))];
% 
% end
function plot_success_traj(file)
data=readtable(file);
% remove trials where the robot went out of domain
data(data.success==2,:)=[];
% if target was not found, then set the timesec as expTime
data.timesec(data.success==0)=data.expTime(data.success==0);

% for condition 4, half the time the target location was not where
% the human was looking for initially
% this means that for a simulation, we are simulating a weird situation
% where the human is just going somewhere where they don't need to but the
% autonomous robots go for the true target location
% best to ignore such trials (meeting on 7/24/24)
% data.timesec(data.whichrobot>1 & data.timesec>data.expTime)=...
%     data.expTime(data.whichrobot>1 & data.timesec>data.expTime);
idx1=data.condition==4 & data.c4flag==1;
fprintf('Sims within yMyT where target was placed else: %d\n',sum(idx1));
data(idx1,:)=[];

% pick trials for each condition
% -- define the domain size
L= [18 9];

cond_desc=["No Map, No Target",...
    "No Map, Yes Target",...
    "Yes Map, No Target",...
    "Yes Map, Yes Target"];

figure(1); gcf; clf;

for cond=1:4
    % autonomous robot found the target
%     idx2show=find(data.condition==cond & data.whichrobot>1);
    % any robot found the target
    idx2show=find(data.condition==cond & data.strategy==1);
    % used 5 for nr=3, 15 for nr=2
    idx2show=idx2show(5); % keep changing the index here
    
    % find the first instance where all 4 rows exist
    sim=load(data.matfile{idx2show});
    
     % -- get the end time
    tf = sim.simdata.time;
    nr = size(sim.simdata.Xs,4);
            
    % -- plot everything nicely
    subplot(2, 4, 2*cond-1);
    cla;
    % -- display the Omron Lab map
    hold on; 
    imagesc([0 L(1)],[0 L(2)], sim.img);
    set(gca,'ydir','reverse');
    set(gca, 'fontsize', 16);
    axis image;
    
    % -- plot the target location
    plot(sim.simdata.tloc(1, 1), ...
        sim.simdata.tloc(1, 2),'s', ...
        'color', [0,77,64]/255, ...
         'LineWidth', 3, 'MarkerSize', 12); % -- Target location

    % -- plot the beginning and end points of the robots
    plot(sim.simdata.Xs(1, 1, 1, 1), ...
        sim.simdata.Xs(2, 1, 1, 1), 'o', ...
        'color', [30, 136, 229]/255, ...
        'LineWidth', 3, 'MarkerSize', 12); % -- experiment robot (human controlled robot start)
    for rr=1:nr-1
    plot(sim.simdata.Xs(1, 1, 1, rr+1), ...
        sim.simdata.Xs(2, 1, 1, rr+1),'go', ...
        'LineWidth', 3, 'MarkerSize', 12); % -- autonomous robot 1 start
    end

    plot(sim.simdata.Xs(1, tf, 1, 1), ...
        sim.simdata.Xs(2, tf, 1, 1), 'x', ...
        'color', [30, 136, 229]/255, ...
        'LineWidth', 3, 'MarkerSize', 12); % -- experiment robot (human controlled robot end)
    for rr=1:nr-1
    plot(sim.simdata.Xs(1, tf, 1, rr+1), ...
        sim.simdata.Xs(2, tf, 1, rr+1), 'gx',...
        'LineWidth', 3, 'MarkerSize', 12); % -- autonomous robot 1 end
    end

    % -- On top of the domain map, plot all trajectories of the robots
    plot(sim.simdata.Xs(1, 1:tf, 1, 1), ...
        sim.simdata.Xs(2, 1:tf, 1, 1), ...
        'k-', 'LineWidth', 2); % -- experiment robot (human controlled robot trajectory)
    for rr=1:nr-1
    plot(sim.simdata.Xs(1, 1:tf, 1, rr+1), ...
        sim.simdata.Xs(2, 1:tf, 1, rr+1), ...
        'g-', 'LineWidth', 2); % -- autonomous robot 1 trajectory
    end
    %                     title(sprintf("\\alpha_k (%s)", cond_names(cond)))
    title(cond_desc(cond), 'fontweight', 'normal')
    
    % plot alpha
    % -- get the end time
    tf = sim.simdata.time;

    % -- plot everything nicely
    subplot(2, 4, 2*cond);
    cla;

    % -- plot the target location
    hold on; plot(1:tf, ...
        sim.simdata.alpha(1:tf), ...
        'k-', 'LineWidth', 3); % -- alpha
    grid on;
    set(gca, 'ylim', [0,1]);
    set(gca, 'fontsize', 16);

    %                     title(cond_names(cond))
    ylabel('$\alpha_k=p(\mathrm{xMxT}|f_{\tau})$', ...
        'interpreter', 'latex')
    xlabel('time (s)');
            
end
end

function plot_performance_comparison(file)
data=readtable(file);

% the way simulations are setup, if the robot that finds the target is more
% than 0, then the robot is autonomous otherwise it is human. The human
% time may not be recorded accurately because it may not have satisfied the
% criteria, therefore in this case, we pick the experiment time.
% the experiment time is already less 5 seconds so no need to reduce it
% further
% data.timesec(data.successfulrobot==0)=data.expTime(data.successfulrobot==0);

fprintf('total sims: %d\n', size(data,1));

% remove trials where the robot went out of domain
data(data.success==2,:)=[];
% if target was not found, then set the timesec as expTime
data.timesec(data.success==0)=data.expTime(data.success==0);

% for condition 4, half the time the target location was not where
% the human was looking for initially
% this means that for a simulation, we are simulating a weird situation
% where the human is just going somewhere where they don't need to but the
% autonomous robots go for the true target location
% best to ignore such trials (meeting on 7/24/24)
% data.timesec(data.whichrobot>1 & data.timesec>data.expTime)=...
%     data.expTime(data.whichrobot>1 & data.timesec>data.expTime);
idx1=data.condition==4 & data.c4flag==1;
fprintf('Sims within yMyT where target was placed else: %d\n',sum(idx1));
data(idx1,:)=[];

% finally any that are more than expTime should be set to expTime
data.timesec(data.whichrobot>1 & data.timesec>data.expTime)=...
    data.expTime(data.whichrobot>1 & data.timesec>data.expTime);


% strategies
% strategies={"alpha_t/TotalDist", "alpha_t/FreezeTime", ...
%     "alpha_t/DistAndFreeze", "alpha_0", ...
%     "alpha_1",  "RandomWalk"};

strategies_labels={'\alpha_k (distance)','\alpha_k (freezing)',...
    '\alpha_k (distance and freezing)', ...
    '\alpha=0 (only assist)', '\alpha=1 (search independently)', 'random'};

markers={'^m', 'sr', 'db', 'ok'};

figure(1); gcf; clf;

subplot(1,3,1);
mu=zeros(4,5);
st=zeros(4,5);
for cc=1:4
    for ss=1:6
        idx=data.condition==cc & data.strategy==ss;
        mu(cc,ss)=mean(data.timesec(idx));
        st(cc,ss)=std(data.timesec(idx));
        %         boxplot(data.timesec(data.condition==cc), ...
        %             data.strategy(data.condition==cc),'PlotStyle','compact');
    end
    errorbar((0:1:5)+cc*.15-.3, mu(cc,:), st(cc,:), markers{cc}, ...
        'MarkerFaceColor', markers{cc}(2), 'MarkerSize', 18, ...
        'LineWidth', 3); hold on;
    hold on;
end
grid on;
set(gca,'fontsize', 24);
set(gca, 'ylim', [0,300]);
set(gca, 'xtick', 0:5);
xticklabels(strategies_labels);
xtickangle(30);
ylabel('Time to find (s)')

subplot(1,3,2);
fracTrials=zeros(4,5);
for cc=1:4
    for ss=1:6
        idx=data.condition==cc & data.strategy==ss;
        fracTrials(cc,ss)=sum(data.timesec(idx)<data.expTime(idx))/sum(idx);
    end
    plot(0:1:5, fracTrials(cc,:), markers{cc}, ...
        'MarkerFaceColor', markers{cc}(2), 'MarkerSize', 18, ...
        'LineWidth', 2); hold on;
end

grid on;

set(gca,'fontsize', 24);
set(gca, 'ylim', [0,1]);
set(gca, 'xtick', 0:5);
xticklabels(strategies_labels);
xtickangle(30);
if str2double(file(end-4))==3
    ylabel({'Fraction of trials where human-robot team',...
        'of 3 robots found target before single human'})
else
        ylabel({'Fraction of trials where human-robot team',...
        'of 2 robots found target before single human'})
end
legend('No Map, No Target',...
    'No Map, Yes Target',...
    'Yes Map, No Target',...
    'Yes Map, Yes Target');

  
subplot(1,3,3);
mu=zeros(4,5);
st=zeros(4,5);
for cc=1:4
    for ss=1:6
        idx=data.condition==cc & data.strategy==ss & data.timesec<data.expTime;
        mu(cc,ss)=mean(data.expTime(idx)-data.timesec(idx));
        st(cc,ss)=std(data.expTime(idx)-data.timesec(idx));
%         boxplot(data.timesec(data.condition==cc), ...
%             data.strategy(data.condition==cc),'PlotStyle','compact');
    end
    errorbar((0:1:5)+cc*.15-.3, mu(cc,:), st(cc,:), markers{cc}, ...
    'MarkerFaceColor', markers{cc}(2), 'MarkerSize', 18, ...
    'LineWidth', 2); hold on;
    hold on;
end
grid on;
set(gca,'fontsize', 24);
set(gca, 'ylim', [0,300]);
set(gca, 'xtick', 0:5);
xticklabels(strategies_labels);
xtickangle(30);
ylabel('Time gained over single human trials (s)')    
    
end

% function plot_performance_comparison_deprecated()
% 
% load('timegained_workspace.mat')
% 
% % -- create figure
% figure(1); clf; gcf;
% 
% subplot(1,2,2);
% % -- plot the results
% errorbar(0:1:4, ...
%     [meanTimeArrayComp_TD(1,1),meanTimeArrayComp_FT(1,1), ...
%     RWmeanTimeArrayComp(1,1), meanTimeArrayComp0(1,1), meanTimeArrayComp1(1,1)],...
%     [stdTimeArrayComp_TD(1,1), stdTimeArrayComp_FT(1,1), ...
%     RWstdTimeArrayComp(1,1), stdTimeArrayComp0(1,1), stdTimeArrayComp1(1,1)],...
%     '^m','markersize', 16, 'LineWidth',2, 'MarkerFaceColor','auto'); hold on;
% 
% errorbar(0:1:4, ...
%     [meanTimeArrayComp_TD(1,2), meanTimeArrayComp_FT(1,2), ...
%     RWmeanTimeArrayComp(1,2), meanTimeArrayComp0(1,2), meanTimeArrayComp1(1,2)],...
%     [stdTimeArrayComp_TD(1,2), stdTimeArrayComp_FT(1,2), ...
%     RWstdTimeArrayComp(1,2), stdTimeArrayComp0(1,2), stdTimeArrayComp1(1,2)],...
%     'sr','markersize', 16, 'LineWidth',2, 'MarkerFaceColor','auto'); hold on;
% 
% errorbar(0:1:4, ...
%     [meanTimeArrayComp_TD(1,3), meanTimeArrayComp_FT(1,3), ...
%     RWmeanTimeArrayComp(1,3), meanTimeArrayComp0(1,3), meanTimeArrayComp1(1,3)],...
%     [stdTimeArrayComp_TD(1,3), stdTimeArrayComp_FT(1,3), ...
%     RWstdTimeArrayComp(1,3), stdTimeArrayComp0(1,3), stdTimeArrayComp1(1,3)],...
%     'db','markersize', 16, 'LineWidth',2, 'MarkerFaceColor','auto'); hold on;
% 
% errorbar(0:1:4, ...
%     [meanTimeArrayComp_TD(1,4), meanTimeArrayComp_FT(1,4), ...
%     RWmeanTimeArrayComp(1,4), meanTimeArrayComp0(1,4), meanTimeArrayComp1(1,4)],...
%     [stdTimeArrayComp_TD(1,4), stdTimeArrayComp_FT(1,4), ...
%     RWstdTimeArrayComp(1,4), stdTimeArrayComp0(1,4), stdTimeArrayComp1(1,4)],...
%     'ok', 'markersize', 16, 'LineWidth',2, 'MarkerFaceColor','auto');
% 
% % legend('No Map, No Target',...
% %        'No Map, Yes Target',...
% %        'Yes Map, No Target',...
% %        'Yes Map, Yes Target');
% 
% ylabel("Time gained (s)");
% 
% % -- make the plot nice
% grid on;
% box off
% ax = gca;
% ax.FontSize = 18;
% set(gca, 'xtick', 0:4)
% xlabel({'Weighting strategy', '(b)'});
% xticklabels({'\alpha_k (distance)', '\alpha_k (freezing)', 'random', '\alpha=0', '\alpha=1'});
% xtickangle(30);
% 
% % -- create single figure
% subplot(1,2,1);
% 
% % -- populate figure
% % Why is this not the same as when we use robot found flag only? -- SB
% plot(0:1:4, [sum(testTime_TD(:,1)>0)/size(testTime_TD,1); ...
%     sum(testTime_FT(:,1)>0)/size(testTime_FT,1); ...
%     sum(RWtestTime(:,1)>0)/size(RWtestTime,1); ...
%     sum(testTime0(:,1)>0)/size(testTime0,1); ...
%     sum(testTime1(:,1)>0)/size(testTime1,1)], ...
%     '^m', 'MarkerSize', 18, 'MarkerFaceColor', 'm', ...
%     'LineWidth', 2); hold on;
% plot(0:1:4, [sum(testTime_TD(:,2)>0)/size(testTime_TD,1); ...
%     sum(testTime_FT(:,2)>0)/size(testTime_FT,1); ...
%     sum(RWtestTime(:,2)>0)/size(RWtestTime,1); ...
%     sum(testTime0(:,2)>0)/size(testTime0,1); ...
%     sum(testTime1(:,2)>0)/size(testTime1,1)], ...
%     'sr', 'MarkerSize', 18,'MarkerFaceColor', 'r', ...
%     'LineWidth', 2); hold on;
% plot(0:1:4, [sum(testTime_TD(:,3)>0)/size(testTime_TD,1); ...
%     sum(testTime_FT(:,3)>0)/size(testTime_FT,1); ...
%     sum(RWtestTime(:,3)>0)/size(RWtestTime,1); ...
%     sum(testTime0(:,3)>0)/size(testTime0,1); ...
%     sum(testTime1(:,3)>0)/size(testTime1,1)], ...
%     'db', 'MarkerSize', 18, 'MarkerFaceColor', 'b',...
%     'LineWidth', 2); hold on;
% plot(0:1:4, [sum(testTime_TD(:,4)>0)/size(testTime_TD,1); ...
%     sum(testTime_FT(:,4)>0)/size(testTime_FT,1); ...
%     sum(RWtestTime(:,4)>0)/size(RWtestTime,1); ...
%     sum(testTime0(:,4)>0)/size(testTime0,1); ...
%     sum(testTime1(:,4)>0)/size(testTime1,1)], ...
%     'ok', 'MarkerSize', 18,'MarkerFaceColor', 'k', ...
%     'LineWidth', 2);
% 
% 
% % -- legend
% legend('No Map, No Target',...
%     'No Map, Yes Target',...
%     'Yes Map, No Target',...
%     'Yes Map, Yes Target');
% 
% % -- make figure look nice
% grid on;
% box off
% ylabel("Fraction of trials target found by auto. robots");
% xlabel({'Weighting strategy', '(a)'});
% ax = gca;
% ax.FontSize = 18;
% set(gca, 'xtick', 0:4, 'ylim', [0,1])
% xticklabels({'\alpha_k (distance)', '\alpha_k (freezing)', 'random', '\alpha=0', '\alpha=1'});
% xtickangle(30);
% end


% function PlotWhoFoundTarget_testSet()
% 
% % -- define the conditions tested
% conditions = ["condition_1",...
%     "condition_2",...
%     "condition_3",...
%     "condition_4"];
% 
% % -- define the mat file name used for all participants
% Matfile = "OmronLab_p=1200_nsim=1_agents=3.mat";
% 
% % -- pull list of test folders within test folder
% parentDir = "data/alpha_t/TotalDist";
% parentDir_RW = "data/RandomWalk";
% parentDir0 = "data/alpha_0";
% parentDir1 = "data/alpha_1";
% 
% files = dir(parentDir);
% RW_files = dir(parentDir_RW);
% files0 = dir(parentDir0);
% files1 = dir(parentDir1);
% 
% % -- initialize the counter
% i = 1;
% 
% % -- begin looping through each test folder
% for test = 1:size(files, 1)
%     
%     % -- make sure that we capture a number not '.' or '..'
%     if (files(test).name ~= "." && files(test).name ~= ".." && files(test).name ~= ".DS_Store")
%         
%         % -- within the test number folder, get the participant numbers
%         subDir = strcat(parentDir, "/", files(test).name);
%         participant_folders = dir(subDir);
%         
%         % -- begin looping through all the participant folders
%         for participant = 1:size(participant_folders, 1)
%             % -- make sure that we capture a number not '.' or '..'
%             if (participant_folders(participant).name ~= "." && participant_folders(participant).name ~= ".." && participant_folders(participant).name ~= ".DS_Store")
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
%                     WhichRobot(cond, i) = TestData.simdata.Which_robot;
%                     
%                 end
%                 i = i + 1;
%             end
%         end
%     end
% end
% 
% i = 1;
% % -- begin looping through each test folder
% for RWtest = 1:size(RW_files, 1)
%     
%     % -- make sure that we capture a number not '.' or '..'
%     if (RW_files(RWtest).name ~= "." && RW_files(RWtest).name ~= ".." && RW_files(RWtest).name ~= ".DS_Store")
%         
%         % -- within the test number folder, get the participant numbers
%         RWsubDir = strcat(parentDir_RW, "/", RW_files(RWtest).name);
%         RWparticipant_folders = dir(RWsubDir);
%         
%         % -- begin looping through all the participant folders
%         for RWparticipant = 1:size(RWparticipant_folders, 1)
%             % -- make sure that we capture a number not '.' or '..'
%             if (RWparticipant_folders(RWparticipant).name ~= "." && RWparticipant_folders(RWparticipant).name ~= ".." && RWparticipant_folders(RWparticipant).name ~= ".DS_Store")
%                 
%                 % -- create participant directory
%                 RWpartDir = strcat(RWsubDir,"/",RWparticipant_folders(RWparticipant).name);
%                 
%                 % -- begin looping through all conditions tested
%                 for cond = 1:size(conditions, 2)
%                     
%                     % -- create condition directory
%                     RWcondDir = strcat(RWpartDir, "/", conditions(cond));
%                     RWcondFile = strcat(RWcondDir, "/", Matfile);
%                     
%                     % -- read the data captured for the test
%                     RWTestData = load(RWcondFile);
%                     RWWhichRobot(cond, i) = RWTestData.simdata.Which_robot;
%                     
%                 end
%                 i = i + 1;
%             end
%         end
%     end
% end
% 
% i = 1;
% % -- begin looping through each test folder
% for test0 = 1:size(files0, 1)
%     
%     % -- make sure that we capture a number not '.' or '..'
%     if (files0(test0).name ~= "." && files0(test0).name ~= ".." && files0(test0).name ~= ".DS_Store")
%         
%         % -- within the test number folder, get the participant numbers
%         subDir0 = strcat(parentDir0, "/", files0(test0).name);
%         participant_folders0 = dir(subDir0);
%         
%         % -- begin looping through all the participant folders
%         for participant0 = 1:size(participant_folders0, 1)
%             % -- make sure that we capture a number not '.' or '..'
%             if (participant_folders0(participant0).name ~= "." && participant_folders0(participant0).name ~= ".." && participant_folders(participant).name ~= ".DS_Store")
%                 
%                 % -- create participant directory
%                 partDir0 = strcat(subDir0,"/",participant_folders0(participant0).name);
%                 
%                 % -- begin looping through all conditions tested
%                 for cond = 1:size(conditions, 2)
%                     
%                     % -- create condition directory
%                     condDir0 = strcat(partDir0, "/", conditions(cond));
%                     condFile0 = strcat(condDir0, "/", Matfile);
%                     
%                     % -- read the data captured for the test
%                     TestData = load(condFile0);
%                     WhichRobot0(cond, i) = TestData.simdata.Which_robot;
%                     
%                 end
%                 i = i + 1;
%             end
%         end
%     end
% end
% 
% i = 1;
% % -- begin looping through each test folder
% for test1 = 1:size(files1, 1)
%     
%     % -- make sure that we capture a number not '.' or '..'
%     if (files1(test1).name ~= "." && files1(test1).name ~= ".." && files1(test1).name ~= ".DS_Store")
%         
%         % -- within the test number folder, get the participant numbers
%         subDir1 = strcat(parentDir1, "/", files1(test1).name);
%         participant_folders1 = dir(subDir1);
%         
%         % -- begin looping through all the participant folders
%         for participant1 = 1:size(participant_folders1, 1)
%             % -- make sure that we capture a number not '.' or '..'
%             if (participant_folders1(participant1).name ~= "." && participant_folders1(participant1).name ~= ".." && participant_folders(participant).name ~= ".DS_Store")
%                 
%                 % -- create participant directory
%                 partDir1 = strcat(subDir1,"/",participant_folders1(participant1).name);
%                 
%                 % -- begin looping through all conditions tested
%                 for cond = 1:size(conditions, 2)
%                     
%                     % -- create condition directory
%                     condDir1 = strcat(partDir1, "/", conditions(cond));
%                     condFile1 = strcat(condDir1, "/", Matfile);
%                     
%                     % -- read the data captured for the test
%                     TestData = load(condFile1);
%                     WhichRobot1(cond, i) = TestData.simdata.Which_robot;
%                     
%                 end
%                 i = i + 1;
%             end
%         end
%     end
% end
% 
% NumTest = 600;
% 
% % -- store the data in a temperary and clean variables
% X = [1, 2];
% testset.Y1 = [sum(WhichRobot(1,:) == 0), sum(WhichRobot(1,:) == 2) + sum(WhichRobot(1,:) == 3)]./NumTest;
% testset.Y2 = [sum(WhichRobot(2,:) == 0), sum(WhichRobot(2,:) == 2) + sum(WhichRobot(2,:) == 3)]./NumTest;
% testset.Y3 = [sum(WhichRobot(3,:) == 0), sum(WhichRobot(3,:) == 2) + sum(WhichRobot(3,:) == 3)]./NumTest;
% testset.Y4 = [sum(WhichRobot(4,:) == 0), sum(WhichRobot(4,:) == 2) + sum(WhichRobot(4,:) == 3)]./NumTest;
% 
% testset.RWY1 = [sum(RWWhichRobot(1,:) == 0), sum(RWWhichRobot(1,:) == 2) + sum(RWWhichRobot(1,:) == 3)]./NumTest;
% testset.RWY2 = [sum(RWWhichRobot(2,:) == 0), sum(RWWhichRobot(2,:) == 2) + sum(RWWhichRobot(2,:) == 3)]./NumTest;
% testset.RWY3 = [sum(RWWhichRobot(3,:) == 0), sum(RWWhichRobot(3,:) == 2) + sum(RWWhichRobot(3,:) == 3)]./NumTest;
% testset.RWY4 = [sum(RWWhichRobot(4,:) == 0), sum(RWWhichRobot(4,:) == 2) + sum(RWWhichRobot(4,:) == 3)]./NumTest;
% 
% testset.Y10 = [sum(WhichRobot0(1,:) == 0), sum(WhichRobot0(1,:) == 2) + sum(WhichRobot0(1,:) == 3)]./NumTest;
% testset.Y20 = [sum(WhichRobot0(2,:) == 0), sum(WhichRobot0(2,:) == 2) + sum(WhichRobot0(2,:) == 3)]./NumTest;
% testset.Y30 = [sum(WhichRobot0(3,:) == 0), sum(WhichRobot0(3,:) == 2) + sum(WhichRobot0(3,:) == 3)]./NumTest;
% testset.Y40 = [sum(WhichRobot0(4,:) == 0), sum(WhichRobot0(4,:) == 2) + sum(WhichRobot0(4,:) == 3)]./NumTest;
% 
% testset.Y11 = [sum(WhichRobot1(1,:) == 0), sum(WhichRobot1(1,:) == 2) + sum(WhichRobot1(1,:) == 3)]./NumTest;
% testset.Y21 = [sum(WhichRobot1(2,:) == 0), sum(WhichRobot1(2,:) == 2) + sum(WhichRobot1(2,:) == 3)]./NumTest;
% testset.Y31 = [sum(WhichRobot1(3,:) == 0), sum(WhichRobot1(3,:) == 2) + sum(WhichRobot1(3,:) == 3)]./NumTest;
% testset.Y41 = [sum(WhichRobot1(4,:) == 0), sum(WhichRobot1(4,:) == 2) + sum(WhichRobot1(4,:) == 3)]./NumTest;
% 
% % -- function that input all test data captured
% PercentTargetFound(testset);
% 
% end

% function PercentTargetFound(testset)
% % -- remember, each variable stored in the struct is [1 x 2]
% % -- [# times target found by human, # times target found by autonomous robot]
%
% % -- create single figure
% figure(2); clf; gcf;
%
% % -- populate figure
% plot(0:1:3, [testset.Y1(1,2); testset.RWY1(1,2); testset.Y10(1,2); testset.Y11(1,2)], ...
%     '^m', 'MarkerSize', 18, 'LineWidth', 2); hold on;
% plot(0:1:3, [testset.Y2(1,2); testset.RWY2(1,2); testset.Y20(1,2); testset.Y21(1,2)], ...
%     'sr', 'MarkerSize', 18, 'LineWidth', 2); hold on;
% plot(0:1:3, [testset.Y3(1,2); testset.RWY3(1,2); testset.Y30(1,2); testset.Y31(1,2)], ...
%     'db', 'MarkerSize', 18, 'LineWidth', 2); hold on;
% plot(0:1:3, [testset.Y4(1,2); testset.RWY4(1,2); testset.Y40(1,2); testset.Y41(1,2)], ...
%     'ok', 'MarkerSize', 18, 'LineWidth', 2);
%
% % -- legend
% legend('No Map, No Target',...
%     'No Map, Yes Target',...
%     'Yes Map, No Target',...
%     'Yes Map, Yes Target');
%
% % -- make figure look nice
% ylabel("Fraction of time target found by auto. robots");
% ax = gca;
% ax.FontSize = 18;
% set(gca, 'xtick', 0:3)
% xticklabels({'\alpha(t)', 'random', '\alpha=0', '\alpha=1'});
% end

% function SingleFigBarPlot(Y1, RWY1, Y2, RWY2, Y3, RWY3, Y4, RWY4)
%
% X = [1, 2, 3, 4];
%
% % -- create figure
% figure(3); clf; gcf;
% subplot(2,2,1); bar(X, [Y1,RWY1]);
% text(1:length(X),[Y1,RWY1]',num2str([Y1,RWY1]'),'vert','bottom','horiz','center','FontSize',16);
% ylabel({"Number of times target";"was found by robot"});
% title("Condition: 1");
% ylim([0 600]);
%
% % -- set figure parameters
% ax = gca;
% ax.FontSize = 16;
% xticklabels({'Ref robot: adaptive a','Auto robot: adaptive a',...
%     'Ref robot: RW', 'Auto robot: RW'});
%
% subplot(2,2,2); bar(X, [Y2,RWY2]);
% text(1:length(X),[Y2,RWY2]',num2str([Y2,RWY2]'),'vert','bottom','horiz','center','FontSize',16);
% ylabel({"Number of times target";"was found by robot"});
% title("Condition: 2");
% ylim([0 600]);
%
% % -- set figure parameters
% ax = gca;
% ax.FontSize = 16;
% xticklabels({'Ref robot: adaptive a','Auto robot: adaptive a',...
%     'Ref robot: RW', 'Auto robot: RW'});
%
% subplot(2,2,3); bar(X, [Y3,RWY3]);
% text(1:length(X),[Y3,RWY3]',num2str([Y3,RWY3]'),'vert','bottom','horiz','center','FontSize',16);
% ylabel({"Number of times target";"was found by robot"});
% title("Condition: 3");
% ylim([0 600]);
%
% % -- set figure parameters
% ax = gca;
% ax.FontSize = 16;
% xticklabels({'Ref robot: adaptive a','Auto robot: adaptive a',...
%     'Ref robot: HRW', 'Auto robot: RW'});
%
% subplot(2,2,4); bar(X, [Y4,RWY4]);
% text(1:length(X),[Y4,RWY4]',num2str([Y4,RWY4]'),'vert','bottom','horiz','center','FontSize',16);
% ylabel({"Number of times target";"was found by robot"});
% title("Condition: 4");
% ylim([0 600]);
%
% % -- set figure parameters
% ax = gca;
% ax.FontSize = 16;
% xticklabels({'Ref robot: adaptive a','Auto robot: adaptive a',...
%     'Ref robot: RW', 'Auto robot: RW'});
%
% end

% function PlotAllTraj(SimExp)
%
% % -- get the list of participants
% loc = "data/OmronLab/3robots/";
% ids = ls(loc); % -- get all in the folder
% ids = ids(3:end-1,1:4); % -- reformat to only have the numbers
%
% % -- get the size of the struct where all data is store for ease of use
% [Np, Nc] = size(SimExp);
%
% % -- define the domain size
% L= [18 9];
%
% % -- loop through all the simulations and plot the trajectories
% for i = 1:Np
%     % -- create individual participant figures
%     figure(9+i); clf; gcf;
%
%     for j = 1:Nc
%
%         % -- get the end time
%         tf = SimExp(i,j).simdata.time;
%
%         % -- create subplots for each condition for each participant
%         subplot(1,Nc,j);
%
%         % -- display the Omron Lab map
%         hold on; imagesc([0 L(1)],[0 L(2)], SimExp(i).img);
%         set(gca,'ydir','reverse');
%         axis image;
%
%         % -- plot the target location
%         hold on; plot(SimExp(i,j).simdata.tloc(1, 1), ...
%             SimExp(i,j).simdata.tloc(1, 2), ...
%             'mx', 'LineWidth', 3, 'MarkerSize', 12); % -- Target location
%
%         % -- plot the beginning and end points of the robots
%         hold on; plot(SimExp(i,j).simdata.Xs(1, 1, 1, 1), ...
%             SimExp(i,j).simdata.Xs(2, 1, 1, 1), ...
%             'ks', 'LineWidth', 3, 'MarkerSize', 12); % -- experiment robot (human controlled robot start)
%         hold on; plot(SimExp(i,j).simdata.Xs(1, 1, 1, 2), ...
%             SimExp(i,j).simdata.Xs(2, 1, 1, 2), ...
%             'rs', 'LineWidth', 3, 'MarkerSize', 12); % -- autonomous robot 1 start
%         hold on; plot(SimExp(i,j).simdata.Xs(1, 1, 1, 3), ...
%             SimExp(i,j).simdata.Xs(2, 1, 1, 3), ...
%             'gs', 'LineWidth', 3, 'MarkerSize', 12); % -- autonomous robot 2 start
%
%         hold on; plot(SimExp(i,j).simdata.Xs(1, tf, 1, 1), ...
%             SimExp(i,j).simdata.Xs(2, tf, 1, 1), ...
%             'kx', 'LineWidth', 3, 'MarkerSize', 12); % -- experiment robot (human controlled robot end)
%         hold on; plot(SimExp(i,j).simdata.Xs(1, tf, 1, 2), ...
%             SimExp(i,j).simdata.Xs(2, tf, 1, 2), ...
%             'rx', 'LineWidth', 3, 'MarkerSize', 12); % -- autonomous robot 1 end
%         hold on; plot(SimExp(i,j).simdata.Xs(1, tf, 1, 3), ...
%             SimExp(i,j).simdata.Xs(2, tf, 1, 3), ...
%             'gx', 'LineWidth', 3, 'MarkerSize', 12); % -- autonomous robot 2 end
%
%         % -- On top of the domain map, plot all trajectories of the robots
%         hold on; plot(SimExp(i,j).simdata.Xs(1, 1:tf, 1, 1), ...
%             SimExp(i,j).simdata.Xs(2, 1:tf, 1, 1), ...
%             'k-', 'LineWidth', 2); % -- experiment robot (human controlled robot trajectory)
%         hold on; plot(SimExp(i,j).simdata.Xs(1, 1:tf, 1, 2), ...
%             SimExp(i,j).simdata.Xs(2, 1:tf, 1, 2), ...
%             'r-', 'LineWidth', 2); % -- autonomous robot 1 trajectory
%         hold on; plot(SimExp(i,j).simdata.Xs(1, 1:tf, 1, 3), ...
%             SimExp(i,j).simdata.Xs(2, 1:tf, 1, 3), ...
%             'g-', 'LineWidth', 2); % -- autonomous robot 2 trajectory
%
%         % -- make the figure look nice
%         xlabel("X (m)");
%         ylabel("Y (m)");
%         if j == 2
%             j = 4;
%         end
%         title(sprintf("Participant ID: %s, Condition: %d", ids(i, :), j));
%         legend('Target','','','','','','','Human robot', 'Autonomous robot 1', 'Autonomous robot 2');
%         ax = gca;
%         ax.FontSize = 12;
%     end
%
% end
%
% end

% function PlotTransientAlpha(SimExp)
%
% % -- get the list of participants
% loc = "data/OmronLab/3robots/";
% ids = ls(loc); % -- get all in the folder
% ids = ids(3:end-1,1:4); % -- reformat to only have the numbers
%
% % -- get the size of the struct where all data is store for ease of use
% [Np, Nc] = size(SimExp);
%
% % -- loop through all the simulations and plot the trajectories
% for i = 1:Np
%     % -- create individual participant figures
%     figure(29+i); clf; gcf;
%
%     for j = 1:Nc
%
%         % -- create subplots for each condition for each participant
%         subplot(2,2,j);
%
%         % -- plot the transient alpha values
%         hold on; plot(SimExp(i,j).simdata.alpha(:,1), ...
%             'k-', 'LineWidth', 2); % -- transient alpha value
%
%         % -- make the figure look nice
%         xlabel("Timestep");
%         ylabel("Alpha");
%         title(sprintf("Participant ID: %s, Condition: %d", ids(i, :), j));
%         ylim([0 1]);
%         axis square;
%         ax = gca;
%         ax.FontSize = 12;
%     end
% end
%
% end

% function ControlInputMapped()
%
% % -- define the conditions tested
% conditions = ["condition_1",...
%     "condition_2",...
%     "condition_3",...
%     "condition_4"];
%
% % -- define the mat file name used for all participants
% Matfile = "OmronLab_p=1200_nsim=1_agents=3.mat";
%
% % -- pull list of test folders within test folder
% parentDir_RW = "data/RandomWalk";
% RW_files = dir(parentDir_RW);
%
% % -- define the domain size
% L= [18 9];
%
% self_color = ["k", "r", "g"];
% r_pos = ["k.", "r.", "g."];
% r_vel = ["k-", "r-", "g-"];
% i=1;
% % -- begin looping through each test folder
% for RWtest = 1:size(RW_files, 1)
%
%     % -- make sure that we capture a number not '.' or '..'
%     if (RW_files(RWtest).name ~= "." && RW_files(RWtest).name ~= ".." && RW_files(RWtest).name ~= ".DS_Store")
%
%         % -- within the test number folder, get the participant numbers
%         RWsubDir = strcat(parentDir_RW, "/", RW_files(RWtest).name);
%         RWparticipant_folders = dir(RWsubDir);
%
%         % -- begin looping through all the participant folders
%         for RWparticipant = 1:size(RWparticipant_folders, 1)
%             % -- make sure that we capture a number not '.' or '..'
%             if (RWparticipant_folders(RWparticipant).name ~= "." && RWparticipant_folders(RWparticipant).name ~= ".." && RWparticipant_folders(RWparticipant).name ~= ".DS_Store")
%
%                 % -- create participant directory
%                 RWpartDir = strcat(RWsubDir,"/",RWparticipant_folders(RWparticipant).name);
%
%                 % -- begin looping through all conditions tested
%                 for cond = 1:size(conditions, 2)
%
%                     % -- create condition directory
%                     RWcondDir = strcat(RWpartDir, "/", conditions(cond));
%                     RWcondFile = strcat(RWcondDir, "/", Matfile);
%
%                     % -- read the data captured for the test
%                     RWTestData = load(RWcondFile);
%
%                     % -- create the figure
%                     figure(1); clf; gcf;
%
%                     % -- loop through the simulation
%                     for k = 1:RWTestData.simdata.time
%                         % -- display the Omron Lab map
%                         subplot(2,2,[1,2]);
%                         hold on; imagesc([0 L(1)],[0 L(2)], RWTestData.img);
%                         set(gca,'ydir','reverse');
%                         axis image;
%                         xlim([0 L(1)]);
%                         ylim([0 L(2)]);
%
%                         % -- plot the target location
%                         plot(RWTestData.simdata.Xs(4,1,1,1), RWTestData.simdata.Xs(5,1,1,1),...
%                             "ms", 'MarkerSize',16, 'LineWidth',3);
%
%                         for r = 1:3
%                             subplot(2,2,[1,2]);
%                             % -- plot the position of the robots in the domain
%                             hold on; plot(RWTestData.simdata.Xs(1,k,1,r), RWTestData.simdata.Xs(2,k,1,r), ...
%                                 r_pos(r), 'MarkerSize',36); % -- ref robot position
%                             %                             hold on; plot(RWTestData.simdata.Xs(1,k,1,2), RWTestData.simdata.Xs(2,k,1,2), ...
%                             %                                           'r.', 'MarkerSize',36); % -- Auto 1 robot position
%                             %                             hold on; plot(RWTestData.simdata.Xs(1,k,1,3), RWTestData.simdata.Xs(2,k,1,3), ...
%                             %                                           'g.', 'MarkerSize',36); % -- Auto 2 robot position
%                             % -- plot the orientation vector
%                             line([RWTestData.simdata.Xs(1,k,1,r),...
%                                 RWTestData.simdata.Xs(1,k,1,r) + 3*cos(RWTestData.simdata.Xs(3,k,1,r)) ],...
%                                 [RWTestData.simdata.Xs(2,k,1,r), ...
%                                 RWTestData.simdata.Xs(2,k,1,r)+3*sin(RWTestData.simdata.Xs(3,k,1,r)) ],...
%                                 'color',self_color(r),'linestyle','-');
%                             ax = gca;
%                             ax.FontSize = 18;
%
%                             % -- In the next subplot, plot the velocities of the robots over time
%                             subplot(2,2,3);
%                             hold on; plot(RWTestData.simdata.vel(1,1:k,1,r), ...
%                                 r_vel(r), 'LineWidth',2); % -- ref robot velocity
%                             %                             hold on; plot(RWTestData.simdata.vel(1,1:k,1,2), ...
%                             %                                           'r-', 'LineWidth',2); % -- Auto 1 robot velocity
%                             %                             hold on; plot(RWTestData.simdata.vel(1,1:k,1,3), ...
%                             %                                           'g-', 'LineWidth',2); % -- Auto 2 robot velocity
%                             xlabel('Time step');
%                             ylabel('Velocity (m/s)');
%                             ax = gca;
%                             ax.FontSize = 18;
%
%                             % -- In the next subplot, plot the turn rates of the robots over time
%                             subplot(2,2,4);
%                             hold on; plot(RWTestData.simdata.omega(1,1:k,1,r), ...
%                                 r_vel(r), 'LineWidth',2); % -- ref robot turn rate
%                             %                             hold on; plot(RWTestData.simdata.omega(1,1:k,1,2), ...
%                             %                                           'r-', 'LineWidth',2); % -- Auto 1 robot turn rate
%                             %                             hold on; plot(RWTestData.simdata.omega(1,1:k,1,3), ...
%                             %                                           'g-', 'LineWidth',2); % -- Auto 2 robot turn rate
%
%                             xlabel('Time step');
%                             ylabel('Turn rate (rad/s)');
%                         end
%                         ax = gca;
%                         ax.FontSize = 18;
%                         drawnow;
%                         pause(0.1);
%                     end
%
%                 end
%                 i = i + 1;
%             end
%         end
%     end
% end
% end

% function GetAverageVelocityExp()
%
% % -- Define folder locations where data is kept
% loc = "data/FILTERED/";
%
% % -- get the ID list
% ID_Data=csvread('../src/IDList_Completed.csv',1,0);
%
% % -- define the Velocity file name for each condition
% condfiles = ["/EKFVel_condition_1.csv",...
%     "/EKFVel_condition_2.csv",...
%     "/EKFVel_condition_3.csv",...
%     "/EKFVel_condition_4.csv"];
%
% % -- create 1D array to contain all velocity data for all participants
% % -- and conditions ran during the experiment
% Velocity = zeros(1, size(condfiles, 2)*size(ID_Data,1));
%
% i = 1; % -- counter
%
% % -- loop through all participants
% for participant = 1:size(ID_Data,1)
%     % -- loop through each condition
%     for cond = 1:size(condfiles, 2)
%         % -- create str that points to the csv file where data is stored
%         VelFile = strcat(loc, num2str(ID_Data(participant,1)), condfiles(cond));
%
%         % -- pull the data
%         VelData = csvread(VelFile);
%
%         % -- collect the averaged out veloctiy data
%         Velocity(i) = mean(VelData);
%
%         i = i + 1;
%     end
% end
%
% % -- get the total average velocity data
% TotalMeanVel = mean(Velocity);
%
% % -- plot the average teleoperated velocity distributions
% %figure(1); clf; gcf;
%
% %scatter(Velocity);
%
%
% end


