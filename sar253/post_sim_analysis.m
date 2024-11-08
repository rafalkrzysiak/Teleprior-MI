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



