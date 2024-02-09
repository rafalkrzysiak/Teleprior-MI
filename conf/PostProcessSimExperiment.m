function PostProcessSimExperiment()

% -- This function will serve as a mediator of other functions to
% -- display data saved from the simulation infoseek2Db_multi_robot.m which
% -- imports experimental data captured using the overhead tracking system
% -- from the NIU Omron Lab.
% -- Written by: Rafal Krzysiak

% -- clear verything prior to running this script
clear variables

% -- plot all trajectories
PlotTrajTestDataSet();

end


function PlotTrajTestDataSet()

% -- define the conditions tested

cond_names=["xMxT","xMyT","yMxT","yMyT"];
cond_nums=["condition_1","condition_2","condition_3","condition_4"];

% -- define the mat file name used for all participants
Matfile = "OmronLab_p=1200_nsim=1_agents=1.mat";

% condition number
cond_type=4;

% -- data to plot
simDir = strcat("./data/", cond_names{cond_type});
expDir = "../data/FILTERED";

files = dir(simDir);

% remove all hidden files
files=files(~ismember({files.name},{'.','..', '.DS_Store'}));

% -- define the domain size
L= [18 9];

figure(1); gcf;clf;

% -- begin looping through each test folder
for ii = 1:size(files, 1)
%     subplot(5,6,ii);
    figure(ii); gcf; clf;
    
    % -- read condition directory
    condFile = strcat(simDir, "/", files(ii).name, "/", Matfile);
    
    trajFile=strcat(expDir,"/", files(ii).name,"/", "EKFtraj_", ...
        cond_nums{cond_type}, ".csv");
    
    trajData=csvread(trajFile);
    
    % -- read the data captured for the test
    TestData = load(condFile);
    
    % -- get the end time
    tf = TestData.simdata.time;
    
    
    % -- real data
    % -- plot everything nicely
    cla;
    % -- display the Omron Lab map
    hold on; imagesc([0 L(1)],[0 L(2)], TestData.img);
    set(gca,'ydir','reverse');
    axis image;
    
    % -- plot the target location
    hold on; plot(TestData.simdata.tloc(1, 1), ...
        TestData.simdata.tloc(1, 2), ...
        'm*', 'LineWidth', 3, 'MarkerSize', 12); % -- Target location
    
    % -- plot the beginning and end points of the robots
    hold on; plot(TestData.simdata.Xs(1, 1, 1, 1), ...
        TestData.simdata.Xs(2, 1, 1, 1), ...
        'ks', 'LineWidth', 3, 'MarkerSize', 12); % -- experiment robot (human controlled robot start)
    
    hold on; plot(TestData.simdata.Xs(1, tf, 1, 1), ...
        TestData.simdata.Xs(2, tf, 1, 1), ...
        'ko', 'LineWidth', 3, 'MarkerSize', 12); % -- experiment robot (human controlled robot end)
    
    
    % -- On top of the domain map, plot all trajectories of the robots
    hold on; plot(TestData.simdata.Xs(1, 1:tf, 1, 1), ...
        TestData.simdata.Xs(2, 1:tf, 1, 1), ...
        'r:', 'LineWidth', 2); % -- experiment robot (human controlled robot trajectory)
    
    % -- show trajectory data of human
    plot(trajData(1,:), trajData(2,:), 'k-', 'LineWidth', 2);
    txy=ginput(1);
    fprintf('%s, %.2f, %.2f\n',files(ii).name, txy(1), txy(2)); 
end

end




