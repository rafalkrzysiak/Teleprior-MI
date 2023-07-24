function PostProcessSimExperiment()

% -- This function will serve as a mediator of other functions to
% -- display data saved from the simulation infoseek2Db_multi_robot.m which 
% -- imports experimental data captured using the overhead tracking system 
% -- from the NIU Omron Lab.
% -- Written by: Rafal Krzysiak

% -- clear verything prior to running this script
clear all variables

% -- import the data saved
SimExp = ImportAllData();

% -- plot which robot found the missing target
% PlotWhoFoundTarget(SimExp);

% -- plot all trajectories 
PlotAllTraj(SimExp)
<<<<<<< HEAD

% -- plot all the timeseries alpha values
% PlotTransientAlpha(SimExp);
=======
>>>>>>> parent of 36ac426 (Data captured)

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

