clear variables
ID_Data = csvread('./IDList_Completed.csv',1);

Ns=size(ID_Data,1);

% Load the omron lab mosaic 
OmronLabMap = imread('../data/maps/OmronLabMosaicCrop_lowres.jpg');

nr=5; % number of rows
nc=4; % number of conditions

for ii = 1:size(ID_Data, 1)
    % -- create figure dedicated to an individual ID
    % -- and create a tiled layout of 1x4
    figure(ii); gcf; clf;% -- contains fig num 1 -> # participants
    
    % -- loop through each of the conditions
    for Condition = 1:4
        subplot(nr,nc,Condition)
        % -- create the string that corresponds to the name of the file
        % -- that contains the trajectory data
        EKFtrajFile = strcat('../data/FILTERED/', num2str(ID_Data(ii,1)), ...
            '/EKFtraj_condition_', num2str(Condition), '.csv');
        
        % -- load the file that contains the trajectory data
        X = load(EKFtrajFile);
        
        % -- begin plotting the data
%             nexttile
        imagesc([-1 15],[-1 7.5], flip(OmronLabMap));
        set(gca,'xdir','reverse','ydir','reverse');
        hold on; plot(X(1,1), X(2,1), 'bs', 'markersize', 6, 'linewidth', 3); % -- starting point
        plot(X(1,end), X(2,end), 'bx', 'markersize', 6, 'linewidth', 3); % -- end point
        plot(X(1,:), X(2,:), 'b-', 'linewidth', 3); % -- trajectory
        plot(X(4,1), X(5,1), 'rs', 'markersize', 6, 'linewidth', 3); % -- Target location
        axis image;
        
        subplot(nr,nc,Condition+nc*1)
        EKFspeedFile = strcat('../data/FILTERED/', num2str(ID_Data(ii,1)), ...
            '/EKFVel_condition_', num2str(Condition), '.csv');
        SP = load(EKFspeedFile);
        dt = 0.5;
        time = 0:dt:size(SP,1)*dt;
        plot(time(1:end-1), SP, 'b-', 'linewidth', 2);
        ylabel('Speed (m/s)');
        xlabel('time (s)');
        
        
        subplot(nr,nc,Condition+nc*2)
        EKFomFile = strcat('../data/FILTERED/', num2str(ID_Data(ii,1)), ...
            '/EKFom_condition_', num2str(Condition), '.csv');
        OM = load(EKFomFile);
        dt = 0.5;
        time = 0:dt:size(OM,1)*dt;
        plot(time(1:end-1), OM, 'b-', 'linewidth', 2);
        ylabel('Turn rate (rad/s)');
        xlabel('time (s)');
        
        drawnow;
    end
    set(gcf, 'position', [54, 511, 1681, 441]);
%     print('-dpng', ['./stats data/', num2str(ID_Data(ii,1)), '_traj.png']);
end