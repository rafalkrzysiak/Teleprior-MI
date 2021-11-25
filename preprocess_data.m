% This will take raw trajectory data, filter it, store secondary measures
% as follows:
% divide filtered condition 4 data into 4a (prior to u turn) and 4b
% likely we'll have 15 4b trials with actual data the rest could be empty
% 1) EKFSpeed
% 2) EKFOmega
% 3) comSpeed
% 4) comOmega
% 5) timeToFind (no scaling)
% 6) pathLength (add successive differences in positions)
% 7) timeStayingInPlace (add the total number of timesteps where speed was
% less than 0.1 m/s)
% 7b) percent of timeStayingInPlace
% 8) commandedAcceleration
% such as speed, turn rates, times, etc. into separate files, 
% all in the subject's folder for later analysis
% revise approach to clip condition 4 into a and b
% plots of trajectories per condition. split condition 4 into 4a and b

function preprocess_data

% -- this function will serve as a main script to use 
% -- that will handle the individual functions that will
% -- analyze the data captured by the tracking system
% -- as well as the control input from participants
% -- Written By: Rafal Krzysiak
% -- Advisor: Dr. SAchit Butail

% -- before running any functions for analysis, clear everything
clc; clear all;

% -- vairables to pass into the functions
PartialRAW = 1;

% -- begin reading from the IDlist file
ID_location = 'IDList_Completed.csv';
ID_Data = csvread(ID_location,1);
ID_List = ID_Data(:,1);
ID_conditions = ID_Data(:,2);

% -- Remember: on the tracking computer, the data for the SAR
%              conditions begins with "0002" and ends with "0005"
%              While onboard RPi data starts with "1" and ends with "4"
trials = ["0002","0003","0004","0005"];
conditions = ["1","2","3","4"];

% -- functions for post trial analysis
% FilterAll(ID_List, trials, conditions, ID_conditions, PartialRAW, ID_Data);
% PlotAllSpeeds(ID_List, trials, conditions, ID_conditions, ID_Data);
% PlotAllTurnRates(ID_List, trials, conditions, ID_conditions);
timeToFind(ID_List, trials, conditions, ID_conditions, ID_Data);
% CompareSpeed(ID_List, trials, conditions, ID_conditions, ID_Data);
% CompareTurnRates(ID_List, trials, conditions, ID_conditions, ID_Data);
% StoreTimeRM(ID_List, trials, conditions, ID_conditions, ID_Data);
% FindTotalPathLength(ID_List, trials, conditions, ID_conditions, ID_Data);
% timeStayingInPlace(ID_List, trials, conditions, ID_conditions, ID_Data);
% DensityTrajMap(ID_List, trials, conditions, ID_conditions, ID_Data);
% CommandedAcceleration(ID_List, trials, conditions, ID_conditions, ID_Data);
% PlotTrajectoryWithAllInfo(ID_List, trials, conditions, ID_conditions, ID_Data); % -- function used at the end to display everything for individual participants
% SaveAllSpeeds(ID_List, trials, conditions, ID_conditions, ID_Data);
% SaveAllTurnrates(ID_List, trials, conditions, ID_conditions, ID_Data);

end

function FilterAll(ID_List, trials, conditions, ID_conditions, PartialRAW, ID_Data)
Fig_suplot = 1;
cond_count = 1;

tiledlayout(size(trials,2),size(ID_List,1));
% tiledlayout(1,size(ID_List,1));
% tiledlayout(4, 4);

% -- loop through every single ID number in the list
for TRIAL = 1:size(trials,2)
    trial = num2str(trials(TRIAL));
    condition = num2str(conditions(TRIAL));
    
    for ID = 1:size(ID_List,1)
        % -- read the data from the csv file
        participantID = num2str(ID_List(ID));
        
        DataFile = strcat(participantID,'/trial_',trial,'/data.csv');
        onBoard = strcat(participantID,'/condition_',condition,'/WheelVel.csv');

        data = csvread(DataFile, 2);
        RpiData = csvread(onBoard);

        [x, y, z, Rx, Ry, Rz, tx, ty, CamID, time, Vel, omega] = ReadData(data, RpiData);
        
        % -- do interp1 smoothing before EKF
        Tq = 0:0.5:time(end);
        X = x(x ~= 0); Y = y(x ~=0);
        RZ = Rz(x ~= 0);
        T_trials = time(x ~= 0);
        Xq = interp1(T_trials, X, Tq);
        Yq = interp1(T_trials, Y, Tq);
        Rzq = interp1(T_trials, RZ, Tq);
        
        idx = ~isnan(Xq);
        Xq = Xq(idx); Yq = Yq(idx); Rzq = Rzq(idx);

        % -- ground truth of what the tracker saw the robot do
        gt = [Xq; Yq; Rzq];
        
        EKF_Vel = zeros(size(gt,1), 1);
        EKF_om = zeros(size(gt,1), 1);
        
        Z = gt; % -- for consistency, make gt the measurement from the tracker
        X0(1:3,1)=gt(1,1:3);

        n=size(X0,1); % number of states for the motion model dim = 3
        m=size(Z,1); % number of states for the measurement model dim = 3
        T=size(Z,2); % number of time steps
        Zt=zeros(m,T);

        Xh=zeros(n,T); % Create the predicted state of the motion of the robot
        Xh_=Xh; % Create the predicted future state of the motion of the robot
        P=zeros(n,n,T); P_=P; % Create the prediction robot
        P0=eye(3)*1; % ^^ initial covariance

        % kalman filter matrices
        % >> constant velocity motion model and its ^^ process noise 
        % Eq. 6.2.2-13 Bar-Shalom, RongLi, Kirubarajan, Estimation with
        % Applications to Tracking and Navigation

%         v=0.5; omega=.1;
%         Ffun=@(X,dt) unicycle1(X,v,omega, dt);
%         Flinfun=@(X,dt) unicycle_lin(X,v, dt); 

        Qk= @(x,dt) [cos(x(3))*dt 0; 
                     sin(x(3))*dt 0; 
                     0 dt]*...
                    [1 0; 
                     0 .1]*...
                    [cos(x(3))*dt 0; 
                     sin(x(3))*dt 0; 
                     0 dt]';

        % >> measurement model and ^^ noise
        % this one is simply identity
        Hfun=@(X) eye(3)*X;

        % because Hfun is linear, this is simply the same function
        Hlinfun=@(X) eye(3); 

        % -- noise of the tracker
%         Rk=diag([.0418^2 .0418^2 .0277^2]*1); 
        Rk=diag([.0418 .0418 .0277]*1); 
        %Qk = Rk; % -- how to calculate Qk?

        k0=1;
        kF=T;

        % mean
        Xh_(:,k0)=X0;

        % covariance
        P_(:,:,k0, 1)=P0;
        t0 = 0;
        v0 = 0;
        om0 = 0;

        % kalman filter
        for k=k0:kF
            % -- update time step
            dt = Tq(k) - t0; % -- get the time difference between timestamps
            t0 = Tq(k); % -- get the current timestamp from tracker

            % -- we only want to update the estimate if we have a measurement
            if Z(1,k)
                if k > 1
                    Z(3,k) = atan2((Z(2,k) - Z(2,k-1)), (Z(1,k) - Z(1,k-1)));
                end
                % -- update function for Extended Kalman Filter
                [Xh(:,k), P(:,:,k)]=ekf_update(Xh_(:,k), ...
                    P_(:,:,k), Rk, Z(:,k), Hfun, Hlinfun);
            else
                % -- because there is no measurement, set dt=0.5s 
                % -- and update the position estimate to be the predicted one
                dt = 0.5;
                t0 = time(k);

                Xh(:,k) = Xh_(:,k);
                P(:,:,k) = P_(:,:,k);
            end
            
            timestep = find(round(Tq(k),1) == round(RpiData(:,1),1));
            if timestep
                v = Vel(timestep(1));
                om = omega(timestep(1));
                v0 = v;
                om0 = om;
            else
                v=v0;
                om=om0;
        %         sprintf('time: %.3f', time(k))
            end
    
            Ffun=@(X,dt) unicycle1(X,v,om,dt);
            Flinfun=@(X,dt) unicycle_lin(X,v,dt);
            
            % -- do a velocity and turn rate calculation using the
            % -- predicted values of position and orientation from EKF
            if k > 1
                vx = (Xh(1,k) - Xh(1,k-1)) / dt;
                vy = (Xh(2,k) - Xh(2,k-1)) / dt;
                
                EKF_Vel(k) = sqrt(vx^2 + vy^2);
                EKF_om(k) = (Xh(3,k) - Xh(3,k-1)) / dt;
            end

            % -- predict
            [Xh_(:,k+1), P_(:,:,k+1)]= ekf_predict(Xh(:,k), ...
                P(:,:,k), Qk(Xh(:,k),dt), Ffun, Flinfun, dt);

            Zt(:,k)=Hfun(Z(:,k));
        end
        
        % -- before we begin to plot everything, make sure to save all
        % -- the EKF velocities and turn rates into a csv file to be used later
        directory_str = strcat('filtered_data/', participantID);
        directory = mkdir(directory_str);
        EKFVelFile = strcat(directory_str, '/EKFVel_condition_', num2str(TRIAL), '.csv');
        EKFomFile = strcat(directory_str, '/EKFom_condition_', num2str(TRIAL), '.csv');
        EKFtrajFile = strcat(directory_str, '/EKFtraj_condition_', num2str(TRIAL), '.csv');
        csvwrite(EKFVelFile, EKF_Vel);
        csvwrite(EKFomFile, EKF_om);
        csvwrite(EKFtrajFile, [Xh; ones(1,size(Xh,2))*tx(end); ones(1,size(Xh,2))*ty(end)]);
        
        if PartialRAW
            newTime = time(x ~= 0);
            x = x(x ~= 0); y = y(y ~= 0);
        else
            newTime = time;
        end

        % -- plot the data and estimates
        %figure(1); gcf;
%         nexttile
%         
%         % -- we want to plot the raw data vs time without the zeros to show gaps
%         % -- then over lay the smoothed EKF on it as well
%         plot(newTime, x, 'r-', 'linewidth', 3);
%         hold on; plot(Tq(1:size(Xh(1,:),2)), Xh(1,:), 'b.-', 'linewidth', 1);
%         set(gca,'xtick',[0:round(Tq(end)/4, -1):Tq(end)])
%         grid on;
%         axis([0 newTime(end) 0 18]);
        
%         if TRIAL < 4
%             subplot(size(trials,2)+1,size(ID_List,1),Fig_suplot);
%         end
        
        % -- looking if its break turst or not
%         if TRIAL == 4
%             if ID_conditions(ID) == 0 % -- keep trust
%                 subplot(size(trials,2)+1,size(ID_List,1),Fig_suplot);
%             else % -- break trust
%                 subplot(size(trials,2)+1,size(ID_List,1),Fig_suplot+size(ID_List,1));
%             end
%         end

        lw = 2;
        ms = 8;
%         OmronLabMap = imread('maps/OmronLabMosaicCrop_lowres.jpg');
%         imagesc([0 15],[-1 7.5], flip(OmronLabMap));
%         if trial == "0002"
%             OmronLabMap = imread('maps/cond1_mosaic.png');
%             imagesc([-1 15],[-1 7.5], flip(OmronLabMap));
%         else
%             OmronLabMap = imread('maps/OmronLabMosaicCrop.png');
%             imagesc([0 14.7],[0 7], flip(OmronLabMap));
%         end
%         set(gca,'xdir','reverse','ydir','reverse');
        % -- EKF data
%         hold on; plot(Xq, Yq, 'linewidth', lw);
%         hold on; plot(Xh(1,:), Xh(2,:), 'linewidth', lw);
%         hold on; plot(Xh(1,1), Xh(2,1), 'bs', 'markersize', ms, 'linewidth', lw);
%         hold on; plot(Xh(1,end), Xh(2,end), 'bx', 'markersize', ms, 'linewidth', lw);
%         hold on; plot(tx(end), ty(end), 'rs', 'markersize', ms, 'linewidth', lw);
%         hold on; plot(Xh(1,ID_Data(ID, 5)), Xh(2,ID_Data(ID, 5)),'k.','markersize', 18);
        %hold on; plot(Zt(1,:), Zt(2,:), 'b*', 'markersize', ms);
        
        % -- RAW data
%         hold on; plot(X, Y, 'r-', 'linewidth', lw);
%         hold on; plot(X(1), Y(1), 'bs', 'markersize', ms, 'linewidth', lw);
%         hold on; plot(X(end), Y(end), 'bx', 'markersize', ms, 'linewidth', lw);
%         hold on; plot(tx(end), ty(end), 'rs', 'markersize', ms, 'linewidth', lw);
        
        % -- label the y axis of the entire set to say the condition number
%         if Fig_suplot == (size(ID_List,1)*str2double(condition) - (size(ID_List,1)+1))
%             ylabel(sprintf('condition: %d \n Y (m)', cond_count), 'fontweight', 'normal', 'fontsize', 10);
%             cond_count = cond_count + 1;
%         end
%         
%         if condition == '4'
%             xlabel('Time (s)', 'fontweight', 'normal', 'fontsize', 16);
%         end
%         
%         if ID == 1
%             ylabel('X (m)', 'fontweight', 'normal', 'fontsize', 10);
%         end
        
%         if condition == '1' && ID == size(ID_List,1)/2
%            title(sprintf('Trajectory \n Time: %.2f s', time(end)), 'fontweight', 'normal', 'fontsize', 8); 
%            
%         else
%             title(sprintf('Time: %.2f s', time(end)), 'fontweight', 'normal', 'fontsize', 8); 
%         end
%         axis image;
        %legend('RAW Trajectory', 'EKF trajectory', 'Robot start', 'Robot End', 'Target location');
%         ax = gca;
%         ax.FontSize = 16;
%         Fig_suplot = Fig_suplot + 1;
    end
end

end

function X = unicycle1(X,v,omega,dt)
X(1,1) = X(1,1) + v*cos(X(3,1))*dt;
X(2,1) = X(2,1) + v*sin(X(3,1))*dt;
X(3,1) = X(3,1) + omega*dt;
end

function flin = unicycle_lin(X, v, dt)

flin=[1 0 -v*sin(X(3,1)*dt);
      0 1 v*cos(X(3,1)*dt);
      0 0 1];
end

function [x, y, z, Rx, Ry, Rz, tx, ty, ID, time, Vel, omega] = ReadData(data, RpiData)
% -- get the robot data from the tarcker system
ID = data(:,1); camID = data(:,2); %time = data(:,3);
x = data(:,4); y = data(:,5); z = data(:,6); 
Rx = data(:,7); Ry = data(:,8); Rz = data(:,9); 
tx = data(:,13); ty = data(:,14); time = data(:,12);

% -- get the velocity and turn rate of the robot
% -- the saved wheel speeds from the iRobot are in mm
% -- make sure to convert them to m before calculating v and omega
Vel = ((RpiData(:,2) + RpiData(:,3))/ 1000) / 2;
omega = ((RpiData(:,2) - RpiData(:,3))/ 1000) / .3084;

% -- omit the zeros
% x = x(x~=0); y = y(y~=0); z = z(z~=0); time = time(time~=0);
% Rz = Rz(Rz~=0); Rx = Rx(Rx~=0); Ry = Ry(Ry~=0);
% tx = tx(tx~=0); ty = ty(ty~=0);
end

function PlotAllSpeeds(ID_List, trials, conditions, ID_conditions, ID_Data)

% -- start the subplot counter
Fig_suplot = 1;
cond_count = 1;
tiledlayout(size(trials,2)+1,size(ID_List,1));

% -- hold all mean velocity values
AllVel = zeros(size(ID_List,1), 5);

% -- create cell array to contain all time/vel data
VelCel = {};
CelColumn = 1;

% -- variables to hold the linewdith and markersize
lw = 2;
ms = 8;

% -- loop through every single ID number in the list
for TRIAL = 1:size(conditions,2)
    
    % -- store the condition value
    condition = num2str(conditions(TRIAL));
    
    for ID = 1:size(ID_List,1)
        
        % -- read the data from the csv file
        participantID = num2str(ID_List(ID));
        onBoard = strcat('RAW\',participantID,'/condition_',condition,'/WheelVel.csv');

        % -- store the data
        RpiData = csvread(onBoard);
        Vel = ((RpiData(:,2) + RpiData(:,3))/ 1000) / 2;
        Vel = abs(Vel);
        time = RpiData(:,1);
        
        % -- Initialize the subplot to be used
        nexttile
%         figure(1); gcf;
%         
%         if TRIAL < 4
%             subplot(size(trials,2)+1,size(ID_List,1),Fig_suplot);
%         end
%         
%         % -- looking if its break turst or not
%         if TRIAL == 4
%             if ID_conditions(ID) == 0 % -- keep trust
%                 subplot(size(trials,2)+1,size(ID_List,1),Fig_suplot);
%             else % -- break trust
%                 subplot(size(trials,2)+1,size(ID_List,1),Fig_suplot+size(ID_List,1));
%             end
%         end
        plot(time, Vel, 'k-', 'linewidth', lw); % -- plot the RAW velocity
        hold on; plot(time, mean(Vel)*ones(size(time)), 'r-', 'linewidth', lw); % -- plot the mean velocity
        
        % -- save the mean vel
        if TRIAL < 4
            % -- for all trials before the last condition
            AllVel(ID, TRIAL) = mean(Vel);
            VelCel{1,CelColumn} = {[[nan; TRIAL; time], [nan; ID_List(ID); Vel]]};
        else
            % -- for the last condition, if keep trust
            if ID_conditions(ID) == 0
                AllVel(ID, TRIAL) = mean(Vel(1:ID_Data(ID, end)*5));
                AllVel(ID, TRIAL+1) = nan;
                VelCel{1,CelColumn} = {[[0; TRIAL; time], [0; ID_List(ID); Vel]]};
            else
                % -- if break trust
                AllVel(ID, TRIAL) = nan;
                AllVel(ID, TRIAL+1) = mean(Vel(ID_Data(ID, end)*5:end));
                VelCel{1,CelColumn} = {[[1; TRIAL; time], [1; ID_List(ID); Vel]]};
            end
        end
        
        CelColumn = CelColumn + 1;
        
        % -- label the y axis of the entire set to say the condition number
        if ID == 1
            ylabel('Velocity (m/s)', 'fontweight', 'normal', 'fontsize', 16);
        end
        
%         if Fig_suplot == (size(ID_List,1)*str2double(condition) - 17)
%             ylabel(sprintf('condition: %d \n Velocity (m/s)', cond_count), 'fontweight', 'normal', 'fontsize', 16);
%             cond_count = cond_count + 1;
%         end
        
        if condition == '4'
            xlabel('Time (s)', 'fontweight', 'normal', 'fontsize', 16)
        end
        
        title(sprintf('Mean Vel: %.2f m/s', mean(Vel)), 'fontweight', 'normal', 'fontsize', 12);
        ax = gca;
        ax.FontSize = 12;
        
%         if condition == '1' && ID == size(ID_List,1)/2
%            title(sprintf('Velocity VS Time \n Mean Vel: %.2f m/s', mean(Vel)), 'fontweight', 'normal', 'fontsize', 12); 
%            
%         else
%             title(sprintf('Mean \n Vel: %.2f m/s', mean(Vel)), 'fontweight', 'normal', 'fontsize', 12); 
%         end
        
        % -- increment the subplot counter
        Fig_suplot = Fig_suplot + 1;
    end
end

anova1(AllVel(:,2:end));
set(gca, 'xticklabel', {'2', '3', '4a', '4b'});
xlabel('Trial Conditions', 'fontweight','normal', 'fontsize', 16);
ylabel('Velocity (m/s)', 'fontweight','normal', 'fontsize', 16);

end

function PlotAllTurnRates(ID_List, trials, conditions, ID_conditions)

% -- start the subplot counter
Fig_suplot = 1;
cond_count = 1;
tiledlayout(size(trials,2)+1,size(ID_List,1));

% -- hold all mean velocity values
Allomega = zeros(size(ID_List,1), 5);

% -- variables to hold the linewdith and markersize
lw = 2;
ms = 8;

% -- loop through every single ID number in the list
for TRIAL = 1:size(conditions,2)
    
    % -- store the condition value
    condition = num2str(conditions(TRIAL));
    
    for ID = 1:size(ID_List,1)
        
        % -- read the data from the csv file
        participantID = num2str(ID_List(ID));
        onBoard = strcat(participantID,'/condition_',condition,'/WheelVel.csv');

        % -- store the data
        RpiData = csvread(onBoard);
        omega = ((RpiData(:,2) - RpiData(:,3))/ 1000) / .3084;
        time = RpiData(:,1);
        
        % -- Initialize the subplot to be used
        nexttile
        omega = abs(omega); % -- convert all turnrates to positive

        plot(time, omega, 'k-', 'linewidth', lw); % -- plot the RAW Trun rate
        hold on; plot(time, mean(omega)*ones(size(time)), 'r-', 'linewidth', lw); % -- plot the mean turn rate
        
        % -- save the mean turn rate
        if TRIAL < 4
            % -- save for all trials before the last condition
            Allomega(ID, TRIAL) = mean(omega);
        else
            % -- for the last condition, if keep trust
            if ID_conditions(ID) == 0
                Allomega(ID, TRIAL) = mean(omega);
                Allomega(ID, TRIAL+1) = nan;
            else
                % -- if break trust
                Allomega(ID, TRIAL) = nan;
                Allomega(ID, TRIAL+1) = mean(omega);
            end
        end
        
        % -- make the figures look nice
        ax = gca; 
        if condition == '4'
            xlabel('Time (s)', 'fontweight', 'normal', 'fontsize', 16);
        end
        
        if ID == 1
            ylabel('Turn rate (rad/s)', 'fontweight', 'normal', 'fontsize', 12);
        end
        title(sprintf('Mean Turn rate: %.2f rad/s', mean(omega)), 'fontweight', 'normal', 'fontsize', 12); 
        ax.FontSize = 12;
        
        % -- increment the subplot counter
        Fig_suplot = Fig_suplot + 1;
    end
end

end

function timeToFind(ID_List, trials, conditions, ID_conditions, ID_Data)

% -- hold all the time values
AllTimes = zeros(size(ID_List,1), size(trials,2)+1);

% -- scaling factor for condition 1
% -- when the participant drove across the whole lab
k_tf = 2.27;

% -- loop through every trial ran
for TRIAL = 1:size(trials,2)
    % -- convert the trial/condition variable into a number
    trial = num2str(trials(TRIAL));
    condition = num2str(conditions(TRIAL));
    
    % -- loop through all IDs 
    for ID = 1:size(ID_List,1)
        % -- read the data from the csv file
        participantID = num2str(ID_List(ID));
        
        % -- read the data from rpi and tracking computer
        DataFile = strcat('RAW/',participantID,'/trial_',trial,'/data.csv');
        onBoard = strcat('RAW/',participantID,'/condition_',condition,'/WheelVel.csv');
        data = csvread(DataFile, 2);
        RpiData = csvread(onBoard);
        [x, y, z, Rx, Ry, Rz, tx, ty, CamID, time, Vel, omega] = ReadData(data, RpiData);
        
        
        
        % -- for the split time array, only the last condition will be split
        % -- Condition 4 time split is at when the participant rotates 180 degrees
        if TRIAL == 4
            % -- for all trials at condition 4
            AllTimes(ID,TRIAL) = time(ID_Data(ID, end));
            
            % -- looking at only those who have been given incorrect target
            % -- location information
            if ID_Data(ID, 2)
                AllTimes(ID,TRIAL+1) = time(end) - time(ID_Data(ID, end));
            end
        else
            % -- for all trials before the last condition
            AllTimes(ID,TRIAL) = time(end);
        end
        
    end
end

% -- plot the time to find for each participant
figure(1); gcf; clf;

% -- loop through all participants
for ii = 1:size(ID_List,1)
    hold on;
    plot([1, 2, 3, 4], AllTimes(ii, 1:4));
end

% -- plot the mean and std of the times
hold on; errorbar([1,2,3,4], mean(AllTimes(:, 1:4)), std(AllTimes(:, 1:4)), 'linewidth', 3);

xlabel('Condition', 'fontsize', 12, 'fontweight', 'normal');
ylabel('Time to find (s)', 'fontsize', 12, 'fontweight', 'normal');

% -- save the times to find the target
csvwrite('stats data\TimeToFind.csv', AllTimes);

end

function CompareSpeeds(ID_List, trials, conditions, ID_conditions, ID_Data)

% -- start the subplot counter
Fig_suplot = 1;
cond_count = 1;
tiledlayout(size(trials,2),size(ID_List,1));

% -- hold all mean velocity values
AllVel = zeros(size(ID_List,1), size(trials,2));
AllEKFVel = AllVel;

% -- variables to hold the linewdith and markersize
lw = 2;
ms = 8;

tiledlayout(size(trials,2)+1,size(ID_List,1));

% -- loop through every single ID number in the list
for TRIAL = 1:size(conditions,2)
    
    % -- store the condition value
    condition = num2str(conditions(TRIAL));
    
    for ID = 1:size(ID_List,1)
        
        % -- read the data from the csv file
        participantID = num2str(ID_List(ID));
        onBoard = strcat('RAW/',participantID,'/condition_',conditions(TRIAL),'/WheelVel.csv');
        DataFile = strcat('RAW/',participantID,'/trial_',trials(TRIAL),'/data.csv');
        data = csvread(DataFile, 2);
        % -- store the data
        RpiData = csvread(onBoard);
        Vel = ((RpiData(:,2) + RpiData(:,3))/ 1000) / 2;
        time = RpiData(:,1);
        [x, y, z, Rx, Ry, Rz, tx, ty, CamID, TIME, ~, omega] = ReadData(data, RpiData);

        % -- read the EKF Velocity and turn rate data previously saved
        EKF_Vel_data = strcat('filtered_data/', participantID, '/EKFVel_condition_', num2str(TRIAL),'.csv');
        EKF_Vel = csvread(EKF_Vel_data);
        TIME = 0:0.5:(size(EKF_Vel,1)*.5)-0.5;
        t1 = size(time,1);
        t2 = size(TIME,2);
        rpiT = floor(t1/t2);
        EKF_Vel = abs(EKF_Vel);
        Vel = abs(Vel); % -- we only want the positive value of the speed

        % -- save the mean speed
        if TRIAL < 4
            % -- for all trials before the last condition
            AllVel(ID, TRIAL) = mean(Vel);
            AllEKFVel(ID, TRIAL) = mean(EKF_Vel);
        else
            % -- for the last condition
                AllVel(ID, TRIAL) = mean(Vel(1:rpiT*ID_Data(ID, end),1));
                AllEKFVel(ID, TRIAL) = mean(EKF_Vel(1:ID_Data(ID, end),1) );
        end
        
        % -- make the figures look nice
        ax = gca; 
        if condition == '4'
            xlabel('Time (s)', 'fontweight', 'normal', 'fontsize', 12);
        end
        
        if ID == 1
            ylabel({'Input';'speed (m/s)'}, 'fontweight', 'normal', 'fontsize', 12);
        end
        
        title(sprintf('Mean speed: %.2f m/s', mean(Vel)), 'fontweight', 'normal', 'fontsize', 12);
        ax.FontSize = 12;
        
        % -- increment the subplot counter
        Fig_suplot = Fig_suplot + 1;
    end
end

end

function CompareTurnRates(ID_List, trials, conditions, ID_conditions, ID_Data)

% -- start the subplot counter
Fig_suplot = 1;
cond_count = 1;
tiledlayout(size(trials,2)+1,size(ID_List,1));

% -- hold all mean velocity values
Allomega = zeros(size(ID_List,1), size(trials,2));
AllEKFomega = Allomega;

% -- variables to hold the linewdith and markersize
lw = 2;
ms = 8;

% -- loop through every single ID number in the list
for TRIAL = 1:size(conditions,2)
    
    % -- store the condition value
    condition = num2str(conditions(TRIAL));
    
    for ID = 1:size(ID_List,1)
        
        % -- read the data from the csv file
        participantID = num2str(ID_List(ID));
        onBoard = strcat('RAW/',participantID,'/condition_',conditions(TRIAL),'/WheelVel.csv');
        DataFile = strcat('RAW/',participantID,'/trial_',trials(TRIAL),'/data.csv');
        data = csvread(DataFile, 2);
        % -- store the data
        RpiData = csvread(onBoard);
        Vel = ((RpiData(:,2) + RpiData(:,3))/ 1000) / 2;
        time = RpiData(:,1);
        [x, y, z, Rx, Ry, Rz, tx, ty, CamID, TIME, Vel, omega] = ReadData(data, RpiData);
        

        % -- read the EKF Velocity and turn rate data previously saved
        EKF_om_data = strcat('filtered_data/', participantID, '/EKFom_condition_', num2str(TRIAL),'.csv');
        EKF_om = csvread(EKF_om_data);
        TIME = 0:0.5:(size(EKF_om,1)*.5)-0.5;
        t1 = size(time,1);
        t2 = size(TIME,2);
        rpiT = floor(t1/t2);
        
        % -- Initialize the subplot to be used
%         nexttile
        EKF_om = abs(EKF_om);
        omega = abs(omega);
%         figure(1); gcf;
%         
%         if TRIAL < 4
%             subplot(size(trials,2)+1,size(ID_List,1),Fig_suplot);
%         end
%         
%         % -- looking if its break turst or not
%         if TRIAL == 4
%             if ID_conditions(ID) == 0 % -- keep trust
%                 subplot(size(trials,2)+1,size(ID_List,1),Fig_suplot);
%             else % -- break trust
%                 subplot(size(trials,2)+1,size(ID_List,1),Fig_suplot+size(ID_List,1));
%             end
%         end
%         plot(time, omega, 'k-', 'linewidth', lw); % -- plot the RAW velocity
% %         hold on; plot(time, mean(omega)*ones(size(time)), 'r-', 'linewidth', lw); % -- plot the mean velocity
% %         hold on; plot(TIME, EKF_om, 'b-', 'linewidth', lw); % -- plot the EKF estimated velocity
% %         hold on; plot(TIME, mean(EKF_om)*ones(size(TIME)), 'c-', 'linewidth', lw); % -- plot the mean velocity of EKF
%         set(gca,'xtick',[0:round(time(end)/4, -1):time(end)])
%         axis([0 time(end) 0 2]);
%         set(gca,'ytick',[0:1:2]);
%         grid on;
        
        % -- save the mean vel
        if TRIAL < 4
            % -- for all trials before the last condition
            Allomega(ID, TRIAL) = mean(omega);
            AllEKFomega(ID, TRIAL) = mean(EKF_om(3:end, :));
        else
            % -- for the last condition, if keep trust
                % -- if break trust
                Allomega(ID, TRIAL) = mean(omega(1:rpiT*ID_Data(ID, end)));
                AllEKFomega(ID, TRIAL) = mean(EKF_om(3:ID_Data(ID, end), :));
        end
        
        % -- label the y axis of the entire set to say the condition number
%         if Fig_suplot == (size(ID_List,1)*str2double(condition) - 23)
%             ylabel(sprintf('condition: %d \n Turn Rate (rad/s)', cond_count), 'fontweight', 'normal', 'fontsize', 12);
%             cond_count = cond_count + 1;
%         end
        
        if condition == '4'
            xlabel('Time (s)', 'fontweight', 'normal', 'fontsize', 12)
        end
        
        if ID == 1
            ylabel({'Input turn';'rate (rad/s)'}, 'fontweight', 'normal', 'fontsize', 12);
        end
        
        title(sprintf('Mean Turn Rate: %.2f rad/s', mean(EKF_om)), 'fontweight', 'normal', 'fontsize', 12);
        

%         if condition == '1' & ID == size(ID_List,1)/2
%            title(sprintf('Turn Rate VS Time \n Mean \n Turn Rate: %.2f rad/s', mean(Vel)), 'fontweight', 'normal', 'fontsize', 8); 
%            
%         else
%             title(sprintf('Mean \n Turn Rate: %.2f rad/s', mean(Vel)), 'fontweight', 'normal', 'fontsize', 8); 
%         end
        
        % -- increment the subplot counter
        Fig_suplot = Fig_suplot + 1;
    end
end

% anova1(Allomega);
% set(gca, 'xticklabel', {'1', '2', '3', '4a', '4b'});
% xlabel('Trial Conditions', 'fontweight','normal', 'fontsize', 16);
% ylabel('Turn Rate (rad/s)', 'fontweight','normal', 'fontsize', 16);
% title('ANOVA on RPi Turn Rate', 'fontweight','normal', 'fontsize', 16);
% 
% anova1(AllEKFomega);
% set(gca, 'xticklabel', {'1', '2', '3', '4a', '4b'});
% xlabel('Trial Conditions', 'fontweight','normal', 'fontsize', 16);
% ylabel('Tracker Turn Rate (rad/s)', 'fontweight','normal', 'fontsize', 16);
% title('ANOVA on EKF Tracker Turn Rate', 'fontweight','normal', 'fontsize', 16);

end

function StoreTimeRM(ID_List, trials, conditions, ID_conditions, ID_Data)
% -- get the number of participants (sample size)
Ns = size(ID_Data,1);

% tiledlayout(1,size(ID_List,1));
tiledlayout(1,Ns);

% -- create matrix to contain the ID of the participant
% -- as well as the time that corresponds to the threshold distance
% -- that the participant has been at in condition 3
% -- top row: ID, bottom row: time split
SplitTime = zeros(2, Ns);
dist = zeros(1, Ns);

for TRIAL = 3:size(conditions,2)
    
    % -- store the condition value
    condition = num2str(conditions(TRIAL));
    
    for ID = 1:size(ID_List,1)
        % -- get the trajectory data from file
        participantID = num2str(ID_List(ID));
        EKF_traj_data = strcat('filtered_data/', participantID, '/EKFtraj_condition_', num2str(TRIAL),'.csv');
        X = csvread(EKF_traj_data);
        
        % -- first we look at trial 3 and get the average distance
        % -- that the participants were from the target at the
        % -- end of condition 3. This will then be used as the splitting
        % -- threshold for condition 4, for the participants that were
        % -- given incorrect target location information
        if TRIAL == 3
            % -- get the average distance between the target location
            % -- and participant location at the end of condition 3
            % -- get the dx and dy of the final positions
            dx = (X(4,end) - X(1,end));
            dy = (X(5,end) - X(2,end));
            dist(1,ID) = sqrt(dx^2 + dy^2);
            
        else % -- no if condition 4 is being looked at

            % -- looking at only the participants that were given incorrect
            % -- target location information
            if ID_Data(ID, 2)
                
                % -- When we give incorrect target location information to
                % -- the participants, we place the target in the opposite X
                % -- direction of the search environment. Because of this
                % -- we need to flip the x position only to where they 
                % -- initially believed the target to be located at.
                if X(4,end) > 8
                    X_t = 3.3;
                else
                    X_t = 12.9;
                end
                
                % -- loop through time of the experiment
                for timestep = 1:size(X,2)
                    % -- calculate the dx and dy between the target
                    % -- location and robot position
                    dx = (X_t - X(1,timestep));
                    dy = (X(5,5) - X(2,timestep));
                    Dist = sqrt(dx^2 + dy^2);
                    
                    % -- check if the distance between the robot and the
                    % -- target is within the threshold, break the loop and
                    % -- save the current atime at which it met the threshold
                    if Dist <= mean(dist)+1.5
                        SplitTime(1,ID) = ID_List(ID);
                        SplitTime(2,ID) = timestep;
                       break; 
                    end
                end
            else
                % -- get the end time for the participants given
                % -- correct target location information
                SplitTime(1,ID) = ID_List(ID);
                SplitTime(2,ID) = size(X,2);
            end
            
            % -- display the split
            EKF_X = movmean(X(1,:),15); % -- smooth the data with window size of 15
            EKF_Y = movmean(X(2,:),15); % -- smooth the data with window size of 15

            nexttile
            plot(EKF_X,'.', 'markersize', 8); disp(ID);
            hold on; plot(SplitTime(2,ID), EKF_X(1,SplitTime(2,ID)),'r.', 'markersize', 12);
            axis square; title(sprintf('%4d', ID_List(ID)),'fontsize', 12, 'fontweight', 'normal');
        end
    end
end

% -- save the data in a csv file
csvwrite('stats data\SplitTime.csv',SplitTime');

end

function FindTotalPathLength(ID_List, trials, conditions, ID_conditions, ID_Data)

% -- create a variable to contain the total distance
% -- that each participant drives during each of the trials
% -- with size (# trials x # participants)
TotalDistance = zeros(size(trials,2), size(ID_List,1));

% -- loop through every trial ran
for TRIAL = 1:size(trials,2)
    % -- convert the trial/condition variable into a number
    trial = num2str(trials(TRIAL));
    condition = num2str(conditions(TRIAL));
    
    % -- loop through all IDs 
    for ID = 1:size(ID_List,1)
        % -- read the data from the csv file
        participantID = num2str(ID_List(ID));
        
        % -- read the data from the filtered data folder
        % -- and store the data in a variable "X"
        % -- State X is 5xT matrix where T is total time
        dir = strcat("filtered_data/", participantID, "/EKFtraj_condition_", condition, ".csv");
        X = csvread(dir);
        
        % -- loop through the duration of the trial starting with t = 1
        for t = 2:size(X,2)
           % -- get the dx and dy of the position 
           % -- then calculate the norm of dx and dy
           dx = X(1,t) - X(1,t-1);
           dy = X(2,t) - X(2,t-1);
           TotalDistance(TRIAL, ID) = TotalDistance(TRIAL, ID) + sqrt(dx^2 + dy^2);
        end
        
    end
end

% -- once the total distance traveled for every participant 
% -- per trial is calculated, plot the distances traveled
% -- against every participant
figure(1); gcf; clf;
for participant = 1:size(ID_List,1)
    hold on; plot([1, 2, 3, 4], TotalDistance(:, participant), '.-', 'linewidth', 2, 'markersize', 2);
end

% -- Make the figure look nice
ax = gca;
axis([1 4 0 50]); grid on;
xlabel('Condition number'); ylabel('Total distance traveled (m)');
ax.XTickLabel = {'1', '', '2', '','3', '', '4'};
ax.FontSize = 18;

% -- save data to csv file
csvwrite('stats data/TotalDistanceTravel.csv',TotalDistance);

end

function timeStayingInPlace(ID_List, trials, conditions, ID_conditions, ID_Data)
% -- create a variable to contain the total time
% -- that each participant statyed in place during each of the trials
% -- with size (# trials x # participants)
TotalTimeInPlace = zeros(size(trials,2)+1, size(ID_List,1));

% -- if speed is < 0.1 m/s save the number of timesteps
% -- where 0.1 m/s is the threshhold
thresh = 0.1;

% -- time step dt
dt = 0.5;

% -- loop through every trial ran
for TRIAL = 1:size(trials,2)
    % -- convert the trial/condition variable into a number
    trial = num2str(trials(TRIAL));
    condition = num2str(conditions(TRIAL));
    
    % -- loop through all IDs 
    for ID = 1:size(ID_List,1)
        % -- read the data from the csv file
        participantID = num2str(ID_List(ID));
        
        % -- read the data from the filtered data folder
        % -- and store the data in a variable "X"
        % -- State X is 5xT matrix where T is total time
        dir = strcat("filtered_data/", participantID, "/EKFVel_condition_", condition, ".csv");
        X = csvread(dir);
        
        % -- check what condition we are looking at
        if TRIAL == 4
            % -- if condition 4b
            if ID_Data(ID, 2) % -- if break condition condition met
                for t = 1:size(X,1) % -- loop throughout the entire time
                    if t >= ID_Data(ID,end) % -- if the timestep is on or past the break trust point, add the timesteps
                        if X(t, 1) < thresh
                            TotalTimeInPlace(TRIAL+1, ID) = TotalTimeInPlace(TRIAL+1, ID) + 1;
                        end
                    else % -- if the timestep is prior to break trust point, add the time steps
                        if X(t, 1) < thresh
                            TotalTimeInPlace(TRIAL, ID) = TotalTimeInPlace(TRIAL, ID) + 1;
                        end
                    end
                end
                
            % -- otherwise, condition 4a
            else
                for t = 1:ID_Data(ID,end)
                    if X(t, 1) < thresh
                        TotalTimeInPlace(TRIAL, ID) = TotalTimeInPlace(TRIAL, ID) + 1;
                    end
                end
            end
            
        % -- conditions 1-3
        else
            % -- loop through the duration of the trial starting with t = 0
            for t = 1:size(X,1)
               if X(t, 1) < thresh
                   TotalTimeInPlace(TRIAL, ID) = TotalTimeInPlace(TRIAL, ID) + 1;
               end
            end
        end
        
    end
end

% -- convert from time steps to total time
TotalTimeInPlace = dt*TotalTimeInPlace;

% -- once the total time in place for every participant 
% -- per trial is calculated, plot the total time in place
% -- against every participant
figure(1); gcf; clf;
for participant = 1:size(ID_List,1)
    hold on; plot([1, 2, 3, 4], TotalTimeInPlace(1:4, participant), '.-', 'linewidth', 2, 'markersize', 2);
end

% -- Make the figure look nice
ax = gca;
axis([1 4 0 300]); grid on;
xlabel('Condition number'); ylabel('Total time stayed in place (s)');
ax.XTickLabel = {'1', '', '2', '','3', '', '4'};
ax.FontSize = 18;

% -- save the data
csvwrite('stats data\TotalTimeInPlace.csv', TotalTimeInPlace);

end

function DensityTrajMap(ID_List, trials, conditions, ID_conditions, ID_Data)

% -- get the number of participants
Ns=size(ID_Data,1);

% -- Load the omron lab mosaic 
OmronLabMap = imread('maps/OmronLabMosaicCrop_lowres.jpg');

% -- loop though all conditions and sub-conditions
for Condition = 1:4
   % -- create a figure that corresponds to the condition number
   figure(Condition+1); gcf; clf;
   
   % -- plot the Search environment
   imagesc([-0.25 15],[-0.5 7.5], flip(OmronLabMap));
   set(gca,'xdir','reverse','ydir','reverse');
   
   % -- 
   if Condition == 4
       % -- create another figure for condition 4
       figure(Condition+2); gcf; clf;

       % -- plot the Search environment
       imagesc([-0.25 15],[-0.5 7.5], flip(OmronLabMap));
       set(gca,'xdir','reverse','ydir','reverse');
   end
   
   % -- loop through all participants of the experiment
   for ii = 1:Ns
       
        % -- create the string that corresponds to the name of the file
        % -- that contains the trajectory data
        EKFtrajFile = strcat('filtered_data/', num2str(ID_Data(ii,1)), ...
            '/EKFtraj_condition_', num2str(Condition), '.csv');
        
        % -- load the file that contains the trajectory data
        X = load(EKFtrajFile);
       
       % -- check what condition it is
       % -- if the condition is not 4 we have to divide up the trajectory
       % -- otherwise just plot everything
       if Condition == 4
           if ID_Data(ii, 2)
               figure(Condition+2);
               % -- plot all data for condition 4b
               hold on; plot(X(1,ID_Data(ii, end)), X(2,ID_Data(ii, end)), 'bs', 'markersize', 10, 'linewidth', 3); % -- starting point
               plot(X(1,end), X(2,end), 'bx', 'markersize', 10, 'linewidth', 3); % -- end point
               plot(X(1,ID_Data(ii, end):end), X(2,ID_Data(ii, end):end), 'b-', 'linewidth', 2); % -- trajectory
               plot(X(4,5), X(5,5), 'rs', 'markersize', 10, 'linewidth', 3); % -- Target location
               axis image; title(sprintf('Condition: %db', Condition), 'fontsize', 18, 'fontweight', 'normal');
               
           end
           
           % -- we want to regardless plot all of condition 4a
           figure(Condition+1);
           % -- plot all data prior to condition 4b
           hold on; plot(X(1,1), X(2,1), 'bs', 'markersize', 10, 'linewidth', 3); % -- starting point
           plot(X(1,ID_Data(ii, end)), X(2,ID_Data(ii, end)), 'bx', 'markersize', 10, 'linewidth', 3); % -- end point
           plot(X(1,1:ID_Data(ii, end)), X(2,1:ID_Data(ii, end)), 'b-', 'linewidth', 2); % -- trajectory
           plot(X(4,5), X(5,5), 'rs', 'markersize', 10, 'linewidth', 3); % -- Target location
           axis image; title(sprintf('Condition: %da', Condition), 'fontsize', 18, 'fontweight', 'normal');
       else
           figure(Condition+1);
           % -- plot all data prior to condition 4
           hold on; plot(X(1,1), X(2,1), 'bs', 'markersize', 10, 'linewidth', 3); % -- starting point
           plot(X(1,end), X(2,end), 'bx', 'markersize', 10, 'linewidth', 3); % -- end point
           plot(X(1,:), X(2,:), 'b-', 'linewidth', 2); % -- trajectory
           plot(X(4,5), X(5,5), 'rs', 'markersize', 10, 'linewidth', 3); % -- Target location
           axis image; title(sprintf('Condition: %d', Condition), 'fontsize', 18, 'fontweight', 'normal');
       end
   end % -- end participant loop
end % -- end condition loop

end

function CommandedAcceleration(ID_List, trials, conditions, ID_conditions, ID_Data)
Ns = size(ID_Data, 1);
conditionlabel = {'No Map, No Target', 'No Map, Yes Target',...
                  'Yes Map, No Target', 'Yes Map, Yes Target'};

% -- loop through all conditions
for Condition = 1:4
    % -- create figure
    figure(Condition+1); gcf; clf;
    
    % -- loop through each participant
    for ii = Ns:Ns
        % -- define the file that contains commanded wheel speeds and load it
        CommandedInputFile = strcat('RAW/', num2str(ID_Data(ii,1)), ...
            '/condition_', num2str(Condition), '/WheelVel.csv');
        U = load(CommandedInputFile);
        
        % -- convert the commanded left and right wheel speeds to 
        % -- commanded speed and turnrate
        % -- commanded wheel speeds saved in mm/s
        CommandedSpeed = (((U(:,2) + U(:,3))/ 1000) / 2);
        
        % -- define the commanded acceleration variable and
        % -- loop through all the commanded speed values and get the
        % -- commanded acceleration
        CommandAccel = zeros(1, size(U, 1)-1);
        
        for t = 2:size(U, 1)
           % -- a=dv/dt
           CommandAccel(1,t-1) = (CommandedSpeed(t) - CommandedSpeed(t-1)) / (U(t) - U(t-1)); 
        end
        
        % -- plot the commanded acceleration 
        hold on; 
        plot(U(2:end,1), CommandAccel);
        
    end
    
    % -- make figure look nice
    xlabel('time (s)', 'fontsize', 18, 'fontweight', 'normal');
    ylabel('Commanded acceleration (m/s^2)', 'fontsize', 18, 'fontweight', 'normal');
    title(conditionlabel(Condition), 'fontsize', 18, 'fontweight', 'normal');
    ax = gca;
    ax.FontSize = 18;
end

end

function PlotTrajectoryWithAllInfo(ID_List, trials, conditions, ID_conditions, ID_Data)

Ns=size(ID_Data,1);

% Load the omron lab mosaic 
OmronLabMap = imread('maps/OmronLabMosaicCrop_lowres.jpg');

nr=7; % number of rows
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
        EKFtrajFile = strcat('filtered_data/', num2str(ID_Data(ii,1)), ...
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
        title(sprintf('Condition:%d', Condition), 'fontsize', 18, 'fontweight', 'normal');
        
        % -- plot the speed of the robot
        subplot(nr,nc,Condition+nc*1)
        EKFspeedFile = strcat('filtered_data/', num2str(ID_Data(ii,1)), ...
            '/EKFVel_condition_', num2str(Condition), '.csv');
        SP = load(EKFspeedFile);
        dt = 0.5;
        time = 0:dt:size(SP,1)*dt;
        plot(time(1:end-1), SP, 'b-', 'linewidth', 2);
        ylabel('Speed (m/s)');
        xlabel('time (s)');
        
        % -- plot the turn rate of the robot
        subplot(nr,nc,Condition+nc*2)
        EKFomFile = strcat('filtered_data/', num2str(ID_Data(ii,1)), ...
            '/EKFom_condition_', num2str(Condition), '.csv');
        OM = load(EKFomFile);
        dt = 0.5;
        time = 0:dt:size(OM,1)*dt;
        plot(time(1:end-1), OM, 'b-', 'linewidth', 2);
        ylabel('Turn rate (rad/s)');
        xlabel('time (s)');
        
        % -- define the file that contains commanded wheel speeds and load it
        CommandedInputFile = strcat('RAW/', num2str(ID_Data(ii,1)), ...
            '/condition_', num2str(Condition), '/WheelVel.csv');
        U = load(CommandedInputFile);
        
        % -- convert the commanded left and right wheel speeds to 
        % -- commanded speed and turnrate
        % -- commanded wheel speeds saved in mm/s
        CommandedSpeed = (((U(:,2) + U(:,3))/ 1000) / 2);
        CommandedTurnRate = (((U(:,2) - U(:,3))/ 1000) / .3084);
        
        % -- define the time range
        time = U(:,1);
        
        % -- plot the commanded speed for each condition
        subplot(nr,nc,Condition+nc*3)
        plot(time, CommandedSpeed, 'b-', 'linewidth', 2);
        ylabel({'Commanded','speed (m/s)'});
        xlabel('time (s)');
        
        % -- plot the commanded turn rate for each condition
        subplot(nr,nc,Condition+nc*4)
        plot(time, CommandedTurnRate, 'b-', 'linewidth', 2);
        ylabel({'Commanded','turn rate (rad/s)'});
        xlabel('time (s)');
        
        % -- commanded acceleration
        CommandAccel = zeros(1, size(CommandedSpeed, 1)-1);
        
        for t = 2:size(CommandedSpeed, 1)
           % -- a=dv/dt
           CommandAccel(1,t-1) = (CommandedSpeed(t) - CommandedSpeed(t-1)) / (U(t) - U(t-1)); 
        end
        
        % -- plot the commanded acceleration 
        subplot(nr,nc,Condition+nc*5)
        plot(U(2:end,1), CommandAccel);
        ylabel({'Commanded', 'acceleration (m/s^2)'});
        xlabel('time (s)');
        
        % -- commanded angular acceleration
        CommandAngAccel = zeros(1, size(CommandedTurnRate, 1)-1);
        
        for t = 2:size(CommandedSpeed, 1)
           % -- a=domega/dt
           CommandAngAccel(1,t-1) = (CommandedTurnRate(t) - CommandedTurnRate(t-1)) / (U(t) - U(t-1)); 
        end
        
        % -- plot the commanded angular acceleration 
        subplot(nr,nc,Condition+nc*6)
        plot(U(2:end,1), CommandAngAccel);
        ylabel({'Commanded Ang.', 'acceleration (rad/s^2)'});
        xlabel('time (s)');
        
        drawnow;
    end
    set(gcf, 'position', [54, 511, 1681, 441]);
%     print('-dpng', ['./stats data/', num2str(ID_Data(ii,1)), '_traj.png']);
end

end

function SaveAllSpeeds(ID_List, trials, conditions, ID_conditions, ID_Data)

% -- number of participants
Ns=size(ID_Data,1);

% -- create arrays that will contain the speeds of the robot
% -- and the commanded input speeds
% -- Format: [c1, c2, c3, c4a, c4b, SQ1, SQ2, SQ3, SQ4, SQ5]
RobotSpeed = zeros(Ns, 10);
CommandedSpeed = RobotSpeed;

% -- loop through all participants
for ii = 1:size(ID_Data, 1)
    
    % -- loop through each of the conditions
    for Condition = 1:4
        % -- get the filtered and RAW data
        filtered_dir = strcat('filtered_data/',num2str(ID_Data(ii,1)),'/EKFVel_condition_',...
                              num2str(Condition),'.csv');
        raw_dir = strcat('RAW/',num2str(ID_Data(ii,1)),'/condition_',num2str(Condition),...
                         '\WheelVel.csv');
        U = load(raw_dir);
        U_EKF = load(filtered_dir);
        
        % -- convert the commanded wheel speed values to overall 
        % -- commended speed
        U_com = abs(((U(:,2) + U(:,3))/ 1000) / 2);
        U_EKF = abs(U_EKF);
        
        % -- check if its condition 4
        if Condition == 4
            % -- check if break trust
            if ID_Data(ii, 2)
                RobotSpeed(ii, Condition+1) = mean(U_EKF(ID_Data(ii, end): end));
                CommandedSpeed(ii, Condition+1) = mean(U_com(5*ID_Data(ii, end): end));
                RobotSpeed(ii, Condition) = mean(U_EKF(1:ID_Data(ii, end)));
                CommandedSpeed(ii, Condition) = mean(U_com(1:5*ID_Data(ii, end)));
            else % -- otherwise store as regular
                RobotSpeed(ii, Condition) = mean(U_EKF);
                CommandedSpeed(ii, Condition) = mean(U_com);
            end
        else
            RobotSpeed(ii, Condition) = mean(U_EKF);
            CommandedSpeed(ii, Condition) = mean(U_com);
        end
    end
    
    % -- populate the questionnaire portion of the array
    RobotSpeed(ii,6:end) = ID_Data(ii, 3:end-1);
    CommandedSpeed(ii,6:end) = ID_Data(ii, 3:end-1);
end

% -- save the arrays as csv files within the stats data folder
csvwrite('stats data\RobotSpeedData.csv', RobotSpeed);
csvwrite('stats data\ComSpeedData.csv', CommandedSpeed);

end

function SaveAllTurnrates(ID_List, trials, conditions, ID_conditions, ID_Data)

% -- number of participants
Ns=size(ID_Data,1);

% -- create arrays that will contain the speeds of the robot
% -- and the commanded input speeds
% -- Format: [c1, c2, c3, c4a, c4b, SQ1, SQ2, SQ3, SQ4, SQ5]
RobotTurnrate = zeros(Ns, 10);
CommandedTurnrate = RobotTurnrate;

% -- loop through all participants
for ii = 1:size(ID_Data, 1)
    
    % -- loop through each of the conditions
    for Condition = 1:4
        % -- get the filtered and RAW data
        filtered_dir = strcat('filtered_data/',num2str(ID_Data(ii,1)),'/EKFom_condition_',...
                              num2str(Condition),'.csv');
        raw_dir = strcat('RAW/',num2str(ID_Data(ii,1)),'/condition_',num2str(Condition),...
                         '\WheelVel.csv');
        U = load(raw_dir);
        U_EKF = load(filtered_dir);
        
        % -- convert the commanded wheel speed values to overall 
        % -- commended speed
        U_com = abs(((U(:,2) - U(:,3))/ 1000) / .3084);
        U_EKF = abs(U_EKF);
        
        % -- check if its condition 4
        if Condition == 4
            % -- check if break trust
            if ID_Data(ii, 2)
                RobotTurnrate(ii, Condition+1) = mean(U_EKF(ID_Data(ii, end): end));
                CommandedTurnrate(ii, Condition+1) = mean(U_com(5*ID_Data(ii, end): end));
                RobotTurnrate(ii, Condition) = mean(U_EKF(1:ID_Data(ii, end)));
                CommandedTurnrate(ii, Condition) = mean(U_com(1:5*ID_Data(ii, end)));
            else % -- otherwise store as regular
                RobotTurnrate(ii, Condition) = mean(U_EKF);
                CommandedTurnrate(ii, Condition) = mean(U_com);
            end
        else
            RobotTurnrate(ii, Condition) = mean(U_EKF);
            CommandedTurnrate(ii, Condition) = mean(U_com);
        end
    end
    
    % -- populate the questionnaire portion of the array
    RobotTurnrate(ii,6:end) = ID_Data(ii, 3:end-1);
    CommandedTurnrate(ii,6:end) = ID_Data(ii, 3:end-1);
end

% -- save the arrays as csv files within the stats data folder
csvwrite('stats data\RobotTurnrateData.csv', RobotTurnrate);
csvwrite('stats data\ComTurnrateData.csv', CommandedTurnrate);

end
