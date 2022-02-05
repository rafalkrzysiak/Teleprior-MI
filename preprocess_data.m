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

% ID Data has columns: id (1), 4byes (2), target loc 1 (3), target loc 2 (4), 
% target loc 3 (5), target loc 4 (6), moment of realization (7)
ID_Data = csvread(ID_location,1);
% flag to process calibration experiments
robot_calib_exp=0;
if robot_calib_exp
%     ID_Data=[9999, 0, 1,1,3,4,3,10]; <--- Old data captured
    ID_Data=[9998, 0, 1,1,3,4,3,10]; % <--- New data from Di'Quan
end

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
% timeToFind(ID_List, trials, conditions, ID_conditions, ID_Data);
% CompareSpeed(ID_List, trials, conditions, ID_conditions, ID_Data);
% CompareTurnRates(ID_List, trials, conditions, ID_conditions, ID_Data);
% StoreTimeSplit(ID_List, trials, conditions, ID_conditions, ID_Data);
% totalDistanceTravelled(ID_List, trials, conditions, ID_conditions, ID_Data);
% timeStayingInPlace(ID_List, trials, conditions, ID_conditions, ID_Data);
% timeTurningInPlace(ID_List, trials, conditions, ID_conditions, ID_Data);
% timeStayingStill(ID_List, trials, conditions, ID_conditions, ID_Data)
% DensityTrajMap(ID_List, trials, conditions, ID_conditions, ID_Data);
% CommandedAcceleration(ID_List, trials, conditions, ID_conditions, ID_Data);
% PlotTrajectoryWithAllInfo(ID_List, trials, conditions, ID_conditions, ID_Data); % -- function used at the end to display everything for individual participants
% SaveAllSpeeds(ID_List, trials, conditions, ID_conditions, ID_Data);
% SaveAllTurnrates(ID_List, trials, conditions, ID_conditions, ID_Data);
% NASATLXData(ID_List, trials, conditions, ID_conditions, ID_Data)
% % KeypressDist(ID_List, trials, conditions, ID_conditions, ID_Data);
% TrackerErrorBoard();
% TrackerErrorMarkersEverywhere();
% TrackerErrorMarkersGrid();
% KeypressDist(ID_List, trials, conditions, ID_conditions, ID_Data);
% C4b_TimeInPlace(ID_List, trials, conditions, ID_conditions, ID_Data);
StoreCommandedData(ID_List, trials, conditions, ID_conditions, ID_Data);

end

function FilterAll(ID_List, trials, conditions, ID_conditions, PartialRAW, ID_Data)
Fig_suplot = 1;
cond_count = 1;

verdate=version('-date');
veryear=str2double(verdate(end-3:end));
if veryear>2018
    tiledlayout(size(trials,2),size(ID_List,1));
end
% tiledlayout(1,size(ID_List,1));
% tiledlayout(4, 4);

% -- loop through every single ID number in the list
for TRIAL = 1:size(trials,2)
    trial = num2str(trials(TRIAL));
    condition = num2str(conditions(TRIAL));
    
    for ID = 1:size(ID_List,1)
        % -- read the data from the csv file
        participantID = num2str(ID_List(ID));
        
        DataFile = strcat('RAW', filesep, participantID, filesep, 'trial_',trial, filesep,'data.csv');
        onBoard = strcat('RAW', filesep, participantID,filesep, 'condition_',condition, filesep, 'WheelVel.csv');

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
                    [0.1 0; 
                     0 0.1]*...
                    [cos(x(3))*dt 0; 
                     sin(x(3))*dt 0; 
                     0 dt]';

        % >> measurement model and ^^ noise
        % this one is simply identity
        Hfun=@(X) eye(3)*X;

        % because Hfun is linear, this is simply the same function
        Hlinfun=@(X) eye(3); 

        % -- noise of the tracker
        Rk=diag([.0418^2 .0418^2 .0277^2]*1); 
        Rk=diag([0.0046216^2 0.006561^2 .036^2]);
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
                % this picks up acute angle only otherwise we will pick up
                % large angles when passing from +ve to -ve
                EKF_om(k) = asin(sin(Xh(3,k) - Xh(3,k-1))) / dt;
            end

            % -- predict
            [Xh_(:,k+1), P_(:,:,k+1)]= ekf_predict(Xh(:,k), ...
                P(:,:,k), Qk(Xh(:,k),dt), Ffun, Flinfun, dt);

            Zt(:,k)=Hfun(Z(:,k));
        end
        
        % -- before we begin to plot everything, make sure to save all
        % -- the EKF velocities and turn rates into a csv file to be used later
        directory_str = strcat('filtered_data',filesep, participantID);
        if ~exist(directory_str, 'dir')
            mkdir(directory_str);
        end
        EKFVelFile = strcat(directory_str, filesep, 'EKFVel_condition_', num2str(TRIAL), '.csv');
        EKFomFile = strcat(directory_str, filesep, 'EKFom_condition_', num2str(TRIAL), '.csv');
        EKFtrajFile = strcat(directory_str, filesep, 'EKFtraj_condition_', num2str(TRIAL), '.csv');
        fprintf('writing data for %s ...\n', participantID);
        csvwrite(EKFVelFile, EKF_Vel); pause(1);
        csvwrite(EKFomFile, EKF_om); pause(1);
        csvwrite(EKFtrajFile, [Xh; ones(1,size(Xh,2))*tx(end); ones(1,size(Xh,2))*ty(end)]); pause(10);
        
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
if size(data,2)>9 % if target is not available then don't bother
    tx = data(:,13); ty = data(:,14); time = data(:,12);
else
    tx = x*0; ty=tx; time=data(:,3);
end

% -- get the velocity and turn rate of the robot
% -- the saved wheel speeds from the iRobot are in mm
% -- make sure to convert them to m before calculating v and omega
Vel = ((RpiData(:,2) + RpiData(:,3))/ 1000) / 2;
omega = ((RpiData(:,2) - RpiData(:,3))/ 1000) / .235;%.3084;

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
%         omega = ((RpiData(:,2) - RpiData(:,3))/ 1000) / .3084;
         omega = ((RpiData(:,2) - RpiData(:,3))/ 1000) / .235;
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
csvwrite('stats data\TimeToFind.csv', [AllTimes, ID_Data(:,3:end-1)]);

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

function StoreTimeSplit(ID_List, trials, conditions, ID_conditions, ID_Data)
% -- get the number of participants (sample size)
Ns = size(ID_Data,1);

% tiledlayout(1,size(ID_List,1));
% tiledlayout(1,Ns);

% -- create matrix to contain the ID of the participant
% -- as well as the time that corresponds to the threshold distance
% -- that the participant has been at in condition 3
% -- top row: ID, bottom row: time split
SplitTime = zeros(2, Ns);
dist = zeros(1, Ns);

OmronLabMap = imread('maps/OmronLabMosaicCrop_lowres.jpg');

% -- plotting parameters
ms = 14;

for TRIAL = 4:size(conditions,2)
    
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
%                     mean(dist)+1.5 = 2.8720
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
            
            if ID_Data(ID,2)
                figure(ID); clf; gcf;
                imagesc([0 15],[-1 7.5], flip(OmronLabMap));
                set(gca,'xdir','reverse','ydir','reverse');
                hold on; plot(X(1,1), X(2,1), 'bs', 'markersize', ms, 'linewidth', 2);
                hold on; plot(X(1,1:ID_Data(ID,end)), X(2,1:ID_Data(ID,end)), 'b-', 'linewidth', 2); % -- condition 4a
                hold on; plot(X(1,ID_Data(ID,end)), X(2,ID_Data(ID,end)), 'c+', 'markersize', ms, 'linewidth', 2);
                hold on; plot(X(1,ID_Data(ID,end):end), X(2,ID_Data(ID,end):end), 'b--', 'linewidth', 2); % -- condition 4b
                hold on; plot(X(1,end), X(2,end), 'bx', 'markersize', ms, 'linewidth', 2);
                hold on; plot(X(4,end), X(5,end), 'rs', 'markersize', ms, 'linewidth', 2);
                xlabel('X(m)'); ylabel('Y(m)');
                ax = gca;
                ax.FontSize = 18;
            end
            

%             nexttile
%             plot(EKF_X,'.', 'markersize', 8); disp(ID);
%             hold on; plot(SplitTime(2,ID), EKF_X(1,SplitTime(2,ID)),'r.', 'markersize', 12);
%             axis square; title(sprintf('%4d', ID_List(ID)),'fontsize', 12, 'fontweight', 'normal');
        end
    end
end

% -- save the data in a csv file
% csvwrite('stats data\SplitTime.csv',SplitTime');

end

function totalDistanceTravelled(ID_List, trials, conditions, ID_conditions, ID_Data)

% -- create a variable to contain the total distance
% -- that each participant drives during each of the trials
% -- with size (# trials x # participants)
TotalDistance = zeros(size(trials,2)+1, size(ID_List,1));

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
        if TRIAL < 4
            for t = 2:size(X,2)
               % -- get the dx and dy of the position 
               % -- then calculate the norm of dx and dy
               dx = X(1,t) - X(1,t-1);
               dy = X(2,t) - X(2,t-1);
               TotalDistance(TRIAL, ID) = TotalDistance(TRIAL, ID) + sqrt(dx^2 + dy^2);
            end
        else
            for t = 2:ID_Data(ID,end)
               % -- get the dx and dy of the position 
               % -- then calculate the norm of dx and dy
               dx = X(1,t) - X(1,t-1);
               dy = X(2,t) - X(2,t-1);
               TotalDistance(TRIAL, ID) = TotalDistance(TRIAL, ID) + sqrt(dx^2 + dy^2);
            end
            
            if ID_Data(ID, 2) % -- if condition 4b
                for t = ID_Data(ID,end):size(X,2)
                   % -- get the dx and dy of the position 
                   % -- then calculate the norm of dx and dy
                   dx = X(1,t) - X(1,t-1);
                   dy = X(2,t) - X(2,t-1);
                   TotalDistance(TRIAL+1, ID) = TotalDistance(TRIAL+1, ID) + sqrt(dx^2 + dy^2);
                end
            end
        end
        
    end
end

% -- once the total distance traveled for every participant 
% -- per trial is calculated, plot the distances traveled
% -- against every participant
figure(1); gcf; clf;
for participant = 1:size(ID_List,1)
    hold on; plot([1, 2, 3, 4, 5], TotalDistance(:, participant), '.-', 'linewidth', 2, 'markersize', 2);
end

% -- Make the figure look nice
ax = gca;
axis([1 5 0 50]); grid on;
xlabel('Condition number'); ylabel('Total distance traveled (m)');
ax.XTickLabel = {'1', '', '2', '','3', '', '4', '', '5'};
ax.FontSize = 18;

% -- save data to csv file
csvwrite('stats data/TotalDistanceTravel.csv',[TotalDistance',ID_Data(:,3:end-1)]);

end

function timeStayingInPlace(ID_List, trials, conditions, ID_conditions, ID_Data)
% -- create a variable to contain the total time
% -- that each participant turned in place during each of the trials
% -- with size (# trials x # participants)
timeStayingInPlace = zeros(size(trials,2)+1, size(ID_List,1));
totalTime=timeStayingInPlace;

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
        
        dir = strcat("filtered_data/", participantID, "/EKFom_condition_", condition, ".csv");
        Y = csvread(dir);
        
        % -- check what condition we are looking at
        if TRIAL == 4
            % -- if condition 4b
            if ID_Data(ID, 2) % -- if break condition condition met
                for t = 1:size(X,1) % -- loop throughout the entire time
                    if t >= ID_Data(ID,end) % -- if the timestep is on or past the break trust point, add the timesteps
                        if X(t, 1) < thresh %&& Y(t,1) > thresh
                            timeStayingInPlace(TRIAL+1, ID) = timeStayingInPlace(TRIAL+1, ID) + 1;
                        end
                        totalTime(TRIAL+1,ID)=totalTime(TRIAL+1,ID)+1;
                    else % -- if the timestep is prior to break trust point, add the time steps
                        if X(t, 1) < thresh %&& Y(t,1) > thresh
                            timeStayingInPlace(TRIAL, ID) = timeStayingInPlace(TRIAL, ID) + 1;
                        end
                        totalTime(TRIAL,ID)=totalTime(TRIAL,ID)+1;
                    end
                end
                
            % -- otherwise, condition 4a for participants not given condition 4b
            else
                for t = 1:ID_Data(ID,end)
                    if X(t, 1) < thresh %&& Y(t,1) > thresh
                        timeStayingInPlace(TRIAL, ID) = timeStayingInPlace(TRIAL, ID) + 1;
                    end
                    totalTime(TRIAL,ID)=totalTime(TRIAL,ID)+1;
                end
            end
            
        % -- conditions 1-3
        else
            % -- loop through the duration of the trial starting with t = 0
            for t = 1:size(X,1)
               if X(t, 1) < thresh %&& Y(t,1) > thresh
                   timeStayingInPlace(TRIAL, ID) = timeStayingInPlace(TRIAL, ID) + 1;
               end
               totalTime(TRIAL,ID)=totalTime(TRIAL,ID)+1;
            end
        end
        
        % -- scale it to be in between 0 and 1
        timeStayingInPlace(TRIAL, ID) = timeStayingInPlace(TRIAL, ID)/totalTime(TRIAL,ID);
    end
end

% for the 4b condition.
TRIAL=4;
for ID = 1:size(ID_List,1)
    if ID_Data(ID, 2)
        timeStayingInPlace(TRIAL+1, ID) = timeStayingInPlace(TRIAL+1, ID)/totalTime(TRIAL+1,ID);
    end
end


% -- convert from time steps to total time
% timeStayingInPlace = timeStayingInPlace;

% -- once the total time in place for every participant 
% -- per trial is calculated, plot the total time in place
% -- against every participant
figure(1); gcf; clf;
for participant = 1:size(ID_List,1)
    hold on; plot([1, 2, 3, 4], timeStayingInPlace(1:4, participant), '.-', 'linewidth', 2, 'markersize', 2);
end

% -- Make the figure look nice
ax = gca;
axis([1 4 0 1]); grid on;
xlabel('Condition number'); ylabel('Total time stayed in place (s)');
ax.XTickLabel = {'1', '', '2', '','3', '', '4'};
ax.FontSize = 18;

% -- save the data
csvwrite(['stats data', filesep, 'timeStayingInPlace.csv'], [timeStayingInPlace', ID_Data(:,3:end-1)]);

end

function timeTurningInPlace(ID_List, trials, conditions, ID_conditions, ID_Data)
% -- create a variable to contain the total time
% -- that each participant turned in place during each of the trials
% -- with size (# trials x # participants)
timeTurningInPlace = zeros(size(trials,2)+1, size(ID_List,1));
fractionTimeTurningInPlace=timeTurningInPlace;
totalTime=timeTurningInPlace;
 
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
        
        dir = strcat("filtered_data/", participantID, "/EKFom_condition_", condition, ".csv");
        Y = csvread(dir);
        
        % -- check what condition we are looking at
        if TRIAL == 4
            % -- if condition 4b
            if ID_Data(ID, 2) % -- if break condition condition met
                for t = 1:size(X,1) % -- loop throughout the entire time
                    if t >= ID_Data(ID,end) % -- if the timestep is on or past the break trust point, add the timesteps
                        if X(t, 1) < thresh && Y(t,1) > thresh
                            timeTurningInPlace(TRIAL+1, ID) = timeTurningInPlace(TRIAL+1, ID) + 1;
                        end
                        totalTime(TRIAL+1,ID)=totalTime(TRIAL+1,ID)+1;
                    else % -- if the timestep is prior to break trust point, add the time steps
                        if X(t, 1) < thresh && Y(t,1) > thresh
                            timeTurningInPlace(TRIAL, ID) = timeTurningInPlace(TRIAL, ID) + 1;
                        end
                        totalTime(TRIAL,ID)=totalTime(TRIAL,ID)+1;
                    end
                end
                
            % -- otherwise, condition 4a
            else
                for t = 1:ID_Data(ID,end)
                    if X(t, 1) < thresh && Y(t,1) > thresh
                        timeTurningInPlace(TRIAL, ID) = timeTurningInPlace(TRIAL, ID) + 1;
                    end
                    totalTime(TRIAL,ID)=totalTime(TRIAL,ID)+1;
                end
            end
            
        % -- conditions 1-3
        else
            % -- loop through the duration of the trial starting with t = 0
            for t = 1:size(X,1)
               if X(t, 1) < thresh && Y(t,1) > thresh
                   timeTurningInPlace(TRIAL, ID) = timeTurningInPlace(TRIAL, ID) + 1;
               end
               totalTime(TRIAL,ID)=totalTime(TRIAL,ID)+1;
            end
        end
        
        % -- scale it to be in between 0 and 1
        fractionTimeTurningInPlace(TRIAL, ID) = timeTurningInPlace(TRIAL, ID)/totalTime(TRIAL,ID);
    end
end
 
% for the 4b condition.
TRIAL=4;
for ID = 1:size(ID_List,1)
    if ID_Data(ID, 2)
        fractionTimeTurningInPlace(TRIAL+1, ID) = timeTurningInPlace(TRIAL+1, ID)/totalTime(TRIAL+1,ID);
    end
end
 
 
% -- convert from time steps to total time
% timeTurningInPlace = timeTurningInPlace;
 
% -- once the total time in place for every participant 
% -- per trial is calculated, plot the total time in place
% -- against every participant
figure(1); gcf; clf;
for participant = 1:size(ID_List,1)
    hold on; plot([1, 2, 3, 4], timeTurningInPlace(1:4, participant), '.-', 'linewidth', 2, 'markersize', 2);
end
 
% -- Make the figure look nice
ax = gca;
axis([1 4 0 1]); grid on;
xlabel('Condition number'); ylabel('Total time stayed in place (s)');
ax.XTickLabel = {'1', '', '2', '','3', '', '4'};
ax.FontSize = 18;
 
% -- save the data
csvwrite(['stats data', filesep, 'fractionTimeTurningInPlace.csv'], ...
        [fractionTimeTurningInPlace', ID_Data(:,3:end-1)]);
 
end

function DensityTrajMap(ID_List, trials, conditions, ID_conditions, ID_Data)

% -- get the number of participants
Ns=size(ID_Data,1);

% -- opacity of the trajectory line
alpha = 0.25;

% -- Load the omron lab mosaic OmronLabMosaicCrop_lowres 
OmronLabMap = imread('maps/blankMap.jpg');

% -- loop though all conditions and sub-conditions
for Condition = 1:4
   % -- create a figure that corresponds to the condition number
   figure(Condition+1); gcf; clf;
   
   % -- plot the Search environment
   imagesc([-0.2 15],[-0.5 7], flip(OmronLabMap));
   set(gca,'xdir','reverse','ydir','reverse');
   
   % -- loop through all participants of the experiment
   for ii = 1:Ns
       
        % -- create the string that corresponds to the name of the file
        % -- that contains the trajectory data
        trajFile = strcat('RAW/', num2str(ID_Data(ii,1)), ...
            '/trial_000', num2str(Condition+1), '/data.csv');
        
        % -- load the file that contains the trajectory data
        Data = csvread(trajFile,2);
        X(:,1) = Data(:,4); X(:,2) = Data(:,5); 
        X(:,3) = Data(:,13); X(:,4) = Data(:,14);
        X = X(X(:,1)~=0,:); % -- remove the zeros
        
        figure(Condition+1);
        % -- plot all data prior to condition 4
        hold on; plot(X(5,1), X(5,2), 'bs', 'markersize', 10, 'linewidth', 3); % -- starting point
        plot(X(end,1), X(end,2), 'bx', 'markersize', 10, 'linewidth', 3); % -- end point
        plot(X(5:end,1), X(5:end,2), 'color',[0,0,0]+alpha, 'linewidth', 1); % -- trajectory
        plot(X(end,3), X(end,4), 'gs', 'markersize', 10, 'linewidth', 3); % -- Target location
        axis image; xlabel('X(m)'); ylabel('Y(m)'); 
        %title(sprintf('Condition: %d', Condition), 'fontsize', 18, 'fontweight', 'normal');
        clear X;
   end % -- end participant loop
   ax = gca;
   ax.FontSize = 18;
end % -- end condition loop

end

function CommandedAcceleration(ID_List, trials, conditions, ID_conditions, ID_Data)
Ns = size(ID_Data, 1);
conditionlabel = {'No Map, No Target', 'No Map, Yes Target',...
                  'Yes Map, No Target', 'Yes Map, Yes Target'};
              
% -- array to hold all mean values of commanded acceleration
% -- for each participant during each trial
CommandedAccel = zeros(Ns, 5);

% -- loop through all conditions
for Condition = 1:4
    
    % -- loop through each participant
    for ii = 1:Ns
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
        Accel = zeros(1, size(CommandedSpeed, 1)-1);
        
        % -- loop through only conditions 1-3 for each participant
        if Condition < 4
            for t = 2:size(U, 1)
               % -- a=dv/dt
               Accel(1,t-1) = (CommandedSpeed(t) - CommandedSpeed(t-1)) / (U(t) - U(t-1)); 
            end
            
            % -- get the mean of the Accel
            CommandedAccel(ii,Condition) = mean(Accel);
            
        else % -- looking at condition 4
            % -- specifically looking at condition 4b
            if ID_Data(ii,2)
                Accel = zeros(1, size(CommandedSpeed(1:5*ID_Data(ii,end)), 2)-1);
                for t = 2:5*ID_Data(ii,end)
                   % -- a=dv/dt
                   Accel(1,t-1) = (CommandedSpeed(t) - CommandedSpeed(t-1)) / (U(t) - U(t-1)); 
                end

                % -- get the mean of the Accel
                CommandedAccel(ii,Condition) = mean(Accel);
                
                Accel = zeros(1, size(CommandedSpeed(5*ID_Data(ii,end):end), 2)-1);
                for t = 5*ID_Data(ii,end):size(CommandedSpeed,1)
                   % -- a=dv/dt
                   Accel(1,t-1) = (CommandedSpeed(t) - CommandedSpeed(t-1)) / (U(t) - U(t-1)); 
                end

                % -- get the mean of the Accel
                CommandedAccel(ii,Condition+1) = mean(Accel);
                
            else
                % -- if not breaking trust (condition 4a)
                for t = 2:size(U, 1)
                   % -- a=dv/dt
                   Accel(1,t-1) = (CommandedSpeed(t) - CommandedSpeed(t-1)) / (U(t) - U(t-1)); 
                end

                % -- get the mean of the Accel
                CommandedAccel(ii,Condition) = mean(Accel);
            end
        end 
    end
    
    % -- make figure look nice
    xlabel('time (s)', 'fontsize', 18, 'fontweight', 'normal');
    ylabel('Commanded acceleration (m/s^2)', 'fontsize', 18, 'fontweight', 'normal');
    title(conditionlabel(Condition), 'fontsize', 18, 'fontweight', 'normal');
    ax = gca;
    ax.FontSize = 18;
end

% -- create csv file to store the commanded acceleration for each
% -- participant during each condition
csvwrite('stats data/CommandedAccel.csv',[CommandedAccel,ID_Data(:,3:end-1)]);

end

function PlotTrajectoryWithAllInfo(ID_List, trials, conditions, ID_conditions, ID_Data)

Ns=size(ID_Data,1);

% Load the omron lab mosaic 
OmronLabMap = imread('maps/OmronLabMosaicCrop_lowres.jpg');

if size(ID_Data,1)>1
    nr=5; % number of rows
    nc=5; % number of conditions (remember: condition 4 is split into 4a and 4b)
else
    nr=4;
    nc=5;
end

% -- forcing terms for the ylim
MaxSpeed = 0.65;
MaxTurnRate = 3.5;

for ii = 1:size(ID_Data, 1)
    
    % -- only want to loop through individuals that hav undergone
    % -- condition 4b (incorrect prior knowledge of target location)
    if (ID_Data(ii, 2) || ID_Data(ii,1) ==9998)
        
        % -- create figure dedicated to an individual ID
        % -- and create a tiled layout of 1x4
        figure(ii); gcf; clf;% -- contains fig num 1 -> # participants
    
        % -- loop through each of the conditions
        for Condition = 1:4
            
            % -- create the string that corresponds to the name of the file
            % -- that contains the trajectory data
            EKFtrajFile = strcat('filtered_data/', num2str(ID_Data(ii,1)), ...
                '/EKFtraj_condition_', num2str(Condition), '.csv');

            % -- load the file that contains the trajectory data
            X = load(EKFtrajFile);
            
            EKFspeedFile = strcat('filtered_data/', num2str(ID_Data(ii,1)), ...
                '/EKFVel_condition_', num2str(Condition), '.csv');
            SP = load(EKFspeedFile);
            dt = 0.5;
            
            EKFomFile = strcat('filtered_data/', num2str(ID_Data(ii,1)), ...
                '/EKFom_condition_', num2str(Condition), '.csv');
            OM = load(EKFomFile);
            
            % -- define the file that contains commanded wheel speeds and load it
            CommandedInputFile = strcat('RAW/', num2str(ID_Data(ii,1)), ...
                '/condition_', num2str(Condition), '/WheelVel.csv');
            U = load(CommandedInputFile);

            % -- convert the commanded left and right wheel speeds to 
            % -- commanded speed and turnrate
            % -- commanded wheel speeds saved in mm/s
            CommandedSpeed = (((U(:,2) + U(:,3))/ 1000) / 2);
            CommandedTurnRate = (((U(:,2) - U(:,3))/ 1000) / .235);
            
            % -- check what condition we are trying to plot
            if Condition < 4 || ID_Data(ii,1) ==9998
                % -- begin plotting the data
                subplot(nr,nc,5*(Condition-1)+1)
                imagesc([-1 15],[-1 7.5], flip(OmronLabMap));
                set(gca,'xdir','reverse','ydir','reverse', 'fontsize', 16);
                hold on; plot(X(1,1), X(2,1), 'bs', 'markersize', 6, 'linewidth', 3); % -- starting point
                plot(X(1,end), X(2,end), 'bx', 'markersize', 6, 'linewidth', 3); % -- end point
                plot(X(1,:), X(2,:), 'b-', 'linewidth', 3); % -- trajectory
                plot(X(4,1), X(5,1), 'rs', 'markersize', 6, 'linewidth', 3); % -- Target location
                axis image; 
                %title(sprintf('Condition:%d', Condition), 'fontsize', 18, 'fontweight', 'normal');

                % -- plot the speed of the robot
                subplot(nr,nc,5*(Condition-1)+2)
                % -- define the time range
                time = U(:,1);
                plot(time, abs(CommandedSpeed), 'k-', 'linewidth', 2);
                ylabel({'Commanded','speed (m/s)'});
                xlabel('time (s)'); grid on;
                set(gca, 'fontsize', 16);
                xlim([0 time(end)]); ylim([0 MaxSpeed]);
                
                % -- plot the turn rate of the robot
                subplot(nr,nc,5*(Condition-1)+3)
                plot(time, abs(CommandedTurnRate), 'k-', 'linewidth', 2);
                ylabel({'Commanded','turn rate (rad/s)'});
                xlabel('time (s)'); grid on;
                set(gca, 'fontsize', 16);
                xlim([0 time(end)]); ylim([0 MaxTurnRate]);
                
                % -- plot the commanded speed for each condition
                subplot(nr,nc,5*(Condition-1)+4)
                time = 0:dt:size(SP,1)*dt;
                plot(time(1:end-1), abs(SP), 'b-', 'linewidth', 2);
                ylabel('Speed (m/s)');
                xlabel('time (s)');
                set(gca, 'fontsize', 16);
                grid on; xlim([0 time(end)]); ylim([0 MaxSpeed]);

                % -- plot the commanded turn rate for each condition
                subplot(nr,nc,5*(Condition-1)+5)
                time = 0:dt:size(OM,1)*dt;
                plot(time(1:end-1), abs(OM), 'b-', 'linewidth', 2);
                ylabel('Turn rate (rad/s)');
                xlabel('time (s)'); grid on;
                set(gca, 'fontsize', 16);
                xlim([0 time(end)]); ylim([0 MaxTurnRate]);

            
            else % -- if we are looking at condition 4
                % -- begin plotting the data
                subplot(nr,nc,5*(Condition-1)+1)
                imagesc([-1 15],[-1 7.5], flip(OmronLabMap));
                set(gca,'xdir','reverse','ydir','reverse');
                hold on; plot(X(1,1), X(2,1), 'bs', 'markersize', 6, 'linewidth', 3); % -- starting point
                plot(X(1,ID_Data(ii,end)), X(2,ID_Data(ii,end)), 'bx', 'markersize', 6, 'linewidth', 3); % -- end point
                plot(X(1,1:ID_Data(ii,end)), X(2,1:ID_Data(ii,end)), 'b-', 'linewidth', 3); % -- trajectory
                plot(X(4,1), X(5,1), 'rs', 'markersize', 6, 'linewidth', 3); % -- Target location
                set(gca, 'fontsize', 16);
                axis image; 
                %title(sprintf('Condition:%d', Condition), 'fontsize', 18, 'fontweight', 'normal');

                % -- plot the speed of the robot
                subplot(nr,nc,5*(Condition-1)+2)
                % -- define the time range
                time = U(:,1);
                plot(time(1:5*ID_Data(ii,end)), abs(CommandedSpeed(1:5*ID_Data(ii,end))), 'k-', 'linewidth', 2);
                ylabel({'Commanded','speed (m/s)'});
                xlabel('time (s)'); grid on;
                set(gca, 'fontsize', 16);
                xlim([0 time(5*ID_Data(ii,end))]); ylim([0 MaxSpeed]);

                % -- plot the turn rate of the robot
                subplot(nr,nc,5*(Condition-1)+3)
                plot(time(1:5*ID_Data(ii,end)), abs(CommandedTurnRate(1:5*ID_Data(ii,end))), 'k-', 'linewidth', 2);
                ylabel({'Commanded','turn rate (rad/s)'});
                xlabel('time (s)'); grid on;
                set(gca, 'fontsize', 16);
                xlim([0 time(5*ID_Data(ii,end))]); ylim([0 MaxTurnRate]);

                % -- plot the commanded speed for each condition
                subplot(nr,nc,5*(Condition-1)+4)
                time = 0:dt:size(SP,1)*dt;
                plot(time(1:ID_Data(ii,end)), abs(SP(1:ID_Data(ii,end))), 'b-', 'linewidth', 2);
                ylabel('Speed (m/s)');
                xlabel('time (s)'); grid on;
                set(gca, 'fontsize', 16);
                xlim([0 time(ID_Data(ii,end))]); ylim([0 MaxSpeed]);

                % -- plot the commanded turn rate for each condition
                subplot(nr,nc,5*(Condition-1)+5)
                time = 0:dt:size(OM,1)*dt;
                plot(time(1:ID_Data(ii,end)), abs(OM(1:ID_Data(ii,end))), 'b-', 'linewidth', 2);
                ylabel('Turn rate (rad/s)');
                xlabel('time (s)'); grid on;
                set(gca, 'fontsize', 16);
                xlim([0 time(ID_Data(ii,end))]); ylim([0 MaxTurnRate]);
                
                % --------------------------------------------
                % -- begin plotting the data for condition 4b
                subplot(nr,nc,5*(Condition)+1)
                imagesc([-1 15],[-1 7.5], flip(OmronLabMap));
                set(gca,'xdir','reverse','ydir','reverse');
                hold on; plot(X(1,ID_Data(ii,end)), X(2,ID_Data(ii,end)), 'bs', 'markersize', 6, 'linewidth', 3); % -- starting point
                plot(X(1,end), X(2,end), 'bx', 'markersize', 6, 'linewidth', 3); % -- end point
                plot(X(1,ID_Data(ii,end):end), X(2,ID_Data(ii,end):end), 'b-', 'linewidth', 3); % -- trajectory
                plot(X(4,1), X(5,1), 'rs', 'markersize', 6, 'linewidth', 3); % -- Target location
                set(gca, 'fontsize', 16);
                axis image; 
                %title(sprintf('Condition:%d', Condition), 'fontsize', 18, 'fontweight', 'normal');

                % -- plot the commanded speed of the robot
                subplot(nr,nc,5*(Condition)+2)
                % -- define the time range
                time = U(:,1);
                plot(time(5*ID_Data(ii,end):end), abs(CommandedSpeed(5*ID_Data(ii,end):end)), 'k-', 'linewidth', 2);
                ylabel({'Commanded','speed (m/s)'});
                xlabel('time (s)'); grid on;
                set(gca, 'fontsize', 16);
                xlim([time(5*ID_Data(ii,end)) time(end)]); ylim([0 MaxSpeed]);

                % -- plot the commanded turn rate of the robot
                subplot(nr,nc,5*(Condition)+3)
                plot(time(5*ID_Data(ii,end):end), abs(CommandedTurnRate(5*ID_Data(ii,end):end)), 'k-', 'linewidth', 2);
                ylabel({'Commanded','turn rate (rad/s)'});
                xlabel('time (s)'); grid on;
                set(gca, 'fontsize', 16);
                xlim([time(5*ID_Data(ii,end)) time(end)]); ylim([0 MaxTurnRate]);

                % -- plot the Robot speed for each condition
                subplot(nr,nc,5*(Condition)+4)
                time = 0:dt:size(SP,1)*dt;
                plot(time(ID_Data(ii,end):end-1), abs(SP(ID_Data(ii,end):end)), 'b-', 'linewidth', 2);
                ylabel('Speed (m/s)');
                xlabel('time (s)'); grid on;
                set(gca, 'fontsize', 16);
                xlim([time(ID_Data(ii,end)) time(end)]); ylim([0 MaxSpeed]);

                % -- plot the Robot turn rate for each condition
                subplot(nr,nc,5*(Condition)+5)
                time = 0:dt:size(OM,1)*dt;
                plot(time(ID_Data(ii,end):end-1), abs(OM(ID_Data(ii,end):end)), 'b-', 'linewidth', 2);
                ylabel('Turn rate (rad/s)');
                xlabel('time (s)'); grid on;
                set(gca, 'fontsize', 16);
                xlim([time(ID_Data(ii,end)) time(end)]); ylim([0 MaxTurnRate]);
            
            end

            drawnow;
    %         
    %         
    %         % -- commanded acceleration
    %         CommandAccel = zeros(1, size(CommandedSpeed, 1)-1);
    %         
    %         for t = 2:size(CommandedSpeed, 1)
    %            % -- a=dv/dt
    %            CommandAccel(1,t-1) = (CommandedSpeed(t) - CommandedSpeed(t-1)) / (U(t) - U(t-1)); 
    %         end
    %         
    %         % -- plot the commanded acceleration 
    %         subplot(nr,nc,Condition+nc*5)
    %         plot(U(2:end,1), CommandAccel);
    %         ylabel({'Commanded', 'acceleration (m/s^2)'});
    %         xlabel('time (s)');
    %         
    %         % -- commanded angular acceleration
    %         CommandAngAccel = zeros(1, size(CommandedTurnRate, 1)-1);
    %         
    %         for t = 2:size(CommandedSpeed, 1)
    %            % -- a=domega/dt
    %            CommandAngAccel(1,t-1) = (CommandedTurnRate(t) - CommandedTurnRate(t-1)) / (U(t) - U(t-1)); 
    %         end
    %         
    %         % -- plot the commanded angular acceleration 
    %         subplot(nr,nc,Condition+nc*6)
    %         plot(U(2:end,1), CommandAngAccel);
    %         ylabel({'Commanded Ang.', 'acceleration (rad/s^2)'});
    %         xlabel('time (s)');

            
        end
        set(gcf, 'position', [54, 511, 1681, 441]);
    %     print('-dpng', ['./stats data/', num2str(ID_Data(ii,1)), '_traj.png']);
    end
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
        filtered_dir = strcat('filtered_data',filesep,num2str(ID_Data(ii,1)),filesep,'EKFom_condition_',...
                              num2str(Condition),'.csv');
        raw_dir = strcat('RAW',filesep,num2str(ID_Data(ii,1)), filesep, 'condition_',num2str(Condition),...
                         filesep,'WheelVel.csv');
        U = load(raw_dir);
        U_EKF = load(filtered_dir);
        
        % -- convert the commanded wheel speed values to overall 
        % -- commended speed
        U_com = abs(((U(:,2) - U(:,3))/ 1000) / .235);
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
csvwrite(['stats data', filesep, 'RobotTurnrateData.csv'], RobotTurnrate);
csvwrite(['stats data', filesep, 'ComTurnrateData.csv'], CommandedTurnrate);

end

function NASATLXData(ID_List, trials, conditions, ID_conditions, ID_Data)

% -- get the number of participants
Ns=size(ID_Data,1);

% -- create variable to store all the mean values for the
% -- NASA TLX questions asked during the experiment
% -- size of (Ns x 7) = (Number of participants x [ID, 6 questions])
% -- Mental demand: Low - High
% -- Physical demand: Low - High
% -- Temporal demand: Low - High
% -- Performance: Perfect - Failure
% -- Effort: Low - High
% -- Frustration: Low - High
AvgResults = zeros(Ns, 6);
results = zeros(4,6,Ns);

% -- loop through all participants
for ii = 1:Ns
     % -- create str that corresponds to NASA TLX data location
     dir = strcat('RAW/', num2str(ID_Data(ii,1)), '/nasa-tlx-results.txt');
     NASATLX = readtable(dir); % -- data as table because of numbers and letters in txt file
     
     % -- extract the values of the answers only
     results(:,:,ii) = table2array(NASATLX(2:end,4:9));
     
     % -- get the mean values of the answers
%      AvgResults(ii,:) = mean(results(:,:,ii));
end

% -- calculate the mean and standard deviation of the NASA TLX results
% -- Remember: The first row for all participants was for condition 0
% --           don't need to include that one, only rows 2-5 for conditions 1-4
% -- it will be a 6x4 array, 6 questions (rows) x 4 conditions (columns)
TLX_mean = zeros(6,4);
TLX_std = TLX_mean;
TLX_question = zeros(Ns, 4); % -- looking at individual quesions paired with all conditions

% -- loop through all rows and columns 
for question = 1:6
    for condition = 1:4
        % -- calculate the mean and std of the questions asked
        % -- look at the same question of a certain condition for all participants
        TLX_mean(question,condition) = mean(results(condition, question, :));
        TLX_std(question,condition) = std(results(condition, question, :));
        
        % -- loop through every participant
        % -- extract the value given for the same question and condition for all participants
        for ii = 1:Ns
            TLX_question(ii,condition) = results(condition,question,ii);
        end
    end
    csvwrite(sprintf('stats data/TLX_question_%d.csv', question), TLX_question);
end

% -- save the results as a csv file later for the live script
% csvwrite('stats data/NASATLXResults.csv', AvgResults);
csvwrite('stats data/NASATLXMean.csv',TLX_mean);
csvwrite('stats data/NASATLXstd.csv',TLX_std);

end

function timeStayingStill(ID_List, trials, conditions, ID_conditions, ID_Data)
% -- create a variable to contain the total time
% -- that each participant statyed in place during each of the trials
% -- with size (# trials x # participants)
timeStayingStill = zeros(size(trials,2)+1, size(ID_List,1));
fractionTimeStayingStill = timeStayingStill;
totalTime=timeStayingStill;
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
        % -- and store the data in a variable "X" and "Y"
        % -- State X and Y are each a 5xT matrix where T is total time
        dir = strcat("filtered_data/", participantID, "/EKFom_condition_", condition, ".csv");
        X = csvread(dir);
        
        diry = strcat("filtered_data/", participantID, "/EKFVel_condition_", condition, ".csv");
        Y = csvread(diry);
        
        % -- check what condition we are looking at
        if TRIAL == 4
            % -- if condition 4b
            if ID_Data(ID, 2) % -- if break condition condition met
                for t = 1:size(X,1) % -- loop throughout the entire time
                    if t >= ID_Data(ID,end) % -- if the timestep is on or past the break trust point, add the timesteps
                        if X(t, 1) < thresh && Y(t,1) < thresh % -- both speed and turn rate below threshold value
                            timeStayingStill(TRIAL+1, ID) = timeStayingStill(TRIAL+1, ID) + 1;
                        end
                        totalTime(TRIAL+1, ID)=totalTime(TRIAL+1, ID)+1;
                    else % -- if the timestep is prior to break trust point, add the time steps
                        if X(t, 1) < thresh && Y(t,1) < thresh
                            timeStayingStill(TRIAL, ID) = timeStayingStill(TRIAL, ID) + 1;
                        end
                        totalTime(TRIAL, ID)=totalTime(TRIAL, ID)+1;
                    end
                end
%                 fractionTimeStayingStill(TRIAL, ID) = timeStayingStill(TRIAL, ID)/size(X,1);
%                 fractionTimeStayingStill(TRIAL+1, ID) = timeStayingStill(TRIAL+1, ID)/size(X,1);
                
            % -- otherwise, condition 4a
            else
                for t = 1:ID_Data(ID,end)
                    if X(t, 1) < thresh && Y(t,1) < thresh
                        timeStayingStill(TRIAL, ID) = timeStayingStill(TRIAL, ID) + 1;
                    end
                totalTime(TRIAL, ID)=totalTime(TRIAL, ID)+1;    
                end
                
%                 fractionTimeStayingStill(TRIAL, ID) = timeStayingStill(TRIAL, ID)/size(X,1);
            end
            
        % -- conditions 1-3
        else
            % -- loop through the duration of the trial starting with t = 0
            for t = 1:size(X,1)
               if X(t, 1) < thresh && Y(t,1) < thresh
                   timeStayingStill(TRIAL, ID) = timeStayingStill(TRIAL, ID) + 1;
               end
               totalTime(TRIAL, ID)=totalTime(TRIAL, ID)+1;
            end
%             fractionTimeStayingStill(TRIAL, ID) = timeStayingStill(TRIAL, ID)/size(X,1);
        end
        fractionTimeStayingStill(TRIAL, ID) = timeStayingStill(TRIAL, ID)/totalTime(TRIAL,ID);
    end
end

% for the 4b condition.
TRIAL=4;
for ID = 1:size(ID_List,1)
    if ID_Data(ID, 2)
        fractionTimeStayingStill(TRIAL+1, ID) = timeStayingStill(TRIAL+1, ID)/totalTime(TRIAL+1,ID);
    end
end

% -- convert from time steps to total time
timeStayingStill = dt*timeStayingStill;

% -- once the total time in place for every participant 
% -- per trial is calculated, plot the total time in place
% -- against every participant
figure(1); gcf; clf;
for participant = 1:size(ID_List,1)
    hold on; plot([1, 2, 3, 4], timeStayingStill(1:4, participant), '.-', 'linewidth', 2, 'markersize', 2);
end

% -- Make the figure look nice
ax = gca;
axis([1 4 0 300]); grid on;
xlabel('Condition number'); ylabel('Total time stayed in place (s)');
ax.XTickLabel = {'1', '', '2', '','3', '', '4'};
ax.FontSize = 18;

% -- save the data
csvwrite(['stats data', filesep, 'timeStayingStill.csv'], [timeStayingStill',ID_Data(:,3:7)]);
csvwrite(['stats data', filesep, 'FractionTimeStayingStill.csv'], [fractionTimeStayingStill',ID_Data(:,3:7)]);

end

function KeypressDist(ID_List, trials, conditions, ID_conditions, ID_Data)

% -- get the number of participants
Ns=size(ID_Data,1);

% -- remember how the data is stored:
% -- [time, vL, vR, key up, key down, key left, key right]
% -- Create array that will store the degree values of the keypresses 
% -- forward: 90, backward: 270, left: 180, right: 0
theta = [90, 270, 0, 180]+45;
dt = 0.1;

% -- loop through every participant
for ii = 1:Ns
    
    % -- loop through all conditions
    for condition = 1:4
       
        % -- get the directory location of the specific
        % -- participant and condition tested
        dir = strcat('RAW/', num2str(ID_Data(ii,1)), '/condition_',...
                     num2str(condition), '/WheelVel.csv');
        U = load(dir);
        
        % -- convert keyboard boolean values as degrees
        % -- then make it a single column vector
        keys = U(:,4:end);
        keys = keys.*theta;
        Theta = sum(keys,2);
        
        % -- create a time vector that matches the length of
        % -- time that was ran for the condition tested
        r = 0:dt:(size(keys,1)-1)*dt;
        
        % -- define the v_dot size
        v_dot = zeros(size(Theta,1),1);
        theta_dot = v_dot;
        r_dot = theta_dot;
        v = theta_dot;
        
        % -- we want to loop throughout time
        % -- v = r*exp(i*theta)
        % -- v_dot = theta_dot*r*exp(i*theta) + r_dot*exp(i*theta)
        for t = 2:size(keys,1)
           % -- calculate v_dot
           v(t,1) = r(t)*(cosd(Theta(t)) + sind(Theta(t)));
           theta_dot(t,1) = (Theta(t) - Theta(t-1))/dt;
           r_dot(t,1) = (r(t) - r(t-1))/dt;
           v_dot(t,1) = theta_dot(t-1,1)*r(t)*(cosd(Theta(t)) + sind(Theta(t))) + ...
                          r_dot(t-1,1)*(cosd(Theta(t)) + sind(Theta(t)));
        end
        
    end
end

end

function TrackerErrorBoard()

% -- Read the data collected for finding the tracker error
% -- Define the directory that holds the csv file
% -- then store the data
% -- Remember: trail_0001 -> board with two markers
dir = "Tracking Error Experiment/trial_0001/data.csv";
X = csvread(dir,1);

% -- Remember: The data stored per marker detected by the tracker in data.csv is stored as:
% -- [Aruco ID, Cam ID, Time, X, Y, Z, Rx, Ry, Rz] <- 9 data points (with pose = 6 pts)
% -- get the number of markers used for analysis with the board knowing
%  -- that the tracker stores 9 data points per marker
Nm = size(X, 2)/9; 

% -- remove all zeros that correspond to each marker
X = X(X(:,4)~=0 & X(:,13)~=0,:);

% -- define the distance between the markers in meters along the markers y axis
L = 2.01136;

% -- Calculate the difference in pose
dx = X(:,4) - X(:,13);
dy = X(:,5) - X(:,14);
dz = X(:,6) - X(:,15);
drx = X(:,7) - X(:,16);
dry = X(:,8) - X(:,17);
drz = X(:,9) - X(:,18);

% -- calculate the length between the markers
dl = sqrt(dx.^2 + dy.^2);

% -- get the mean and standard deviation of the difference
% -- where column 1 is mean and column 2 is standard deviation
data = zeros(5, Nm);
data(:, 1) = [mean(dl); mean(dz); mean(drx); mean(dry); mean(drz)];
data(:, 2) = [std(dl); std(dz); std(drx); std(dry); std(drz)];

% -- create a figure that displays all the marker difference in pose errors
figure(1); clf; gcf;

% ----------------
% -- plot the dl
subplot(2,3,1);
plot(dl, 'k-', 'linewidth', 2);
hold on; plot(1:size(dl,1), L*ones(size(dl,1),1), 'b-', 'linewidth', 2);

% -- plot the mean and standard deviation of dy
hold on; plot(1:size(dl,1), data(1,1)*ones(size(dl,1),1), 'r-', 'linewidth', 2);
hold on; plot(1:size(dl,1), (data(1,1)+data(1,2))*ones(size(dl,1),1), 'r--', 'linewidth', 2);
hold on; plot(1:size(dl,1), (data(1,1)-data(1,2))*ones(size(dl,1),1), 'r--', 'linewidth', 2);

% -- make subplot look nice
xlabel('Time (s)', 'fontweight', 'normal', 'fontsize', 18);
ylabel('Distance between markers (m)', 'fontweight', 'normal', 'fontsize', 18);
title(sprintf('True: %.3f m, Mean: %.3f $\\pm$ %.3f m', L, data(1,1), data(1,2)), 'Interpreter', 'latex', 'fontsize', 18);

% ----------------
% -- plot the dz
subplot(2,3,2);
plot(dz, 'k-', 'linewidth', 2);
hold on; plot(1:size(dz,1), zeros(size(dy,1),1), 'b-', 'linewidth', 2);

% -- plot the mean and standard deviation of dy
hold on; plot(1:size(dz,1), data(2,1)*ones(size(dz,1),1), 'r-', 'linewidth', 2);
hold on; plot(1:size(dz,1), (data(2,1)+data(2,2))*ones(size(dz,1),1), 'r--', 'linewidth', 2);
hold on; plot(1:size(dz,1), (data(2,1)-data(2,2))*ones(size(dz,1),1), 'r--', 'linewidth', 2);

% -- make subplot look nice
xlabel('Time (s)', 'fontweight', 'normal', 'fontsize', 18);
ylabel('Difference in Z axis (m)', 'fontweight', 'normal', 'fontsize', 18);
title(sprintf('Mean: %.3f $\\pm$ %.3f m', data(2,1), data(2,2)), 'Interpreter', 'latex', 'fontsize', 18);

% ------------------
% -- plot the drx
subplot(2,3,3);
plot(drx, 'k-', 'linewidth', 2);
hold on; plot(1:size(drx,1), zeros(size(drx,1),1), 'b-', 'linewidth', 2);

% -- plot the mean and standard deviation of drx
hold on; plot(1:size(drx,1), data(3,1)*ones(size(drx,1),1), 'r-', 'linewidth', 2);
hold on; plot(1:size(drx,1), (data(3,1)+data(3,2))*ones(size(drx,1),1), 'r--', 'linewidth', 2);
hold on; plot(1:size(drx,1), (data(3,1)-data(3,2))*ones(size(drx,1),1), 'r--', 'linewidth', 2);

% -- make subplot look nice
xlabel('Time (s)', 'fontweight', 'normal', 'fontsize', 18);
ylabel('Difference in RX (rad)', 'fontweight', 'normal', 'fontsize', 18);
title(sprintf('Mean: %.3f $\\pm$ %.3f rad', data(3,1), data(3,2)), 'Interpreter', 'latex', 'fontsize', 18);

% ------------------
% -- plot the dry
subplot(2,3,4);
plot(dry, 'k-', 'linewidth', 2); 
hold on; plot(1:size(dry,1), zeros(size(dry,1),1), 'b-', 'linewidth', 2);

% -- plot the mean and standard deviation of drx
hold on; plot(1:size(dry,1), data(4,1)*ones(size(dry,1),1), 'r-', 'linewidth', 2);
hold on; plot(1:size(dry,1), (data(4,1)+data(4,2))*ones(size(dry,1),1), 'r--', 'linewidth', 2);
hold on; plot(1:size(dry,1), (data(4,1)-data(4,2))*ones(size(dry,1),1), 'r--', 'linewidth', 2);

% -- make subplot look nice
xlabel('Time (s)', 'fontweight', 'normal', 'fontsize', 18);
ylabel('Difference in RY (rad)', 'fontweight', 'normal', 'fontsize', 18);
title(sprintf('Mean: %.3f $\\pm$ %.3f rad', data(4,1), data(4,2)), 'Interpreter', 'latex', 'fontsize', 18);

% -----------------
% -- plot the drz
subplot(2,3,5);
plot(drz, 'k-', 'linewidth', 2);
hold on; plot(1:size(drz,1), zeros(size(drz,1),1), 'b-', 'linewidth', 2);

% -- plot the mean and standard deviation of drx
hold on; plot(1:size(drz,1), data(5,1)*ones(size(drz,1),1), 'r-', 'linewidth', 2);
hold on; plot(1:size(drz,1), (data(5,1)+data(5,2))*ones(size(drz,1),1), 'r--', 'linewidth', 2);
hold on; plot(1:size(drz,1), (data(5,1)-data(5,2))*ones(size(drz,1),1), 'r--', 'linewidth', 2);

% -- make subplot look nice
xlabel('Time (s)', 'fontweight', 'normal', 'fontsize', 18);
ylabel('Difference in RZ (rad)', 'fontweight', 'normal', 'fontsize', 18);
title(sprintf('Mean: %.3f $\\pm$ %.3f rad', data(5,1), data(5,2)), 'Interpreter', 'latex', 'fontsize', 18);

end

function TrackerErrorMarkersEverywhere()

% -- read the data collected for finding the tracker error
% -- Define the directory that holds the csv file
% -- Remember: trail_0002-0004 -> markers all over Lab
% --           With the time collected increasing from 0002 -> 0004
dir = "Tracking Error Experiment/Experiment 1/trial_0004/data.csv";
X = csvread(dir, 1);

% -- get the number of markers used in the experiment
Nm = size(X, 2)/9;

% -- define the distance between each pair of markers in meters
L = 0.211137;

% -- define the markers used
markers = [20:1:50, 55:1:67];

% -- calculate the difference in pose for each marker pair
Np = Nm/2; % -- number of marker pairs used

% -- create array that will store the mean and standard deviation for each
% -- marker pair pose measurement
% -- Array of size [5 x 2 x Number of marker pairs]
% -- where the rows are (dl,z,rx,ry,rz) and columns are (mean, std)
data = zeros(5, 2, Np);

% -- plotting parameters
fs = 8;

for ii = 1:Np
    
    % -- create series step that will get the correct data from X
    % -- that correponds to the specific marker pose 
    % -- Marker #1: 4 -> X, 5 -> Y, 6 -> Z, 7 -> Rx, 8 -> Ry, 9 -> Rz
    % -- Marker #2: 13 -> X, 14 -> Y, 15 -> Z, 16 -> Rx, 17 -> Ry, 18 ->Rz
    sx1 = 4 + 18*(ii - 1); % -- X column for marker 1
    sx2 = 13 + 18*(ii - 1); % -- X column for marker 2
    
    % -- store the two markers in the pair as a temp variable
    % -- eliminating the zeros from both
    XX = X(X(:,sx1)~=0 & X(:,sx2)~=0,(sx1-3):(sx2+5));
    
    % -- caluclate the difference in pose between each marker
    dx = XX(:,4) - XX(:,13);
    dy = XX(:,5) - XX(:,14);
    dz = XX(:,6) - XX(:,15);
    drx = XX(:,7) - XX(:,16);
    dry = XX(:,8) - XX(:,17);
    drz = XX(:,9) - XX(:,18);
    
    % -- calculate the length between the markers
    dl = sqrt(dx.^2 + dy.^2);
    
    % -- calculate the mean and standard deviation for each marker pair
    data(:, 1, ii) = [mean(dl); mean(dz); mean(drx); mean(dry); mean(drz)];
    data(:, 2, ii) = [std(dl); std(dz); std(drx); std(dry); std(drz)];
    
    % -- create figure
    figure(1); gcf;
    
    % -- plot dl
    subplot(2,3,1);
    hold on; plot(dl);
    hold on; plot(1:size(dl,1), L*ones(size(dl,1),1), 'b-', 'linewidth', 2);
    xlabel('Time (s)', 'fontweight', 'normal', 'fontsize', fs);
    ylabel('Distance between markers (m)', 'fontweight', 'normal', 'fontsize', fs);
    
    % -- plot the dz
    subplot(2,3,2);
    hold on; plot(dz);
    xlabel('Time (s)', 'fontweight', 'normal', 'fontsize', fs);
    ylabel('Difference in Z height (m)', 'fontweight', 'normal', 'fontsize', fs);
    
    % -- plot the drx
    subplot(2,3,3);
    hold on; plot(drx);
    xlabel('Time (s)', 'fontweight', 'normal', 'fontsize', fs);
    ylabel('Difference in Rx (rad)', 'fontweight', 'normal', 'fontsize', fs);
    
    % -- plot the dry
    subplot(2,3,4);
    hold on; plot(dry);
    xlabel('Time (s)', 'fontweight', 'normal', 'fontsize', fs);
    ylabel('Difference in Ry (rad)', 'fontweight', 'normal', 'fontsize', fs);
    
    % -- plot the drz
    subplot(2,3,5);
    hold on; plot(drz);
    xlabel('Time (s)', 'fontweight', 'normal', 'fontsize', fs);
    ylabel('Difference in Rz (rad)', 'fontweight', 'normal', 'fontsize', fs);
    
    % -- save everything in the tracker error folder within the stats data folder
%     csvwrite(sprintf('stats data/TrackerError/Experiment 1/Marker pairs/Markerpair%03d.csv',ii), [dl,dz,drx,dry,drz]);
end

% -- Create title for each of the subplots
figure(1);
subplot(2,3,1); 
title(sprintf('True: %.3f m, Mean: %.3f $\\pm$ %.3f m', L, mean(data(1,1,:)), std(data(1,2,:))), 'Interpreter', 'latex', 'fontsize', 18);

subplot(2,3,2); 
title(sprintf('Mean: %.3f $\\pm$ %.3f m', mean(data(2,1,:)), std(data(2,2,:))), 'Interpreter', 'latex', 'fontsize', 18);

subplot(2,3,3); 
title(sprintf('Mean: %.3f $\\pm$ %.3f m', mean(data(3,1,:)), std(data(3,2,:))), 'Interpreter', 'latex', 'fontsize', 18);

subplot(2,3,4); 
title(sprintf('Mean: %.3f $\\pm$ %.3f m', mean(data(4,1,:)), std(data(4,2,:))), 'Interpreter', 'latex', 'fontsize', 18);

subplot(2,3,5); 
title(sprintf('Mean: %.3f $\\pm$ %.3f m', mean(data(5,1,:)), std(data(5,2,:))), 'Interpreter', 'latex', 'fontsize', 18);

% -- compare various distances between the measured markers
% -- populated as follows: [marker ID 1, Marker ID 2, distance measured real world]
measured = csvread('MarkerDist.csv');

% -- get the number of measured marker pairs throughout the lab
% -- measurements were done from marker to marker
Nmp = size(measured, 1);

% -- create array that will store the mean and standard deviation for each
% -- marker pair pose measurement
% -- Array of size [5 x 2 x Number of marker pairs]
% -- where the rows are (dl,z,rx,ry,rz) and columns are (mean, std)
data_mpair = zeros(5, 2, Np);

% -- calculate the errors of each measurement from the tracker to the real
% -- world measurement and calculate the overall error
% -- rows correspond to the marker pair, column is difference between measurement
error = zeros(Nmp, 2);

% -- loop through every measured marker pair and get the distance  
% -- and difference in pose between the markers in the pair
for ii = 1:Nmp
    
    % -- get the pair of markers that were measured
    m1 = find(markers == measured(ii,1));
    m2 = find(markers == measured(ii,2));
    
    % -- create series step that will get the correct data from X
    % -- that correponds to the specific marker pose 
    % -- Marker #1: 4 -> X, 5 -> Y, 6 -> Z, 7 -> Rx, 8 -> Ry, 9 -> Rz
    % -- Marker #2: 13 -> X, 14 -> Y, 15 -> Z, 16 -> Rx, 17 -> Ry, 18 ->Rz
    sx1 = 4 + 9*(m1 - 1); % -- X column for marker 1
    sx2 = 4 + 9*(m2 - 1); % -- X column for marker 2
    
    % -- store the two markers in the pair as a temp variable
    % -- eliminating the zeros from both
    XX = X(X(:,sx1)~=0 & X(:,sx2)~=0,[(sx1-3):(sx1+5), (sx2-3):(sx2+5)]);
    
    % -- caluclate the difference in pose between each marker
    dx = XX(:,4) - XX(:,13);
    dy = XX(:,5) - XX(:,14);
    dz = XX(:,6) - XX(:,15);
    drx = XX(:,7) - XX(:,16);
    dry = XX(:,8) - XX(:,17);
    drz = XX(:,9) - XX(:,18);
    
    % -- calculate the length between the markers
    dl = sqrt(dx.^2 + dy.^2);
    
    % -- calculate the mean and standard deviation for each marker pair
    data_mpair(:, 1, ii) = [mean(dl); mean(dz); mean(drx); mean(dry); mean(drz)];
    data_mpair(:, 2, ii) = [std(dl); std(dz); std(drx); std(dry); std(drz)];
    
    % -- create figure
    figure(2); gcf;
    
    % -- plot dl
    subplot(4,7,ii);
    hold on; plot(dl);
    hold on; plot(1:size(dl,1), measured(ii, 3)*ones(size(dl,1),1), 'b-', 'linewidth', 2);
    xlabel('Time (s)', 'fontweight', 'normal', 'fontsize', fs);
    ylabel('Distance (m)', 'fontweight', 'normal', 'fontsize', fs);
    title(sprintf('Distance between Marker %d to marker %d \nTrue: %.3f m, Mean: %.3f $\\pm$ %.3f m',...
          measured(ii, 1), measured(ii, 2), measured(ii, 3), data_mpair(1,1,ii), data_mpair(1,2,ii)), 'Interpreter', 'latex', 'fontsize', fs);
      
    legend('dl', 'true');
    
    % -- calculate the errors of each measurement from the tracker to the real
    error(ii, 1) = measured(ii,3) - data_mpair(1,1,ii); % -- difference in measurement between markers (m)
    error(ii, 2) = data_mpair(end,1,ii); % -- orientation between markers (rad)
    
    % -- save everything in the tracker error folder within the stats data folder
    csvwrite(sprintf('stats data/TrackerError/Measured pairs/Measuredpair%03d.csv',ii), [dl,dz,drx,dry,drz]);
    csvwrite(sprintf('stats data/TrackerError/Measured pairs/MeasuredpairData%03d.csv',ii), data_mpair(:,:,ii));
end

% -- get the mean and std of the error
LengthErrorData = [mean(error(:,1)), std(error(:,1))];
OrientationErrorData = [mean(error(:,2)), std(error(:,2))];

% -- save the tracker error values as a csv file
csvwrite('stats data/TrackerError/TrackerErrorData.csv', [LengthErrorData; OrientationErrorData]);

% -- plot the position of the marker on a figure to see the spread
for ii = 1:Nm
   figure(3); gcf;
   
   sx1 = 4 + 9*(ii - 1); % -- X column for marker 1
   % -- store the two markers in the pair as a temp variable
   % -- eliminating the zeros from both
   XX = X(X(:,sx1)~=0,(sx1-3):(sx1+5));
   
   hold on; plot(mean(XX(:,4)), mean(XX(:,5)), 's', 'markersize', 10, 'linewidth', 3);
end

end

function TrackerErrorMarkersGrid()

% -- read the data collected for finding the tracker error
% -- Define the directory that holds the csv file
% -- Remember: trail_0002-0004 -> markers all over Lab
% --           With the time collected increasing from 0002 -> 0004
dir = "Tracking Error Experiment/Experiment 2/trial_0003/data.csv";
X = csvread(dir, 1);

% -- get the number of markers used in the experiment
Nm = size(X, 2)/9;

% -- compare various distances between the measured markers
% -- populated as follows: [marker ID 1, Marker ID 2, distance measured real world]
measured = csvread('MarkerDist_Ex2.csv');

% -- get the number of measured marker pairs throughout the lab
% -- measurements were done from marker to marker
Nmp = size(measured, 1);

% -- define the markers used
markers = [25,21:1:24,26:1:34];

% -- plotting parameters to make figures nice
fs = 12;
ms = 2;

% -- we want to get the std values for the EKF
% -- loop through all markers 
for ii = 1: Nm
    
    % -- get the state of the marker
    sx1 = 4 + 9*(ii - 1); % -- X column for marker ii
    
    % -- store the markers state a temp variable
    % -- eliminating the zeros
    XX = X(X(:,sx1)~=0,(sx1-3):(sx1+5));
    
    % -- get the mean of the X and Y measurement separately
    Xmean = mean(XX(:, 4)); % -- 4th column is the X position data
    Ymean = mean(XX(:, 5)); % -- 5th column is the Y position data
    
    % -- subtract the mean from the measurement
    XX(:,4) = XX(:,4)-Xmean;
    XX(:,5) = XX(:,5)-Ymean;
    
    % -- append the X and Y data separately into a cell array
    CellDataX{ii} = XX(:,4);
    CellDataY{ii} = XX(:,5);
end

% -- convert the cell array into a single column array then
% -- get the standard deviation of the X and Y position data
STD_X_Data = std(cell2mat(CellDataX(:)));
STD_Y_Data = std(cell2mat(CellDataY(:)));
meanData = mean([cell2mat(CellDataX(:)) ; cell2mat(CellDataY(:))])
stdData = std([cell2mat(CellDataX(:)) ; cell2mat(CellDataY(:))])

% -- make a nice table to display when done running
STD = ["X"; "Y"];
Value = [STD_X_Data; STD_Y_Data];
DataTable = table(STD, Value)

% -- loop through every measured marker pair and get the distance  
% -- and difference in pose between the markers in the pair
for ii = 1:Nmp
    
    % -- get the pair of markers that were measured
    m1 = find(markers == measured(ii,1));
    m2 = find(markers == measured(ii,2));
    
    % -- create series step that will get the correct data from X
    % -- that correponds to the specific marker pose 
    % -- Marker #1: 4 -> X, 5 -> Y, 6 -> Z, 7 -> Rx, 8 -> Ry, 9 -> Rz
    % -- Marker #2: 13 -> X, 14 -> Y, 15 -> Z, 16 -> Rx, 17 -> Ry, 18 ->Rz
    sx1 = 4 + 9*(m1 - 1); % -- X column for marker 1
    sx2 = 4 + 9*(m2 - 1); % -- X column for marker 2
    
    % -- store the two markers in the pair as a temp variable
    % -- eliminating the zeros from both
    XX = X(X(:,sx1)~=0 & X(:,sx2)~=0,[(sx1-3):(sx1+5), (sx2-3):(sx2+5)]);
    
    % -- caluclate the difference in pose between each marker
    dx = XX(:,4) - XX(:,13);
    dy = XX(:,5) - XX(:,14);
    dz = XX(:,6) - XX(:,15);
    drx = XX(:,7) - XX(:,16);
    dry = XX(:,8) - XX(:,17);
    drz = XX(:,9) - XX(:,18);
    
    % -- calculate the length between the markers
    dl = sqrt(dx.^2 + dy.^2);
    
    % -- calculate the mean and standard deviation for each marker pair
    data_mpair(:, 1, ii) = [mean(dl); mean(dz); mean(drx); mean(dry); mean(drz)];
    data_mpair(:, 2, ii) = [std(dl); std(dz); std(drx); std(dry); std(drz)];
    
    % -- create figure
    figure(2); gcf;
    
    % -- plot dl
    subplot(4,7,ii);
    hold on; plot(dl);
    hold on; plot(1:size(dl,1), measured(ii, 3)*ones(size(dl,1),1), 'b-', 'linewidth', 2);
    xlabel('Time (s)', 'fontweight', 'normal', 'fontsize', fs);
    ylabel('Distance (m)', 'fontweight', 'normal', 'fontsize', fs);
    title(sprintf('Distance between Marker %d to marker %d \nTrue: %.3f m, Mean: %.3f $\\pm$ %.3f m',...
          measured(ii, 1), measured(ii, 2), measured(ii, 3), data_mpair(1,1,ii), data_mpair(1,2,ii)), 'Interpreter', 'latex', 'fontsize', fs);
      
    legend('dl', 'true');
    
    % -- calculate the errors of each measurement from the tracker to the real
    error(ii, 1) = measured(ii,3) - data_mpair(1,1,ii); % -- difference in measurement between markers (m)
    error(ii, 2) = data_mpair(end,1,ii); % -- orientation between markers (rad)
    
    % -- save everything in the tracker error folder within the stats data folder
    csvwrite(sprintf('stats data/TrackerError/Experiment 2/Measured pairs/Measuredpair%03d.csv',ii), [dl,dz,drx,dry,drz]);
    csvwrite(sprintf('stats data/TrackerError/Experiment 2/Measured pairs/MeasuredpairData%03d.csv',ii), data_mpair(:,:,ii));
end

% -- get the mean and std of the error
LengthErrorData = [mean(error(:,1)), std(error(:,1))];
OrientationErrorData = [mean(error(:,2)), std(error(:,2))];

% -- make a nice table to display when done running
Measurement = ["Distance"; "Rz Orientation"];
Mean = [mean(error(:,1)); mean(error(:,2))];
STD = [std(error(:,1)); std(error(:,2))];
DataTable = table(Measurement, Mean, STD)

% -- save the tracker error values as a csv file
% -- the first column is the mean and the second column is the standard deviation
csvwrite('stats data/TrackerError/Experiment 2/TrackerErrorData.csv', [LengthErrorData; OrientationErrorData]);

end

function C4b_TimeInPlace(ID_List, trials, conditions, ID_conditions, ID_Data)


% -- if speed is < 0.1 m/s save the number of timesteps
% -- where 0.1 m/s is the threshhold
thresh = 0.1;

% -- time step dt
dt = 0.5;

% -- bin size (the number of timesteps)
BinSize = 40;

% -- create a counter for the number of participants
% -- that will have their data saved
count = 1;

% -- loop through every trial ran
for TRIAL = 4:size(trials,2)
    % -- convert the trial/condition variable into a number
    trial = num2str(trials(TRIAL));
    condition = num2str(conditions(TRIAL));
    
    % -- loop through all IDs 
    for ID = 1:size(ID_List,1)
        
        % -- looking only at the participants that were given condition 4b
        if ID_Data(ID,2)
            % -- read the data from the csv file
            participantID = num2str(ID_List(ID));

            % -- read the data from the filtered data folder
            % -- and store the data in a variable "X"
            % -- State X is 5xT matrix where T is total time
            dir = strcat("filtered_data/", participantID, "/EKFVel_condition_", condition, ".csv");
            X = csvread(dir);
            
            diry = strcat("filtered_data/", participantID, "/EKFVel_condition_", condition, ".csv");
            Y = csvread(diry);
            
            % -- create an array that will be the size of the data
            % -- this will contain the number of timesteps that will
            % -- be within the threshold set for speed and turn rate
%             Nbins = floor(size(X(ID_Data(ID,end):end,1), 1)/BinSize);
            Nbins = floor(size(X, 1)/BinSize);
            TimeInPlace = zeros(Nbins, 1);
            
            % -- begin looping through each bin
            t = 0; %ID_Data(ID,2); % -- initialize counter
            for ii = 1:Nbins
                for jj = 1:BinSize
                    t = t + 1; % -- increment counter
                    if X(t,1) < thresh && Y(t,1) < thresh
                        TimeInPlace(ii, 1) = TimeInPlace(ii, 1) + 1;
                    end 
                end
            end
            
            % -- plot the results
            figure(1); gcf;
            hold on; plot(TimeInPlace * dt, 'LineWidth', 3, "Color", [0.4, 0.4, 0.4, 0.2]);
            
            % -- hold the values of all the bins to later get the mean
            % -- and std across all participants within a cell array
            BinData{count} = TimeInPlace * dt; % -- convert to seconds then save
            count = count + 1; % -- increment the counter by one
        end
        
    end
    
end

% -- get the max number of bins used for the data
% -- create an empty array that will hold the values of the bin data
Ns = size(BinData,2); % -- number of participants
size0 = 0;
for ii = 1:Ns % -- loop through all participants
    SizeVal = size(BinData{1,ii},1); % -- get the size of the bins used in data
    if SizeVal > size0 % -- check if its larger
       size0 = SizeVal; % -- save the current largest bins used
    end
end

MaxBinSize = size0; % -- relabel to new variable
BinValues = zeros(MaxBinSize,Ns); % -- size determined by the max number of bins found for a single participant

% -- calculate the mean and std values of the time to stay in place 
% -- for each participant
for participant = 1:Ns
    for Bin = 1:size(BinData{1,participant}(:,1),1)
        BinValues(Bin,participant) = BinValues(Bin,participant) + BinData{1,participant}(Bin,1);
    end
end

% -- calculate the mean and std of the bin values captured for each
% -- participant, the mean and std will correspond to each bin number
BinCalc = zeros(2,MaxBinSize);

for Bin = 1:MaxBinSize%size(BinData{1,participant}(:,1),1)
    if sum(BinValues(Bin,:))
        BinCalc(1,Bin) = mean(BinValues(Bin,[BinValues(Bin,:)~=0])); % -- mean calculation
        BinCalc(2,Bin) = std(BinValues(Bin,[BinValues(Bin,:)~=0])); % -- std calculation
    end
end

x = 1:numel(BinCalc(1,:));
curve1 = BinCalc(1,:) + BinCalc(2,:);
curve2 = BinCalc(1,:) - BinCalc(2,:);
x2 = [x, fliplr(x)];
inBetween = [curve1, fliplr(curve2)];
figure(1); gcf; 
% hold on; patch(x2, inBetween, [0 0 0.1], 'FaceAlpha', 0.35); % -- plotting the std shaded region
hold on; plot(BinCalc(1,:), 'LineWidth', 3, "Color", [0.4, 0.4, 0.4, 1]); % -- plotting the mean line


% -- make the figure look nice
fs = 18;
xlim([1 MaxBinSize]);
xlabel('Bin number');
ylabel('Total time in staying still (s)');
ax = gca;
ax.FontSize = fs;
grid on;

end

function StoreCommandedData(ID_List, trials, conditions, ID_conditions, ID_Data)

for TRIAL = 1:size(trials,2)
    trial = num2str(trials(TRIAL));
    condition = num2str(conditions(TRIAL));
    
    for ID = 1:size(ID_List,1)
        % -- read the data from the csv file
        participantID = num2str(ID_List(ID));
        onBoard = strcat('RAW', filesep, participantID,filesep, 'condition_',condition, filesep, 'WheelVel.csv');
        RpiData = csvread(onBoard); % -- store data in variable

        % -- begin to extract the individual wheel velocities and convert
        % -- them into commanded turn rate and speed 
        % -- Remember: the wheel speeds are stored in mm/s, convert to m/s
        Vel = ((RpiData(:,2) + RpiData(:,3))/ 1000) / 2;
        omega = ((RpiData(:,2) - RpiData(:,3))/ 1000) / .235;
        Vel = abs(Vel); omega = abs(omega); % -- take the absolute value of all values

        % -- create variables that define the location where the data will
        % -- be stored under the filtered data folder
        ComSpeedLoc = strcat('filtered_data', filesep, participantID, filesep, 'ComSpeed_condition_', num2str(TRIAL) ,'.csv');
        ComTurnRateLoc = strcat('filtered_data', filesep, participantID, filesep, 'ComTurnRate_condition_', num2str(TRIAL) ,'.csv');
        fprintf('Writing Com speed and turn rate data for participant: %4d of condition: %d \n', ID_List(ID), TRIAL);
        csvwrite(ComSpeedLoc, Vel);  pause(1);
        csvwrite(ComTurnRateLoc, omega); pause(1);
    end
end

end