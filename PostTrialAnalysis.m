function PostTrialAnalysis

% -- this function will serve as a main script to use 
% -- that will handle the individual functiosn that will
% -- analyze the data captured by the tracking system
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
trials = ["0002","0003","0004","0005"];
conditions = ["1","2","3","4"];

% -- functions for post trial analysis
FilterData(PartialRAW, ID_Data); % -- pass the data collected into an EKF and save everything
% FilterMany(ID_List, trials, conditions, ID_conditions, PartialRAW, ID_Data);
% AnimateTraj(); % -- function that will animate the trajectories
% PlotTraj(); % -- plot the trajectory of the robot in the Omron Lab
% PlotRotations();
% PlotAllVelocities(ID_List, trials, conditions, ID_conditions);
% PlotAllTurnRates(ID_List, trials, conditions, ID_conditions);
% AvgTime(ID_List, trials, conditions, ID_conditions, ID_Data);
% CompareVelocities(ID_List, trials, conditions, ID_conditions, ID_Data);
% CompareTurnRates(ID_List, trials, conditions, ID_conditions, ID_Data);
% SaveAllDataRPi(ID_List, trials, conditions, ID_conditions);
% UserKeyboard(ID_List, trials, conditions, ID_conditions);
% StoreTimeRM(ID_List, trials, conditions, ID_conditions,ID_Data);

end

function FilterData(PartialRAW, ID_Data)

% -- read the data from the csv file
participantID = '2276';
trial = '0005';
condition = '4';
DataFile = strcat(participantID,'/trial_',trial,'/data.csv');
onBoard = strcat(participantID,'/condition_',condition,'/WheelVel.csv');

data = csvread(DataFile, 2);
RpiData = csvread(onBoard);

[x, y, z, Rx, Ry, Rz, tx, ty, ID, time, Vel, omega] = ReadData(data, RpiData);

% -- do interp1 smoothing before EKF
Tq = 0:0.5:time(end);
X = x(x ~= 0); Y = y(x ~=0);
RZ = Rz(x ~= 0);
T_trials = time(x ~= 0);
Xq = interp1(T_trials, X, Tq);
Yq = interp1(T_trials, Y, Tq);
Rzq = interp1(T_trials, RZ, Tq);

% -- ground truth of what the tracker saw the robot do
gt = [Xq(1:end-2); Yq(1:end-2); Rzq(1:end-2)];
% gt = [ones(size(x,1),1), 5*ones(size(x,1),1), zeros(size(x,1),1)];
% gt(:,1) = linspace(0,14,size(x,1));
% gt(100:200,1) = 0;
Z = gt; % -- for consistency, make gt the measurement from the tracker
X0(1:3,1)=gt(1,1:3)';

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

% -- motion model for a differentially driven robot
% -- non-linear motion model
% v = 0.15; om = 0.15;
% Ffun=@(X,dt) unicycle1(X,v,om,dt);
% Flinfun=@(X,dt) unicycle_lin(X,v,dt); 

% -- process noise covariance matrix
Qk= @(x,dt) [cos(x(3))*dt 0; 
             sin(x(3))*dt 0; 
             0 dt]*...
            [1 0; 
             0 1]*...
            [cos(x(3))*dt 0; 
             sin(x(3))*dt 0; 
             0 dt]';
 
% >> measurement model and ^^ noise
% this one is simply identity
Hfun=@(X) eye(3)*X;

% because Hfun is linear, this is simply the same function
Hlinfun=@(X) eye(3); 

% -- noise for the tracker
Rk=diag([.0418^2 .0418^2 .0277^2]*1); 
% Rk=diag([.0418^2 .0418^2 .0277*5]*1);

% -- Kalman filter start and end point
k0=1;
kF=T;
 
% mean
Xh_(:,k0)=X0;
 
% covariance
P_(:,:,k0)=P0;
t0 = 0;
v0 = 0;
om0 = 0;



% kalman filter
for k=k0:kF
    
    % -- update time step
%     dt = time(k) - t0; % -- get the time difference between timestamps
%     t0 = time(k); % -- get the current timestamp from tracker
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
    timestep = find(round(time(k),1) == round(RpiData(:,1),1));
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
    
    % -- predict function for Extended Kalman Filter
    [Xh_(:,k+1), P_(:,:,k+1)]= ekf_predict(Xh(:,k), ...
        P(:,:,k), Qk(Xh(:,k),dt), Ffun, Flinfun, dt);
    
    % -- Extended Kalman Filter measurements
    Zt(:,k)=Hfun(Z(:,k));
end

% -- strictly for plotting purposes only!
% -- we want to omit the 0 measurement to make it clean
if PartialRAW
    newTime = time(x ~= 0);
    x = x(x ~= 0); y = y(y ~= 0);
else
    newTime = time;
end

% -- plot the data and estimates
figure(2); clf; gcf; 
% subplot(2,2,[1 2])

% -- linewdith and markersize variables
lw = 2;
ms = 16;

% -- get the omron lab mosaic and plot all data on top of it
OmronLabMap = imread('maps/OmronLabMosaicCrop_lowres.jpg');
imagesc([0 15],[-1 7.5], flip(OmronLabMap));
% if trial == '0002'
%     OmronLabMap = imread('maps/cond1_mosaic.png');
%     imagesc([-1 15],[-1 7.5], flip(OmronLabMap));
% else
%     OmronLabMap = imread('maps/OmronLabMosaicCrop.png');
%     imagesc([0 14.7],[0 7], flip(OmronLabMap));
% end

set(gca,'xdir','reverse','ydir','reverse');
hold on; plot(tx(1), ty(1), 'rs', 'markersize', ms, 'linewidth', lw);
hold on; plot(Xq(1), Yq(1), 'bs', 'markersize', ms, 'linewidth', lw);
hold on; plot(Xq(end), Yq(end), 'bx', 'markersize', ms, 'linewidth', lw);
% hold on; plot(Xq, Yq, 'linewidth', lw);
hold on; plot(Xh(1,:), Xh(2,:), 'b-', 'linewidth', lw);
hold on; plot(Xq(145), Yq(145), 'cx', 'markersize', ms, 'linewidth', lw);
% hold on; plot(Zt(1,:), Zt(2,:), 'b*', 'markersize', ms);
xlabel('X (m)', 'fontweight', 'normal', 'fontsize', 12);
ylabel('Y (m)', 'fontweight', 'normal', 'fontsize', 12);
axis image;
ax = gca;
ax.FontSize = 18;
% legend('Target location','Robot start', 'Robot End','RAW Trajectory','EKF trajectory','EKF measurements');

% % -- plotting the x vs time for RAW and filtered data
% subplot(2,2,3);
% plot(Tq(1:end-2), Xq(1:end-2), 'linewidth', lw); % -- measured
% % hold on; plot(time, Xh(1,1:end), 'linewidth', lw); % -- filtered
% hold on; errorbar(Tq(1:end-2), Xh(1,1:end), squeeze(sqrt(P(1,1,:))), 'linewidth', 1);
% % hold on; plot(time, Xh_(1,1:end-1), 'linewidth', lw); % -- filtered predicted
% grid on;
% xlabel('Time (s)', 'fontweight', 'normal', 'fontsize', 12);
% ylabel('X (m)', 'fontweight', 'normal', 'fontsize', 12);
% legend('RAW X', 'EKF X', 'EKF pred');
% 
% % -- plotting the y vs time for RAW and filtered data
% subplot(2,2,4);
% plot(Tq(1:end-2), Yq(1:end-2), 'linewidth', lw); % -- measured
% % hold on; plot(time, Xh(2,1:end), 'linewidth', lw); % -- filtered
% hold on; errorbar(Tq(1:end-2), Xh(2,1:end), squeeze(sqrt(P(2,2,:))), 'linewidth', 1);
% % hold on; plot(time, Xh_(2,1:end-1), 'linewidth', lw); % -- filtered predicted
% grid on;
% xlabel('Time (s)', 'fontweight', 'normal', 'fontsize', 12);
% ylabel('Y (m)', 'fontweight', 'normal', 'fontsize', 12);
% legend('RAW Y', 'EKF Y');

% -- save the filtered data
%SaveXh = strcat(participantID,'/trial_',trial,'/Xh.csv');
%SaveZt = strcat(participantID,'/trial_',trial,'/Zt.csv');
%csvwrite(SaveXh, Xh);
%csvwrite(SaveZt, Zt);

end

function FilterMany(ID_List, trials, conditions, ID_conditions, PartialRAW, ID_Data)
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
                     0 .05]*...
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

function AnimateTraj()

% -- read the data from the csv file
data = csvread('data.csv', 2);
[x, y, z, Rx, Ry, Rz, tx, ty, ID, time] = ReadData(data);

% -- smooth out the trajectory data so it doesn't look as jagged
sx = movmean(x, 5);
sy = movmean(y, 5);

% -- create empty figure
figure(1); clf; gcf;

% -- plot the target location and the starting position of the robot
plot(tx, ty, 'kd', 'markersize', 12, 'linewidth', 3); hold on;
plot(x(1), y(1), 'bx', 'markersize', 12, 'linewidth', 3); hold on;
axis([0 16 0 7]); % -- set the boundary of the domain
xlabel('X (m)', 'fontsize', 28, 'fontweight', 'normal'); % -- X-axis label
ylabel('Y (m)', 'fontsize', 28, 'fontweight', 'normal'); % -- Y-axis label

% -- loop throughout the data set
for i = 1:size(x,1)
    % -- begin ploting the robot position over time
    plot(x(1:i), y(1:i), 'b-', 'linewidth', 2); hold on; % -- raw robot trajectory path
    plot(sx(1:i), sy(1:i), 'k-', 'linewidth', 2); hold on; % -- smoothed robot trajectory path
    plot(x(i), y(i), 'r*', 'markersize', 6); hold on; % -- raw robot trajectory position points
    axis([0 16 0 7]); % -- set the boundary of the domain
    title(sprintf('Time: %.4f', time(i)), 'fontsize', 28, 'fontweight', 'normal'); % -- display time on the title
    pause(.1);
end

end

function PlotTraj()
% -- read the data from the csv file
data = csvread('data.csv', 2);
[x, y, z, Rx, Ry, Rz, tx, ty, ID, time] = ReadData(data);

% -- smooth out the trajectory data so it doesn't look as jagged
sx = movmean(x, 5);
sy = movmean(y, 5);

% -- create empty figure
figure(1); clf; gcf;
ax = gca; % -- we want to get the properties of the figure
ms = 24;
mlw = 4;

plot(x, y, 'b', 'linewidth', 2); % -- robot trajectory
hold on; plot(sx, sy, 'r', 'linewidth', 2); % -- moving average of robot trajectory
hold on; plot(x(1), y(1), 'sb', 'markersize', ms, 'linewidth', mlw); % -- starting point of robot
hold on; plot(x(end), y(end), 'bx', 'markersize', ms, 'linewidth', mlw); % -- end point of robot
hold on; plot(tx(1), ty(1), 'kd', 'markersize', ms, 'linewidth', mlw);
xlabel('X(m)', 'fontsize', 24); ylabel('Y(m)', 'fontsize', 24)
title(sprintf('2D trajectory of robot on ground, time: %.2f s', time(end)), 'fontsize', 24);
legend('Robot Trajectory', 'MA Robot Trajectory', 'Robot Start', 'Robot End', 'Target Location',...
       'fontsize', 16, 'fontweight', 'normal');
grid on; axis([0 16 0 7]);
ax.FontSize = 24; % -- make the fontsize equal across the board
end

function PlotRotations()
% -- read the data from the csv file
data = csvread('data.csv', 2);
[x, y, z, Rx, Ry, Rz, tx, ty, ID, time] = ReadData(data);

% -- make sure that the time vector is the same length as the others
if size(time,1) ~= size(Rz,1)
   time = [0;time]; 
end

% -- store the rotations in a single variable
rotation = [Rx, Ry, Rz];

% -- store all the title strings into a single variable
titleLabels = ['Rotation about the X-axis',...
               'Rotation about the Y-axis',...
               'Rotation about the Z-axis'];

% -- create an empty figure
figure(1); clf; gcf;
           
for AX = 1:3
    % -- create subplot and store as a variable to access properties
    ax = subplot(1,3,AX);
    
    % -- plot the rotations
    plot(time, rotation(:,AX));
    ylabel('rotation(rad)'); xlabel('time(s)') % -- label the axis
    title(titleLabels(AX)); % -- add the title
    grid on; grid minor; axis([0 time(end) -pi pi]); axis square; % -- make the plot nice
    ax.FontSize = 24; % -- modify all font sizes
end

end

function PlotAllVelocities(ID_List, trials, conditions, ID_conditions)

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
        onBoard = strcat(participantID,'/condition_',condition,'/WheelVel.csv');

        % -- store the data
        RpiData = csvread(onBoard);
        Vel = ((RpiData(:,2) + RpiData(:,3))/ 1000) / 2;
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
                AllVel(ID, TRIAL) = mean(Vel);
                AllVel(ID, TRIAL+1) = nan;
                VelCel{1,CelColumn} = {[[0; TRIAL; time], [0; ID_List(ID); Vel]]};
            else
                % -- if break trust
                AllVel(ID, TRIAL) = nan;
                AllVel(ID, TRIAL+1) = mean(Vel);
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
        omega = abs(omega);

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
        
        % -- label the y axis of the entire set to say the condition number
%         if Fig_suplot == (size(ID_List,1)*str2double(condition) - 17)
%             ylabel(sprintf('condition: %d \n Turn rate (rad/s)', cond_count), 'fontweight', 'normal', 'fontsize', 16);
%             cond_count = cond_count + 1;
%         end
        
        if condition == '4'
            xlabel('Time (s)', 'fontweight', 'normal', 'fontsize', 16);
        end
        
        if ID == 1
            ylabel('Turn rate (rad/s)', 'fontweight', 'normal', 'fontsize', 12);
        end
        title(sprintf('Mean Turn rate: %.2f rad/s', mean(omega)), 'fontweight', 'normal', 'fontsize', 12); 
        ax = gca; 
        ax.FontSize = 12;
        
%         if condition == '1' && ID == size(ID_List,1)/2
%            title(sprintf('Turn rate VS Time \n Mean Turn rate: %.2f rad/s', mean(omega)), 'fontweight', 'normal', 'fontsize', 12); 
%            
%         else
%             title(sprintf('Mean Turn rate: %.2f rad/s', mean(omega)), 'fontweight', 'normal', 'fontsize', 12); 
%         end
        
        % -- increment the subplot counter
        Fig_suplot = Fig_suplot + 1;
    end
end

anova1(Allomega);
set(gca, 'xticklabel', {'1', '2', '3', '4a', '4b'});
xlabel('Trial Conditions', 'fontweight','normal', 'fontsize', 16);
ylabel('Turn Rate (rad/s)', 'fontweight','normal', 'fontsize', 16);

end

function AvgTime(ID_List, trials, conditions, ID_conditions, ID_Data)

% -- hold all the time values
AllTimes = zeros(size(ID_List,1), size(trials,2));

k_tf = 2.27;

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
        
        % -- for all trials before the last condition
        if TRIAL < 4
            if TRIAL == 1
                % -- scale the time using the value determined through testing
                AllTimes(ID, TRIAL) = time(end)/k_tf;
            else
                AllTimes(ID, TRIAL) = time(end);
            end
        else
            % -- looking at the last condition
            AllTimes(ID, TRIAL) = time(ID_Data(ID, end));
        end
        
    end
end

% -- plot the ANOVA
% p = anova1(AllTimes);
% set(gca, 'xticklabel', {'1', '2', '3', '4a', '4b'});
% xlabel('Trial Conditions', 'fontweight','normal', 'fontsize', 16);
% ylabel('Time (s)', 'fontweight','normal', 'fontsize', 16);
end

function CompareVelocities(ID_List, trials, conditions, ID_conditions, ID_Data)

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
        onBoard = strcat(participantID,'/condition_',conditions(TRIAL),'/WheelVel.csv');
        DataFile = strcat(participantID,'/trial_',trials(TRIAL),'/data.csv');
        data = csvread(DataFile, 2);
        % -- store the data
        RpiData = csvread(onBoard);
        Vel = ((RpiData(:,2) + RpiData(:,3))/ 1000) / 2;
        time = RpiData(:,1);
        [x, y, z, Rx, Ry, Rz, tx, ty, CamID, TIME, ~, omega] = ReadData(data, RpiData);

        % -- read the EKF Velocity and turn rate data previously saved
        EKF_Vel_data = strcat('filtered_data/', participantID, '/EKFVel_condition_', num2str(TRIAL),'.csv');
        EKF_om_data = strcat('filtered_data/', participantID, '/EKFom_condition_', num2str(TRIAL),'.csv');
        EKF_Vel = csvread(EKF_Vel_data);
        EKF_om = csvread(EKF_om_data);
        TIME = 0:0.5:(size(EKF_Vel,1)*.5)-0.5;
        t1 = size(time,1);
        t2 = size(TIME,2);
        rpiT = floor(t1/t2);
        
%         % -- Initialize the subplot to be used
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
%         nexttile
%         plot(time, Vel, 'k-', 'linewidth', lw); % -- plot the RAW velocity
% %         hold on; plot(time, mean(Vel)*ones(size(time)), 'r-', 'linewidth', lw); % -- plot the mean velocity
% %         plot(TIME, EKF_Vel, 'b-', 'linewidth', lw); % -- plot the EKF estimated velocity
%         %hold on; plot(TIME, mean(EKF_Vel)*ones(size(TIME)), 'c-', 'linewidth', lw); % -- plot the mean velocity of EKF
%         set(gca,'xtick',[0:round(time(end)/4, -1):time(end)])
%         axis([0 time(end) 0 .5]);
%         set(gca,'ytick',[0:0.25:.5]);
%         grid on;

%         % -- save the mean vel
        if TRIAL < 4
            % -- for all trials before the last condition
            AllVel(ID, TRIAL) = mean(Vel);
            AllEKFVel(ID, TRIAL) = mean(EKF_Vel);
        else
            % -- for the last condition
                AllVel(ID, TRIAL) = mean(Vel(1:rpiT*ID_Data(ID, end),1));
                AllEKFVel(ID, TRIAL) = mean( EKF_Vel(1:ID_Data(ID, end),1) );
        end
        
        % -- label the y axis of the entire set to say the condition number
%         if Fig_suplot == (size(ID_List,1)*str2double(condition) - 23)
%             ylabel(sprintf('condition: %d \n Velocity (m/s)', cond_count), 'fontweight', 'normal', 'fontsize', 12);
%             cond_count = cond_count + 1;
%         end
        
        if condition == '4'
            xlabel('Time (s)', 'fontweight', 'normal', 'fontsize', 12);
        end
        
        if ID == 1
            ylabel({'Input';'speed (m/s)'}, 'fontweight', 'normal', 'fontsize', 12);
        end
        
        title(sprintf('Mean speed: %.2f m/s', mean(Vel)), 'fontweight', 'normal', 'fontsize', 12);
        
        ax = gca; 
        ax.FontSize = 12;
        
%         if condition == '1' & ID == size(ID_List,1)/2
%            title(sprintf('Velocity VS Time \n Mean \n Vel: %.2f m/s', mean(Vel)), 'fontweight', 'normal', 'fontsize', 8); 
%            
%         else
%             title(sprintf('Mean \n Vel: %.2f m/s', mean(Vel)), 'fontweight', 'normal', 'fontsize', 8); 
%         end
        
        % -- increment the subplot counter
        Fig_suplot = Fig_suplot + 1;
    end
end

% anova1(AllVel);
% set(gca, 'xticklabel', {'1', '2', '3', '4a', '4b'});
% xlabel('Trial Conditions', 'fontweight','normal', 'fontsize', 16);
% ylabel('Velocity (m/s)', 'fontweight','normal', 'fontsize', 16);
% title('ANOVA on RPi Velocity', 'fontweight','normal', 'fontsize', 16);
% 
% anova1(AllEKFVel);
% set(gca, 'xticklabel', {'1', '2', '3', '4a', '4b'});
% xlabel('Trial Conditions', 'fontweight','normal', 'fontsize', 16);
% ylabel('Tracker Velocity (m/s)', 'fontweight','normal', 'fontsize', 16);
% title('ANOVA on EKF Tracker Velocity', 'fontweight','normal', 'fontsize', 16);

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
        onBoard = strcat(participantID,'/condition_',conditions(TRIAL),'/WheelVel.csv');
        DataFile = strcat(participantID,'/trial_',trials(TRIAL),'/data.csv');
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

function SaveAllDataRPi(ID_List, trials, conditions, ID_conditions)
cond_count = 1;

% -- hold all mean velocity values
AllVel = zeros(size(ID_List,1), 5);

% -- create cell array to contain all time/vel data
DataCel = {};
EKF_cel = {};
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
        onBoard = strcat(participantID,'/condition_',condition,'/WheelVel.csv');
        EKF_Vel_data = strcat('filtered_data/', participantID, '/EKFVel_condition_', num2str(TRIAL),'.csv');
        EKF_om_data = strcat('filtered_data/', participantID, '/EKFom_condition_', num2str(TRIAL),'.csv');
        DataFile = strcat(participantID,'/trial_',trials(TRIAL),'/data.csv');
        data = csvread(DataFile, 2);
        RpiData = csvread(onBoard);
        [~, ~, ~, ~, ~, ~, ~, ~, ~, TIME, ~, ~] = ReadData(data, RpiData);
        
        EKF_Vel = csvread(EKF_Vel_data);
        EKF_om = csvread(EKF_om_data);

        % -- store the data
        Vel = ((RpiData(:,2) + RpiData(:,3))/ 1000) / 2;
        omega = ((RpiData(:,2) - RpiData(:,3))/ 1000) / .3084;
        time = RpiData(:,1);
        
        % -- save the mean vel
        if TRIAL < 4
            % -- for all trials before the last condition
            DataCel{1,CelColumn} = {[[nan; TRIAL; time], [nan; ID_List(ID); Vel], [nan; ID_List(ID); omega]]};
            EKF_cel{1,CelColumn} = {[[nan; TRIAL; TIME], [nan; ID_List(ID); EKF_Vel], [nan; ID_List(ID); EKF_om]]};
        else
            % -- for the last condition, if keep trust
            if ID_conditions(ID) == 0
                DataCel{1,CelColumn} = {[[0; TRIAL; time], [0; ID_List(ID); Vel], [0; ID_List(ID); omega]]};
                EKF_cel{1,CelColumn} = {[[0; TRIAL; TIME], [0; ID_List(ID); EKF_Vel], [0; ID_List(ID); EKF_om]]};
            else
                % -- if break trust
                DataCel{1,CelColumn} = {[[1; TRIAL; time], [1; ID_List(ID); Vel], [1; ID_List(ID); omega]]};
                EKF_cel{1,CelColumn} = {[[1; TRIAL; TIME], [1; ID_List(ID); EKF_Vel], [1; ID_List(ID); EKF_om]]};
            end
        end
        
        CelColumn = CelColumn + 1;
        
    end
end

end

function UserKeyboard(ID_List, trials, conditions, ID_conditions)

tiledlayout(size(trials,2)+1,size(ID_List,1));

time_key_pressed = zeros(1,4);


% -- loop through every single ID number in the list
for TRIAL = 1:size(conditions,2)
    
    % -- store the condition value
    condition = num2str(conditions(TRIAL));
    
    for ID = 1:size(ID_List,1)
        
        % -- read the data from the csv file
        participantID = num2str(ID_List(ID));
        onBoard = strcat(participantID,'/condition_',conditions(TRIAL),'/WheelVel.csv');
        
        % -- store the data
        % -- remember how the data is stored:
        % -- [time, vL, vR, key up, key down, key left, key right]
        RpiData = csvread(onBoard);
        time = RpiData(:, 1);
        keys = RpiData(:, 4:end);
        
        % -- get the average dt of the system
        all_dt = zeros(1, size(time, 1)-1);
        for timestep = 1:size(time,1)-1
            all_dt(1,timestep) = time(timestep +1) - time(timestep);
        end
        
        avg_dt = mean(all_dt);
        
        for key = 1:size(keys, 2)
            time_key_pressed(1,key) = avg_dt*sum(keys(:,key));
        end
        
        overall = sum(time_key_pressed);
        
        % -- convert the total time pressed into percent
        time_key_pressed = 100*(time_key_pressed/overall);
        
        nexttile
        bar([time_key_pressed(1,1), time_key_pressed(1,2), time_key_pressed(1,3)+time_key_pressed(1,4)]);
        
        if ID == 1
           ylabel('Total Percent', 'fontweight', 'normal', 'fontsize', 12); 
        end
        
        if TRIAL == 4
           xlabel('Manuever', 'fontweight', 'normal', 'fontsize', 12); 
        end
        
        ax = gca; 
%         ax.XTickLabel = {'Forward', 'Backward', 'rotate left', 'rotate right'};
        ax.XTickLabel = {'F', 'B', 'R'};
        ax.FontSize = 12;
        ax.YLim = [0, 100];
        
    end
end

end

function StoreTimeRM(ID_List, trials, conditions, ID_conditions, ID_Data)
tiledlayout(1,size(ID_List,1));

for TRIAL = 4:size(conditions,2)
    
    % -- store the condition value
    condition = num2str(conditions(TRIAL));
    
    for ID = 1:size(ID_List,1)
        
        % -- read the data from the csv file
        participantID = num2str(ID_List(ID));
        EKF_traj_data = strcat('filtered_data/', participantID, '/EKFtraj_condition_', num2str(TRIAL),'.csv');
        EKF_traj = csvread(EKF_traj_data);
        EKF_X = movmean(EKF_traj(1,:),15); % -- smooth the data with window size of 15
        
        nexttile
        %plot();
        plot(EKF_traj(1,:), EKF_traj(2,:),'.', 'markersize', 8);
        hold on; plot(EKF_traj(1,ID_Data(ID, 5)), EKF_traj(2,ID_Data(ID, 5)),'k.','markersize', 18);
        
    end
end

end