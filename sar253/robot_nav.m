function robot_nav
% this example runs an ekf on a setup with nonlinear motion model 
% and nonlinear measurement model


% seeding random number generator
rng(2);

% to lengthen or shorten the simulation, change Tfinal
% to modify the measurement step, change dt
dt=1/5;
Tfinal = 100;
tspan = 0:dt:Tfinal; % time for simulation

T = size(tspan,2); % the length of the tspan vector

% simulate the position of the target
% the target position can be stationary or dynamic
[xj, yj]=simulate_target(tspan, dt);

% ready to filter
% initialize robot location and orientation
X0(1:3,1)=rand(3,1)*.5;
X0(4:5,1)=20+randn(2,1); 

P0=eye(5)*1; % ^^ initial covariance
%P0(4,4) = 100; P0(5,5) = 100;

try_extended_kalman_filter(dt, X0, P0, xj, yj, T, 0, tspan);

function [xj, yj]=simulate_target(T, dt)

% % ode with initial conditions 
% i1=[2 2 0];
% v = 10; % initial velocity and rotation
% [~,gt] = ode45(@unicycle_ode,tspan,i1,[],v);
% % ^^ add noise with intensity w0; gt is ground truth
% 
% % measurement model is for a radar
% eta=[1 .1];
% gt = [gt , ones(numel(gt(:,1)),1)*gt(end,1), ones(numel(gt(:,1)),1)*gt(end,2)];
% Z=radar1(gt')+[randn(1, size(gt,1))*eta(1);
%                randn(1, size(gt,1))*eta(2)];

% the target is static
xj=ones(1,numel(T))*20;
yj=ones(1,numel(T))*20;

% the target is dynamic, but it only moves a little bit
% but the movements are gaussian distributed
% the motion is very erratic, not recommended to use at all
% but very interesting to look at
% xj=ones(1,numel(T))*10 + 2*randn(1,numel(T));
% yj=ones(1,numel(T))*10 + 2*randn(1,numel(T));

% the target is dynamic, but it only moves a little bit
% but the movements are only in the x direction
% xmove = zeros(1,numel((T)));
% for i = 2:numel(T)
%     xmove(i) = xmove(i-1) + 1;
% end
% xj=ones(1,numel(T))*10 + xmove;
% yj=ones(1,numel(T))*10;

% the target is dynamic, but it only moves a little bit
% but the movements are only in the x direction
% ymove = zeros(1,numel((T)));
% for i = 2:numel(T)
%     ymove(i) = ymove(i-1) + 1;
% end
% xj=ones(1,numel(T))*10;
% yj=ones(1,numel(T))*10 + ymove;

% the target is dynamic, but it only moves a little bit
% and the movements are in both the x and y direction
% xmove = zeros(1,numel((T)));
% ymove = zeros(1,numel((T)));
% for i = 2:numel(T)
%     xmove(i) = xmove(i-1) + 1;
%     ymove(i) = ymove(i-1) + 1;
% end
% xj=ones(1,numel(T))*10 + xmove;
% yj=ones(1,numel(T))*10 + ymove;
 
function try_extended_kalman_filter(dt, X0, P0, xj, yj, T, close_loop, tspan)
k0=1; % beginning
kF=T; % final

n=size(X0,1); % get the size of the state

% set up the matricies for covariance and estimate/predicted state
Xh=zeros(n,kF); 
Xs=zeros(3,kF); Xs(:,1)=X0(1:3,1);
Xh_=Xh;
P=zeros(n,n,kF); P_=P;

% motion noise covariance matrix
% unicycle model discrete form
v0=1; omega0=.1;
Ffun=@(x,v,omega,dt) unicycle1(x,v,omega,dt); % for the virtual sim
 
% ^^ this is where we plug in the Jacobian of a nonlinear motion model 
% evaluated at x  
Flinfun=@(x,v,dt) unicycle_lin(x,v,dt); % for the virtual sim
% 
% Qk= @(x,dt) blkdiag([cos(x(3))*dt 0; sin(x(3))*dt 0; 0 dt;]*...
%             [1 0; 
%             0 .1]*[cos(x(3))*dt 0; sin(x(3))*dt 0; 0 dt;]', eye(2)*.1);


Qk= @(x,dt) [cos(x(3))*dt 0 0 0; 
             sin(x(3))*dt 0 0 0; 
             0 dt 0 0;
             0 0 1 0;
             0 0 0 1]*...
            [0 0 0 0; 
            0 0 0 0
            0 0 0 0
            0 0 0 0]*...
            [cos(x(3))*dt 0 0 0; 
             sin(x(3))*dt 0 0 0; 
             0 dt 0 0;
             0 0 1 0;
             0 0 0 1]';

 
% >> measurement model and ^^ noise
Hfun=@(x) radar1(x); 
Hlinfun=@(x) radar1_lin(x);

% measurement noise covariance matrix
eta=[.1 pi/180];
Rk=[eta(1) 0;
    0 eta(2)];

Xh_(:,k0)=X0; % mean
P_(:,:,k0, 1)=P0; % covariance 

% for running the motion model virtually
omega = .5:.1:1.5;
v = 6:.5:12;

[W, V] = ndgrid(omega, v);

% Z=zeros(2,size((T),1));
Z=zeros(2,kF); % create space for all of the measurements

% kalman filter
for k=k0:kF
    
    % get the distance and orientation measurements from
    % the robots perspective

    Z(:,k)=get_measurements(Xs(:,k), xj(k),yj(k)) + [randn*eta(1);
                                                     randn*eta(2)]; 
    
    % you have to run the motion model virtually over a range of omega 0.5
    % to 1.5 and v from 6 to 12 so that you have n_omega x n_v different
    % values of P_ and therefore S
    % pick the value of omega and v that minimize norm(S)
    % use the omega and v to update your Ffun, which you pass next
    if close_loop
        s = zeros(numel(W),1);

        for kk = 1:numel(W)
         

           % predict
           [Xh_(:,k+1), P_(:,:,k+1,1)]= ekf_predict1(Xh(:,k), ...
               P(:,:,k,1), Qk(Xh(:,k),dt), Ffun(Xh(:,k), V(kk), W(kk), dt), ...
               Flinfun(Xh(:,k), V(kk), dt));
             % update
           [~,~, S]=ekf_update(Xh_(:,k+1), ...
               P_(:,:,k+1,1), Rk, Z(:,k), Hfun, Hlinfun);

            s(kk) = norm(S);
        end

        [val, idx]=min(s);

        % update
        [Xh(:,k), P(:,:,k,1)]=ekf_update(Xh_(:,k), ...
            P_(:,:,k,1), Rk, Z(:,k), Hfun, Hlinfun);

        % predict
        [Xh_(:,k+1), P_(:,:,k+1,1)]= ekf_predict1(Xh(:,k), ...
            P(:,:,k,1), Qk(Xh(:,k),dt), Ffun(Xh(:,k), V(idx), W(idx), dt), ...
            Flinfun(Xh(:,k), V(idx), dt));
        
        % simulate the robot
        Xs(:,k+1)=unicycle1(Xs(:,k),V(idx),W(idx), dt);
    else
        % update
        [Xh(:,k), P(:,:,k,1)]=ekf_update(Xh_(:,k), ...
            P_(:,:,k,1), Rk, Z(:,k), Hfun, Hlinfun);

        % predict
        [Xh_(:,k+1), P_(:,:,k+1,1)]= ekf_predict1(Xh(:,k), ...
            P(:,:,k,1), Qk(Xh(:,k),dt), Ffun(Xh(:,k), v0, omega0, dt), ...
            Flinfun(Xh(:,k), v0, dt));
        
        % simulate the robot
%         Xs(:,k+1)=unicycle1(Xs(:,k),v0,omega0, dt);
        Xs(:,k+1)=unicycle1(Xs(:,k),v0,omega0, dt);

  
    end
    
end
 
show_the_results(Z, Xs, Xh, P, xj, yj, tspan);
 
function show_the_results(Z, Xs, Xh, P, xj, yj, tspan)

sr=2; % number of rows in the figure
sc=4; % number of columns in the figure
 
% plots the results of the estimated state of both
% the robot and the target in the inertial frame
figure(1); gcf; clf;
ylabels={'x_1', 'x_2', '\theta', 'x_j', 'y_j'};
for ii=1:5
    subplot(sr,sc,ii); gca;
    errorbar(tspan, Xh(ii,:), squeeze(sqrt(P(ii,ii,:))), 'linewidth', 1);
    hold on;
    if ii<=3
        plot(tspan, Xs(ii,1:end-1), 'r');
    end
    set(gca, 'fontname', 'times', 'fontsize', 10);
    grid on; axis image; axis square;
    ylabel(ylabels{ii}); xlabel('time (s)');
    
    if ii==2
        title('Model C','fontsize',20);
    end
    
    if ii == 4
        hold on;
        plot(tspan,xj, 'r'); % plot the true X position of the target 
        % plot the confidence regions
%         hold on; plot(tspan, 1*sqrt(cat(1,squeeze(P(ii,ii,:))))'+xj, 'k--', 'linewidth', 3);
%         hold on; plot(tspan, -1*sqrt(cat(1,squeeze(P(ii,ii,:))))'+xj, 'k--', 'linewidth', 3);
    end
    
    if ii == 5
        hold on;
        plot(tspan,yj, 'r'); % plot the true Y position of the target
        % plot the confidence regions
%         hold on; plot(tspan, 1*sqrt(cat(1,squeeze(P(ii,ii,:))))'+yj, 'k--', 'linewidth', 3);
%         hold on; plot(tspan, -1*sqrt(cat(1,squeeze(P(ii,ii,:))))'+yj, 'k--', 'linewidth', 3);
    end
end
xlabel('time');

% plot the robots estimated path along with 
% it's time step position, start and end point and target position
subplot(sr,sc,6);
plot(Xs(1,:), Xs(2,:), 'r', 'linewidth', 2); % estimate
hold on; plot(Xh(1,:), Xh(2,:), 'co', 'Markersize', 8); % Time step position of estimate 
plot(Xh(1,1), Xh(2,1), '^k', 'linewidth', 2, 'markersize', 16); % beginning point
plot(Xh(1,end), Xh(2,end), 'k.', 'linewidth', 5, 'markersize', 25); % stopping point
plot(xj, yj, 'sg', 'linewidth', 2, 'markersize', 16); % Target Position
axis image; axis square; xlabel('x_1'); ylabel('x_2');
grid on; grid minor;
set(gca, 'fontname', 'times', 'fontsize', 10);
legend('Estimate','Time step position estimate',...
       'Robot Start', 'Robot End', 'Target Position', 'location', 'northwest');

% plot the original target position and 
% the estimated state of the target position
subplot(sr,sc,7);
plot(xj, yj, 'sg', 'linewidth', 2, 'markersize', 16); % Target Position
hold on; plot(Xh(4,:), Xh(5,:), 'k.', 'markersize', 6); % estimated state of the target
axis image; axis square; xlabel('x_j'); ylabel('y_j');
grid on; grid minor;
legend('Target Position', 'Estimated Target position');

function Xdot = unicycle_ode(t,X,v)
Xdot(1,1) = v*cos(X(3));
Xdot(2,1) = v*sin(X(3));
% set omega as a function of time to create interesting trajectory
Xdot(3,1) = 1*sin(.5*t);
 
function X = unicycle1(X,v,omega, dt)
X(1,1) = X(1,1) + v*cos(X(3,1))*dt;
X(2,1) = X(2,1) + v*sin(X(3,1))*dt;
X(3,1) = X(3,1) + omega*dt;
 
function flin = unicycle_lin(X,v,dt)
flin=[1 0 -v*sin(X(3,1))*dt 0 0;
      0 1 v*cos(X(3,1))*dt 0 0;
      0 0 1 0 0;
      0 0 0 1 0;
      0 0 0 0 1];
 
function z = get_measurements(X, xj,yj)
% from the notes created through the meetings
z(1,:)=sqrt((xj-X(1,:)).^2 + (yj-X(2,:)).^2);
z(2,:)=atan2((yj-X(2,:)), (xj-X(1,:))-X(3,:));  
  
function z = radar1(X)
% from the notes created through the meetings
z(1,:)=sqrt((X(4,:)-X(1,:)).^2 + (X(5,:)-X(2,:)).^2);
z(2,:)=atan2((X(5,:)-X(2,:)), (X(4,:)-X(1,:))-X(3,:));

function Hk = radar1_lin(X)
xij = X(4,1)-X(1,1); yij = X(5,1)-X(2,1);
rij = sqrt(xij^2 + yij^2);

Hk=[-xij/rij, -yij/rij, 0, xij/rij, yij/rij
    yij/rij^2, -xij/rij^2, -1, -yij/rij^2, xij/rij^2];