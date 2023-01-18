function example03
% this example runs an ekf on a setup with linear motion model 
% and nonlinear measurement model
 
% NOTE
% >> in the script means a big change is needed for you to adapt it to your
% setup. e.g. a new likelihood function or measurement model
% ^^ means there are parameters that can be tuned to see if it changes the
% performance
 
% generating 2D data from a mobile robot with unicycle kinematics
% set the parameters
dt=1/5;
tspan = 0:dt:20; % time for simulation
 
% ode with initial conditions 
i1=[2 2 0]; % x,y,theta
v=10; 
% this is just a simulation so we keep the unicycle model because it makes
% nice tracks.
[~,gt] = ode45(@unicycle_ode,tspan,i1,[],v);
 
% measurement model is for a radar
eta=.1;
global xj yj
xj = -23.9374; yj = 32.575;

Z=radar1([gt(:,1:2)'*dt; gt(:,1).*cos(gt(:,3))'; gt(:,2).*sin(gt(:,3))'], xj, yj)...
    +randn(2, size(gt,1))*eta;
 
% ready to filter
% initialize
X0(1:2,1)=gt(1,1:2); 
% X0(3:4,1)=[1 0]';
X0(3:4,1)=rand(2,1); % the theta is just arbitrary
P0=eye(4)*1; % ^^ initial covariance
 
try_extended_kalman_filter(dt, Z, gt', X0, P0);
 
function try_extended_kalman_filter(dt, Z, gt, X0, P0)
global xj yj
n=size(X0,1); % number of states for the motion model
m=size(Z,1); % number of states for the measurement model
T=size(Z,2); % number of time steps
 
Xh=zeros(n,T); % Create the predicted state of the motion of the robot
Xh_=Xh; % Create the predicted future state of the motion of the robot
P=zeros(n,n,T); P_=P; % Create the prediction robot
 
% kalman filter matrices
% >> constant velocity motion model and its ^^ process noise 
% Eq. 6.2.2-13 Bar-Shalom, RongLi, Kirubarajan, Estimation with
% Applications to Tracking and Navigation
Fk=  [ 1, 0, dt, 0
       0, 1, 0, dt
       0, 0, 1, 0
       0, 0, 0, 1];

Ffun=@(x) Fk*x;
 
% ^^ this is where we plug in the Jacobian of a nonlinear motion model 
% evaluated at x  
Flinfun=@(x) Fk; 
 
Qk= eye(4)*1;
 
% >> measurement model
Hfun=@(x) radar1(x, xj, yj);
 
% ^^ this is where we plug in the Jacobian of a nonlinear measurement model
% diff=.2; Hfun(X)-(Hfun(X-diff) + Hlinfun(X-diff)*(ones(4,1)*diff))
Hlinfun=@(x) radar1_lin(x, xj, yj); % because Hfun is linear
 
Rk=diag([2 .2]*10); 
 
k0=1;
kF=T;
 
% mean
Xh_(:,k0)=X0;
 
% covariance
P_(:,:,k0, 1)=P0;
 
% kalman filter
for k=k0:kF
    % update
    [Xh(:,k), P(:,:,k,1)]=ekf_update(Xh_(:,k), ...
        P_(:,:,k,1), Rk, Z(:,k), Hfun, Hlinfun);
     
    % predict
    [Xh_(:,k+1), P_(:,:,k+1,1)]= ekf_predict(Xh(:,k), ...
        P(:,:,k,1), Qk, Ffun, Flinfun);
end
 
show_the_results(gt, Z, Xh, P,dt, T, Qk, Rk);
 
function show_the_results(gt, Z, Xh, P,dt, T, Q, R)
global xj yj
m=size(Z,1); % number of states for the measurement model
 
% show the results
figure(1); gcf; clf;
ylabels={'r_1 Position (m)', 'r_2 Position (m)'};
for ii=1:2
    subplot(2,3,ii); gca;
    plot((1:T)*dt, gt(ii,:), 'k', 'linewidth', 2); % true position
    hold on;
%     plot(0:1/5:20, Xh(ii,:), 'r', 'linewidth', 2); % Kalman filter 
    errorbar((1:T)*dt, Xh(ii,:), squeeze(sqrt(P(ii,ii,:))), 'linewidth', 1);
    set(gca, 'fontname', 'times', 'fontsize', 10);
    grid on; grid minor; axis image; axis square;
    ylabel(ylabels{ii}); xlabel('Time (s)');
    legend('True State','Error in Position');
    if ii==1
        title(sprintf('model A, Q=%.1f, R=%.1f', Q(1,1), R(1,1)));
    end
end

subplot(2,3,3); gca;
plot(gt(1,:), gt(2,:), 'k', 'linewidth', 2);
hold on;
plot(Xh(1,:), Xh(2,:), 'r', 'linewidth', 2);
plot(Xh(1,1), Xh(2,1), '^k', 'linewidth', 2, 'markersize', 16);
plot(xj, yj, 'sg', 'linewidth', 2, 'markersize', 16);
axis image; axis square;
grid on; grid minor;
xlabel('r_1'); ylabel('r_2');
set(gca, 'fontname', 'times', 'fontsize', 10);
legend('ground truth','estimate');

% Plotting the velocity
ylabels={'r_1 Velocity (m/s)', 'r_2 Velocity (m/s)'};
v=diff(gt(1:2,:),1,2)*dt;
v=[v v(:,end)]; % to make it same size as time
for jj=1:2
    subplot(2,3,3+jj);
    plot((1:T)*dt, v(jj,:), 'k', 'linewidth', 2); % Predicted X velocity
    hold on; errorbar((1:T)*dt, Xh(jj+2,:), squeeze(sqrt(P(jj+2,jj+2,:))), 'linewidth', 1);
    hold on; axis image; axis square;
    grid on; grid minor;
    xlabel('Time (s)'); ylabel(ylabels{jj});
    set(gca, 'fontname', 'times', 'fontsize', 10);
    legend('Predicted Velocity','Error in Velocity');
end

function Xdot = unicycle_ode(t,X,v)
Xdot(1,1) = v*cos(X(3));
Xdot(2,1) = v*sin(X(3));
% set omega as a function of time to create interesting trajectory
Xdot(3,1) =1*sin(.5*t);
 
function z = radar1(X, xj, yj)
% from the notes created through the meetings
z(1,:)=sqrt((xj-X(1,:)).^2 + (yj-X(2,:)).^2);
z(2,:)=atan2((yj-X(2,:)), (xj-X(1,:)))-atan2(X(4,:), X(3,:));
 
function Hk = radar1_lin(X, xj, yj)
rij = sqrt((xj-X(1,1))^2+(yj-X(2,1))^2);
xij = xj-X(1,1); yij = yj-X(2,1);
Hk=[-xij/rij, -yij/rij, 0, 0
    -yij/rij^2, xij/rij^2, X(4,1)/(X(3,1)^2+X(4,1)^2), -X(3,1)/(X(3,1)^2+X(4,1)^2)];
