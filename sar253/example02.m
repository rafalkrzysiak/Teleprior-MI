function example02
% this example runs an ekf on a setup with nonlinear motion model 
% and linear measurement model
 
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
i1=[2 2 0];
v = 10; omega=1; % initial velocity and rotation
[~,gt] = ode45(@unicycle_ode,tspan,i1,[],v);
% ^^ add noise with intensity w0; gt is ground truth

global xj yj
xj = gt(end,1); yj = gt(end,2);
% measurement model is for an overhead camera
eta=0.2;
Z1=radar1(gt(:,1:3)',xj, yj);
Z=radar1(gt(:,1:3)',xj, yj)+randn(2, size(gt,1))*eta;
 
% ready to filter
% initialize
X0(1:2,1)=gt(1,1:2); X0(3,1)=0; % the theta is just arbitrary
P0=eye(3)*.5; % ^^ initial covariance
 
try_extended_kalman_filter(dt, Z, gt', X0, P0, v, omega, Z1);
 
function try_extended_kalman_filter(dt, Z, gt, X0, P0, v, omega, Z1)
 
global xj yj
n=size(X0,1);
m=size(Z,1);
T=size(Z,2);
 
Xh=zeros(n,T); 
Xh_=Xh;
P=zeros(n,n,T); P_=P;
 
% kalman filter matrices
% unicycle model discrete form
Ffun=@(x) unicycle1(x,v,omega,dt);
 
% ^^ this is where we plug in the Jacobian of a nonlinear motion model 
% evaluated at x  
Flinfun=@(x) unicycle_lin(x,v,dt); 
 
Qk= eye(3)*10; % motion noise covariance matrix
 
% >> measurement model and ^^ noise
% this one is simply identity
Hfun=@(x) radar1(x, xj, yj); 
Hlinfun=@(x) radar1_lin(x, xj, yj);%Hk; % because Hfun is linear
 
Rk=diag([2 2]/2); % measurement noise covariance matrix
 
k0=1; % beginning
kF=T; % final

Xh_(:,k0)=X0; % mean
P_(:,:,k0, 1)=P0; % covariance 
 
% kalman filter
for k=k0:kF
     
    % update
    [Xh(:,k), P(:,:,k,1)]=ekf_update(Xh_(:,k), ...
        P_(:,:,k,1), Rk, Z(:,k), Hfun, Hlinfun);
     
    % predict
    [Xh_(:,k+1), P_(:,:,k+1,1)]= ekf_predict(Xh(:,k), ...
        P(:,:,k,1), Qk, Ffun, Flinfun);
end
 
show_the_results(gt, Z, Xh, P, Qk, Rk, dt, T, v, Z1);
 
function show_the_results(gt, Z, Xh, P, Qk, Rk, dt, T, v, Z1)
global xj yj
m=size(Z,1);
 
% show the results
figure(1); gcf; clf;
ylabels={'x_1', 'x_2', '\theta'};
for ii=1:3
    subplot(2,3,ii); gca;
    plot((1:T)*dt, gt(ii,:), 'k', 'linewidth', 2);
    hold on;
    plot((1:T)*dt, Xh(ii,:), 'r', 'linewidth', 2); hold on;
    errorbar((1:T)*dt, Xh(ii,:), squeeze(sqrt(P(ii,ii,:))), 'linewidth', 1);
    set(gca, 'fontname', 'times', 'fontsize', 10);
    grid on; axis image; axis square;
    ylabel(ylabels{ii}); xlabel('time (s)');
    if ii==1
        title(sprintf('Model B, Qk=%.1f, Rk=%.1f, Velocity=%.1f, dt=%.1f', Qk(1,1), Rk(1,1), v, dt),'fontsize',20);
    end
end
xlabel('time');

subplot(2,3,4);
plot(gt(1,:), gt(2,:), 'k', 'linewidth', 2); % ground truth
hold on; plot(gt(1,:), gt(2,:), 'bo', 'Markersize', 8); % Time step position of ground truth 
hold on; plot(Xh(1,:), Xh(2,:), 'r', 'linewidth', 2); % estimate
hold on; plot(Xh(1,:), Xh(2,:), 'co', 'Markersize', 8); % Time step position of estimate 
plot(Xh(1,1), Xh(2,1), '^k', 'linewidth', 2, 'markersize', 16); % beginning point
plot(xj, yj, 'sg', 'linewidth', 2, 'markersize', 16); % end point
axis image; xlabel('x_1'); ylabel('x_2');
grid on; grid minor;
set(gca, 'fontname', 'times', 'fontsize', 10);
legend('ground truth','Time step position','estimate','Time step position estimate','Start','End');


subplot(2,3,5);
plot(Z1(1,:), 'r');
hold on;
plot(Z(1,:), 'k.');
grid on;
ylabel('range');

subplot(2,3,6);
plot(Z1(2,:), 'r');
hold on;
plot(Z(2,:), 'k.');
grid on;
ylabel('phi');
 
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
flin=[1 0 -v*sin(X(3,1))*dt;
      0 1 v*cos(X(3,1))*dt;
      0 0 1];
 
function z = radar1(X, xj, yj)
% from the notes created through the meetings
z(1,:)=sqrt((xj-X(1,:)).^2 + (yj-X(2,:)).^2);
z(2,:)=atan2((yj-X(2,:)), (xj-X(1,:)))-X(3,:);

function Hk = radar1_lin(X, xj, yj)
xij = xj-X(1,1); yij = yj-X(2,1);
rij = sqrt(xij^2 + yij^2);

Hk=[-xij/rij, -yij/rij, 0
    -(X(2,1)-yj)/rij^2, (X(1,1)-xj)/rij^2, -1];