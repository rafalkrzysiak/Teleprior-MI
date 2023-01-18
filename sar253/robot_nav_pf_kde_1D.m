function robot_nav_pf_kde_1D
% this example runs an ekf on a setup with nonlinear motion model 
% and nonlinear measurement model

% seeding random number generator
rng(2);

% to lengthen or shorten the simulation, change Tfinal
% to modify the measurement step, change dt
dt=1/2;
Tfinal = 30;
tspan = 0:dt:Tfinal; % time for simulation

T = size(tspan,2); % the length of the tspan vector

particles = 500; % number of particles to predict where robot is
Target_motion = 0; % boolean value for motion of target
num_targets = 1; % number of targets in the simulation
num_robots = 1; % number of robots in the simulation
closed_loop = 1; % boolean value for using the closed loop
w = 1; % disturbance

% initialize robot location and orientation as well as the target
% X0(1:3,1)=rand(3,1)*.5;
% X0(4:5,1)=20+randn(2,1);
X0 = num_robot_target(num_robots, num_targets);

try_particle_filter(dt, X0, T, tspan, w, particles, Target_motion, ...
                    num_robots, num_targets, closed_loop);

function try_particle_filter(dt, X0, T, tspan, w, particles, ...
                            Target_motion, num_robots, num_targets, closed_loop)

k0=1; % the beginning
kF=T; % the ending
n=size(X0,1);

% ^^ number of particles (change this to see how it affects the
% performance)
N=particles; 

% particles n x N x T
p=zeros(n,N,T);

% pepper the initial estimate with some noise 
eta0=1;
p(:,:,1)=X0*ones(1,N) + randn(n,N)*eta0; 

% >> the measurement model is non-linear
glfn=@(Z,x) glfn1(Z,x);

% we still need these to show stuff
Xh=zeros(n,T);
P=zeros(n,n,T); 
Xs=zeros(5,kF); Xs(1:5,1)=X0(1:5,1);

% the motion model used is the nonlinear one
Ffun=@(X,v,omega,dt) unicycle1(X,v,omega,dt);

eta0=[.1 pi/180]; % noise for the measurements
Z=zeros(2,T); % to hold all of the measurements taken

% particle filter weights
wts=ones(1,N);

% Robot and Target conditions for open loop
vr = .2; omegar = .1;
vt = 1; omegat = pi/8;

% for running the motion model virtually
omega = -1:.1:1;
v = 0:.1:3;

ent_val = zeros(kF,1);

for k=k0:kF
    
    Z(:,k)=get_measurements(Xh(:,k), Xs(4,k),Xs(5,k)) + [randn*eta0(1);
                                                         randn*eta0(2)];
     
    % update                                                 
    [p(:,:,k), wts] = pf_update(p(:,:,k), wts, Z(:,k), glfn);
    
     % this function pulls out the estimate from the distribution
    % ^^ the flag can be 1,2, or 3 and can give different estimates
    flag=1;
    Xh(:,k)=postest(p(:,:,k), wts, flag);
    P(:,:,k)=cov(p(:,:,k)');
    
    if closed_loop
        % create entropy vector for all combinations of velocity and omega
        entropy_kde = zeros(numel(omega),1);
        d=numel(Xh(4:5,k));
        m=4^d;
        b=5; l=1;
        
        for kk = 1:numel(v)
            for jj = 1:numel(omega)
                % update the motion model with every iteration because the robot moves
                gmot=@(x) Ffun(Xs(:,k),v(kk),omega(jj),1);

                % predict
                p(:,:,k+1) = pf_predict(p(:,:,k),gmot, w);

                support=mean(p(4:5,:,k+1),2)*ones(1,m)+(-l/2+rand(d,m)*b);
                entropy_kde(kk,jj) = ent_kde(p(4:5,:,k+1),1,support);
            end
        end
        
        % hold the chosen values to view them later
        [val, row]=min(entropy_kde);
        [minval, col] = min(val);
        row = row(col);
        ent_val(k) = entropy_kde(row,col);
        
        % update the motion model with every iteration because the robot moves
        gmot=@(x) Ffun(Xs(:,k),v(row),omega(col),dt);

        % predict
        p(:,:,k+1) = pf_predict(p(:,:,k),gmot, w);

        % Put Robot and target sim, this is the simulated state space
        Xs(:,k+1) = Robot_Target_Sim(Xs(:,k),v(row),omega(col),dt,vt,omegat,Target_motion);
        
%         figure(4); gcf; clf;
%         surf(v,omega,entropy_kde)
%         xlabel('Velocity (m/s)'); ylabel('Omega (rad/s)');
%         title('entropy KDE');
%         colorbar; drawnow;
    
    else
        % update the motion model with every iteration because the robot moves
        gmot=@(x) Ffun(Xs(:,k),vr,omegar,dt);

        % predict
        p(:,:,k+1) = pf_predict(p(:,:,k),gmot, w);

        % Put Robot and target sim, this is the simulated state space
        Xs(:,k+1) = Robot_Target_Sim(Xs(:,k),vr,omegar,dt,vt,omegat,Target_motion);
    end
    
end

show_the_results(N, Z, Xs, Xh, P, tspan, closed_loop);
 
function show_the_results(N, Z, Xs, Xh, P, tspan, closed_loop)

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
    
    if ii==1
        title('Model C','fontsize',20);
    end
    
    if ii==2
       if closed_loop
          title('Closed Loop', 'fontsize',20);
       else
          title('Open Loop', 'fontsize',20);
       end
    end
    
    if ii==3
       title(sprintf('%d Particles',N),'fontsize',20);
    end
    
    if ii == 4
        hold on;
        plot(tspan, Xs(ii,1:end-1), 'r'); % plot the true X position of the target 
        title('Simulated Target X position with estimate');
    end
    
    if ii == 5
        hold on;
        plot(tspan, Xs(ii,1:end-1), 'r'); % plot the true Y position of the target
        title('Simulated Target Y position with estimate');
    end
end
xlabel('time');

% plot the robots estimated path along with 
% it's time step position, start and end point and target position
subplot(sr,sc,6);
plot(Xs(1,:), Xs(2,:), 'r'); % Simulated
hold on; plot(Xh(1,:), Xh(2,:), 'b'); % Estimated 
hold on; plot(Xh(1,:), Xh(2,:), 'co', 'Markersize', 8); % Time step position of estimate 
hold on; plot(Xh(1,1), Xh(2,1), '^k', 'linewidth', 2, 'markersize', 16); % beginning point
hold on; plot(Xh(1,end), Xh(2,end), 'k.', 'linewidth', 5, 'markersize', 25); % stopping point
hold on; plot(Xs(4,:), Xs(5,:), 'sg', 'linewidth', 2, 'markersize', 10); % Target Position
hold on; plot(Xs(4,1), Xs(5,1), 'sb', 'linewidth', 2, 'markersize', 16); % Initial Target Position
hold on; plot(Xs(4,end), Xs(5,end), 'sr', 'linewidth', 2, 'markersize', 16); % Initial Target Position
axis image; axis square; xlabel('x_1'); ylabel('x_2');
grid on; grid minor;
set(gca, 'fontname', 'times', 'fontsize', 10);
legend('Simulated','Estimate','Time step position estimate',...
       'Robot Start', 'Robot End', 'Target time step position',...
       'Target Begin','Target End','location', 'northwest');

% plot the original target position and 
% the estimated state of the target position
subplot(sr,sc,7);
plot(Xs(4,:), Xs(5,:), 'sg', 'linewidth', 2, 'markersize', 10); % Target Position
hold on; plot(Xh(4,:), Xh(5,:), 'k.', 'markersize', 6); % estimated state of the target
hold on; plot(Xs(4,1), Xs(5,1), 'sb', 'linewidth', 2, 'markersize', 16); % Initial Target Position
hold on; plot(Xs(4,end), Xs(5,end), 'sr', 'linewidth', 2, 'markersize', 16); % Initial Target Position
axis image; axis square; xlabel('x_j'); ylabel('y_j');
grid on; grid minor;
legend('Target time step position', 'Estimated Target position',...
        'Target Begin','Target End');
 
function X0 = num_robot_target(num_robots, num_targets)
X = zeros(3,1); Xt = zeros(2,1); % temporary place holders for robot and target
rx = 1; rth = 3; % initial positions of the robot state space

% placing the ith robot into the state space
for i = 1:num_robots
    for j = 1:3
       X(j) = randn; 
    end
    X0(rx:rth,1) = X(1:3,1);
    rx = rx + 3;
    rth = rth + 3; 
end

% placing the kth target into the state space after the robots
r_size = length(X0);
tx = r_size+1; ty = r_size+2;
for k = 1:num_targets
   for kk = 1:2
       Xt(kk) = 5+randn;
   end
   X0(tx:ty,1) = Xt(1:2,1);
   tx = tx + 2;
   ty = ty + 2;
end

X0(3)=atan2((X0(5)-X0(2)),(X0(4)-X0(1)));
    
function X = unicycle1(X,v,omega, dt)
X(1,1) = X(1,1) + v*cos(X(3,1))*dt;
X(2,1) = X(2,1) + v*sin(X(3,1))*dt;
X(3,1) = X(3,1);% + omega*dt;

function X = Robot_Target_Sim(X,vr,omegar,dt,vt,omegat,Target_motion)
eta = [.1 .1 .01]; % scaling factor for the process noise
if Target_motion
    % Wild Trajectory for both Target and Robot
    X(1,1) = X(1,1) + vr*cos(pi/4*X(3,1))*dt + randn*eta(1)*cos(X(3,1))*dt;
    X(2,1) = X(2,1) + vr*sin(pi+X(3,1))*dt + randn*eta(1)*sin(X(3,1))*dt;
    X(3,1) = X(3,1) + omegar*dt + randn*eta(2)*dt;
    X(4,1) = X(4,1) + vt*cos(omegat*2*X(3,1))*dt;
    X(5,1) = X(5,1) + vt*cos(omegat+X(3,1))*dt;
else
    % Basic Trajectory for robot, target stationary
    X(1,1) = X(1,1) + vr*cos(X(3,1))*dt;
    X(2,1) = X(2,1) + vr*sin(X(3,1))*dt;
    X(3,1) = X(3,1) + omegar*dt;
    X(4,1) = X(4,1);
    X(5,1) = X(5,1);
end
 
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

function wts=glfn1(Z, X)

% >> convert from actual value to Z

% >> this line for example would instead consist of the full nonlinear
% measurment model like the epipolar model or the camera model
% Zh=p(1:2);
Zh(1)=sqrt((X(4,:)-X(1,:)).^2 + (X(5,:)-X(2,:)).^2);
Zh(2)=atan2((X(5,:)-X(2,:)), (X(4,:)-X(1,:))-X(3,:));

% ^^ noise values these should be changed depending on the measurement
% model above
eta1=diag([10 10]); 

wts=normpdf(Zh(1), Z(1), eta1(1,1)).*...
    normpdf(Zh(2), Z(2), eta1(2,2));

function [eX, eY]=covellipse(xc, yc, cov)
[V, D]=eig(cov);
 
[eX, eY]=myellipse(xc, yc, sqrt(D(2,2)), sqrt(D(1,1)), atan2(V(2), V(1)));
 
function [eX eY] = myellipse(xc, yc, a, b, phi)
%function [eX eY] = myellipse(xc, yc, a, b, phi)
%
% xc, yc as the center
% a, b semi-major and minor axis
% phi the angle (in radians) semi-major axis makes with x axis. 
 
elem=50;
th=linspace(0,2*pi, elem);
 
x = sqrt(a^2)*cos(th);
y = sqrt(b^2)*sin(th);
 
trm=[cos(phi) -sin(phi) xc; 
        sin(phi) cos(phi) yc;
        0 0 1];
 
eX =  x*trm(1,1) + y*trm(1,2) + trm(1,3)*ones(1,elem);
eY =  x*trm(2,1) + y*trm(2,2) + trm(2,3)*ones(1,elem);