function robot_nav_pf_mutual_info(particles, etaz, closed_loop, dt, Tfinal)
% this example runs an ekf on a setup with nonlinear motion model daa
% and nonlinear measurement model

% seeding random number generator
rng(2);

if nargin < 1
    % all the assignments below
    particles = 500; % number of particles to predict where robot is
    etaz=[10 pi/180]; % noise to add onto the measurement 
    closed_loop = 1; % boolean value for using the closed loop
    dt=1;
    Tfinal = 30;
end

% to lengthen or shorten the simulation, change Tfinal
% to modify the measurement step, change dt
tspan = 0:dt:Tfinal; % time for simulation

T = size(tspan,2); % the length of the tspan vector
Target_motion = 0; % boolean value for motion of target
num_targets = 1; % number of targets in the simulation
num_robots = 1; % number of robots in the simulation

w = 1; % disturbance

% initialize robot location and orientation as well as the target
% X0(1:3,1)=rand(3,1)*.5;
% X0(4:5,1)=20+randn(2,1);
X0 = num_robot_target(num_robots, num_targets);

try_particle_filter(dt, X0, T, tspan, w, particles, etaz, Target_motion, ...
                    num_robots, num_targets, closed_loop);

function try_particle_filter(dt, X0, T, tspan, w, particles, etaz, ...
                            Target_motion, num_robots, num_targets, closed_loop)

k0=1; % the beginning
kF=T; % the ending
n=size(X0,1);

% ^^ number of particles (change this to see how it affects the
% performance)
N=particles; 

% particles n x N x T
p=zeros(n,N,T);
sp=zeros(n-1,N,T);

% pepper the initial estimate with some noise 
eta0=1;
p(:,:,1)=X0*ones(1,N) + randn(n,N)*eta0; 
sp(:,:,1) = ones(1,N) + randn(4,N)*eta0;
Zh = zeros(2,N,T); Zh(:,:,1)=ones(1,N) + randn(2,N)*eta0;

% >> the measurement model is non-linear
glfn=@(Z,x) glfn1(Z,x);

% we still need these to show stuff
Xh=zeros(n,T);
P=zeros(n,n,T); 
Xs=zeros(5,kF); Xs(1:5,1)=X0(1:5,1);

% the motion model used is the nonlinear one
Ffun=@(X,v,omega,dt) unicycle1(X,v,omega,dt);

Z=zeros(2,T); % to hold all of the measurements taken

% particle filter weights
wts=ones(1,N);

% Robot and Target conditions for open loop
vr = .2; omegar = .1;
vt = 1; omegat = pi/8;

% for running the motion model virtually
omega = 0:.2:2;
v = 0:.2:2;

mutual_val = zeros(kF,1);
mutual_kde = zeros(numel(v),numel(omega),T);
mutual_kde(:,:,1) = randn(numel(v),numel(omega))*eta0;

for k=k0:kF
    
    Z(:,k)=get_measurements(Xh(:,k), Xs(4:5,k)) + [randn*etaz(1);
                                                   randn*etaz(2)];
     
    % update                                                 
    [p(:,:,k), wts] = pf_update(p(:,:,k), wts, Z(:,k), glfn);
    
     % this function pulls out the estimate from the distribution
    % ^^ the flag can be 1,2, or 3 and can give different estimates
    flag=1;
    Xh(:,k)=postest(p(:,:,k), wts, flag);
    P(:,:,k)=cov(p(:,:,k)');
    
    if closed_loop
        % create entropy vector for all combinations of velocity and omega
%         mutual_kde = zeros(numel(v),1);
        d=numel(Xh(1:4,k));
        m=2^d;
        b=2; l=1;
        
        for kk = 1:numel(v)
            for jj = 1:numel(omega)
                % update the motion model with every iteration because the robot moves
                gmot=@(x) Ffun(Xs(:,k),v(kk),omega(jj),2);

                % predict
                p(:,:,k+1) = pf_predict(p(:,:,k),gmot, w);
                
                % using every predicted particle state, calculated each
                % measurement (range and bearing) to the target
                Zh(:,:,k+1) = radar1(p(:,:,k+1));
                
                % the matrix that will be used for the support of kde
                % also the matrix that will be sent into the kde as the
                % combined stated for the target and measurement
                sp(:,:,k+1) = [p(4:5,:,k+1);Zh(:,:,k+1)];

                support=mean(sp(:,:,k+1),2)*ones(1,m)+(-l/2+rand(d,m)*b);
                mutual_kde(kk,jj,k+1) = mutual_info(sp(:,:,k+1),support);
            end
        end
        
        % hold the chosen values to view them later
        [val, row]=max(mutual_kde(:,:,k+1));
        [maxval, col] = max(val);
        row = row(col);
        mutual_val(k) = mutual_kde(row,col,k+1);
        
        % update the motion model with every iteration because the robot moves
        gmot=@(x) Ffun(Xs(:,k),v(row),omega(col),dt);

        % predict
        p(:,:,k+1) = pf_predict(p(:,:,k),gmot, w);

        % Put Robot and target sim, this is the simulated state space
        Xs(:,k+1) = Robot_Target_Sim(Xs(:,k),v(row),omega(col),dt,vt,omegat,Target_motion);
    
    else
        % update the motion model with every iteration because the robot moves
        gmot=@(x) Ffun(Xs(:,k),vr,omegar,dt);

        % predict
        p(:,:,k+1) = pf_predict(p(:,:,k),gmot, w);

        % Put Robot and target sim, this is the simulated state space
        Xs(:,k+1) = Robot_Target_Sim(Xs(:,k),vr,omegar,dt,vt,omegat,Target_motion);
    end
    
end

% get the average mutual information for every combination of omega and
% velocity for the entire time
avg_mutual = avg_mutual_info(v, omega, kF, mutual_kde);

if closed_loop
    
    save(sprintf('data_12_14_19/closedloop=%d_particles=%d_etar=%.2f_etath=%.4f.mat',...
                 closed_loop, N, etaz(1), etaz(2)), 'P', 'p', 'Xs', 'Xh', 'avg_mutual',...
                 'mutual_kde', 'dt', 'tspan', 'N', 'v', 'omega', 'etaz');
else
    save(sprintf('data_12_14_19/closedloop=%d_particles=%d_etar=%.2f_etath=%.4f.mat',...
                 closed_loop, N, etaz(1), etaz(2)), 'P', 'p', 'Xs', 'Xh', 'dt',...
                 'tspan', 'N', 'v', 'omega', 'etaz');
end

show_the_results(N, Z, Xs, Xh, P, v, omega, tspan, closed_loop, etaz, avg_mutual);
 
function show_the_results(N, Z, Xs, Xh, P, v, omega, tspan, closed_loop, etaz, avg_mutual)

sr=2; % number of rows in the figure
sc=4; % number of columns in the figure
 
% plots the results of the estimated state of both
% the robot and the target in the inertial frame
% figure(1); gcf; clf;
figure('Position', get(0, 'Screensize')); gcf; clf;
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
hold on; plot(Xh(4,:), Xh(5,:), 'k.', 'markersize', 10); % estimated state of the target
axis image; axis square; xlabel('x_1'); ylabel('x_2');
grid on; grid minor;
set(gca, 'fontname', 'times', 'fontsize', 10);
legend('Simulated','Estimate','Time step position estimate',...
       'Robot Start', 'Robot End', 'Target Location','Estimated Target',...
       'location', 'southeast');

% plot the original target position and 
% the estimated state of the target position
subplot(sr,sc,7);
plot(Xs(4,:), Xs(5,:), 'sg', 'linewidth', 2, 'markersize', 12); % Target Position
hold on; plot(Xh(4,:), Xh(5,:), 'k.', 'markersize', 10); % estimated state of the target
axis image; axis square; xlabel('x_j'); ylabel('y_j');
grid on; grid minor;
legend('Target True position', 'Estimated Target position');

subplot(sr,sc,8);
surf(v,omega,avg_mutual)
xlabel('Velocity (m/s)'); ylabel('Omega (rad/s)');
title('Avg Mutual Info'); axis image; axis square;
colorbar; view(0,90);

saveas(gcf,sprintf('data_12_14_19/images/closedloop=%d_particles=%d_etar=%.2f_etath=%.4f.png',...
                 closed_loop, N, etaz(1), etaz(2)));
             
% close(gcf); % close the figure
    
function X = unicycle1(X,v,omega, dt)
X(1,1) = X(1,1) + v*cos(X(3,1))*dt;
X(2,1) = X(2,1) + v*sin(X(3,1))*dt;
X(3,1) = X(3,1) + omega*dt;

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
 
function z = get_measurements(X, Xs)
% from the notes created through the meetings
z(1,:)=sqrt((Xs(1,:)-X(1,:)).^2 + (Xs(2,:)-X(2,:)).^2);
z(2,:)=wrapToPi(atan2((Xs(2,:)-X(2,:)), (Xs(1,:)-X(1,:))-X(3,:)));  
  
function z = radar1(X)
% from the notes created through the meetings
z(1,:)=sqrt((X(4,:)-X(1,:)).^2 + (X(5,:)-X(2,:)).^2);
z(2,:)=wrapToPi(atan2((X(5,:)-X(2,:)), (X(4,:)-X(1,:))-X(3,:)));

function wts=glfn1(Z, X)

% >> convert from actual value to Z

% >> this line for example would instead consist of the full nonlinear
% measurment model like the epipolar model or the camera model
% Zh=p(1:2);
Zh(1)=sqrt((X(4,:)-X(1,:)).^2 + (X(5,:)-X(2,:)).^2);
Zh(2)=wrapToPi(atan2((X(5,:)-X(2,:)), (X(4,:)-X(1,:))-X(3,:)));

% ^^ noise values these should be changed depending on the measurement
% model above
eta1=diag([10 pi/180]); 

wts=normpdf(Zh(1), Z(1), eta1(1,1)).*...
    normpdf(Zh(2), Z(2), eta1(2,2));
