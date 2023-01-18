function robot_nav_pf_multi_robot(particles, etaz, closed_loop, dt, Tfinal,...
                                  num_robots, num_targets, w, omega, v, b, l)
                              
% seeding random number generator
rng(2);

if nargin < 1
    % all the assignments below
    particles = 500; % number of particles to predict where robot is
    etaz=[10 pi/180]; % noise to add onto the measurement 
    closed_loop = 1; % boolean value for using the closed loop
    dt=1/2; % time step
    Tfinal = 30; % overall simulation time
    num_targets = 1; % number of targets in the simulation
    num_robots = 2; % number of robots in the simulation
    w = 1; % disturbance
    % for running the motion model virtually
    omega = -1:.1:1;
    v = 0:.1:1;
    b = 1;
    l = 1;
end

% to lengthen or shorten the simulation, change Tfinal
% to modify the measurement step, change dt
tspan = 0:dt:Tfinal; % time for simulation

T = size(tspan,2); % the length of the tspan vector
Target_motion = 0; % boolean value for motion of target

% initialize robot location and orientation as well as the target
X0 = num_robot_target(num_robots, num_targets);

try_particle_filter(dt, X0, T, tspan, w, particles, etaz, Target_motion, ...
                    num_robots, num_targets, closed_loop, v, omega, b, l);

function try_particle_filter(dt, X0, T, tspan, w, particles, etaz, Target_motion,...
                             num_robots, num_targets, closed_loop, v, omega, b, l)

k0=1; % the beginning
kF=T; % the ending
n=size(X0,1);

% ^^ number of particles (change this to see how it affects the
% performance)
N=particles; 

% particles n x N x T
p=zeros(n,N,T,num_robots); % holds the particles of the original state
sp=zeros(4,N,T,num_robots); % holds the particles of the measurement and target
Z = zeros(2,1,T,num_robots); % holds the measurement for each robot
Zh = zeros(2,N,T,num_robots); % holds the measurement from each particle to the target
mutual_val = zeros(kF,1);
mutual_kde = zeros(numel(v),numel(omega),T,num_robots);
velocity = zeros(num_robots,T);
Omega = zeros(num_robots,T);

% pepper the initial estimate with some noise 
eta0=1;

% initialize the first 'time step' of the robot and target
for i = 1:num_robots
    p(:,:,1,i) = X0*ones(1,N) + randn(n,N)*eta0; 
    sp(:,:,1,i) = ones(1,N) + randn(4,N)*eta0;
    Zh(:,:,1,i) = ones(1,N) + randn(2,N)*eta0;
    mutual_kde(:,:,1,i) = randn(numel(v),numel(omega))*eta0;
end

% >> the measurement model is non-linear
glfn=@(Z,x) glfn1(Z,x);

% we still need these to show stuff
Xh=zeros(n,T); Xh_pas = zeros(3,kF);
P=zeros(n,n,T,num_robots); 
Xs=zeros(n,kF); Xs(1:end,1)=X0(1:end,1);

% the motion model used is the nonlinear one
Ffun=@(X,v,omega,dt) unicycle1(X,v,omega,dt);

Z=zeros(2,T); % to hold all of the measurements taken

% particle filter weights
wts=ones(1,N);

% Robot and Target conditions for open loop
vr = .2; omegar = .1;
vt = 1; omegat = pi/8;

for k=k0:kF
    
    for agent = 1:num_robots
        Xh_pas(:,k) = Xh((3*agent-2):3*agent,k);
        Z(:,1,k,agent)=get_measurements(Xh_pas(:,k), Xs(7:8,k)) + [randn*etaz(1);
                                                                   randn*etaz(2)];
     
        % update                                                 
        [p(:,:,k,agent), wts] = pf_update(p(:,:,k,agent), wts, Z(:,1,k,agent), glfn);

         % this function pulls out the estimate from the distribution
        % ^^ the flag can be 1,2, or 3 and can give different estimates
        flag=1;
        Xh(:,k)=postest(p(:,:,k,agent), wts, flag);
        P(:,:,k,agent)=cov(p(:,:,k,agent)');

        if closed_loop
            % create entropy vector for all combinations of velocity and omega
    %         mutual_kde = zeros(numel(v),1);
            d=numel(Xh(1:4,k));
            m=5^d;
%             b=5; l=1;

            for kk = 1:numel(v)
                for jj = 1:numel(omega)
                    % update the motion model with every iteration because the robot moves
                    gmot=@(x) Ffun(Xs(:,k),v(kk),omega(jj),1);

                    % predict
                    p(:,:,k+1,agent) = pf_predict(p(:,:,k,agent),gmot, w);
                    
                    % --- run pf update on 
                    [ptemp, wts] = pf_update(p(:,:,k+1,agent), wts, Z(:,1,k,agent), glfn);
                    % minimize entropy of ptemp(4:5);

                    % using every predicted particle state, calculated each
                    % measurement (range and bearing) to the target
                    Zh(:,:,k+1,agent) = radar1(p(:,:,k+1,agent));

                    % the matrix that will be used for the support of kde
                    % also the matrix that will be sent into the kde as the
                    % combined stated for the target and measurement
                    sp(:,:,k+1,agent) = [p(end-1:end,:,k+1,agent);Zh(:,:,k+1,agent)];

                    support=mean(sp(:,:,k+1,agent),2)*ones(1,m)+(-l/2+rand(d,m)*b);
                    mutual_kde(kk,jj,k+1,agent) = mutual_info(sp(:,:,k+1,agent),support);
                end
            end

            % hold the chosen values to view them later
            [val, row]=max(mutual_kde(:,:,k+1,agent));
            [maxval, col] = max(val);
            row = row(col);
            mutual_val(k) = mutual_kde(row,col,k+1,agent);
            
            velocity(agent,k) = v(row); % store the optimal velocity 
            Omega(agent,k) = omega(col); % store the optimal omega 

            % update the motion model with every iteration because the robot moves
            gmot=@(x) Ffun(Xs(:,k),velocity(agent,k),Omega(agent,k),dt);

            % predict
            p(:,:,k+1,agent) = pf_predict(p(:,:,k,agent),gmot, w);
            
            

        else
            % update the motion model with every iteration because the robot moves
            gmot=@(x) Ffun(Xs(:,k),vr,omegar,dt);

            % predict
            p(:,:,k+1,agent) = pf_predict(p(:,:,k,agent),gmot, w);

            % Put Robot and target sim, this is the simulated state space
            Xs(:,k+1) = Robot_Target_Sim(Xs(:,k),vr,omegar,dt,vt,omegat,Target_motion);
        end
    end
    
    % Put Robot and target sim, this is the simulated state space
    Xs(:,k+1) = Robot_Target_Sim(Xs(:,k),velocity(:,k),Omega(:,k),dt,agent);
    
end

% get the average mutual information for every combination of omega and
% velocity for the entire time
avg_mutual = avg_mutual_info(v, omega, kF, mutual_kde, agent);

if closed_loop
    
    save(sprintf('data_12_18_19/particles=%d_etar=%.2f_etath=%.4f_b=%d_L=%d.mat',...
                  N, etaz(1), etaz(2), b, l), 'P', 'p', 'Xs', 'Xh', 'avg_mutual',...
                 'mutual_kde', 'dt', 'tspan', 'N', 'v', 'omega', 'etaz', 'velocity', 'Omega');
else
    save(sprintf('data_12_18_19/closedloop=%d_particles=%d_etar=%.2f_etath=%.4f.mat',...
                 closed_loop, N, etaz(1), etaz(2)), 'P', 'p', 'Xs', 'Xh', 'dt',...
                 'tspan', 'N', 'v', 'omega', 'etaz');
end

show_the_results(N, Z, Xs, Xh, P, v, omega, tspan, closed_loop, etaz, avg_mutual, agent, dt, b, l);
 
function show_the_results(N, Z, Xs, Xh, P, v, omega, tspan, closed_loop, etaz, avg_mutual, agent, dt, b, l)

sr=3; % number of rows in the figure
sc=4; % number of columns in the figure
 
% plots the results of the estimated state of both
% the robot and the target in the inertial frame
% figure(1); gcf; clf;
figure('Position', get(0, 'Screensize')); gcf; clf;

% plot the simulated trajectory, the estimated trajectory and the target(s)
subplot(sr,sc,1);
for i = 1:agent
    plot(Xs(3*i-2,:), Xs(3*i-1,:), 'r'); % Simulated
    hold on; plot(Xh(3*i-2,:), Xh(3*i-1,:), 'b'); % Estimated
    hold on; plot(Xh(3*i-2,:), Xh(3*i-1,:), 'co', 'Markersize', 8); % Time step position of estimate 
    hold on; plot(Xh(3*i-2,1), Xh(3*i-1,1), '^k', 'linewidth', 2, 'markersize', 16); % beginning point
    hold on; plot(Xh(3*i-2,end), Xh(3*i-1,end), 'k.', 'linewidth', 5, 'markersize', 25); % stopping point
    hold on; plot(Xs(end-1,:), Xs(end,:), 'sg', 'linewidth', 2, 'markersize', 20); % Target Position
    hold on; plot(Xh(end-1,:), Xh(end,:), 'k.', 'markersize', 10); % estimated state of the target
end
axis image; axis square; xlabel('x_1'); ylabel('x_2'); title('Model C','fontsize',20);
grid on; grid minor; 
set(gca, 'fontname', 'times', 'fontsize', 10);
legend('Simulated','Estimate','Time step position estimate',...
       'Robot Start', 'Robot End', 'Target Location','Estimated Target',...
       'location', 'northwest');
   
% plot the original target position and 
% the estimated state of the target position
subplot(sr,sc,2);
plot(Xs(end-1,:), Xs(end,:), 'sg', 'linewidth', 2, 'markersize', 12); % Target Position
hold on; plot(Xh(end-1,:), Xh(end,:), 'k.', 'markersize', 10); % estimated state of the target
axis image; axis square; xlabel('x_j'); ylabel('y_j');
title(sprintf('%d Particles',N),'fontsize',20); grid on; grid minor;
legend('Target True position', 'Estimated Target position');

ylabels = {'x_1', 'x_2', '\theta_1', ...
           'x_3', 'x_4', '\theta_2', ...
           'x_j', 'y_j'};

for ii=1:3*agent+2
    subplot(sr,sc,ii+2); gca;
    for j = 1:agent
        errorbar(tspan, Xh(ii,:), squeeze(sqrt(P(ii,ii,:,agent))), 'b', 'linewidth', 1);
        hold on;
    end
    
    if ii<=6
        plot(tspan, Xs(ii,1:end-1), 'r');
    end
    set(gca, 'fontname', 'times', 'fontsize', 10);
    grid on; axis image; axis square;
    ylabel(ylabels{ii}); xlabel('time (s)');
    
    if ii == 1
       title('Simulated robot_1 X position with estimate'); 
    end
    
    if ii == 2
       title('Simulated robot_1 Y position with estimate'); 
    end
    
    if ii == 3
       title('Simulated robot_1 \theta_1 orientation with estimate'); 
    end
    
    if ii == 4
       title('Simulated robot_2 X position with estimate'); 
    end
    
    if ii == 5
       title('Simulated robot_2 Y position with estimate'); 
    end
    
    if ii == 6
       title('Simulated robot_2 \theta_2 orientation with estimate'); 
    end
    
    if ii == 7
        hold on;
        plot(tspan, Xs(ii,1:end-1), 'r'); % plot the true X position of the target 
        title('Simulated Target X position with estimate');
    end
    
    if ii == 8
        hold on;
        plot(tspan, Xs(ii,1:end-1), 'r'); % plot the true Y position of the target
        title('Simulated Target Y position with estimate');
    end
end

% for j = 1:agent
%     subplot(sr,sc,j+10);
%     surf(v,omega,avg_mutual(:,:,1,j));
%     xlabel('Velocity (m/s)'); ylabel('Omega (rad/s)');
%     title('Avg Mutual Info'); axis image; axis square;
%     colorbar; view(0,90);
% end

saveas(gcf,sprintf('data_12_18_19/images/particles=%d_etar=%.2f_etath=%.4f_dt=%.2f_b=%d_l=%d.png',...
                  N, etaz(1), etaz(2), dt, b, l));
             
close(gcf); % close the figure
    
function X = unicycle1(X,v,omega, dt)
X(1,1) = X(1,1) + v*cos(X(3,1))*dt;
X(2,1) = X(2,1) + v*sin(X(3,1))*dt;
X(3,1) = X(3,1) + omega*dt;

function X = Robot_Target_Sim(X,vr,omegar,dt,agent)

for i = 1:agent
    X(3*i-2,1) = X(3*i-2,1) + vr(i)*cos(X(3*i,1))*dt;
    X(3*i-1,1) = X(3*i-1,1) + vr(i)*sin(X(3*i,1))*dt;
    X(3*i,1) = X(3*i,1) + omegar(i)*dt;
end
X(end-1,1) = X(end-1,1);
X(end,1) = X(end,1);
 
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
