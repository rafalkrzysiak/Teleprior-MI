function example01
% this example compares the performance of different filtering algorithms
% on a dataset obtained by simulating a lorenz attractor. A high sampling
% rate is assumed and therefore a constant velocity motion model is assumed
% The measurement model is also linear 

% NOTE
% >> in the script means a big change is needed for you to adapt it to your
% setup. e.g. a new likelihood function or measurement model
% ^^ means there are parameters that can be tuned to see if it changes the
% performance

% generating 3D data of lorenz attractor
% set the parameters
sigma = 10; % 
b = 8/3; % beta
r = 28; %10 % rho
dt=1/10;
tspan = 0:dt:20; % time for simulation

% ode with initial conditions [0 2 0]
i1=[0 2 0];
[~,gt] = ode45(@lorenz_ode,tspan,i1,[],sigma,b,r);
% ^^ add noise with intensity w0; gt is ground truth
w0=2;
Z=gt'+randn(size(gt,2), size(gt,1))*w0;


% ready to filter
% initialize
X0(1:3,1)=Z(:,1);
X0(4:6,1)=(Z(:,2)-Z(:,1))/dt;
P0=eye(6)*.5; % ^^ initial covariance

% uncomment one of these below to see how it performs, see the function
% itself to see how it works

% try_kalman_filter(dt, Z, gt', X0, P0);
% try_extended_kalman_filter(dt, Z, gt', X0, P0);
% try_kalman_filter_with_smoothing(dt, Z, gt', X0, P0);
try_particle_filter(dt, Z, gt', X0);

function try_particle_filter(dt, Z, gt, X0)

n=size(X0,1);
T=size(Z,2);

% ^^ number of particles (change this to see how it affects the
% performance)
N=1000; 

% particles n x N x T
p=zeros(n,N,T);

% ^^ pepper the initial estimate with some noise 
eta0=1;
p(:,:,1)=X0*ones(1,N) + randn(n,N)*eta0; 

% >> the measurement model is now replaced by a likelihood function
glfn=@(Z,p) glfn1(Z,p);

% we still need these to show stuff
Xh=zeros(n,T);
P=zeros(n,n,T);


% >> motion model
Fk=  [ 1, 0, 0, dt,  0,  0
    0, 1, 0,  0, dt,  0
    0, 0, 1,  0,  0, dt
    0, 0, 0,  1,  0,  0
    0, 0, 0,  0,  1,  0
    0, 0, 0,  0,  0,  1];

% ^^disturbance noise
w=10;
gmot=@(x) Fk*x;


k0=1;
kF=T;

% particle filter
wts=ones(1,N);
for k=k0:kF

    % update
    [p(:,:,k), wts] = pf_update(p(:,:,k), wts, Z(:,k), glfn);
    
    % this function pulls out the estimate from the distribution
    % ^^ the flag can be 1,2, or 3 and can give different estimates
    flag=2;
    Xh(:,k)=postest(p(:,:,k), wts, flag);
    P(:,:,k)=cov(p(:,:,k)');
    
    % predict
    p(:,:,k+1) = pf_predict(p(:,:,k), gmot, w);
end


show_the_results(gt, Z, Xh, P);

function try_extended_kalman_filter(dt, Z, gt, X0, P0)

n=size(X0,1);
m=size(Z,1);
T=size(Z,2);


Xh=zeros(n,T);
Xh_=Xh;
P=zeros(n,n,T); P_=P;


% kalman filter matrices
% >> constant velocity motion model and its ^^ process noise 
% Eq. 6.2.2-13 Bar-Shalom, RongLi, Kirubarajan, Estimation with
% Applications to Tracking and Navigation
Fk=  [ 1, 0, 0, dt,  0,  0
    0, 1, 0,  0, dt,  0
    0, 0, 1,  0,  0, dt
    0, 0, 0,  1,  0,  0
    0, 0, 0,  0,  1,  0
    0, 0, 0,  0,  0,  1];

Ffun=@(x) Fk*x;

% ^^ this is where we plug in the Jacobian of a nonlinear motion model 
% evaluated at x  
Flinfun=@(x) Fk; 
vd=50;
Qk=  [ (dt^3*vd^2)/3,             0,             0, (dt^2*vd^2)/2,             0,             0
    0, (dt^3*vd^2)/3,             0,             0, (dt^2*vd^2)/2,             0
    0,             0, (dt^3*vd^2)/3,             0,             0, (dt^2*vd^2)/2
    (dt^2*vd^2)/2,             0,             0,       dt*vd^2,             0,             0
    0, (dt^2*vd^2)/2,             0,             0,       dt*vd^2,             0
    0,             0, (dt^2*vd^2)/2,             0,             0,       dt*vd^2];

% >> measurement model and ^^ noise
% this one is simply identity
Hk=[eye(m), zeros(m)];

Hfun=@(x) Hk*x;

% ^^ this is where we plug in the Jacobian of a nonlinear measurement model
Hlinfun=@(x) Hk; % because Hfun is linear

Rk=diag([2 2 2]*10);


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

show_the_results(gt, Z, Xh, P);


function try_kalman_filter(dt, Z, gt, X0, P0)

n=size(X0,1);
m=size(Z,1);
T=size(Z,2);


Xh=zeros(n,T);
Xh_=Xh;
P=zeros(n,n,T); P_=P;


% kalman filter matrices
% >> constant velocity motion model and its ^^ process noise 
% Eq. 6.2.2-13 Bar-Shalom, RongLi, Kirubarajan, Estimation with
% Applications to Tracking and Navigation
Fk=  [ 1, 0, 0, dt,  0,  0
    0, 1, 0,  0, dt,  0
    0, 0, 1,  0,  0, dt
    0, 0, 0,  1,  0,  0
    0, 0, 0,  0,  1,  0
    0, 0, 0,  0,  0,  1];
vd=50;
Qk=  [ (dt^3*vd^2)/3,             0,             0, (dt^2*vd^2)/2,             0,             0
    0, (dt^3*vd^2)/3,             0,             0, (dt^2*vd^2)/2,             0
    0,             0, (dt^3*vd^2)/3,             0,             0, (dt^2*vd^2)/2
    (dt^2*vd^2)/2,             0,             0,       dt*vd^2,             0,             0
    0, (dt^2*vd^2)/2,             0,             0,       dt*vd^2,             0
    0,             0, (dt^2*vd^2)/2,             0,             0,       dt*vd^2];

% >> measurement model and ^^ noise
% this one is simply identity
Hk=[eye(m), zeros(m)];
Rk=diag([2 2 2]*10);


Klmn.F = Fk;
Klmn.Q= Qk;
Klmn.H = Hk;
Klmn.R= Rk;


k0=1;
kF=T;

% mean
Xh_(:,k0)=X0;

% covariance
P_(:,:,k0, 1)=P0;

% kalman filter
for k=k0:kF
    
    
    
    % update
    [Xh(:,k), P(:,:,k,1)]=kf_update(Xh_(:,k), ...
        P_(:,:,k,1), Z(:,k), Klmn);
    
    % predict
    [Xh_(:,k+1), P_(:,:,k+1,1)]= kf_predict(Xh(:,k), ...
        P(:,:,k,1), Klmn);
end

show_the_results(gt, Z, Xh, P);



function try_kalman_filter_with_smoothing(dt, Z, gt, X0, P0)
n=size(X0,1);
m=size(Z,1);
T=size(Z,2);


Xh=zeros(n,T);
Xh_=Xh;
P=zeros(n,n,T); P_=P;


% kalman filter matrices
% >> constant velocity motion model and its ^^ process noise 
% Eq. 6.2.2-13 Bar-Shalom, RongLi, Kirubarajan, Estimation with
% Applications to Tracking and Navigation
Fk=  [ 1, 0, 0, dt,  0,  0
    0, 1, 0,  0, dt,  0
    0, 0, 1,  0,  0, dt
    0, 0, 0,  1,  0,  0
    0, 0, 0,  0,  1,  0
    0, 0, 0,  0,  0,  1];
vd=50;
Qk=  [ (dt^3*vd^2)/3,             0,             0, (dt^2*vd^2)/2,             0,             0
    0, (dt^3*vd^2)/3,             0,             0, (dt^2*vd^2)/2,             0
    0,             0, (dt^3*vd^2)/3,             0,             0, (dt^2*vd^2)/2
    (dt^2*vd^2)/2,             0,             0,       dt*vd^2,             0,             0
    0, (dt^2*vd^2)/2,             0,             0,       dt*vd^2,             0
    0,             0, (dt^2*vd^2)/2,             0,             0,       dt*vd^2];

% >> measurement model and ^^ noise
% this one is simply identity
Hk=[eye(m), zeros(m)];

Rk=diag([2 2 2]*10);


Klmn.F = Fk;
Klmn.Q= Qk;
Klmn.H = Hk;
Klmn.R= Rk;


k0=1;
kF=T;

% mean
Xh_(:,k0)=X0;

% covariance
P_(:,:,k0, 1)=P0;

% kalman filter
for k=k0:kF
 
    % update
    [Xh(:,k), P(:,:,k,1)]=kf_update(Xh_(:,k), ...
        P_(:,:,k,1), Z(:,k), Klmn);
    
    % predict
    [Xh_(:,k+1), P_(:,:,k+1,1)]= kf_predict(Xh(:,k), ...
        P(:,:,k,1), Klmn);
end

% Rauch, H. E., Tung, F., & Striebel, C. T. (1965).
% Maximum likelihood estimates of linear dynamic systems.
% AIAA journal, 3(8), 1445-1450.
% N here means the last step
RTS=Klmn;

for k=kF-1:-1:k0
   
    Ck=P(:,:,k)*RTS.F'/P_(:,:,k+1);
    Xh(:,k)=Xh(:,k) + ...
        Ck*(Xh(:,k+1) - RTS.F*Xh(:,k));
    P(:,:,k)=P(:,:,k) + ...
        Ck*(P(:,:,k+1) - P_(:,:,k+1))*Ck';
end

show_the_results(gt, Z, Xh, P);

function wts=glfn1(Z, p)

% >> convert from actual value to Z

% >> this line for example would instead consist of the full nonlinear
% measurment model like the epipolar model or the camera model
Zh=p(1:3);

% ^^ noise values these should be changed depending on the measurement
% model above
eta=diag([2 2 2]); 

wts=normpdf(Zh(1), Z(1), eta(1,1)).*...
    normpdf(Zh(2), Z(2), eta(2,2)).*...
    normpdf(Zh(3), Z(3), eta(3,3));

function show_the_results(gt, Z, Xh, P)

m=size(Z,1);

% show the results
figure(1); gcf; clf;
for ii=1:m
    subplot(m,2,ii*2); gca;
    plot(gt(ii,:), 'k', 'linewidth', 2);
    hold on;
    plot(Z(ii,:), 'k*');
    plot(Xh(ii,:), 'r', 'linewidth', 2);
    set(gca, 'fontname', 'times', 'fontsize', 24);
end
xlabel('time');
ylabel('value');

subplot(m,2,[1 3 5]);
plot3(gt(1,:), gt(2,:), gt(3,:), 'k', 'linewidth', 2);
hold on;
plot3(Z(1,:), Z(2,:), Z(3,:), 'k*');
plot3(Xh(1,:), Xh(2,:), Xh(3,:), 'r', 'linewidth', 2);
axis image;

set(gca, 'fontname', 'times', 'fontsize', 24);
legend('ground truth', 'measurement', 'estimate');


function xdot = lorenz_ode(t,x,sigma,b,r)
xdot(1,1) = sigma*(x(2)-x(1));
xdot(2,1) = r*x(1)-x(2)-x(1)*x(3);
xdot(3,1) = x(1)*x(2)-b*x(3);


