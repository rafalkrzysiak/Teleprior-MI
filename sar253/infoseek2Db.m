function infoseek2Db(dt, T, r_visible, eta, N, nsim, closedloop, past, debug)

% read the map from the folder
% img = imread('maps/basic_map.jpg');
img = imread('maps/u_map.jpg');
% img = imread('maps/complex_map.jpg');
img = imbinarize(img); % for matlab 2016 and up
% img = rot90(rot90(img(:,:,1)));
% thresh = graythresh(img); % use if using matlab 2016 or lower
% img=im2bw(img,thresh); % use if using matlab 2016 or lower
% figure(1); gcf; clf;
% imagesc([0 20],[0 20],img); hold on;
% grid on; grid minor;
% set(gca,'ydir','normal');

% preset values when not running this script from another script
if nargin < 1
    dt=1; % time step
    T=1000; % total simulation time
    r_visible = 3; % visible range of the robot
    eta=.5; % ######### Changing eta value 
    N=100; % Number of particles
    nsim=1; % number of simulations when debug mode isn't enabled
    closedloop=1; % enable or disable opimization of mutual information
    debug=1; % enable or disable debug mode
    past = 25; % how far to look back throughout time to make sure robot isnt stuck
    r_robot = 1;
    
    if debug
        figure(10); gcf; clf;
%         nsim=5; % number of simulations when debug mode is enabled
    %     rng(6);
    end
end

% X represents combined state
% robot location/orientation and target location
% X0 = [9 5 3*pi/2 15 5]'; % basic map
X0 = [5 5 3*pi/2 16 16]'; % u map
% X0 = [2 17 0 15 3]'; % complex map


% Robot and Target dynamics
v=([-1:.5:1])*2; % possible velocity values for the robot
om=([-1:.5:1])*.5; % possible omega values for the robot
omega0=randn*.5;
vel0=rand;
w=diag([.01, .01, .01, .25, .25]); % disturbance

% measurement model | bearing only measurement     
hfun=@(X) wrapToPi(atan2((X(5,:)-X(2,:)), (X(4,:)-X(1,:)))-X(3,:));
% hfun=@(X) atan2((X(5,:)-X(2,:)), (X(4,:)-X(1,:)))-X(3,:);

% the likelihood function 
lfn=@(Z,ZZ,p,bin_map) lfn1(Z, ZZ, p, hfun, eta, bin_map, r_visible);

% the optimize mutual information function
O_MI = @(k, p, wts, bin_map, Z) optimize_MI(k, p, v, dt, N, wts, w, om, lfn, bin_map, Z);

n=size(X0,1); % The state size
Num_target_found = zeros(nsim,1);
time = Num_target_found;

for jj=1:nsim
    
    fprintf('Simulation: %d \n',jj)
    
    % initialize
    % particles n x N x T
    p=zeros(n,N,T);
    % particle filter weights
    wts=ones(1,N);
    Z = zeros(1,T);
    
    % holds all the optimized velocity and omega data
    vel_data = zeros(1,T);
    omega_data = vel_data;

    % ^^ initial estimate of target location
    % is just uniformly distributed, so we have no idea where the target is
    w0=.15;
    
    p(:,:,1)=[X0(1)+randn(1,N)*w0;
              X0(2)+randn(1,N)*w0;
              X0(3)+randn(1,N)*w0;
              rand(1,N)*20;
              rand(1,N)*20]; 

    % we still need these to show stuff
%     Xh=zeros(n,T);
    Xh(:,1)=mean(p(:,:,1),2);

    % simulated
    Xs=Xh;
    Xs(:,1)=X0;
    
    % beginning and ending setps for the filter
    k0=1;
    kF=T;
    
    % mesh grid for the robot location
    [xGrid,yGrid] = meshgrid(1:1:410,1:1:410);
    
    for k=k0:kF
        
        fprintf('Time step: %d\n',k); % display the time step number
        
        bin_map = img;
        
        % check if the robot hasn't really moved over some time
        % if the robot hasn't moved for some time (is stuck), say that the
        % robot will never find the target and end that simulation
        if k > 30
            diff = std(Xs(k-past:k),0,2);
            if diff(1) < 3 && diff(2) < 3
                time(jj) = kF;
                Xs(:,k+1:kF) = Xs(:,k).*ones(5,kF-k);
                Xh(:,k:kF) = Xh(:,k-1).*ones(5,kF-(k-1));
               break 
            end
            
            
        end
        
        if Xs(1,k) < 0 || Xs(2,k) > 20 || Xs(1,k) > 20 || Xs(2,k) < 0
            time(jj) = kF;
            Xs(:,k+1:kF) = Xs(:,k).*ones(5,kF-k);
            Xh(:,k:kF) = Xh(:,k-1).*ones(5,kF-(k-1));
            break
        end
        
        % check the distance between simulated robot and target
        rij = sqrt((Xs(4,k)-Xs(1,k))^2+(Xs(2,k)-Xs(5,k))^2);
        
        if rij < r_visible % only gets a measurement when within range
            % noiseless
            Z(:,k)=get_measurements(Xs(:,k));
            
            if rij < r_visible/2 % must be closer to get a diffinitive answer
                Num_target_found(jj) = 1; % save a yes to the sim number where target was found
                time(jj) = k; % mark the time it was found
                disp('The target was found!');
                Xs(:,k+1:kF) = Xs(:,k).*ones(5,kF-k);
                Xh(:,k:kF) = Xh(:,k-1).*ones(5,kF-(k-1));
                break
            end
        else
            Z(:,k) = 0; % gets to measurment
            time(jj) = k; % continue counting the time
        end
        
        % update
        [p(:,:,k), wts] = pf_update(p(:,:,k), wts, Z(:,k), lfn, bin_map);

        % this function pulls out the estimate from the distribution
        % ^^ the flag can be 1,2, or 3 and can give different estimates
        flag=1;
        Xh(:,k)=postest(p(:,:,k), wts, flag);
        P(:,:,k)=cov(p(:,:,k)');
        
        % Only enable when wanting to debug to see whats happening
        if debug
            debug_plot(Xs(:,k), Xh(:,k), k, p(:,:,k), r_visible, r_robot, bin_map)
        end
        
        if closedloop
            % maximize mutual information
%             [omega,vel]=optimize_MI(k, p, v, dt, N, wts, w, eta, hfun, om, r_visible);
%             [omega,vel]=optimize_MI(k, p, v, dt, N, wts, w, om, lfn);
            [omega,vel] = O_MI(k, p, wts, bin_map, Z(:,k));
%             omega=optimize_Htz(k, p, v, dt, N, wts, w, eta, hfun, lfn, [-1 1]);
        else
            % 1 or -1 with equal probability
            omega0=omega0+0.1*randn; 
            vel0=vel0+rand*.1;
            
            omega=omega0;
            vel=vel0;
        end
        
        % update the position of the robot in the map to see where it is
        BW = sqrt((xGrid - Xs(1,k)*(410/20)).^2 + (yGrid - Xs(2,k)*(410/20)).^2) <= r_robot*(410/20);
        white = sum(~BW.*img(:,:,1) == 1,'all');
        
        if white > 110010 && Xs(3,k) > 0
            omega = omega*pi;
            
        elseif white > 110010 && Xs(3,k) < 0
            omega = omega*-pi;
        end

        % simulate
        Xs(:,k+1)=rt1d(Xs(:,k),vel, omega, dt);
        
        % predict
        mmdl1=@(x) rt1d(x, vel, omega, dt);
        p(:,:,k+1) = pf_predict(p(:,:,k), mmdl1, diag(w));
   
    end
    simdata(jj).Xh=Xh;
    simdata(jj).Xs=Xs;
    simdata(jj).p=p;
    
    vel_data(jj) = vel;
    omega_data(jj) = omega;
end

% plot the results of the simulation (trajectories)
traj_plot(simdata, X0, N, nsim, eta, Num_target_found, time, dt, img)

save(sprintf('data/basic3_particles=%d_nsim=%d_etath=%.4f.mat', N, nsim, eta), ...
             'P', 'p', 'simdata', 'dt', 'T', 'r_visible', 'N', 'vel_data',...
             'omega_data', 'eta', 'img');

function [omega, vel]=optimize_MI(k, p, vel_range, dt, N, wts, w, omega_range, lfn, bin_map, Z)

MIvals=zeros(numel(omega_range),numel(vel_range));

for oo=1:numel(omega_range)
    for dd=1:numel(vel_range)

        mmdl1=@(x) rt1d(x, vel_range(dd), omega_range(oo), dt);
        p(:,:,k+1) = pf_predict(p(:,:,k), mmdl1, diag(w));

        % Support for Z 
        Zsup=-pi:pi/6:pi;
        M=size(Zsup,2);

        pZ=zeros(1,M);
        for ll=1:M
            % p(Z_ll)=int_kk p(T_kk)*p(Z_ll|T_kk)
%             pZ(ll)=sum(wts.*lfn1(Zsup(:,ll),p(:,:,k+1), hfun, eta, r_visible));
            pZ(ll)=sum(wts.*lfn(Zsup(:,ll),Z,p(:,:,k+1),bin_map));
        end
        pZ=pZ(pZ~=0);
        
        % entropy of z
        Hz=-sum(pZ.*log(pZ));
        
        % work on this to speed up
        pZT=zeros(N,M);
        pZcT=zeros(N,M);
        for jj=1:N
            for ll=1:M
                % p(Z,T) = p(T)*p(Z|T)
%                 pZT(jj,ll)=wts(jj)*lfn1(Zsup(:,ll),p(:,jj,k+1), hfun, eta, r_visible);
                pZT(jj,ll)=wts(jj)*lfn(Zsup(:,ll),Z,p(:,jj,k+1),bin_map);
                % p(Z|T)
%                 pZcT(jj,ll)=lfn1(Zsup(:,ll),p(:,jj,k+1), hfun, eta, r_visible);
                pZcT(jj,ll)=lfn(Zsup(:,ll),Z,p(:,jj,k+1),bin_map);
            end
        end
    
    % entropy 
    pZcT1=pZcT(pZcT~=0 & pZT~=0);
    pZT1=pZT(pZcT~=0 & pZT~=0);
    Hzt=-sum(pZT1(:).*log(pZcT1(:)));
    MIvals(oo,dd)=(Hz-Hzt);
    end
end

[val, row]=max(MIvals);
[~, col] = max(val);
row = row(col);
omega=omega_range(row);
vel=vel_range(col);

function omega=optimize_Htz(k, p, v, dt, N, wts, w, eta, hfun, lfn, omega_range)

Htz=zeros(1, numel(omega_range));

pTkcZk=p(:,:,k);
mu_pTkcZk=mean(pTkcZk,2);
for oo=1:numel(omega_range)

    
    mmdl1=@(x) rt1d(x, v, omega_range(oo), dt);
    p_ = pf_predict(pTkcZk, mmdl1, diag(w));

    % Support for Z should be something that
    Zsup=[1 -1];
%     Z1=mean(hfun(p(:,:,k)));
%     Zsup=linspace(Z1-10,Z1+10,10);
%     Zsup=hfun(p(:,:,k));
%     Zsup=Zsup(:,1:2:end);
    M=size(Zsup,2);

    pTk1cZk1=zeros(2, numel(wts),M);
    for ll=1:M
        pTk1cZk1(:,:,ll)=pf_update(p_, wts, Zsup(:,ll),lfn);
    end
    % entropy of z
    pTk1cZZk1=squeeze(pTk1cZk1(2,:,:));
    Htz(oo)=ent(pTk1cZZk1(:)', 10, ...
        [mu_pTkcZk(2)-5, mu_pTkcZk(2)+5], 'x');
%     Htz(oo)=ent(pTk1cZZk1(:)', 20, ...
%         [1, 15], 'x');
%     Htz(oo)=ent_kde(pTk1cZZk1(:)', .5, 5:1:15);

end

[~, idx]=min(Htz);
omega=omega_range(idx);
    
function wts=lfn1(Z, ZZ, p, hfun, eta, bin_map, r_visible)

% >> this line for example would instead consist of the full nonlinear
% measurment model like the epipolar model or the camera model
Zh=hfun(p);

% ^^ noise values these should be changed depending on the measurement
% model above
wts=ones(1,size(p,2));

% place a 1 for every particle position of the target
for kk = 1:size(p,2)
    aa = abs(round(p(4,kk).*(410/20)));
    bb = abs(round(p(5,kk).*(410/20)));

    if aa>0 && bb>0 && aa<410 && bb<410
        if bin_map(aa,bb)>0
            wts(kk)=1;
        else
            wts(kk) = 0;
        end
        
    else
        wts(kk) = 0;
    end
    
end

if ZZ(1)
    for ii=1:size(eta,1)
        wts=wts(ii).*normpdf(Zh(ii,:), Z(ii,:), eta(ii));
    end
else
    dist_from_robot=sqrt(sum((p(4:5,:)-p(1:2,:)).^2));
    idx=(dist_from_robot<(r_visible*.75));
    wts(idx)=0;
%     for i = 1:numel(dist_from_robot)
%         wts(i) = wts(i)*(1-normpdf(dist_from_robot(i),0,1));
%     end
end
    
function X = rt1d(X, v, omega, dt)

% robot
X(1,1) = X(1,1) + v*cos(X(3,1))*dt;
X(2,1) = X(2,1) + v*sin(X(3,1))*dt;
X(3,1) = X(3,1) + omega*dt;

% target
X(4,1) = X(4,1);% + .1*randn;
X(5,1) = X(5,1);% + .1*randn;

function z = get_measurements(X)

z(1,:)=wrapToPi(atan2((X(end,:)-X(2,:)), (X(end-1,:)-X(1,:)))-X(3,:));
% z(1,:)=atan2((X(end,:)-X(2,:)), (X(end-1,:)-X(1,:)))-X(3,:);
