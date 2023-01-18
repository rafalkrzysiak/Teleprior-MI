function infoseek1Db

dt=.1;
T=100;

closedloop=1;
debug=0;

% X represents combined state
% robot location and target location
X0=[5 -20]'; 

% >> robot and target dynamics
v=20;
w=diag([1, .01]); % disturbance

% >> measurement model | distance only measurement
hfun=@(X) sign(X(2,:)-X(1,:));
eta=diag(1);
% the likelihood function 
lfn=@(Z,p) lfn1(Z,p, hfun, eta);

n=size(X0,1);

% ^^ number of particles (change this to see how it affects the
% performance)
N=100; 
nsim=10;

figure(1); gcf; clf;
if debug
    figure(10); gcf; clf;
    nsim=1;
    rng(6);
end

for jj=1:nsim
    
    % initialize
    % particles n x N x T
    p=zeros(n,N,T);
    % particle filter weights
    wts=ones(1,N);

    % ^^ initial estimate of target location
    % is just uniformly distributed, so we have no idea where the target is
    w0=1;
    p(:,:,1)=[X0(1)+randn(1,N)*w0;
             -20 + rand(1,N)*40]; 

    % we still need these to show stuff
    Xh=zeros(n,T);
    Xh(:,1)=mean(p(:,:,1),2);

    % simulated
    Xs=Xh;
    Xs(:,1)=X0;

    k0=1;
    kF=T;

    for k=k0:kF

        % noisy sensor
%         Z(:,k)=get_measurements(Xs(1,k), Xs(2,k))*(double(rand>0.05)*2-1);

        % noiseless
        Z(:,k)=get_measurements(Xs(1,k), Xs(2,k));

        % update
        [p(:,:,k), wts] = pf_update(p(:,:,k), wts, Z(:,k), lfn);

        % this function pulls out the estimate from the distribution
        % ^^ the flag can be 1,2, or 3 and can give different estimates
        flag=2;
        Xh(:,k)=postest(p(:,:,k), wts, flag);
        P(:,:,k)=cov(p(:,:,k)');

        % control
        if debug
            figure(10); gcf;
            subplot(1,2,1);
            hist(p(2,:,k), -20:2:20);
        end
        if closedloop
            % maximize mutual information
            omega=optimize_MI(k, p, v, dt, N, wts, w, eta, hfun, lfn, [1 -1]);
%             omega=optimize_Htz(k, p, v, dt, N, wts, w, eta, hfun, lfn, [-1 1]);
        else
            % 1 or -1 with equal probability
            omega=double(rand>0.5)*2-1; 
        end

        % simulate
        Xs(:,k+1)=rt1d(Xs(:,k),v, omega, dt);

        % predict
        mmdl1=@(x) rt1d(x, v, omega, dt);
        p(:,:,k+1) = pf_predict(p(:,:,k), mmdl1, diag(w));

        simdata(jj).Xh=Xh;
        simdata(jj).Xs=Xs;
    end
end


% plot things online. 
% Later, when happy with the performance, modify and move this section
% to outside the loop
figure(1); gcf; clf;
hold on;
for ii=1:size(simdata,2)
    plot(simdata(ii).Xh(2,:), 'color', [1 0 0]+(ones(1,3)-[1 0 0])*0.5);
    plot(simdata(ii).Xh(1,:), 'color', [0 0 0]+(ones(1,3)-[0 0 0])*0.5);
    if debug
        plot(simdata(ii).Xs(1,:), 'k');
        plot(simdata(ii).Xs(2,:), 'r');
    end
end

function omega=optimize_MI(k, p, v, dt, N, wts, w, eta, hfun, lfn, omega_range)

MIvals=zeros(1, numel(omega_range));

for oo=1:numel(omega_range)

    mmdl1=@(x) rt1d(x, v, omega_range(oo), dt);
    p(:,:,k+1) = pf_predict(p(:,:,k), mmdl1, diag(w));

    % Support for Z 
    Zsup=[1 -1];
    M=size(Zsup,2);

    pZ=zeros(1,M);
    for ll=1:M
        % p(Z_ll)=int_kk p(T_kk)*p(Z_ll|T_kk)
        pZ(ll)=sum(wts.*lfn1(Zsup(:,ll),p(:,:,k+1), hfun, eta));
    end
    pZ=pZ(pZ~=0);
    
    % entropy of z
    Hz=-sum(pZ.*log(pZ));


    pZT=zeros(N,M);
    pZcT=zeros(N,M);
    for jj=1:N
        for ll=1:M
            % p(Z,T) = p(T)*p(Z|T)
            pZT(jj,ll)=wts(jj)*lfn1(Zsup(:,ll),p(:,jj,k+1), hfun, eta);
            % p(Z|T)
            pZcT(jj,ll)=lfn1(Zsup(:,ll),p(:,jj,k+1), hfun, eta);
        end
    end
    % entropy 
    pZcT1=pZcT(pZcT~=0 & pZT~=0);
    pZT1=pZT(pZcT~=0 & pZT~=0);
    Hzt=-sum(pZT1(:).*log(pZcT1(:)));

    MIvals(oo)=(Hz-Hzt);
end

[~, idx]=max(MIvals);
omega=omega_range(idx);

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
    
function wts=lfn1(Z, p, hfun, eta)

% >> convert from actual value to Z

% >> this line for example would instead consist of the full nonlinear
% measurment model like the epipolar model or the camera model
Zh=hfun(p);

% ^^ noise values these should be changed depending on the measurement
% model above
wts=ones(1,size(p,2));
for ii=1:size(eta,1)
    wts=wts.*(Zh(ii,:)==Z(ii,:));
end


function X = rt1d(X,v, omega, dt)

% robot
X(1,1) = X(1,1) + v*omega*dt;

% target
X(2,1) = X(2,1);


function z = get_measurements(R, T)
z(1,:)=sign(T-R);



