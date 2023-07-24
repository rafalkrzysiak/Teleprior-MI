function test_alpha_compute
% check alpha computation here first, this runs directly on experimental
% data

% load distribution
Exp = load('../src/pdist_tau=15s.mat');

% load data
[xs, tf, tloc, file_id] = RobotExperimentDataSet();

% calculate d
d = sqrt(sum((xs(1:2,2:end) - xs(1:2,1:end-1)).^2))';

% calculate alpha
alpha=zeros(1,tf);
tau=15*2; % since tau is in seconds but the one passed is in frames
param.alphaBegin=0.5; % the only parameter we need in this function
for k=1:tf-1
    [alpha(k+1), TotalDist_k, pDxMxT_k, pDyMyT_k] = ...
        UpdateAlpha(d, Exp.p_dist, Exp.x_dist, k, param, tau, alpha(k));
end
figure(1); gcf; clf;
plot(1:tf, alpha, 'linewidth', 2);
xlabel('frame');
ylabel('alpha');