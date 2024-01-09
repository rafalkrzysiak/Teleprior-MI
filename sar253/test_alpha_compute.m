function test_alpha_compute
% check alpha computation here first, this runs directly on experimental
% data

addpath ../src
ID_Data=csvread('../src/IDList_Completed.csv',1,0);

% calcluate distribution
% here we create the test/train scenario 
% use 26 randomly selected ids to create pdistr and xdistr and run it on 3
% remaining ones only
numSubjects=size(ID_Data,1);
idsall=randperm(numSubjects);

num_train=floor(0.9*numSubjects);

ids_train=ID_Data(idsall(1:num_train),:);
ids_test=ID_Data(idsall(num_train+1:numSubjects),:);

dtTrack=1/2;
dtCommand=1/10;
fps=1/dtTrack;

% tau should be param.tau but for that paramconfig should be called outside
% the for loop--can we do that?
% set tau 15 seconds for distance and 20 seconds for freezing -- SB

tau=15;
data=extract_dist_data(tau, ids_train, ...
            dtTrack, dtCommand); % distance
[p_dist, x_dist]=calc_pdf(data);

% -- getting the values for the freeze time
% tau=30;
% data=extract_freezing_data(tau, ids_train, ...
%             dtTrack, dtCommand); % distance
% [p_dist, x_dist]=calc_pdf(data);

figure(1); gcf; clf;
subplot(2,4,1);
plot(x_dist,p_dist);

% load data
check=sprintf('%d', ids_test(1,1));
for ii=1:4
[xs, tf, tloc, file_id] = RobotExperimentDataSet(check,num2str(ii)); 

% calculate d
d = sqrt(sum((xs(1:2,2:end) - xs(1:2,1:end-1)).^2))';

% calculate alpha
alpha=ones(1,tf)*0.5;
for k=fps*tau:tf-1
    feature_k=sum(d(k-fps*tau+1:k, 1));
    alpha(k+1) = ...
        UpdateAlpha(feature_k, p_dist, x_dist, alpha(k));
end

subplot(2,4,ii+4);
plot(1:tf, alpha, 'linewidth', 2);
xlabel('frame');
ylabel('alpha');
end