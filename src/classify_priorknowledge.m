clear variables
%addpath /Users/sachit/Dropbox/EASeL/common/MYLIB/entropy

%% Robot Speed
clear variables
ID_Data = csvread('IDList_Completed.csv',1);
type='EKFVel';
% type='ComSpeed';
dT=10; % section of time to observe in frames
conditions= {'xMxT', 'xMyT','yMxT','yMyT'};
subjects=dir('../data/FILTERED/*');
isub = [subjects(:).isdir];
nameFolds = {subjects(isub).name}';
nameFolds(ismember(nameFolds,{'.','..'})) = [];

for cond=1:4
    speeddata{cond}=[];
    for ii=1:size(nameFolds,1)
        append1=csvread(['../data/FILTERED/', ...
            nameFolds{ii}, '/', type, '_condition_', num2str(cond), '.csv']);
        idx=ID_Data(:,1)==str2double(nameFolds{ii});
        if ID_Data(idx,2) && cond==4
            append1=append1(1:ID_Data(idx,end));
        end
        speeddata{cond} =[speeddata{cond}; append1];
    end
end
                
support_speed=0:0.1:.5;
figure(1);gcf;clf;
for k=1:4
    data = speeddata{k};
    
    toremove=mod(numel(data),dT);
    data1=data(1:end-toremove);
    
    data=mean(reshape(data1, [], dT),2);
    
    
    
    subplot(2,2,k)

    [n,x]=hist(data,support_speed);
    p_speed(:,k)= n/sum(n);
    plot(x,p_speed(:,k))
    set(gca,'ylim',[0,1])
    xlabel('speed(m/s)')
    ylabel('p')
    legend(conditions{k})
end
figure(2);gcf;clf;
plot(x,p_speed)
set(gca,'ylim',[0,1])
 xlabel('speed(m/s)')
    ylabel('p')
legend(conditions)


%% Robot distance travelled
clear variables
ID_Data = csvread('IDList_Completed.csv',1);
type='EKFtraj';
% type='ComSpeed';
dT=30; % section of time to observe in frames
conditions= {'xMxT', 'xMyT','yMxT','yMyT'};
subjects=dir('../data/FILTERED/*');
isub = [subjects(:).isdir];
nameFolds = {subjects(isub).name}';
nameFolds(ismember(nameFolds,{'.','..'})) = [];

for dT=50 %5:10:120
for cond=1:4
    distdata{cond}=[];
    for ii=1:size(nameFolds,1)
        append1=csvread(['../data/FILTERED/', ...
            nameFolds{ii}, '/', type, '_condition_', num2str(cond), '.csv']);
        idx=ID_Data(:,1)==str2double(nameFolds{ii});
        append1=append1(1:2,:);
        if ID_Data(idx,2) && cond==4
            append1=append1(:,1:ID_Data(idx,end));
        end
        distdata{cond} =[distdata{cond}; sum(diff(append1,1,2).^2,1)'];
    end
end
            
    
support_dist=0:0.1:1;
figure(1);gcf;clf;
for k=1:4
    data = distdata{k};
    
    toremove=mod(numel(data),dT);
    data1=data(1:end-toremove);
    
    data=sum(reshape(data1, [], dT),2);
    
    
    
    subplot(2,2,k)

    [n,x]=hist(data,support_dist);
    p_dist(:,k)= n/sum(n);
    plot(x,p_dist(:,k))
    set(gca,'ylim',[0,1])
    xlabel('distance (m)')
    ylabel('p')
    legend(conditions{k})
end
figure(2);gcf;clf;
plot(x,p_dist)
set(gca,'ylim',[0,1])
 xlabel('distance(m)')
    ylabel('p')

% look at the KL distance between each condition    
KLdist=zeros(4);
for ii=1:4
    for jj=ii+1:4
        KLdist(ii,jj)=kldiv(p_dist(:,ii), p_dist(:,jj));
    end
end
KLdist
fprintf('dT=%d, dist=%.2f\n', dT, mean(KLdist(KLdist~=0)))

% Wasserstein distance
WSdist=zeros(4);
for ii=1:4
    for jj=ii+1:4
        WSdist(ii,jj)=ws_distance(p_dist(:,ii), p_dist(:,jj));
    end
end
WSdist
fprintf('dT=%d, dist=%.2f\n', dT, mean(WSdist))
end
legend(conditions)

%% Robot angle rotated
clear variables
ID_Data = csvread('IDList_Completed.csv',1);
type='EKFtraj';
dT=20; % section of time to observe in frames
conditions= {'xMxT', 'xMyT','yMxT','yMyT'};
subjects=dir('../data/FILTERED/*');
isub = [subjects(:).isdir];
nameFolds = {subjects(isub).name}';
nameFolds(ismember(nameFolds,{'.','..'})) = [];

for cond=1:4
    distdata{cond}=[];
    for ii=1:size(nameFolds,1)
        append1=csvread(['../data/FILTERED/', ...
            nameFolds{ii}, '/', type, '_condition_', num2str(cond), '.csv']);
        idx=ID_Data(:,1)==str2double(nameFolds{ii});
        append1=append1(3,:);
        if ID_Data(idx,2) && cond==4
            append1=append1(:,1:ID_Data(idx,end));
        end
        distdata{cond} =[distdata{cond}; abs(asin(sin(diff(append1,1,2))))'];
    end
end
            
    
support_dist=0:0.2:4*pi;
figure(1);gcf;clf;
for k=1:4
    data = distdata{k};
    
    toremove=mod(numel(data),dT);
    data1=data(1:end-toremove);
    
    data=sum(reshape(data1, [], dT),2);
    
    
    
    subplot(2,2,k)

    [n,x]=hist(data,support_dist);
    p_speed(:,k)= n/sum(n);
    plot(x,p_speed(:,k))
    set(gca,'ylim',[0,1])
    xlabel('angle (rad)')
    ylabel('p')
    legend(conditions{k})
end
figure(2);gcf;clf;
plot(x,p_speed)
set(gca,'ylim',[0,1])
 xlabel('angle (rad)')
    ylabel('p')
legend(conditions)


%% Robot Turnrate
clear variables
ID_Data = csvread('IDList_Completed.csv',1);
conditions= {'xMxT', 'xMyT','yMxT','yMyT'};
type='EKFom';
% type='ComTurnRate';
dT=20; % section of time to observe in frames
subjects=dir('../data/FILTERED/*');
isub = [subjects(:).isdir];
nameFolds = {subjects(isub).name}';
nameFolds(ismember(nameFolds,{'.','..'})) = [];

for cond=1:4
    trdata{cond}=[];
    for ii=1:size(nameFolds,1)
        append1=csvread(['../data/FILTERED/', ...
            nameFolds{ii}, '/', type, '_condition_', num2str(cond), '.csv']);
        idx=ID_Data(:,1)==str2double(nameFolds{ii});
        if ID_Data(idx,2) && cond==4
            append1=append1(1:ID_Data(idx,end));
        end
        trdata{cond} =[trdata{cond}; append1];
    end
end
            
    
support_tr=0:0.1:2;
figure(1);gcf;clf;
for k=1:4
    data = abs(trdata{k});
    
    toremove=mod(numel(data),dT);
    data1=data(1:end-toremove);
    
    data=mean(reshape(data1, [], dT),2);
    
    subplot(2,2,k)

    [n,x]=hist(data,support_tr);
    p_tr(:,k)= n/sum(n);
    plot(x,p_tr(:,k))
    set(gca,'ylim',[0,1])
    xlabel('turn rate (rad/s)')
    ylabel('p')
    legend(conditions{k})
end
figure(2);gcf;clf;
plot(x,p_tr)
set(gca,'ylim',[0,1])
 xlabel('turn rate (rad/s)')
    ylabel('p')
legend(conditions)


%% Entropy H(X[k]|X[k-1]) where X[k]=orientation
clear variables
ID_Data = csvread('IDList_Completed.csv',1);
conditions= {'xMxT', 'xMyT','yMxT','yMyT'};
type='EKFtraj';
% type='ComTurnRate';
subjects=dir('../data/FILTERED/*');
isub = [subjects(:).isdir];
nameFolds = {subjects(isub).name}';
nameFolds(ismember(nameFolds,{'.','..'})) = [];

for cond=1:4
    trdata{cond}=[];
    for ii=1:size(nameFolds,1)
        append1=csvread(['../data/FILTERED/', ...
            nameFolds{ii}, '/', type, '_condition_', num2str(cond), '.csv']);
        append1=append1(3,:)';
        idx=ID_Data(:,1)==str2double(nameFolds{ii});
        if ID_Data(idx,2) && cond==4
            append1=append1(1:ID_Data(idx,end),:);
        end
        trdata{cond} =[trdata{cond}; append1];
    end
end
            
    
support_tr=0:0.2:2;
figure(1);gcf;clf;
for k=1:4
    
    % need to reshape the data so that we compute conditional entropy on T
    % second segments
    data = trdata{k};
    
    subplot(2,2,k)

    [n,x]=hist(data,support_tr);
    p_tr(:,k)= n/sum(n);
    plot(x,p_tr(:,k))
    set(gca,'ylim',[0,1])
    xlabel('turn rate (rad/s)')
    ylabel('p')
    legend(conditions{k})
end
figure(2);gcf;clf;
plot(x,p_tr)
set(gca,'ylim',[0,1])
 xlabel('turn rate (rad/s)')
    ylabel('p')
legend(conditions)

%%
clear variables
conditions= {'xMxT', 'xMyT','yMxT','yMyT'};
markers= {'o', '.','s','x'};
TRdata = csvread('../data/RobotTurnrateData.csv');
support_TR=0:0.1:1;
speeddata = csvread('../data/RobotSpeedData.csv');
support_speed=0:0.05:0.3;
figure(1);gcf;clf;
for k=1:4
    subplot(2,2,k)
    plot(speeddata(:,k),TRdata(:,k),'.');
    xlabel('Speed(m/s)')
    ylabel('Turn rate(rads/s)')
    set(gca,'ylim',[0,1])
    set(gca,'xlim',[0,0.3])
end
figure(2);gcf;clf;
for k=1:4
    
    scatter(speeddata(:,k),TRdata(:,k), 'marker', markers{k});
    hold on
    
end
xlabel('Speed(m/s)')
    ylabel('Turn rate(rads/s)')
    set(gca,'ylim',[0,1])
    set(gca,'xlim',[0,0.3])
legend(conditions)