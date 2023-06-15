% this script creates distributions of features of robot movement such as average speed,
% distance moved, turn rate etc. over a short period of time based on the
% experimental condition. 
% it then computes the KL distance between the distributions to determine
% if the feature is good enough to be used as a classifier of prior
% knowledge. ideally we'd like short time (to enable real-time detection)
% and high KL divergence (to ensure that the distributions are far enough)
% Right now the function runs on single features. Next we'll combine all of
% them to get the best classification

clear variables

dtTrack=1/2;
dtCommand=1/10;
ID_Data = csvread('IDList_Completed.csv',1);
conditions= {'xMxT', 'xMyT','yMxT','yMyT'};
subjects=dir('../data/FILTERED/*');
isub = [subjects(:).isdir];
nameFolds = {subjects(isub).name}';
nameFolds(ismember(nameFolds,{'.','..'})) = [];
obsvTime=15; % seconds

kldist14=zeros(1,5);
wsdist14=zeros(1,5);

figure(1); gcf; clf;
dstr_speed=extract_speed_data(obsvTime, ID_Data, ...
    dtTrack, dtCommand, nameFolds); % speed
subplot(2,3,1);
[p_speed, x_speed, kldist14(1)]=calc_pdf(dstr_speed, ...
        'average speed (m/s)', conditions);

dstr_dist=extract_dist_data(obsvTime, ID_Data, ...
    dtTrack, dtCommand, nameFolds); % distance
subplot(2,3,2);
[p_dist, x_dist, kldist14(2)]=calc_pdf(dstr_dist,  ...
    'distance (m)', conditions);

dstr_tr=extract_turnrate_data(obsvTime, ID_Data, ...
    dtTrack, dtCommand, nameFolds); % distance
subplot(2,3,3);
[p_tr, x_tr, kldist14(3)]=calc_pdf(dstr_tr, ...
    'average turn rate (rad/s)', conditions);

dstr_angle=extract_angle_data(obsvTime, ID_Data, ...
    dtTrack, dtCommand, nameFolds); % distance
subplot(2,3,4);
[p_angle, x_angle, kldist14(4)]=calc_pdf(dstr_angle,  ...
    'angle turned (rad)', conditions);

dstr_freeze=extract_freezing_data(obsvTime, ID_Data, ...
    dtTrack, dtCommand, nameFolds); % distance
subplot(2,3,5);
[p_frz, x_frz, kldist14(5)]=calc_pdf(dstr_freeze, ...
    'time staying still (fraction)', conditions);

sum(kldist14)

save(sprintf('pdist_tau=%ds.mat', obsvTime), 'p_dist', 'x_dist');

function [pdata, x, kldist14, wsdist14]=calc_pdf(dstr, xlbl, conditions)
% this function calculates the pdf and KL divergence
    
    alldata=[dstr{:}];
    edges=linspace(min(alldata), max(alldata), 10);

    % calculate probabilities
    for k=1:4
        data = dstr{k};
        [n,x]=histcounts(data, edges);
        pdata(:,k)= n/sum(n);
    end

    % Wasserstein distance
    WSdist=zeros(4);
    for ii=[1,4]
        for jj=ii+1:4
            WSdist(ii,jj)=ws_distance(pdata(:,ii), pdata(:,jj));
        end
    end
    WSdist
    wsdist14=WSdist(1,4);
    
    % KL Divergence
    KLdist=zeros(4);
    for ii=1:4
        for jj=ii+1:4
            KLdist(ii,jj)=kldiv(pdata(:,ii), pdata(:,jj));
        end
    end
    KLdist
    kldist14=KLdist(1,4);
    
    % plot everything
    gca; cla;
    for k=[1,4]
        plot(x(1:end-1),pdata(:,k), 'linewidth', 2);
        hold on;
    end
    grid on;
    set(gca,'ylim',[0,1])
    set(gca, 'fontsize', 16);
    xlabel(xlbl)
    ylabel('p')
    title(sprintf('Wass.=%.2f, KL=%.2f', ...
            wsdist14, kldist14));
    legend(conditions([1,4]))
    drawnow;
    
    x=x(1:end-1); % because we need equal edges and p values
end

function dstr=extract_speed_data(obsvTime,ID_Data, ...
    dtTrack, dtCommand,nameFolds)

    obsvFrames=obsvTime/dtTrack;
    type='EKFVel';
    % concatenate data into each 
    for cond=1:4
        dstr{cond}=[];
        for ii=1:size(nameFolds,1)
            append1=csvread(['../data/FILTERED/', ...
                nameFolds{ii}, '/', type, '_condition_', num2str(cond), '.csv']);
            idx=ID_Data(:,1)==str2double(nameFolds{ii});
            [r,c]=size(append1);
            % to keep everything consistent
            if r>c, append1=append1'; end 
            if ID_Data(idx,2) && cond==4
                append1=append1(:,1:ID_Data(idx,end));
            end
            % create distribution after rejecting last 10% of the trial
            data=append1(:,1:round(0.9*size(append1,2)));
            toremove=mod(size(data,2),obsvFrames);
            data1=data(:,1:end-toremove);

            % data operation
            data1=data1; % use data as is
            data=mean(reshape(data1, [], obsvFrames),2);

            dstr{cond}=[dstr{cond}, data'];
        end
    end
end


function dstr=extract_dist_data(obsvTime,ID_Data, ...
    dtTrack, dtCommand,nameFolds)

    obsvFrames=obsvTime/dtTrack;
    type='EKFtraj';
    % concatenate data into each 
    for cond=1:4
        dstr{cond}=[];
        for ii=1:size(nameFolds,1)
            append1=csvread(['../data/FILTERED/', ...
                nameFolds{ii}, '/', type, '_condition_', num2str(cond), '.csv']);
            idx=ID_Data(:,1)==str2double(nameFolds{ii});
            [r,c]=size(append1);
            % to keep everything consistent
            if r>c, append1=append1'; end 
            append1=append1(1:2,:);
            if ID_Data(idx,2) && cond==4
                append1=append1(:,1:ID_Data(idx,end));
            end
            % create distribution after rejecting last 10% of the trial
            data=append1(:,1:round(0.9*size(append1,2)));
            toremove=mod(size(data,2)-1,obsvFrames); % because of diff
            data1=data(:,1:end-toremove); 

            % data operation
            data1=sqrt(sum(diff(data1,1,2).^2,1)); % use data as is
            data=sum(reshape(data1, [], obsvFrames),2);

            dstr{cond}=[dstr{cond}, data'];
        end
    end
end

function dstr=extract_turnrate_data(obsvTime,ID_Data, ...
    dtTrack, dtCommand,nameFolds)

    obsvFrames=obsvTime/dtTrack;
    type='EKFom';
    % concatenate data into each 
    for cond=1:4
        dstr{cond}=[];
        for ii=1:size(nameFolds,1)
            append1=csvread(['../data/FILTERED/', ...
                nameFolds{ii}, '/', type, '_condition_', num2str(cond), '.csv']);
            idx=ID_Data(:,1)==str2double(nameFolds{ii});
            [r,c]=size(append1);
            % to keep everything consistent
            if r>c, append1=append1'; end 
            if ID_Data(idx,2) && cond==4
                append1=append1(:,1:ID_Data(idx,end));
            end
            % create distribution after rejecting last 10% of the trial
            data=append1(:,1:round(0.9*size(append1,2)));
            toremove=mod(size(data,2),obsvFrames); % because of diff
            data1=abs(data(:,1:end-toremove)); 

            % data operation
            data=mean(reshape(data1, [], obsvFrames),2);

            dstr{cond}=[dstr{cond}, data'];
        end
    end
end

function dstr=extract_angle_data(obsvTime,ID_Data, ...
    dtTrack, dtCommand,nameFolds)

    obsvFrames=obsvTime/dtTrack;
    type='EKFtraj';
    % concatenate data into each 
    for cond=1:4
        dstr{cond}=[];
        for ii=1:size(nameFolds,1)
            append1=csvread(['../data/FILTERED/', ...
                nameFolds{ii}, '/', type, '_condition_', num2str(cond), '.csv']);
            idx=ID_Data(:,1)==str2double(nameFolds{ii});
            [r,c]=size(append1);
            % to keep everything consistent
            if r>c, append1=append1'; end 
            append1=append1(3,:);
            if ID_Data(idx,2) && cond==4
                append1=append1(:,1:ID_Data(idx,end));
            end
            % create distribution after rejecting last 10% of the trial
            data=append1(:,1:round(0.9*size(append1,2)));
            toremove=mod(size(data,2)-1,obsvFrames); % because of diff
            data1=data(:,1:end-toremove); 

            % data operation
            data1=abs(asin(sin(diff(data1,1,2)))); 
            data=sum(reshape(data1, [], obsvFrames),2);

            dstr{cond}=[dstr{cond}, data'];
        end
    end
end

function dstr=extract_freezing_data(obsvTime,ID_Data, ...
    dtTrack, dtCommand,nameFolds)

    obsvFrames=obsvTime/dtTrack;
    type1='EKFVel'; type2='EKFom';
    % concatenate data into each 
    for cond=1:4
        dstr{cond}=[];
        for ii=1:size(nameFolds,1)
            % speed
            append1=csvread(['../data/FILTERED/', ...
                nameFolds{ii}, '/', type1, '_condition_', num2str(cond), '.csv']);
            idx=ID_Data(:,1)==str2double(nameFolds{ii});
            [r,c]=size(append1);
            % to keep everything consistent
            if r>c, append1=append1'; end 
            if ID_Data(idx,2) && cond==4
                append1=append1(:,1:ID_Data(idx,end));
            end
            % create distribution after rejecting last 10% of the trial
            data=append1(:,1:round(0.9*size(append1,2)));
            toremove=mod(size(data,2),obsvFrames);
            data1=data(:,1:end-toremove);

            % data operation
            speed_data=reshape(data1, [], obsvFrames);
            
            
            % turn rate
            append2=csvread(['../data/FILTERED/', ...
                nameFolds{ii}, '/', type2, '_condition_', num2str(cond), '.csv']);
            idx=ID_Data(:,1)==str2double(nameFolds{ii});
            [r,c]=size(append2);
            % to keep everything consistent
            if r>c, append2=append2'; end 
            if ID_Data(idx,2) && cond==4
                append2=append2(:,1:ID_Data(idx,end));
            end
            % create distribution after rejecting last 10% of the trial
            data=append2(:,1:round(0.9*size(append1,2)));
            toremove=mod(size(data,2),obsvFrames);
            data2=abs(data(:,1:end-toremove));

            % data operation
            tr_data=reshape(data2, [], obsvFrames);

            freezing_fraction=sum(speed_data<0.1 & tr_data<0.1,2)/obsvFrames;
            dstr{cond}=[dstr{cond}, freezing_fraction'];
        end
    end
end

