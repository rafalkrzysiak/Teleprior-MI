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
features={'speed', 'distance', 'turn rate', 'freezing'};
% features={'speed', 'distance', 'turn rate', 'angle', 'freezing'};
obsvTime=5:5:30;
% obsvTime=20;

kldist14=zeros(numel(obsvTime),4);
wsdist14=zeros(numel(obsvTime),4);
markertypes={'o', 's', 'd', '^'};

for ii=1:numel(obsvTime)
    
    figure(ii); gcf; clf;
    dstr_speed=extract_speed_data(obsvTime(ii), ID_Data, ...
        dtTrack, dtCommand); % speed
    subplot(2,2,1);
    [p_speed, x_speed]=calc_pdf(dstr_speed, ...
        'average speed (m/s)', conditions);
    [kldist14(ii,1),wsdist14(ii,1)]=calc_pdf_dist(p_speed, x_speed);
    
    dstr_dist=extract_dist_data(obsvTime(ii), ID_Data, ...
        dtTrack, dtCommand); % distance
    subplot(2,2,2);
    [p_dist, x_dist]=calc_pdf(dstr_dist,  ...
        'distance (m)', conditions);
    [kldist14(ii,2),wsdist14(ii,2)]=calc_pdf_dist(p_dist, x_dist);
    
    dstr_tr=extract_turnrate_data(obsvTime(ii), ID_Data, ...
        dtTrack, dtCommand); % turn rate
    subplot(2,2,3);
    [p_tr, x_tr]=calc_pdf(dstr_tr, ...
        'average turn rate (rad/s)', conditions);
    [kldist14(ii,3), wsdist14(ii,3)]=calc_pdf_dist(p_tr, x_tr);
    
%     dstr_angle=extract_angle_data(obsvTime(ii), ID_Data, ...
%         dtTrack, dtCommand); % angle
%     subplot(2,2,4);
%     [p_angle, x_angle]=calc_pdf(dstr_angle,  ...
%         'angle turned (rad)', conditions);
%     [kldist14(ii,4), wsdist14(ii,4)]=calc_pdf_dist(p_angle, x_angle);
    
    dstr_freeze=extract_freezing_data(obsvTime(ii), ID_Data, ...
        dtTrack, dtCommand); % freezing
    subplot(2,2,4);
    [p_frz, x_frz]=calc_pdf(dstr_freeze, ...
        'time staying still (fraction)', conditions);
    [kldist14(ii,4), wsdist14(ii,4)]=calc_pdf_dist(p_frz, x_frz);
    
%     print('-dpng', ...
%         sprintf('../doc/plots/pdistr_%ds.png', obsvTime(ii)))
end

figure(ii+1); gcf;clf;
% subplot(1,2,1);
% plot(obsvTime, kldist14, 'linewidth', 2);
% set(gca, 'fontsize', 16);
% xlabel('observation time (s)');
% ylabel('KL distance');
% legend(features)
% 
% subplot(1,2,2);

for ii=1:size(wsdist14,2)
    plot(obsvTime, wsdist14(:,ii),['k-', markertypes{ii}],'linewidth', 2, ...
        'Markersize', 18);
    hold on;
end
set(gca, 'fontsize', 16);
xlabel('Observation time (s)');
ylabel('Wasserstein distance');
legend(features)


% uncomment after running once and setting obsvTime to the one that gives
% the best estimate
% save(sprintf('pdist_tau=%ds.mat', obsvTime), 'p_dist', 'x_dist');



function [kldist14, wsdist14]=calc_pdf_dist(pdata, x)

% Wasserstein distance
WSdist=zeros(4);
for ii=[1,4]
    for jj=ii+1:4
        WSdist(ii,jj)=ws_distance(pdata(:,ii), pdata(:,jj));
    end
end

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

title(sprintf('WD=%.3f, KLD=%.2f', ...
    wsdist14, kldist14), 'fontweight', 'normal');

end










