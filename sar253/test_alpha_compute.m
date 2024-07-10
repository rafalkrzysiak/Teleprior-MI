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

% tau should be param.tau but for that paramconfig should be called
% outside the for loop--can we do that?
% set tau 15 seconds for distance and 20 seconds for freezing -- SB
ConfigSetup="alpha_t/DistAndFreeze";

if ConfigSetup=="alpha_t/TotalDist"
    tau=15;
    data=extract_dist_data(tau, ids_train, ...
        dtTrack, dtCommand); % distance
    [p_dist, x_dist]=calc_pdf(data);
elseif ConfigSetup=="alpha_t/FreezeTime"
    % -- getting the values for the freeze time
    tau=30;
    data=extract_freezing_data(tau, ids_train, ...
        dtTrack, dtCommand); % distance
    [p_dist, x_dist]=calc_pdf(data);
elseif ConfigSetup=="alpha_t/DistAndFreeze"
    tau=15;
    dstr_dist=extract_dist_data(tau, ids_train, ...
        dtTrack, dtCommand); % distance
    dstr_freeze=extract_freezing_data(tau, ids_train, ...
        dtTrack, dtCommand); % freeze
    [p_dist, x_dist1, x_frz1]=calc_pdf_2D(dstr_dist, dstr_freeze);
    x_dist=[x_dist1; x_frz1];
end

figure(1); gcf; clf;
subplot(2,3,1);
if size(x_dist,1)==1
    plot(x_dist,p_dist);
else
    % plotting just for first condition only
    imagesc(x_dist(1,1:end-1), x_dist(2,1:end-1), p_dist(:,:,1));
    title('xMxT');
end


% load data
check=sprintf('%d', ids_test(1,1));
for ii=1:4
    [xs, speed, tr, tf] = RobotExperimentDataSet(check,num2str(ii));
    
    d = zeros(size(xs,2), 1);
    f_time = d;
    
    if ConfigSetup=="alpha_t/TotalDist"
        % calculate d
        d = sqrt(sum((xs(1:2,2:end) - xs(1:2,1:end-1)).^2))';
    elseif ConfigSetup=="alpha_t/FreezeTime"
        for k=1:tf
            if abs(speed(k)) < 0.1 && abs(tr(k)) < 0.1
                % -- add up individual time steps of the
                % -- simulation if the human controlled robot
                % -- is "freezing"
                f_time(k, 1) = 1; % this was 0.5 but should be 1 because we are counting frames --SB
            else
                % -- if the human controlled robot is
                % -- understood to not be "freezing" under the
                % -- required conditions, add 0.0s
                f_time(k, 1) = 0;
            end
        end
    elseif ConfigSetup=="alpha_t/DistAndFreeze"
        % calculate d
        d = sqrt(sum((xs(1:2,2:end) - xs(1:2,1:end-1)).^2))';
        for k=1:tf
            if abs(speed(k)) < 0.1 && abs(tr(k)) < 0.1
                % -- add up individual time steps of the
                % -- simulation if the human controlled robot
                % -- is "freezing"
                f_time(k, 1) = 1; % this was 0.5 but should be 1 because we are counting frames --SB
            else
                % -- if the human controlled robot is
                % -- understood to not be "freezing" under the
                % -- required conditions, add 0.0s
                f_time(k, 1) = 0;
            end
        end
    end
    % calculate alpha
    % alpha=ones(1,tf)*0.5;
    pK=0.25*ones(4,1)*ones(1,tf);
    for k=fps*tau:tf-1
        if ConfigSetup=="alpha_t/TotalDist"
            feature_k=sum(d(k-fps*tau+1:k, 1));
        elseif ConfigSetup=="alpha_t/FreezeTime"
            feature_k = sum(f_time(k-fps*tau+1:k, 1))/...
                (fps*tau);
        elseif ConfigSetup=="alpha_t/DistAndFreeze"
             feature_k=[sum(d(k-fps*tau+1:k, 1)), sum(f_time(k-fps*tau+1:k, 1))/...
                (fps*tau)];
        end
        %     alpha(k+1) = ...
        %         UpdateAlpha(feature_k, p_dist, x_dist, alpha(k));
        pK(:,k+1)=UpdateAlpha(feature_k, p_dist, x_dist, pK(:,k));
        % alpha=pK(1,:);
    end
    subplot(2,3,ii+1);
    plot(1:tf, pK, 'linewidth', 2);
    xlabel('frame');
    ylabel('probabilities');
    legend('xMxT', 'xMyT', 'yMxT', 'yMyT')
    title(sprintf('condition %d', ii))
    drawnow;
end