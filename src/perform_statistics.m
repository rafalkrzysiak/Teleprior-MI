% Run preprocess_data.m first, this will take raw trajectory ../data, filter it, store secondary measures, such as speed, turn rates, times, etc. into separate files, all in the subject's folder for later analysis
% Run plot_traj.m next. This will plot the ../data for visual analysis
% Run this file next

%% initialize variables
xticklbl={'\begin{tabular}{c} No Map \\ No Target\end{tabular}',...
                 '\begin{tabular}{c} No Map \\ Yes Target\end{tabular}',...
                 '\begin{tabular}{c} Yes Map \\ No Target\end{tabular}',...
                 '\begin{tabular}{c} Yes Map \\ Yes Target\end{tabular}'};


%% Time to find etc.
figure(1); gcf; clf;
dfile ='H1-stats.txt';
if exist(dfile, 'file') ; delete(dfile); end
diary(dfile)
diary on

subplot(231);
% time to find
% hyp: time to find will depend on prior knowledge. 
timeArray = readtable('../data/TimeToFind.csv');
timeArray.c1sec=timeArray.c1sec/1.135; % scaling to compensate for length and obstacles
plot_and_stats(timeArray, 2:5, xticklbl, 'Time to find (s)','(a)',[],1);


% Total distance traveled
subplot(232);
TotalDistArray = csvread('../data/TotalDistanceTravel.csv');
TotalDistArray(:,1)=TotalDistArray(:,1)/1.135;
t_TotalDist = array2table(TotalDistArray,...
    'VariableNames',{'C1','C2','C3','C4','C5','SQ1','SQ2','SQ3','SQ4','SQ5'});
plot_and_stats(t_TotalDist, 1:4,  xticklbl, 'Distance travelled (m)','(b)',[], 1);

% Fraction time staying in place
subplot(233);
timeStayingInPlaceArray = csvread('../data/timeStayingInPlace.csv');
t_timeStayingInPlace = array2table(timeStayingInPlaceArray,...
    'VariableNames',{'C1','C2','C3','C4','C5','SQ1','SQ2','SQ3','SQ4','SQ5'});
plot_and_stats(t_timeStayingInPlace, 1:4,  xticklbl, 'Fraction time staying in place',...
                    '',[0,1],1);


% Fraction time spent turning in place
subplot(234);
timeTurningInPlaceArray = csvread('../data/fractionTimeTurningInPlace.csv');
%timeTurningInPlaceArray(:,1)=timeTurningInPlaceArray(:,1)/2.27; % not needed
t_timeTurningInPlace = array2table(timeTurningInPlaceArray,...
    'VariableNames',{'C1','C2','C3','C4','C5','SQ1','SQ2','SQ3','SQ4','SQ5'});
plot_and_stats(t_timeTurningInPlace, 1:4,  xticklbl, ...
    'Fraction of time turning in place', '(c)',[0 1], 1);




% Fraction time staying still
% hyp: stopping percentage will depend on prior knowledge. 
% higher stopping percentage when map is not known
% Total time in place traveled
TotalTimeStoppingArray = csvread('../data/timeStayingStill.csv'); 
TotalTimeStoppingArray(:,1) = TotalTimeStoppingArray(:,1)/2.27;  % <-- Should be dvided by 2.27
t_TotalTimeStoppingArray = array2table(TotalTimeStoppingArray,...
    'VariableNames',{'C1','C2','C3','C4','C5','SQ1','SQ2','SQ3','SQ4','SQ5'});
plot_and_stats(t_TotalTimeStoppingArray, 1:4,  xticklbl, ...
    'Average time spent staying still (s)', '', [], 0);

% fraction
subplot(235);
FractionTimeStayingStillArray = csvread('../data/FractionTimeStayingStill.csv');
t_FractionTimeStayingStillArray = array2table(FractionTimeStayingStillArray,...
    'VariableNames',{'C1','C2','C3','C4','C5','SQ1','SQ2','SQ3','SQ4','SQ5'});
plot_and_stats(t_FractionTimeStayingStillArray, 1:4,  xticklbl,...
    'Fraction of time staying still (freezing)', '(d)',[0,1],1);

% frequency of stops
subplot(236);
FreqStopsArray = csvread('../data/FreqStops.csv');
t_FreqStopsArray = array2table(FreqStopsArray,...
    'VariableNames',{'C1','C2','C3','C4','C5'});
plot_and_stats(t_FreqStopsArray, 1:4,  xticklbl,...
    'Frequency of stops', '',[],1);

diary off

%% commanded speed etc.
figure(2); gcf; clf;
dfile ='H2-stats.txt';
if exist(dfile, 'file') ; delete(dfile); end
diary(dfile)
diary on

subplot(221)
% hyp: commanded speed will depend on prior knowledge. 
% higher speeds when target is known (c2 with c1, c4 with c3)
% higher speeds when map is known (c3 with c1)
comSpeedArray = csvread('../data/ComSpeedData.csv');
t_comSpeed = array2table(comSpeedArray,...
    'VariableNames',{'C1','C2','C3','C4','C5','SQ1','SQ2','SQ3','SQ4','SQ5'});
plot_and_stats(t_comSpeed, 1:4,  xticklbl, ...
    'Commanded speed (m/s)', '(a)', [], 1);
set(gca, 'YLim', [0, 0.3]);

% commanded turn rate
subplot(222)
% hyp: commanded turn rate will depend on prior knowledge. 
% Higher turn rate when target is not known
% no difference when map is known
comTurnRateArray = csvread('../data/ComTurnrateData.csv');
t_comTurnRate = array2table(comTurnRateArray,...
    'VariableNames',{'C1','C2','C3','C4','C5','SQ1','SQ2','SQ3','SQ4','SQ5'});
plot_and_stats(t_comTurnRate, 1:4,  xticklbl, ...
    'Commanded turn rate (rad/s)','(b)',  [], 1);
set(gca, 'YLim', [0, 1/2]);

%% robot speed
figure(3); gcf; clf;
% Robot speed
subplot(121);
% hyp: speed will depend on prior knowledge.
% higher speeds when target is known (c2 with c1, c4 with c3)
% higher speeds when map is known (c3 with c1)
EKFSpdData = csvread('../data/RobotSpeedData.csv');
t_EKFSpd = array2table(EKFSpdData,...
    'VariableNames',{'C1','C2','C3','C4','C5','SQ1','SQ2','SQ3','SQ4','SQ5'});
plot_and_stats(t_EKFSpd, 1:4,  xticklbl, ...
    'Robot speed (m/s)', '(a)', [], 1);
set(gca, 'YLim', [0, 0.3]);

% Robot turn rate
subplot(122);
% hyp: turn rate will depend on prior knowledge. 
% Higher turn rate when target is not known
% no difference when map is known
EKFOmegaArray = csvread('../data/RobotTurnrateData.csv');
t_EKFOmega = array2table(EKFOmegaArray,...
    'VariableNames',{'C1','C2','C3','C4','C5','SQ1','SQ2','SQ3','SQ4','SQ5'});
plot_and_stats(t_EKFOmega, 1:4,  xticklbl, ...
    'Robot turn rate (rad/s)', '(b)', [],1);
set(gca, 'YLim', [0, 1.1]);

diary off;

%% H3, Does accuracy in prior knowledge affect motion
figure(4); gcf; clf;
dfile ='H3-stats.txt';
if exist(dfile, 'file') ; delete(dfile); end
diary(dfile)
diary on

xticklbl={'\begin{tabular}{c} No Target\end{tabular}',...
          '\begin{tabular}{c} Accurate \end{tabular}',...
          '\begin{tabular}{c} Inaccurate \end{tabular}'};
% speed between 3, 4a and 4b
% knowledge of the target not present in location 1 implies it's presence in location 2, 
% thus leading to an increase in speed which resembles when target location is known in 4a
subplot(231);
EKFSpdData = csvread('../data/RobotSpeedData.csv');
EKFSpdData = EKFSpdData(EKFSpdData(:,5)~=0,[3,4,5]);
t_EKFSpd = array2table(EKFSpdData,...
    'VariableNames',{'C1','C2','C3'});
plot_and_stats(t_EKFSpd, 1:3,  xticklbl, 'Robot speed (m/s)', '(a)', [],1);

subplot(232);
EKFOmegaArray = csvread('../data/RobotTurnrateData.csv');
EKFOmegaArray = EKFOmegaArray((EKFOmegaArray(:,5) ~= 0), [3,4,5]);
t_EKFOmega = array2table(EKFOmegaArray,...
    'VariableNames',{'C1','C2','C3'});
plot_and_stats(t_EKFOmega, 1:3,  xticklbl, 'Robot turn rate (rad/s)', '(b)',[],1);

% commanded speed
subplot(233);
% hyp: commanded speed will depend on prior knowledge. 
% higher speeds when target is known (c2 with c1, c4 with c3)
% higher speeds when map is known (c3 with c1)
comSpeedArray = csvread('../data/ComSpeedData.csv');
comSpeedArray = comSpeedArray((comSpeedArray(:,5) ~= 0), [3,4,5]);
t_comSpeed = array2table(comSpeedArray,...
    'VariableNames',{'C1','C2','C3'});
plot_and_stats(t_comSpeed, 1:3,  xticklbl, 'Commanded speed (m/s)', '(c)', [],1);


% commanded turn rate
subplot(234);
% hyp: commanded turn rate will depend on prior knowledge. 
% Higher turn rate when target is not known
% no difference when map is known
comTurnRateArray = csvread('../data/ComTurnrateData.csv');
comTurnRateArray = comTurnRateArray((comTurnRateArray(:,5) ~= 0), [3,4,5]);
t_comTurnRate = array2table(comTurnRateArray,...
    'VariableNames',{'C1','C2','C3'});
plot_and_stats(t_comTurnRate, 1:3,  xticklbl, 'Commanded turn rate (rad/s)','(d)', [],1);


%Total time staying in place 
fractionTimeStayingInPlaceArray = csvread('../data/timeStayingInPlace.csv');
% timeTurningInPlaceArray(:,1)=timeTurningInPlaceArray(:,1)/2.27;
fractionTimeStayingInPlaceArray = fractionTimeStayingInPlaceArray((fractionTimeStayingInPlaceArray(:,5) ~= 0), [3,4,5]);
t_fractionTimeStayingInPlaceArray = array2table(fractionTimeStayingInPlaceArray,...
    'VariableNames',{'C1','C2','C3'});
plot_and_stats(t_fractionTimeStayingInPlaceArray, 1:3,  xticklbl, ...
    'Fraction time spent staying in place', '',[],0);

%Total time turning in place 
subplot(235);
timeTurningInPlaceArray = csvread('../data/fractionTimeTurningInPlace.csv');
% timeTurningInPlaceArray(:,1)=timeTurningInPlaceArray(:,1)/2.27;
timeTurningInPlaceArray = timeTurningInPlaceArray((timeTurningInPlaceArray(:,5) ~= 0), [3,4,5]);
t_timeTurningInPlace = array2table(timeTurningInPlaceArray,...
    'VariableNames',{'C1','C2','C3'});
plot_and_stats(t_timeTurningInPlace, 1:3,  xticklbl, ...
    'Fraction time spent turning in place','(e)', [0 1],1);

subplot(236);
FractionTimeStayingStillArray = csvread('../data/FractionTimeStayingStill.csv');
FractionTimeStayingStillArray = FractionTimeStayingStillArray((FractionTimeStayingStillArray(:,5) ~= 0), [3,4,5]);
t_FractionTimeStayingStillArray = array2table(FractionTimeStayingStillArray,...
    'VariableNames',{'C1','C2','C3'});
plot_and_stats(t_FractionTimeStayingStillArray, 1:3,  xticklbl, ...
    'Fraction time spent staying still','(f)', [0 1],1);

diary off

%% Workload         
figure(5); gcf; clf;
dfile ='Workload-stats-C4a.txt';
if exist(dfile, 'file') ; delete(dfile); end
diary(dfile)
diary on

refDataforc4 = csvread('../data/ComTurnrateData.csv');
refDataforc4=refDataforc4(:,5);

xticklbl={'\begin{tabular}{c} No Map \\ No Target\end{tabular}',...
                 '\begin{tabular}{c} No Map \\ Yes Target\end{tabular}',...
                 '\begin{tabular}{c} Yes Map \\ No Target\end{tabular}',...
                 '\begin{tabular}{c} Yes Map \\ Yes Target\end{tabular}'};

NASATLXArray(:,:,1) = csvread('../data/TLX_question_1.csv');
NASATLXArray(:,:,2) = csvread('../data/TLX_question_2.csv');
NASATLXArray(:,:,3) = csvread('../data/TLX_question_3.csv');
NASATLXArray(:,:,4) = csvread('../data/TLX_question_4.csv');
NASATLXArray(:,:,5) = csvread('../data/TLX_question_5.csv');
NASATLXArray(:,:,6) = csvread('../data/TLX_question_6.csv');
figure(1); gcf; clf;
xlbl={'(a)', '(b)', '(c)', '(d)', '(e)', '(f)'};


ylbl={'Mental','Physical','Temporal','Performance','Effort','Frustration'};

for ii=1:6
    subplot(2,3,ii)
    plot_and_stats(array2table(NASATLXArray(refDataforc4==0,:,ii)), 1:4,  xticklbl, ...
        ylbl{ii}, xlbl{ii}, [0 100],1);
%     hold on;
%     plot(4, NASATLXArray(refDataforc4==0,4,ii), 'o', 'markersize', ...
%         12, 'color', 'k', 'markerface', ones(1,3));
end

diary off


function plot_and_stats(data_table, colidx, xticklbl, ylbl, xlbl, ylim, disp1)

fprintf('[%s] \n', ylbl);

if disp1
% time_bp=boxplot(table2array(t_time(:,1:4)), 'labels', {'C1','C2','C3','C4'});
% plot(table2array(data_table(:,colidx))', '-o', 'markersize', 12, 'color', ones(1,3)*.5, 'markerface', ones(1,3)*.75);
% boxplot(table2array(data_table(:,colidx)), 'color', ones(1,3)*.5);
mu=mean(table2array(data_table(:,colidx)));
st=std(table2array(data_table(:,colidx)),[],1);
bar(mu, 'Facecolor', ones(1,3)*.5);
hold on;
errorbar(1:size(mu,2), mu, st, 'k.');
% hold on;
% plot(median(table2array(data_table(:,colidx))), 'k+', 'markersize', 16, 'linewidth', 2);
set(gca, 'xtick', 1:4)
ylabel(ylbl);
xlabel(xlbl);
end
% friedman test (non-parametric)
[p_data, tbl_data, stats_data]=friedman(table2array(data_table(:,colidx)), 1, 'off');
tbl_data
if p_data < 0.05
    res_data=multcompare(stats_data, 'Ctype','bonferroni', 'Display', 'off')
    if disp1
        sigstar([],res_data,1);
    end
end

% ANOVA repeated measures
% if numel(colidx)==4
%     within4=array2table([1 2 3 4]', 'variablenames', {'cond'});
%     rm_data=fitrm(data_table,'C1-C4~1', 'WithinDesign', within4);
% else
%     within3=array2table([1 2 3]', 'variablenames', {'cond'});
%     rm_data=fitrm(data_table,'C1-C3~1', 'WithinDesign', within3);
% end
% stats_data = ranova(rm_data)
% if stats_data.pValue(1) < 0.05
%     res_data=multcompare(rm_data, 'cond', 'ComparisonType','bonferroni')
%     if disp1
%     sigstar([],res_data,1);
%     end
% end
% -- make figure look nice
if disp1
box off;
ax = gca;
ax.FontSize = 18;
ax.TickLabelInterpreter = 'latex';
ax.XTickLabel = xticklbl(1:size(mu,2));
if ~isempty(ylim)
    set(gca, 'ylim', ylim);
end
end
end