function dstr=extract_speed_data(obsvTime,ID_Data, ...
    dtTrack, dtCommand)

obsvFrames=obsvTime/dtTrack;
type='EKFVel';
% concatenate data into each
for cond=1:4
    dstr{cond}=[];
    for ii=1:size(ID_Data,1)
        append1=csvread(['../data/FILTERED/', ...
            num2str(ID_Data(ii,1)), '/', type, '_condition_', num2str(cond), '.csv']);
        [r,c]=size(append1);
        % to keep everything consistent
        if r>c, append1=append1'; end
        if ID_Data(ii,2) && cond==4
            append1=append1(:,1:ID_Data(ii,end));
        end
        % create distribution after rejecting last 5 seconds of the trial
        data1=append1(:,1:(size(append1,2)-10));
%         toremove=mod(size(data,2),obsvFrames);
%         data1=data(:,1:end-toremove);
        
        % data operation
%         data=data1; % use data as is
        
%         data=mean(reshape(data1, [], obsvFrames),2);
        data=calculate_feature_over_tau(data1, obsvFrames, 'speed');
        dstr{cond}=[dstr{cond}, data'];
    end
end
end