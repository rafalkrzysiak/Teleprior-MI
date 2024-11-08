function dstr=extract_angle_data(obsvTime,ID_Data, ...
    dtTrack, dtCommand)

obsvFrames=obsvTime/dtTrack;
type='EKFtraj';
% concatenate data into each
for cond=1:4
    dstr{cond}=[];
    for ii=1:size(ID_Data,1)
        append1=csvread(['../data/FILTERED/', ...
            num2str(ID_Data(ii,1)), '/', type, '_condition_', num2str(cond), '.csv']);
        [r,c]=size(append1);
        % to keep everything consistent
        if r>c, append1=append1'; end
        append1=append1(3,:);
        if ID_Data(ii,2) && cond==4
            append1=append1(:,1:ID_Data(ii,end));
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