function dstr=extract_freezing_data(obsvTime,ID_Data, ...
    dtTrack, dtCommand)

obsvFrames=obsvTime/dtTrack;
type1='EKFVel'; type2='EKFom';
% concatenate data into each
for cond=1:4
    dstr{cond}=[];
    for ii=1:size(ID_Data,1)
        % speed
        append1=csvread(['../data/FILTERED/', ...
            num2str(ID_Data(ii,1)), '/', type1, '_condition_', num2str(cond), '.csv']);
        [r,c]=size(append1);
        % to keep everything consistent
        if r>c, append1=append1'; end
        if ID_Data(ii,2) && cond==4
            append1=append1(:,1:ID_Data(ii,end));
        end
        % create distribution after rejecting last 10% of the trial
        speed_data=append1(:,1:round(0.9*size(append1,2)));
%         toremove=mod(size(data,2),obsvFrames);
%         data1=data(:,1:end-toremove);
        
        % data operation
%         speed_data=reshape(data1, [], obsvFrames);

        
        
        % turn rate
        append2=csvread(['../data/FILTERED/', ...
            num2str(ID_Data(ii,1)), '/', type2, '_condition_', num2str(cond), '.csv']);
        [r,c]=size(append2);
        % to keep everything consistent
        if r>c, append2=append2'; end
        if ID_Data(ii,2) && cond==4
            append2=append2(:,1:ID_Data(ii,end));
        end
        % create distribution after rejecting last 10% of the trial
        tr_data=append2(:,1:round(0.9*size(append1,2)));
%         toremove=mod(size(data,2),obsvFrames);
%         data2=abs(data(:,1:end-toremove));
        
        % data operation
%         tr_data=reshape(data2, [], obsvFrames);
        
%         freezing_fraction=sum(speed_data<0.1 & tr_data<0.1,2)/obsvFrames;
        freezing_fraction=calculate_feature_over_tau([speed_data(:)'; tr_data(:)'],...
            obsvFrames, 'freezing');
        dstr{cond}=[dstr{cond}, freezing_fraction'];
    end
end
end
