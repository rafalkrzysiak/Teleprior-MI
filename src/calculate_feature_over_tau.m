function data1=calculate_feature_over_tau(data, tauframes, feature)

data1=zeros(1,size(data,2)-tauframes+1);
switch feature
    case 'speed'
        for ii=tauframes:size(data,2)
            % note that we use past tau frames, which makes it useful for 
            % real-time applications
            data1(ii-tauframes+1)=mean(data(ii-tauframes+1:ii));
        end
    case 'distance'
        for ii=tauframes:size(data,2)
            data1(ii-tauframes+1)=sum(data(ii-tauframes+1:ii));
        end
    case 'turnrate'
        for ii=tauframes:size(data,2)
            data1(ii-tauframes+1)=mean(data(ii-tauframes+1:ii));
        end
    case 'freezing'
        for ii=tauframes:size(data,2)
            speed_data=data(1,ii-tauframes+1:ii);
            tr_data=data(2,ii-tauframes+1:ii);           
            data1(ii-tauframes+1)= ...
                sum(speed_data<0.1 & tr_data<0.1,2)/tauframes;
        end    
end

data1=data1';