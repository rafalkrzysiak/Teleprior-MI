function avg_mutual = avg_mutual_info(v, omega, kF, mutual_kde, agent)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
sum_mutual = zeros(numel(v),numel(omega),1,agent);
avg_mutual = zeros(numel(v),numel(omega),1,agent);

for i = 1:agent
    for kkk = 1:kF
        sum_mutual(:,:,1,i) = mutual_kde(:,:,kkk,i) + sum_mutual(:,:,1,i);
    end
    avg_mutual(:,:,1,i) = sum_mutual(:,:,1,i)/kF;
end

end

