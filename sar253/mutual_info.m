function m_info = mutual_info(sp,support)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
m_info = ent_kde(sp(1:2,:),1,support(1:2,:)) + ...
         ent_kde(sp(3:4,:),1,support(3:4,:)) - ...
         ent_kde(sp,1,support);
end

