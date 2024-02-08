function p = pf_predict(p, gmot, w, robot)
%function [Xh, P] = pf_predict(p, F, dist, n)
%
% p is d x N matrix with d dimensional state and N particles
% gmot is motion model
% w is the disturbance variance (assumed constant along all dimensions)

% -- get the dimension of the particle state
d=size(p,1);

for jj=1:size(p,2)
    p(:,jj)=gmot(p(:,jj)) + w.*randn(d,1);
end