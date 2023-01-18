function h=ent_kde(X, bw, s)
% ent_kde (beta version experimental)
%
% X is d x n samples from a distribution 
% s is d x m support (e.g. uniform random samples from the region)
% bw is bandwidth
% h is entropy in bits
%
% example
% calculate the entropy of 5 dimensional random variables on a support of 
% 10 points in the space of 3 sigma
% ent_kde(randn(5,100),1, -3+rand(5,10)*6)
% ent_kde(-3+rand(5,100)*6,1, -3+rand(5,10)*6)

p=kde(X, bw, s);
% [bw p]=kde3(X,10/bw);
h=-sum(p(p~=0).*log2(p(p~=0)));