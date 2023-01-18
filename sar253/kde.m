function [px, s]=kde(x, bw, s)
% function px=kde(x, bw)
% x is d x n vector
% bw is the bandwidth
% a gaussian symmetric kernel is used
% s is the d x m support on which px is computed
%
% example
% probability of a 1D normal random variable
% [px, s]=kde(randn(1,10), 1, -3+rand(1,15)*6); 
% [val,idx]=sort(s); 
% plot(s(idx),px(idx))
%
% 2D 
% [px, s]=kde(randn(2,100), 1, -3+rand(2,50)*6);
% plot3(s(1,:), s(2,:), px, '.')
%
% Sachit Butail, 2014

nx=size(x,2);
d=size(x,1);

if d>nx, error('x should be d x n matrix'); end

ker=@(x) (2*pi)^(-d/2)*exp(-1/2*sum(x.*x,1)); %x'*x
H=bw*eye(d);
kerh=@(x) det(H)^(-d)*ker(H^(-1)*x);

% px=zeros(1,n);
% for i=1:n
%     px(i)=1/n*sum(kerh(x-x(:,i)*ones(1,n)));
% end

m=size(s,2);
px=zeros(1,m);
for i=1:m
    px(i)=1/m*sum(kerh(x-s(:,i)*ones(1,nx)));
end
px=px/sum(px);