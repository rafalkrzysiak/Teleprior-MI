function kldist=kldiv(pX,pY)
% function kldist=kldiv(X,Y)
%

% pX=pX+eps;
% pY=pY+eps;
kldist=0;
for ii=1:numel(pX)
    if pX(ii) && pY(ii)
        kldist=kldist+pX(ii)*log2(pX(ii)/pY(ii));
    end
end
