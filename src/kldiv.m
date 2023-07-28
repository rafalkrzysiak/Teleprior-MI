function kldist=kldiv(pX,pY)
% function kldist=kldiv(X,Y)
%

% pX=pX+eps;
% pY=pY+eps;
% kldist=0;

% find which pX and pY are non-zero
idx=pX & pY;
% redo the probabilities because KLDIv is only defined
% if pY=0 -> pX=0. Since that is not necessarily true, we need 
% to rewrite and normalize the probabilities

pY=pY(idx); pY=pY/sum(pY);
pX=pX(idx); pX=pX/sum(pX);

kldist=sum(pX.*log2(pX./pY));

% for ii=1:numel(pX)
%     if pX(ii) && pY(ii)
%         kldist=kldist+pX(ii)*log2(pX(ii)/pY(ii));
%     end
% end
