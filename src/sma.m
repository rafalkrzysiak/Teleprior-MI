function y=sma(x, win, method)
% function y=sma(x, win)
%
% simple moving average (default) / median filter
% x is a single dimension vector
% win is a positive number that denotes the size of the sliding window
% for moving average
% method average if not specified, or 'median' 
% y is the output


[nr, nc]=size(x);
column=0;
if(nr > nc)
    column=1;
end

if(column)
    x=x';
    nelem=nr;
else
    nelem=nc;
end
tx=zeros(win, nelem+win-1);
for ii=1:win
    tx(ii,ii:nelem+ii-1)=x;
end

if nargin ==3 
    if strcmp(method,'median')
        m_tx=median(tx,1);
    end
else
    s_tx=sum(tx);
    nz_tx=tx~=0;
    m_tx=s_tx./sum(nz_tx);
    m_tx(sum(nz_tx)==0)=0;
end

y=m_tx(win-1:nelem+win-2);
if(column)
    y=y';
end