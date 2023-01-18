function h=ent(data, numberOfBins, support, form, correction)
% function h=ent(data, numberOfBins, support, form, correction)

% ent(X, numberOfBins, support) assumes that you have multiple values of a variable and you
% want to bin it into numberOfBins bins. p doesn't mean p but a variable

h=0;

% data may be discrete or continuous and that would in turn determine how
% the bins should be placed

% is the data continuous or discrete, this will be decided on the basis of
% the support which if it is less than the number of bins is assumed
% continuous


switch form
    case 'x' % a single variable entropy
        if numel(support) < numberOfBins
            bins=linspace(support(1), support(2), numberOfBins);
        else
            bins=support;
        end

        % histograms
        X=data(1,:);
        p=hist(X, bins);
        p=p/sum(p);
%         p(p==0)=1;
        p=p(p~=0);
        h=-sum(p(:).*log2(p(:)));
        
        % Miller-Madow correction
        if nargin >4
            if strcmp(correction, 'millermadow')
                h=h+(numel(p)-1)/(2*size(X,2));
            end
        end
        
    case 'x;y' % joint entropy
        
        X=data(1,:); 
        Y=data(2,:);
        
        for jj=1:2
            if size(support,2) < numberOfBins(jj)
                bins{jj}=linspace(support(jj,1), support(jj,2), numberOfBins(jj));
                binwidth=bins{jj}(2)-bins{jj}(1);
                bb=bins{jj};
                bins{jj}=[-inf bins{jj}(1)-binwidth/2 bins{jj}+binwidth/2];
                bins{jj}(end)=bb(end);
            else
                bins{jj}=[-inf support(jj,:)];
            end
            nbins(jj)=numel(bins{jj});
        end
        % p(X[markovOrder-1], Y[markovOrder-1])
        pXY=zeros(nbins(1)-1, nbins(2)-1);
        for jj=1:nbins(1)-1
            for kk=1:nbins(2)-1
                % ( ] 
                pXY(jj,kk)=sum(X>bins{1}(jj) & X<=bins{1}(jj+1) & ...
                               Y>bins{2}(kk) & Y<=bins{2}(kk+1));
%                 pXY(jj,kk)=sum(X>=bins{1}(jj) & X<=bins{1}(jj+1) & ...
%                                        Y>=bins{2}(kk) & Y<=bins{2}(kk+1));
            end
        end
        if size(support,2) < numberOfBins(1)
            pXY=pXY(2:end,2:end);
        end
        pXY=pXY(pXY~=0);

        
        pXY=pXY/sum(pXY(:));
                
        h=-sum(pXY(:).*log2(pXY(:)));
    
    case 'x;y;z' % joint entropy
        
        X=data(1,:); 
        Y=data(2,:);
        Z=data(3,:);
        
        for jj=1:3
            if size(support,2) < numberOfBins(jj)
                bins{jj}=linspace(support(jj,1), support(jj,2), numberOfBins(jj));
                binwidth=bins{jj}(2)-bins{jj}(1);
                bb=bins{jj};
                bins{jj}=[-inf bins{jj}(1)-binwidth/2 bins{jj}+binwidth/2];
                bins{jj}(end)=bb(end);
            else
                bins{jj}=[-inf support(jj,:)];
            end
            nbins(jj)=numel(bins{jj});
        end
        % p(X[markovOrder-1], Y[markovOrder-1])
        pXYZ=zeros(nbins(1)-1, nbins(2)-1, nbins(3)-1);
        for ii=1:nbins(1)-1
            for jj=1:nbins(2)-1
                for kk=1:nbins(3)-1
                    pXYZ(ii,jj,kk)=sum(X>bins{1}(ii) & X<=bins{1}(ii+1) & ...
                                       Y>bins{2}(jj) & Y<=bins{2}(jj+1) & ...
                                       Z>bins{3}(kk) & Z<=bins{3}(kk+1));
                end
            end
        end
        if size(support,2) < numberOfBins(1)
            pXYZ=pXYZ(2:end,2:end,2:end);
        end
        
        pXYZ=pXYZ(pXYZ~=0);

        
        pXYZ=pXYZ/sum(pXYZ(:));
                
        h=-sum(pXYZ(:).*log2(pXYZ(:)));
        
    case 'w;x;y;z' % joint entropy
        
        W=data(1,:);
        X=data(2,:); 
        Y=data(3,:);
        Z=data(4,:);
        
        for jj=1:4
            if size(support,2) < numberOfBins(jj)
                bins{jj}=linspace(support(jj,1), support(jj,2), numberOfBins(jj));
                binwidth=bins{jj}(2)-bins{jj}(1);
                bb=bins{jj};
                bins{jj}=[-inf bins{jj}(1)-binwidth/2 bins{jj}+binwidth/2];
                bins{jj}(end)=bb(end);
            else
                bins{jj}=[-inf support(jj,:)];
            end
            nbins(jj)=numel(bins{jj});
            
        end
        % p(X[markovOrder-1], Y[markovOrder-1])
        pWXYZ=zeros(nbins(1)-1, nbins(2)-1, nbins(3)-1, nbins(4)-1);
        for ii=1:nbins(1)-1
            for jj=1:nbins(2)-1
                for kk=1:nbins(3)-1
                    for ll=1:nbins(4)-1
                        pWXYZ(ii,jj,kk,ll)=sum( W>bins{1}(ii) & W<=bins{1}(ii+1) & ...
                                                X>bins{2}(jj) & X<=bins{2}(jj+1) & ...
                                                Y>bins{3}(kk) & Y<=bins{3}(kk+1) & ...
                                                Z>bins{4}(ll) & Z<=bins{4}(ll+1));
                    end
                end
            end
        end
        if size(support,2) < numberOfBins(1)
            pWXYZ=pWXYZ(2:end,2:end,2:end,2:end);
        end
        
        pWXYZ=pWXYZ(pWXYZ~=0);

        
        pWXYZ=pWXYZ/sum(pWXYZ(:));
                
        h=-sum(pWXYZ(:).*log2(pWXYZ(:)));    
        
    case 'x|y' % conditional entropy
        
        X=data(1,:); 
        Y=data(2,:);
        
        if numel(support)<numberOfBins
            bins=linspace(support(1), support(2), numberOfBins);
            binwidth=bins(2)-bins(1);
            bb=bins;
            bins=[-inf bins(1)-binwidth/2 bins+binwidth/2];
            bins(end)=bb(end);
        else
            bins=[-inf support];
        end
        nbins=numel(bins);
        
        % p(X[markovOrder-1], Y[markovOrder-1])
        pXY=zeros(nbins-1, nbins-1);
        for jj=1:nbins-1
            for kk=1:nbins-1
                pXY(jj,kk)=sum(X>bins(jj) & X<=bins(jj+1) & ...
                               Y>bins(kk) & Y<=bins(kk+1));
            end
        end
        if numel(support) < numberOfBins
            pXY=pXY(2:end,2:end);
        end
        pXY=pXY/sum(pXY(:));
        
        % pY
        pY=zeros(nbins-1,1);
        for jj=1:nbins-1
            pY(jj)=sum(Y>bins(jj) & Y<=bins(jj+1));
        end
        pY=pY(2:end);
        pY=pY/sum(pY(:));
        
        for ii=1:nbins-2
            for jj=1:nbins-2
                if pXY(ii,jj) && pY(ii)
                    h=h-pXY(ii,jj)*log2(pXY(ii,jj)/pY(ii));
                end
            end
        end    
end

