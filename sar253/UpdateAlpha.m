function pKk_fk = UpdateAlpha(feature_k, pdist, x_dist, pK_km1)
% UpdateAlpha will update the weighting factor alpha_k which is the
% posterior probability p(pK_k=xMxT|feature_k)
% pdist=p(feature_k|K) for each of the four types of prior knowledge
% feature_k is average speed, turn rate, fraction freezing time
% x_dist is the support/range of the pdf
% 
% pK_k= p(K_k|feature_k) 
% pK_km1=p(K_km1|feature_k)

% -- calculate p(d|xMxT) and p(d|yMyT)
% -- discussed during zoom meeting 5/23/2023

% 
% if feature_k < max(x_dist)
%     p_f_xMxT = interp1(x_dist, pdist(:,1), feature_k); % p(f|xMxT)
%     p_f_xMyT = interp1(x_dist, pdist(:,2), feature_k); % p(f|xMyT)
%     p_f_yMxT = interp1(x_dist, pdist(:,3), feature_k); % p(f|xMyT)
%     p_f_yMyT = interp1(x_dist, pdist(:,4), feature_k); % p(f|yMyT)
% else
%     p_f_xMxT = 0.00001;
%     p_f_xMyT = 0.00001;
%     p_f_yMxT = 0.00001;
%     p_f_yMyT = 0.00001;
% end
% 
% if isnan(p_f_xMxT)
%     p_f_xMxT = 0.00001;
% end
% 
% if isnan(p_f_xMyT)
%     p_f_xMyT = 0.00001;
% end
% 
% if isnan(p_f_yMxT)
%     p_f_yMxT = 0.00001;
% end
% 
% if isnan(p_f_yMyT)
%     p_f_yMyT = 0.00001;
% end
% 
% % -- avoid zero probabilities
% if ~p_f_xMxT
%     p_f_xMxT = 0.00001;
% end
% 
% if ~p_f_xMyT
%     p_f_xMyT = 0.00001;
% end
% 
% if ~p_f_yMxT
%     p_f_yMxT = 0.00001;
% end
% 
% if ~p_f_yMyT
%     p_f_yMyT = 0.00001;
% end

% update for thri revision
p_f_Kk=zeros(1,4);
pKk_fk=zeros(1,4);

for ii=1:4
    if numel(feature_k)==2
        if feature_k(1) < max(x_dist(1,:)) && feature_k(2) < max(x_dist(2,:))
                p_f_Kk(ii)=interp2(x_dist(1,:), x_dist(2,:), pdist(:,:,ii), ...
                                        feature_k(1), feature_k(2));
        else
            p_f_Kk(ii)=0.00001;
        end
    else
        if feature_k < max(x_dist)
            p_f_Kk(ii)=interp1(x_dist, pdist(:,ii), feature_k);
        else
            p_f_Kk(ii)=0.00001;
        end
    end
    
    if isnan(p_f_Kk(ii))
        p_f_Kk(ii)=0.00001;
    end
    
    % -- avoid zero probabilities
    if ~p_f_Kk(ii)
        p_f_Kk(ii)=0.00001;
    end
end

% -- update alpha
% Bayes' rule
% p(K[k]=xMxT|f_k)=p(f_k|xMxT)*p(xMxT[k-1])/{normalization factor}
% where 
% {normalization factor}    =   p(f_k|xMxT)*p(xMxT[k-1]) + ...
%                               p(f_k|xMyT)*p(xMyT[k-1]) + ...
%                               p(f_k|yMxT)*p(yMxT[k-1]) + ...
%                               p(f_k|yMyT)*p(yMyT[k-1])

for ii=1:4
    pKk_fk(ii)=p_f_Kk(ii)*pK_km1(ii);
end

% normalization factor
normf=sum(pKk_fk);

for ii=1:4
    pKk_fk(ii)=pKk_fk(ii)/normf;
end

% if any of the probabilities are << 0 then make them small so that they
% can recover
for ii=1:4
    if pKk_fk(ii) < 0.001
        pKk_fk(ii)=0.001;
    end
end

% makes the assumption that the operator can either have no
% knowledge or full knowledge
% alpha = (p_f_xMxT * alpha)/(p_f_xMxT*alpha + p_f_yMyT*(1-alpha));
% alpha = (p_f_yMyT * alpha)/(p_f_yMyT*alpha + p_f_xMxT*(1-alpha));



% -- make sure that alpha is not too far into 0
% -- If alpha is zero, alpha will never recover above zero
% if alpha < 0.01
%     alpha = 0.01;
% end



% else
%     % -- preset the alpha to be the alpha within the config file
%     alpha = param.alphaBegin;
% end

end