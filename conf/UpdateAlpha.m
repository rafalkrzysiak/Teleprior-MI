function [alpha, pDxMxT_k, pDyMyT_k] = UpdateAlpha(feature_k, pdist, x, alpha)
% -- this function will serve to update alpha given the distance
% -- traveled by the reference robot. Probabilities are calculated
% -- using experimental data captured by the Omron lab teleoperation

% alpha is the posterior which recursively gets updated every time step.

% -- Tau was determined in post process (pass tau)
% tau = 50;

% -- We need at minimum 2 timesteps to calculate the distance
% -- traveled by reference robot. If the condition is not met,
% -- set alpha to the pre determined value param.alphaBegin
% -- which is defined in the ParamConfig.m file
% if k > 1
% -- calculate the distance traveled by the reference robot
% -- within the environment
% calculate d once outside and send as a vector of T-1 elements where
% each element is just the distance between the k and k-1-th time-step
%     d(k, 1) = sqrt(sum((Xh(6:7,k) - Xh(6:7,k-1)).^2)); % comment or delete

% -- only enter when we have at minimum of param.alphaBuffer steps available
% -- param.alphaBuffer is entered in the ParamConfig.m file
% -- the param.alphaBuffer value is arbitrarly chosen for right now,
% -- possible more optimized value will replace in future
% SB: alphbuffer is tau in 2023-05-19.pdf and will be decided based on
% an optimization that runs to maximize the KLdistance between the pdfs
% of the random vector X=[speed, turn rate, ...] so, we can set it here
% and not treat it as a control design parameter.
% for example, set tau=50
% -- needed a buffer at the beginning of the alpha/totaldist calculation
% -- the believed location of the reference robot jumps meters in timesteps [1 3]
% -- 3 was added to avoid the massive jump in distance which created a
% -- Nan value for pDxMxT_k and pDyMyT_k using the interp1 function

% -- calculate p(d|xMxT) and p(d|yMyT)
% -- discussed during zoom meeting 5/23/2023
if feature_k < max(x)
    pDxMxT_k = interp1(x, pdist(:,1), feature_k); % p(d|xMxT)
    pDyMyT_k = interp1(x, pdist(:,4), feature_k); % p(d|yMyT)
else
    pDxMxT_k = 0.00001;
    pDyMyT_k = 0.00001;
end

if isnan(pDxMxT_k)
    pDxMxT_k = 0.00001;
end

if isnan(pDyMyT_k)
    pDyMyT_k = 0.00001;
end

% -- avoid zero probabilities
if ~pDxMxT_k
    pDxMxT_k = 0.00001;
end

if ~pDyMyT_k
    pDyMyT_k = 0.00001;
end

% % -- avoid zero probabilities
% p_k = pDxMxT_k + pDyMyT_k;
% if ~p_k
%     pDxMxT_k = 0.001;
%     pDyMyT_k = 0.001;
% end

% -- update alpha
% -- we can play around with this calculation
% -- TO DO:
% -- 1. Need to make more dynamic numerator calculation, meaning
% --    The alpha calculation depends on which condition we are running
% Bayes' rule
% p(xMxT|d)=p(d|xMxT)*p(xMxT)/marginal_p(d)
% where marginal_p(d)=p(d|xMxT)*p(xMxT) + p(d|yMyT)*(1-p(xMxT))
% makes the assumption that the operator can either have no
% knowledge or full knowledge
alpha = (pDxMxT_k * alpha)/(pDxMxT_k*alpha + pDyMyT_k*(1-alpha));
%alpha = (pDyMyT_k * alpha)/(pDyMyT_k*alpha + pDxMxT_k*(1-alpha));



% -- make sure that alpha is not too far into 0
% -- If alpha is zero, alpha will never recover above zero
if alpha < 0.01
    alpha = 0.01;
end



% else
%     % -- preset the alpha to be the alpha within the config file
%     alpha = param.alphaBegin;
% end

end