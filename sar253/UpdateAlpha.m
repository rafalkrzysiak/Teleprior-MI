function alpha = UpdateAlpha(TotalDist, d, pdist, x, k, param, Xh, alpha)
% -- this function will serve to update alpha given the distance
% -- traveled by the reference robot. Probabilities are calculated
% -- using experimental data captured by the Omron lab teleoperation

% alpha is the posterior which recursively gets updated every time step.

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
    if k > param.alphaBuffer
        TotalDist_k = sum(d(k-param.alphaBuffer:k, 1)); % d(k-tau:k,1)...

        % -- calculate p(d|xMxT) and p(d|yMyT)
        % -- discussed during zoom meeting 5/23/2023
        pDxMxT_k = interp1(x, pdist(:,1), TotalDist_k);
        pDyMyT_k = interp1(x, pdist(:,4), TotalDist_k);
        % -- update alpha 
        % -- we can play around with this calculation 
        % -- TO DO: 
        % -- 1. Need to make more dynamic numerator calculation, meaning
        % --    The alpha calculation depends on which condition we are running
        alpha = (pDxMxT_k * alpha)/(pDxMxT_k + pDyMyT_k);
        
    else
        % -- Temporary
        % this goes out of the if condition because we set it to 0.5 since
        % we cannot calculate it
        alpha = param.alphaBegin;

    end
    


    
% else
%     % -- preset the alpha to be the alpha within the config file
%     alpha = param.alphaBegin;
% end

end