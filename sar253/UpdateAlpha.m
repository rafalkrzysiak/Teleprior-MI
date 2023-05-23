function alpha = UpdateAlpha(TotalDist, d, pdist, x, k, param, Xh)
% -- this function will serve to update alpha given the distance
% -- traveled by the reference robot. Probabilities are calculated
% -- using experimental data captured by the Omron lab teleoperation

% -- We need at minimum 2 timesteps to calculate the distance
% -- traveled by reference robot. If the condition is not met,
% -- set alpha to the pre determined value param.alphaBegin
% -- which is defined in the ParamConfig.m file
if k > 1
    % -- calculate the distance traveled by the reference robot
    % -- within the environment
    d(k, 1) = sqrt(sum((Xh(6:7,k) - Xh(6:7,k-1)).^2));
    
    % -- only enter when we have at minimum of param.alphaBuffer steps available
    % -- param.alphaBuffer is entered in the ParamConfig.m file
    % -- the param.alphaBuffer value is arbitrarly chosen for right now, 
    % -- possible more optimized value will replace in future
    if k > param.alphaBuffer
        TotalDist(k, 1) = sum(d(k-param.alphaBuffer:k, 1));

        % -- calculate p(d|xMxT) and p(d|yMyT)
        % -- discussed during zoom meeting 5/23/2023
        pDxMxT = interp1(x, pdist(:,1), TotalDist(k,1));
        pDyMyT = interp1(x, pdist(:,4), TotalDist(k,1));

        % -- Temporary
        pxMxT = 0.5;

        % -- update alpha 
        % -- we can play around with this calculation 
        % -- TO DO: 
        % -- 1. Need to make more dynamic numerator calculation, meaning
        % --    The alpha calculation depends on which condition we are running
        alpha = (pDxMxT * pxMxT)/(pDxMxT + pDyMyT);

    else
        % -- preset the alpha to be the alpha within the config file
        % -- if the timestep is not > param.alphaBuffer
        alpha = param.alphaBegin;
    end
else
    % -- preset the alpha to be the alpha within the config file
    alpha = param.alphaBegin;
end

end