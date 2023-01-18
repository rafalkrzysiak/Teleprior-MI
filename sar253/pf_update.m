function [p, wts] = pf_update(p, wts, Z, Zr, glfn, bin_map, robot, param, maps)
% function [p wts] = pf_update(p, wts, Z, gh)
%
% p is d x N matrix with d dimensional state and N particles
% wts is 1 x N vector of weights for each particle
% Z is the measurement
% glfn is the measurement model

% resampling every time
for jj=1:size(p,2)
      % ^^ each choice has different performance
    if maps == "basic_map"
        wts(1,jj) = glfn(Z, Zr, p(:,jj), bin_map, robot, param); % for basic map
    elseif maps == "OmronLab"
        wts(1,jj)=glfn(Z, Zr, p(:,jj), bin_map(:,:,1), robot, param); % for complex/umap map
    else
        wts(1,jj)=glfn(Z, Zr, p(:,jj), bin_map(:,:,1)', robot, param); % for complex/umap map
    end
end

% -- make sure weights are never 0
% -- Line 411 in ParticleFilter.m in rvctoolbox
% if ~sum(wts)
%     wts=wts + 0.001;
% end
% 
% wts=wts./sum(wts);
% neff=1/sum(wts.^2);
% if (neff <=  size(p,2))    
%     p=p(:,resample(wts));
%     wts=ones(1,numel(wts))/numel(wts);
% end 


% % resampling selectively
% for jj=1:size(p,2)
%       % ^^ each choice has different performance
%     wts(jj)=wts(jj).*glfn(Z, Z, p(:,jj),bin_map); 
% end
% 
% wts=wts/sum(wts);
% neff=1/sum(wts.^2);
% if (neff <=  size(p,2)/2)
%     p=p(:,resample(wts));
%     wts=ones(1,numel(wts))/numel(wts);
% end 

% resampling selectively with *current weights* only
% for jj=1:size(p,2)
%       % ^^ each choice has different performance
%     wts(jj)=glfn(Z, Z, p(:,jj), bin_map); 
% end
% 
% wts=wts/sum(wts);
% neff=1/sum(wts.^2);
% if (neff <=  size(p,2)/2)
%     p=p(:,resample(wts));
%     wts=ones(1,numel(wts))/numel(wts);
% end 