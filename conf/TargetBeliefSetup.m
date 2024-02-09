function p = TargetBeliefSetup(param, p, xs, w0)
% -- This function will serve as a method to set up the belief of where
% -- the human subjects may percceive the target location to be.
% -- During the no target conditions (xMxT, yMxT), the belief is to be set as a uniform
% -- distribution. While during the yes target conditions (xMyT, yMyT), the
% -- belief is to be a gaussian distribution. However, for the yMyT
% -- condition, the gaussian distribution is to be tighter/smaller than
% -- that of xMyT.

% -- set the belief the robot itself
p(1:3, :, 1) = [xs(1,1)+randn(1,param.N)*w0;
                xs(2,1)+randn(1,param.N)*w0;
                xs(3,1)+randn(1,param.N)*w0];

% belief can go through the obstacles but robot can't
if param.config == "xMxT"
    p(4:5, :, 1) = ones(2, param.N).* ...
                    [rand(1,param.N)*param.L(1);
                     rand(1,param.N)*param.L(2)];
% Gaussian but scaled to 
% likelihood does not prevent obstacles
elseif param.config == "xMyT"
    Knowledge_scale = 5; % -- 5 times less confident than "yMyT"
    p(4:5, :, 1) = [xs(4,1)+randn(1,param.N)*w0*Knowledge_scale;
                    xs(5,1)+randn(1,param.N)*w0*Knowledge_scale];
% belief cannot through the obstacles AND robot can't
% Likelihood prevents obstacles
elseif param.config == "yMxT"
    p(4:5, :, 1) = ones(2, param.N).* ...
                    [rand(1,param.N)*param.L(1);
                     rand(1,param.N)*param.L(2)];
% Gaussian
elseif param.config == "yMyT"
    p(4:5, :, 1) = [xs(4,1)+randn(1,param.N)*w0;
                    xs(5,1)+randn(1,param.N)*w0];

end

end