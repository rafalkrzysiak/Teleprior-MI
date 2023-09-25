function infoseek2Db_multi_robot(param, img, saveIn, maps, ...
                                saveFrames, target_loc, exp_id, condition, ...
                                pdstr, xdstr)

% preset values when not running this script from another script
if nargin < 1
    % read the map from the folder
    img = imread('maps/basic_map.jpg');
    % img = imread('maps/u_map.jpg');
    % img = imread('maps/complex_map.jpg');
    img = imbinarize(img); % for matlab 2016 and up
    % img = rot90(rot90(img(:,:,1)));
    % thresh = graythresh(img); % use if using matlab 2016 or lower
    % img=im2bw(img,thresh); % use if using matlab 2016 or lower
    % figure(1); gcf; clf;
    % imagesc([0 20],[0 20],img); hold on;
    % grid on; grid minor;
    % set(gca,'ydir','normal');
    dt = 1; % time step
    T = 2000; % total simulation time
    r_visible = 3; % visible range of the robot
    eta = .35; % ######### Changing eta value 
    N = 100; % Number of particles
    nsim = 2; % number of simulations when debug mode isn't enabled
    closedloop = 1; % enable or disable opimization of mutual rb_information
    debug = 1; % enable or disable debug mode
    r_robot = .3; % -- the radius of each agent
    agents = 2; % -- number of agents in the simulation
    share_info = 1; % -- enable/disable sharing of rb_information between robots
    share_info_MI = 0;
    particle_subsample = 1; % -- flag to determine whether to subsample particles or not
    dn = 5; % -- the change in particles skipped (EX. with Time -> dt = 10)
    iter_num = 1;
    saveIn = strcat("data/combination", num2str(iter_num));
    alpha = 1;
    % -- the disturbance in the particles jusitification:
    % -- 1. the robot itself can successfully localize itself great
    % -- 2. the target has a small amount of jitter, it might move ever so slightly
    % -- 3. no robot knows the speed of the human robot, it is 50% of the max 
    % --    possible velocity speed of the creates platform
    w = diag([.01, .01, .001, .05, .05, .35, .35]); % -- disturbance
    
    % -- Robot and Target dynamics
    v=([0:.1:.4])*1; % -- possible velocity values for the robot (creates platform)
    om=([-.2:.1:.2])*1; % -- possible omega values for the robot (creates platform)
    
    omega0 = randn*.35;
    vel0 =.1+rand*.5;
    if debug
        figure(10); gcf; clf;
%         nsim=5; % number of simulations when debug mode is enabled
    %     rng(6);
    end
end

% -- capture the rb_information sharing flag parameter
og_share_info = param.share;

% -- set up the matrix that will show which robot can see which neighbor
% -- the robot cannot see itself!
info_SI = zeros(param.agents, param.agents);
info_MI = zeros(param.agents, param.agents);

% -- location for the simulation
location = strcat(saveIn, maps);

% -- use edge detection to get all of the edges of the environment
% -- get all coordinates of the edges to help with collisions
% -- row = y coordinate
% -- column = x coordinate
% -- scale the maps coordinates from pixel to meters!
bw = edge(img(:,:,1),'sobel');
[imh, imw]=size(img(:,:,1));
[row, col] = find(bw == 1);
fiducial = zeros(numel(row),2);
fiducial(:,2) = row*(param.L(2)/imh);
fiducial(:,1) = col*(param.L(1)/imw);

% -- measurement model | bearing only measurement     
hfun=@(X) wrapToPi(atan2((X(5,:)-X(2,:)), (X(4,:)-X(1,:)))-X(3,:));

% the likelihood function for pf_update
lfn = @(Z, Zr, p, bin_map, robot, param) ...
      lfn1(Z, Zr, p, hfun, bin_map, robot, param);
       
% -- the likelihood function for optimizing mutual rb_informtation
% -- around the human operated robot
lfnMI = @(Z, Zsup, p, robot, bin_map) ...
        lfn_MI(Z, Zsup, p, hfun, robot, bin_map, param);

% -- the optimize mutual rb_information function
O_MI = @(k, p, wts, bin_map, Z, robot, Zr, alpha) ...
       optimize_MI(k, p, wts, lfn, lfnMI, bin_map, Z, ...
                   robot, Zr, param, alpha); % pass alpha here


Num_target_found = zeros(param.nsim,1);
Which_robot = Num_target_found;
time = Num_target_found;
result = time;

% -- temporarily store the target location, this will be used for
% -- when we slightly move the location of the target
% tx = target_loc(1);
% ty = target_loc(2);

% -- Load the pdist.mat file created from the analysis portion of the 
% -- Human-subjects-experiment. We will be using this .mat file multiple
% -- times, best to pull it prior to running the simulations
% instead of calling this here, pass this directly from the wrapper script
% commenting this line below and passing Exp as argument -SB
% Exp = load('../src/pdist_tau=15s.mat');

% -- Pull list of 

% -- loop through the number of simulations per combination
for jj=1:param.nsim
    
    fprintf('Simulation: %d \n',jj) % -- display the simulation number
    sim_end = 0; % -- flag that states the simulation has not ended
    bin_map = img; % -- rename the img as bin_map
    rij = zeros(param.agents,1); % -- create matrix that holds the bearing measurement of target
    Rr = rij; % -- create matrix that holds the bearing measurement of human robot
    r_distance = ones(param.agents,param.agents)*20; % -- distance from autonomous robot to human robot
    
    % -- initialize the robots, target, particles and control inputs
    % -- before we set up the location of the target, add some small disturbance
    % -- so that the target is not in the same spot for every simulation
%     target_loc = [tx+randn, ty+randn];
    X0 = robot_position(maps, target_loc); % -- initialize the position and orientation of the robot(s) and target
    n=size(X0,1); % The state size
    n = n+2; % -- state size is 7 for us now (self, position of target, position of human)
    p=zeros(n,param.N,param.T,param.agents); % -- particles (n x N x T)
    wts=ones(1,param.N,1,param.agents); % -- particle filter weights
    Z = zeros(1,param.T,1,param.agents); % -- target bearing measurement
    Zr = Z; % -- human robot bearing measurement
    omega = Z; % -- omega values used every time step
    vel = omega; % -- velocity values used every time step
    I = zeros(5,param.T,1,param.agents); % -- holds rb_information regarding the mutual rb_info
    
    % -- holds all the optimized velocity and omega data
    vel_data = zeros(1,param.T,1,param.agents);
    omega_data = vel_data;

    % ^^ initial estimate of target location
    % is just uniformly distributed, so we have no idea where the target is
    w0=.1;
    
    % -- hold all simulated positions of the robots as well as the estimates
    Xh = zeros(n,param.T,1,param.agents);
    Xs = Xh;
    
    % -- initialize the particles, estimates and positions of the robots
    for r = 1:param.agents
        % -- initialize the start of the robot location particles
        % -- make sure the last 4 dim a copy for the autonomous robots only
        % -- should be a common estimate
        p(1:3,:,1,r)=[X0(1,1,1,r)+randn(1,param.N)*w0;
                      X0(2,1,1,r)+randn(1,param.N)*w0;
                      X0(3,1,1,r)+randn(1,param.N)*w0]; 
                
%         if r == 1 % human operated robot
%            p(4:7,:,1,r:param.agents) = ones(4, param.N, 1, param.agents).* ...
%                                        [rand(1,param.N)*param.L(1);
%                                         rand(1,param.N)*param.L(2);
%                                         rand(1,param.N)*param.L(1);
%                                         rand(1,param.N)*param.L(2)];
%         end

        % -- should assign each value separately -- SB
%         if r == 1 % human operated robot
           p(4:7,:,1,r) = ones(4, param.N, 1, 1).* ...
                                       [rand(1,param.N)*param.L(1);
                                        rand(1,param.N)*param.L(2);
                                        rand(1,param.N)*param.L(1);
                                        rand(1,param.N)*param.L(2)];
%         end
                
        % -- only for the human teleoperated robot, initialize the target
        % -- particle distribution to be placed as a gaussian distribution
        % -- at the lower corner of the usable domain with a large variance
        if r == 1 && param.bias
            % -- NEW*: 
            % -- set the experimental trajectory data captured with the 
            % -- Omron overhead tracking system for robot 1 only
            %p(1:3,:,1,r) = RobotExperimentDataSet();

           %if param.bias
               p(4:5,:,1,r) = [(param.L(2)*sqrt(2)/2)*cosd(45-30)+randn(1,param.N)*(.15*param.L(2));
                               (param.L(2)*sqrt(2)/2)*sind(45-30)+randn(1,param.N)*(.15*param.L(2))]; 
           %end
        end
        
        Xh(:,1,1,r)=mean(p(:,:,1,r),2);
        if r == 1
            [Xs(1:3,:,1,r), ~, ~]=RobotExperimentDataSet(exp_id, condition);
            Xs(4:5,1,1,r)=X0(4:5,:,1,r);
        else
            Xs(1:5,1,1,r)=X0(:,:,1,r);
        end
%         rij(r,1) = 20;
%         Rr(r,1) = 20;
        r_distance(r,r) = 0;
    end
    
    
    % beginning and ending setps for the filter
    k0 = 1;
%     kF = param.T;

    % -- NEW:
    % -- get the total time of the robot Human Subject Trial
    kF = size(Xs(1:3,:,1,1),2);

    % -- create distance vector shared for all *autonomous* robots
    % -- this calculation will be used to influence the change in alpha
    % -- For now, lets leave the dimension to be equal to total robots
    % -- running in simulation, which will leave robot_1 with all zeros
    d = zeros(kF, 1);
    TotalDist = zeros(kF, 1, 2);
    alpha = d;
    pDxMxT = d;
    pDyMyT = d;

    % -- set the alpha to be what it was in the beginning 
    alpha(1,1) = param.alphaBegin;
    
    % -- begin simulating the condition
    for k = k0:kF
        
%         fprintf('Sim: %d, Time step: %d\n',jj,k); % -- display the time step number
        
        % -- loop for every robot in the environment
        for robot = 1:param.agents
            
            % -- check the distances between the robots
            % -- if the parameter for sharing rb_information was enabled
            % -- we want the robots to share rb_information between each other
            % -- when they see one of their neighbors
            for rob = 2:param.agents
                for neighbor = 2:param.agents
                    % -- make sure the robot doesn't look at itself
                    if neighbor ~= rob
                        r_distance(neighbor,rob) = sqrt((Xs(1,k,1,neighbor)-Xs(1,k,1,rob))^2+...
                                                          (Xs(2,k,1,neighbor)-Xs(2,k,1,rob))^2);

                        % -- if one robot sees another robot, enable sharing
                        % -- rb_information, if it doesn't see it, no sharing.
                        % -- as long as it does not interfere with the preset parameters
                        if r_distance(neighbor,rob) < 20*param.r_visible(rob)
                            share_info = 1;
                            info_SI(neighbor,rob) = 1;
                        else
                            share_info = 0;
                            info_SI(neighbor,rob) = 0;
                        end
                    end 
                end
            end
            
            % -- look at the matrix that shows which robot sees which
            % -- used to determine sharing of rb_information!
            [Neighbors, ~] = find(info_SI(:,robot) == 1);
            
            kappa = 5;
            rij0 = 1.5;
            phuman = 1-(1-1)./(1+exp(-kappa*(rij(robot,1)-rij0)));
            probot = 1-(1-1)./(1+exp(-kappa*(Rr(robot,1)-rij0)));
            
            % -- check the distance between robot and simulated target
            rij(robot,1) = sqrt((Xs(4,k,1,robot)-Xs(1,k,1,robot))^2+...
                                (Xs(5,k,1,robot)-Xs(2,k,1,robot))^2);
            
            % -- check the distance between autonomous robot and human operated robot       
            if robot ~= 1
                Rr(robot,1) = sqrt((Xs(1,k,1,1)-Xs(1,k,1,robot))^2+...
                                   (Xs(2,k,1,1)-Xs(2,k,1,robot))^2);
            end
            
            % -- if the target is within range
            if rij(robot,1) < param.r_visible(robot) && phuman > rand % -- only gets a measurement when within range
                % -- noiseless measurement
                Z(:,k,1,robot)=get_measurements(Xs(1:5,k,1,robot));
                
                % -- if the measurement taken by the robot is within the field of view
                if abs(Z(:,k,1,robot)) < param.r_FOV(robot)/2
                    Z(:,k,1,robot) = Z(:,k,1,robot);
                    if rij(robot,1) < .75*param.r_visible(robot) % -- must be closer to get a diffinitive answer
                        Num_target_found(jj) = 1; % -- save a yes to the sim number where target was found
                        time(jj) = k; % -- mark the time the target was found
                        result(jj) = 1; % -- the result was that the target was found
                        disp('The target was found!');
                        Which_robot(jj) = robot; % -- this will save the robot that found the target first
                        
                        % -- once the target was found, for every robot,
                        % -- fill in the rest of the time possible to the
                        % -- final position the robot is in when the target
                        % -- was found
                        for rob = 1:param.agents
                            Xs(:,k+1:kF,1,rob) = Xs(:,k,1,rob).*ones(7,kF-k);
                            Xh(:,k:kF,1,rob) = Xh(:,k-1,1,rob).*ones(7,kF-(k-1));
                        end
                        
                        sim_end = 1; % -- signal that the simulation is ended
                        break
                    end
                else
                    % -- if the measurement taken is not within the field
                    % -- of view, then state that the measurement is 0
                    Z(:,k,1,robot) = 0;
                end
            else
                Z(:,k,1,robot) = 0; % -- target not seen, measurement = 0
                time(jj) = k; % -- continue counting the time
                result(jj) = 0; % -- not found the target
                Num_target_found(jj) = 0; % -- not found the target
            end
            
            % -- for all autonomous robots, if the human operated robot
            % -- is within range
            if robot ~= 1 && Rr(robot,1) < 100*param.r_visible(robot) && probot > rand
                Zr(:,k,1,robot) = get_measurements([Xs(1:3,k,1,robot);Xs(1:2,k,1,1)]) + randn*param.eta(1);
            else
                Zr(:,k,1,robot) = 0; % -- if the robot doesn't see another robot, no measurement
            end
            
            % -- if the robot sees its neighbor, create an array that
            % -- stores the robot number it sees, the robot cannot
            % -- see itself and cannot communicate with the human robot
%             if robot ~= 1 && param.share
%                 param.Neighbors(:,robot) = [robot;Neighbors];
%                 param.Neighbors(:,robot) = sort(Neighbors);
%             end
        end
        
        % -- we want to share the weights of the particles of the autonomous 
        % -- robots only, sharing the rb_information of both the target and human state
        if param.share
            wts_share = ones(1, param.N, 1, param.agents);
            for robot = 1:param.agents

                % -- Eq 3.5 & 3.6 from Thesis document!
                % -- w^i_{k,q} = prod_{l,N_i}{p(Z^{l,\theta}_k,Z^{l,H}_k|X^l_k)}
                % -- using the same function as the pf_update
%                 [~, wts_share(:,:,1,robot)] = pf_update([p(1:3,:,k,param.Neighbors(robot-1,robot));p(4:7,:,k,robot)], wts(:,:,1,robot), ...
%                                               Z(:,k,1,param.Neighbors(robot-1,robot)), Zr(:,k,1,param.Neighbors(robot-1,robot)), ...
%                                               lfn, bin_map, param.Neighbors(robot-1,robot), param, maps);
                [~, wts_share(:,:,1,robot)] = pf_update(p(:,:,k,robot), wts(:,:,1,robot), ...
                                                        Z(:,k,1,robot), Zr(:,k,1,robot), ...
                                                        lfn, bin_map, robot, param, maps);
                % -- only for the human robot, we want to keep the particles
                if robot == 1
                    wts(:,:,:,robot) = wts_share(:,:,1,robot);
                end
            end
        end
        
        % -- take a product of the weights that were calculated 
        % -- (Eq 3.21 from Thesis) prod function
        for robot = 2:param.agents
           wts(:,:,:,robot) = prod(wts_share(:,:,:,2:param.agents),4);
        end
        
        % -- After running the robots to get their measurements of the 
        % -- target and human teleoperated robot, update the particles
        for robot = 2:param.agents
            
            % -- update the particle locations and their weights
%             [p(:,:,k,robot), wts(:,:,1,robot)] = pf_update(p(:,:,k,robot), wts(:,:,1,robot), Z(:,k,1,robot), Zr(:,k,1,robot), lfn, bin_map, robot, param, maps);
%             wts(:,:,1,robot) = wts(:,:,1,robot).*wts_share(:,:,1,robot);

            % -- we want to resample the particles with the shared weights caluclated
            if ~sum(wts(:,:,:,robot))
                wts(:,:,:,robot)=wts(:,:,:,robot) + 0.001;
            end

            wts(:,:,:,robot)=wts(:,:,:,robot)./sum(wts(:,:,:,robot));
            neff=1/sum(wts(:,:,:,robot).^2);
            if (neff <= size(p,2))    
                p(:,:,k,robot)=p(:,resample(wts(:,:,:,robot)),k,robot);
                wts(:,:,:,robot)=ones(1,numel(wts(:,:,:,robot)))/numel(wts(:,:,:,robot));
            end 
            
            % this function pulls out the estimate from the distribution
            % ^^ the flag can be 1,2, or 3 and can give different estimates
            flag=1;
            Xh(:,k,1,robot) = postest(p(:,:,k,robot), wts(:,:,1,robot), flag);
            P(:,:,k,robot) = cov(p(:,:,k,robot)');
            
            % -- check if any of the robots left the domain
            if Xs(1,k,1,robot) < 0 ||  Xs(1,k,1,robot) > param.L(1)  || Xs(2,k,1,robot) < 0 || Xs(2,k,1,robot) > param.L(2)
                time(jj) = kF; % -- set the final time to the max time possible
                result(jj) = 2; % -- Enable specified flag that solely represents that the robot left the domain
                
                % -- If a robot has left the domain, fill in the rest of
                % -- the possible time with the current positions of the robot
                for rob = 2:param.agents
                    Xs(:,k+1:kF,1,rob) = Xs(:,k,1,rob).*ones(7,kF-k);
                    Xh(:,k:kF,1,rob) = Xh(:,k-1,1,rob).*ones(7,kF-(k-1));
                end
                
                % -- signal the end of the simulation
                sim_end = 1;
                break
            end
                   
            if param.closedloop
                % calculate alpha here and pass to O_MI
                % algorithm on how to calculate alpha (put inside a
                % function that admits Xs and timestep k)
                % extract last 15 seconds of robot data from Xs
                    % calculate d(k)=the distance moved (this can be done by
                    % simply adding the difference between successive
                    % locations)
                    % load pdist.mat which has the probability
                    % distributions for distance
                    % later we'll expand to include everything
                    %
                    % calculate the p(d(k)|xMxT), p(d(k)|yMyT) 
                    % next calculate p(xMxT|d(k))
                    % 
                    % since alpha in our setup means independence, we can
                    % set alpha=p(yMyT|d(k)) or
                    % set alpha=p(xMxT|d(k))
                % -- make sure that we have 2 times ran before we try and
                % -- calculate the distance that the reference robot has moved
                if k > 2
%                     d(k, 1) = sqrt(sum((Xh(6:7,k,1,robot) - Xh(6:7,k-1,1,robot)).^2));
                    d(k, 1) = sqrt(sum((Xs(1:2,k,1,1) - Xs(1:2,k-1,1,1)).^2));
                else
                    d(k, 1) = 0;
                end
                
                % -- begin alpha caluclation based on distance traveled 
                % -- by reference robot within the environment
                % feature is distance but could be anything e.g. turn rate,
                % fraction of time spent staying in place, which is why we
                % update the function to work on features instead of
                % distance
                if k > 2*param.tau+3
                    feature_k = sum(d(k-2*param.tau:k, 1));

                    [alpha(k+1,1), pDxMxT(k+1,1), pDyMyT(k+1,1)] = ...
                        UpdateAlpha(feature_k, pdstr, xdstr, alpha(k,1));
                else
                    % move this out of UpdateAlpha
                    alpha(k+1,1)=param.alphaBegin;
                    pDxMxT(k+1,1)=0.00001;
                    pDyMyT(k+1,1)=0.00001;
                end
                % -- set alpha to be constant value
                %alpha(k+1,1) = 0.0;
                % maximize mutual rb_information
    %             [omega,vel]=optimize_MI(k, p, v, dt, N, wts, w, eta, hfun, om, r_visible);
    %             [omega,vel]=optimize_MI(k, p, v, dt, N, wts, w, om, lfn);
                 [omega(1,k,1,robot),vel(1,k,1,robot),I(:,k,1,robot)] = ...
                     O_MI(k, p, wts, bin_map, Z(:,k,1,:), robot, Zr(:,k,1,:), alpha(k+1,1));
            else
                % 1 or -1 with equal probability
                omega(1,k,1,robot)=0.5*rand-0.25; %param.omega0+0.1*randn; 

                vel(1,k,1,robot)=param.vel0;
            end

            % -- get the range and bearing rb_information for every fiducial
            % -- for every robot in the simulation
%             for marker = 1:size(fiducial,1)
%                rb_info1(marker,1) = sqrt((fiducial(marker,1)-Xs(1,k,1,robot))^2 ...
%                                     +(fiducial(marker,2)-Xs(2,k,1,robot))^2); 
%                                 
%                rb_info1(marker,2) = wrapToPi(atan2((fiducial(marker,2)-Xs(2,k,1,robot)), ...
%                                        (fiducial(marker,1)-Xs(1,k,1,robot))) - Xs(3,k,1,robot));
%             end
            
            % speeding up -- SB
            rb_info(:,1)=sqrt((fiducial(:,1)-Xs(1,k,1,robot)).^2+(fiducial(:,2)-Xs(2,k,1,robot)).^2);
            rb_info(:,2)=wrapToPi(atan2(fiducial(:,2)-Xs(2,k,1,robot), fiducial(:,1)-Xs(1,k,1,robot)) - ...
                                    Xs(3,k,1,robot));

            % -- look at the points that are only within the collision FOV
            % -- and visible range of the robot. Then find the smallest 
            % -- value in the range
%             pts = find(rb_info(:,2) < pi/6 & rb_info(:,2) > -pi/6 & rb_info(:,1) < param.r_visible(robot));
             % collision should be closer than visible range -- SB
            pts = find(rb_info(:,2) < pi/6 & rb_info(:,2) > -pi/6 & rb_info(:,1) < param.r_visible(robot));
            
            % -- If the range of the closest point of the wall is within 
            % -- 2/3 of the visible range of the robot, begin collision 
            % -- avoidance protocol
            if size(pts,1) > 0 && robot ~= 1
                pts_interest = rb_info(pts,:);
                [~, idx_pt] = min(pts_interest(:,1));
%                 b_pt = find(pts_interest(:,1) == r_pt);
                pt_bearing = pts_interest(idx_pt,2);
                if param.debug
                    plot(fiducial(pts,1), fiducial(pts,2), 'rx');
                end
                % update the collision avoidance to be parallel to the wall
                omega(1,k,1,robot) = -param.kc*(sign(pt_bearing)*pi/2-pt_bearing);
                vel(1,k,1,robot) = vel(1,k,1,robot)/10; % SB updated
            end
            
            % -- collision avoidance of autonomous robots with human robot
            if Rr(robot,1) < 1*param.r_visible(robot) && robot ~= 1 && abs(Zr(:,k,1,robot)) < pi/4 && robot ~= 1
               omega(1,k,1,robot) = -param.kc*(sign(Zr(:,k,1,robot))*pi/2 - Zr(:,k,1,robot)); 
%                omega(1,k,1,robot) = -param.kc*Zr(:,k,1,robot); 
               vel(1,k,1,robot) = vel(1,k,1,robot)/10;
            end

            % -- simulate the motion of the robot with the determined
            % -- omega and velocity values from optimize_MI function
            if robot ~= 1
                Xs(:,k+1,1,robot)=rt2d(Xs(:,k,1,robot),vel(1,k,1,robot), omega(1,k,1,robot), param.dt);
            else
                continue
            end
            
            % -- predict
            mmdl1=@(x) rt2d(x, vel(1,k,1,robot), omega(1,k,1,robot), param.dt);
            p(:,:,k+1,robot) = pf_predict(p(:,:,k,robot), mmdl1, diag(param.w), robot);

        end
        
        % -- whether or not to show the plots during the simulation
        if param.debug
            debug_plot(Xs(:,k,1,:), Xh(:,k,1,:), k, p(:,:,k,:), param, ...
                       bin_map, fiducial, jj, X0, saveFrames, target_loc, ...
                       I(:,:,1,:), alpha, TotalDist, pDxMxT, pDyMyT)
        end
        
        % -- end the msimulation if the target was found
        if Num_target_found(jj) == 1 || sim_end == 1
           break 
        end
   
    end
    
    % -- after the simulation ended, save all the data into a single struct
    % -- doing this will hold alot of rb_information in a single variable
    simdata(jj).param = param; % -- save all the parameters used in the simulation
    simdata(jj).Xh = Xh; % -- estimate robot position
    simdata(jj).Xs = Xs; % -- simulated robot position
    %simdata(jj).p = p; % -- particle state of all robots
    %simdata(jj).P = P; % -- covariance
    simdata(jj).vel = vel; % -- velocity values throughout the whole simulation
    simdata(jj).omega = omega; % -- omega values throughout the whole simulation
    simdata(jj).tloc = target_loc; % -- target location for the simulation
    simdata(jj).time = time(jj); % -- time it took to finish the simulation
    simdata(jj).success = result(jj); % -- flags that state which sim found the target
    simdata(jj).Z = Z; % -- measurement of the target w.r.t robot
    simdata(jj).Zr = Zr; % -- measurement of the human robot w.r.t autonomous robot
    simdata(jj).I = I; % -- save the mutual rb_information values for target/human robot
    simdata(jj).Which_robot = Which_robot; % -- save the robot number that found the target
end

% plot the results of the simulation (trajectories)
% traj_plot(simdata, X0, N, nsim, eta, Num_target_found, time, dt, img, agents, ...
%           location, og_share_info, og_share_info_MI, target_loc, alpha)

% -- save all rb_information into the newly created directory
save(strcat(location, sprintf('_p=%d_nsim=%d_agents=%d.mat', param.N, param.nsim, param.agents)), ...
     'simdata', 'img', '-v7.3');

% save(sprintf('data/umap_particles=%d_nsim=%d_agents=%d.mat', N, nsim, agents), ...
%              'P', 'p', 'simdata', 'dt', 'T', 'r_visible', 'N', 'eta', 'img');
         
function wts_share = sharing_info(p, agents, Z, Zr, hfun, robot, row1)

% -- get the number of particles
np = size(p(:,:,1,robot),2);

% -- get the number of agents that see each other
n_agents = size(row1,1);

% -- create a matrix to hold weights for every particle of the agent
% -- that is compared to every particle of its neighbor
weights = zeros(np,np);
w = zeros(1,np);
wts_share = ones(1,np);

% -- we now send the target position particles from one robot to the other
% -- only the autonomous robot can share particles, the human robot cannot
% -- inmprove its target particle estimate by using the other robots
% -- particles. Other autonomous robots are allowed to use the human
% -- controlled robot's target estimate particles
if robot ~= 1
    for agent = robot % -- starting at agent 1, eventually looping through every robot
       for neighbor = 1:n_agents % -- looping through every neighbor that the agents sees
          if row1(neighbor) ~= agent % -- if robot is not itself, begin sharing rb_information
             for agent_p = 1:np % -- loop through every particle of the agent
                 for neighbor_p = 1:np % -- loop through every particle of the neighbor
                     particles = [p(1:3,neighbor_p,1,row1(neighbor)); p(4:5,agent_p,1,agent)]; % -- use the agent's own target estimate and neighbors robot estimate
                     particle = [p(1:3,neighbor_p,1,row1(neighbor)); p(6:7,agent_p,1,agent)];
                     Zh = hfun(particles); % -- get a measurement from the combined particle set
                     ZH = hfun(particle);
                     wts_robot = normpdf(ZH, Zr, pi/4);
                     wts_target = normpdf(Zh, Z, pi/4);
                     weights(agent_p, neighbor_p) = wts_robot.*wts_target;
%                      weights(agent_p, neighbor_p) = 1-((1-normpdf(Zh, Z, pi/4)).*(1-normpdf(ZH, Zr, pi/4))); % -- get the weights of the measurement compared to the actual measurement
                 end
             end
             w = sum(weights,2); % -- sum the columns of the NxN weights matrix to get a 1xN vector
             if numel(w) > 1
                 w_norm = w/sum(w); % -- normalize the new weights of the particles
                 wts_share = w_norm.*wts_share; % -- multiply the new weights with the current weights of the neighbors particles
             else
                 wts_share = w.*wts_share; % -- multiply the new weights with the current weights of the neighbors particles
             end
          end
       end
    end
end

% pass alpha to optimize_MI
function [omega, vel, I]=optimize_MI(k, p, wts, lfn, lfnMI, bin_map, Z, ...
                                     robot, Zr, param, alpha)

% -- Store the mutual rb_information values
MIvals=zeros(numel(param.om),numel(param.v));
MI_t = MIvals;
MI_r = MI_t;
MI_T = MI_t;
MI_R = MI_r;

% -- get the number of particles
np = size(p,2);

% -- Create the support for Z 
% -- get the size of the z support
% -- create variable to store values of the likelihood function
Zsup = -pi:pi/4:pi;
M=size(Zsup,2); 

for oo=1:numel(param.om)
    for dd=1:numel(param.v)
        
        % -- motion model for the robot in 2D
        mmdl1=@(x) rt2d(x, param.v(dd), param.om(oo), param.dt);
        
        % -- take the particles and move them in the future with a varying
        % -- combination of omega and velocity
        p(:,:,k+1,robot) = pf_predict(p(:,:,k,robot), mmdl1, diag(param.w), robot);

        % ** if lfn(Zsup(:,ll),Z(:,:,1,robot),p(:,:,k+1,robot),bin_map) without the Z assignment
        % is called 3 times in the code, then better just call it once in a
        % loop over ll and then assign as part of pZ and pZT, pZcT
        pZ = zeros(1,M);
        pZcT = zeros(np,M);
        pZT = zeros(np,M);
        
        % -- get the conditional probability values 
        for ll = 1:M
            pZcT(:,ll) = lfnMI(Z(:,:,1,robot), Zsup(:,ll), p(1:5,:,k+1,robot), robot, bin_map);
            
            % -- we want to avoid zero probabilities
            if ~sum(pZcT(:,ll))
                pZcT(:,ll) = .001;
            end
            
            % -- we want to avoid zero probabilities
            part_0 = find(pZcT(:,ll) == 0);
            
            if part_0
                pZcT(part_0,ll) = .00001;
            end
        end
        
        % -- for conditional probabilities the sum p(Z_1|T_q) + p(Z_2|T_q) + ...
        % -- should sum up to 1
        pZcT=pZcT./(sum(pZcT,2)*ones(1,M));
        
        for ll=1:M
            % p(Z_ll)=int_kk p(T_kk)*p(Z_ll|T_kk)
            % -- Equation 3.9 (inside the square parentheses)
            pZ(ll) = sum(wts(:,:,1,robot).*pZcT(:,ll)'); % -- joint probability
            pZT(:,ll)=wts(:,:,1,robot).*pZcT(:,ll)';
        end
        
        % -- entropy of z (Measurement)
        % -- Equation 3.9 (From Thesis document!)
        % -- after getting pZ (The joint probability), we must remove the 
        % -- zeros, because log(0)=-inf
        pZ = pZ(pZ~=0);
        
        % -- the probabilities must sum up to 0
        if sum(pZ)
            pZ=pZ/sum(pZ);
        end
        
        % probabilities must sum to 1
        if sum(pZT)
            pZT=pZT/sum(pZT(:));
        end
        
        % -- entropy calculation
        Hz = -sum(pZ.*log(pZ));
        
        % -- Eq 3.10 from thesis
        % -- entropy calculation for the target from robots perspective
        % -- after getting pZT (the joint probability) and 
        % -- pZcT (the conditional probability), we must remove the 
        % -- 0 values from both to omit an inf value, log(0)=-inf
        % -- must get the positions where pZct and pZT ~=0 to maintain dimensions
        pZcT1 = pZcT(pZcT~=0 & pZT ~=0);
        pZT1 = pZT(pZT~=0 & pZcT~=0);
        Hzt = -sum(pZT1(:).*log(pZcT1(:)));
        
        Hzr = 0;
        Hztr = 0;
        
        % -- Social rb_information for the MI function ^^^
        if robot ~= 1
            
            % ** if lfn(Zsup(:,ll),Z(:,:,1,robot),p(:,:,k+1,robot),bin_map) without the Z assignment
            % is called 3 times in the code, then better just call it once in a
            % loop over ll and then assign as part of pZ and pZT, pZcT
            pZr = zeros(1,M);
            pZcTr = zeros(np,M);
            pZTr = zeros(np,M);
            
            for ll = 1:M
                % -- get the conditional probability values of the robot
                pZcTr(:,ll) = lfnMI(Z(:,:,1,robot), Zsup(:,ll), [p(1:3,:,k+1,robot);p(6:7,:,k+1,robot)], robot, bin_map);
                
                % -- we want to avoid zero probabilities
                part_0 = find(pZcTr(:,ll) == 0);
                
                if part_0
                    pZcTr(part_0,ll) = .00001;
                end
                % -- we want to avoid zero probabilities
                if ~sum(pZcTr(:,ll))
                    pZcTr(:,ll) = .001;
                end
            end
            
            % -- for conditional probabilities the sum p(Z_1|T_q) + p(Z_2|T_q) + ...
            % -- should sum up to 1
            pZcTr=pZcTr./(sum(pZcTr,2)*ones(1,M));

            for ll=1:M
                % p(Z_ll)=int_kk p(T_kk)*p(Z_ll|T_kk)
                % -- Equation 3.9 (inside the square parentheses)
                pZr(ll) = sum(wts(:,:,1,robot).*pZcTr(:,ll)'); % -- joint probability
                pZTr(:,ll) = wts(:,:,1,robot).*pZcTr(:,ll)';
            end

            % -- entropy of z (Measurement)
            % -- Equation 3.9 (From Thesis document!)
            % -- after getting pZ (The joint probability), we must remove the 
            % -- zeros, because log(0)=-inf
            
            pZr = pZr(pZr~=0);
            % -- the probabilities must sum up to 0
            if sum(pZr)
                pZr=pZr/sum(pZr);
            end

            % probabilities must sum to 1
            if sum(pZTr)
                pZTr=pZTr/sum(pZTr(:));
            end
            
            % -- entropy calculation
            Hzr = -sum(pZr.*log(pZr));

            % -- Eq 3.10 from thesis
            % -- entropy calculation for the target from robots perspective
            % -- after getting pZT (the joint probability) and 
            % -- pZcT (the conditional probability), we must remove the 
            % -- 0 values from both to omit an inf value, log(0)=-inf
            % -- must get the positions where pZct and pZT ~=0 to maintain dimensions
            pZcT1r = pZcTr(pZcTr~=0 & pZTr ~=0);
            pZT1r = pZTr(pZcTr~=0 & pZTr ~=0);
            Hztr = -sum(pZT1r(:).*log(pZcT1r(:)));
        end
        
        if Hztr - Hzr > 0.01 || isnan(Hz) || isnan(Hzt) || Hzt - Hz > 0.01 || isnan(Hzr) || isnan(Hztr)
            keyboard
            error('Mutual rb_information cannot be less than 0');
        end
        
        % -- calculate the mutual in formation for each robot
        % -- only use the alpha value for the autonomous robots
        if param.norm
            if robot == 1
                MI_T(oo,dd) = (Hz-Hzt);
                MI_t(oo,dd) = (Hz-Hzt)/Hz;
            else
                MI_t(oo,dd) = (Hz-Hzt)/Hz;
                MI_r(oo,dd) = (Hzr-Hztr)/Hzr;
                MI_T(oo,dd) = (Hz-Hzt);
                MI_R(oo,dd) = (Hzr-Hztr);
            end
        else
            if robot == 1
                MI_T(oo,dd) = (Hz-Hzt);
                MI_t(oo,dd) = (Hz-Hzt);
            else
                MI_t(oo,dd) = (Hz-Hzt);
                MI_r(oo,dd) = (Hzr-Hztr);
                MI_T(oo,dd) = (Hz-Hzt);
                MI_R(oo,dd) = (Hzr-Hztr);
            end
        end
        
    end
end

% -- we want to normalize the mutual rb_information for the individual arrays
% -- that represent the human robot and the target with themselves
% [tval, trow] = max(MI_t);
% [~, tcol] = max(tval);
% MI_t = MI_t./MI_t(trow(tcol),tcol);
% 
% [Rval, Rrow] = max(MI_r);
% [~, Rcol] = max(Rval);
% MI_r = MI_r./MI_r(Rrow(Rcol),Rcol);

% -- get the max value of mutual rb_information
% -- get the velocity and omega values that correspond to the max value for MI
if robot ~= 1
    %MIvals = param.alpha.*MI_t + (1-param.alpha).*MI_r;
    MIvals = alpha*MI_t + (1-alpha)*MI_r;
else
    MIvals = MI_t;
end

[val, row] = max(MIvals);
[~, col] = max(val);
row = row(col);

% -- save the chosen velocity and omega values
omega = param.om(row);
vel = param.v(col);

% -- save the MI vals computed for the target, robot and weighted combined
I(1) = MIvals(row,col); % -- over all MI, weighted with alpha
I(2) = MI_t(row,col); % -- MI normalized of the target
I(3) = MI_r(row,col); % -- MI normalized of the human robot
I(4) = MI_T(row,col); % -- MI not normalized of the target
I(5) = MI_R(row,col); % -- MI not normalized of the human robot

function omega=optimize_Htz(k, p, v, dt, N, wts, w, eta, hfun, lfn, omega_range)

Htz=zeros(1, numel(omega_range));

pTkcZk=p(:,:,k);
mu_pTkcZk=mean(pTkcZk,2);
for oo=1:numel(omega_range)

    
    mmdl1=@(x) rt1d(x, v, omega_range(oo), dt);
    p_ = pf_predict(pTkcZk, mmdl1, diag(w));

    % Support for Z should be something that
    Zsup=[1 -1];
%     Z1=mean(hfun(p(:,:,k)));
%     Zsup=linspace(Z1-10,Z1+10,10);
%     Zsup=hfun(p(:,:,k));
%     Zsup=Zsup(:,1:2:end);
    M=size(Zsup,2);

    pTk1cZk1=zeros(2, numel(wts),M);
    for ll=1:M
        pTk1cZk1(:,:,ll)=pf_update(p_, wts, Zsup(:,ll),lfn);
    end
    % entropy of z
    pTk1cZZk1=squeeze(pTk1cZk1(2,:,:));
    Htz(oo)=ent(pTk1cZZk1(:)', 10, ...
        [mu_pTkcZk(2)-5, mu_pTkcZk(2)+5], 'x');
%     Htz(oo)=ent(pTk1cZZk1(:)', 20, ...
%         [1, 15], 'x');
%     Htz(oo)=ent_kde(pTk1cZZk1(:)', .5, 5:1:15);

end

[~, idx]=min(Htz);
omega=omega_range(idx);

% -- likelihood function for pf_update only!
% -- this function combines both looking for the target as well
% -- as looking for the human operated robot
function wts=lfn1(Z, Zr, p, hfun, bin_map, robot, param)

% -- number of particles 
np = size(p,2);

% -- non-linear measurement model
Zh = hfun(p);

% -- initialize a set of particle weights to one
wts = ones(1,np);
wwts = wts;

% -- get the domain size to get the upper boundaries of the domain
X_limit = size(bin_map,2);
Y_limit = size(bin_map,1);

% -- for every particle, check if the estimate is within the domain
% -- if the particle position is within the domain, let the initialized
% -- weight for that particle = 1, if it is not in the domain, the 
% -- initialized particle weight = 0.
for kk = 1:np
    % -- covert particle from meters to pixels for the map
    X_target = round(p(4,kk).*(X_limit/param.L(1)));
    Y_target = round(p(5,kk).*(Y_limit/param.L(2)));
    X_human = round(p(6,kk).*(X_limit/param.L(1)));
    Y_human = round(p(7,kk).*(Y_limit/param.L(2)));

    % -- check if the particle is within the outer boundary of the map
    if (X_target > 0 && Y_target > 0 && X_target < X_limit && Y_target < Y_limit ...
        && X_human > 0 && Y_human > 0 && X_human < X_limit && Y_human < Y_limit)

        % -- check if the particle representing the target position 
        % -- is within a usable portion of the map
        if bin_map(Y_target,X_target)
            wts(kk) = 1/param.N;
        else
            wts(kk) = 0;
        end
        
        if robot ~= 1
            % -- for all robots that are autonomous, check if the particle
            % -- representing the human operated robot is within the usable domain
            % -- and if it corresponds with a usable target position particle
            if bin_map(Y_human,X_human) == 1 && wts(kk) ~= 0
                wwts(kk) = 1/param.N;
                wts(kk) = 1/param.N;
            else
                wwts(kk) = 0;
                wts(kk) = 0;
            end
        end

    else
        % -- if outside the boundary, wts = 0
        wts(kk) = 0; 
        wwts(kk) = 0;
    end
end

if Z(1) % -- if the robot sees the target
    if wts % -- only update the weights if the particle weight is nonzero
        for ii=1:size(param.eta,1)
            wts=wts.*normpdf(Zh(ii), Z(ii), param.eta(robot));
        end
    end
else % -- if the robot does not see the target
    dist_from_robot = sqrt(sum((p(4:5,:)-p(1:2,:)).^2));
%     idx = (dist_from_robot < (r_visible*.75));
    % -- if the robot does not see the target, we reset the weights for 90%  of
    % -- the particles
    idx = (dist_from_robot < param.r_visible(robot) && abs(Zh) < param.r_FOV(robot)/2);
%     if rand < .9
        wts(idx) = 0;
%     end
end

if robot ~= 1
    % -- get a measurement of the human robot from the autonomous robot
    % -- using the autonomous robot's particle estimate
    ZH = hfun([p(1:3,:); p(6:7,:)]);
    
    if Zr(1) % -- if the autonoumous robot sees the human controlled robot
        if wwts % -- only update the weights if the particle weight is nonzero
            for ii=1:size(param.eta,1)
                wwts=wwts.*normpdf(ZH(ii), Zr(ii), param.eta(robot));
            end
        end
    else
        dist_from_human = sqrt(sum((p(6:7,:)-p(1:2,:)).^2));
        idy = (dist_from_human < param.r_visible(robot) && abs(ZH) < param.r_FOV(robot)/2);
        
%         if rand < .9
            wwts(idy) = 0;
%         end
    end
    % -- for every robot except the human operated one, update the weights
    % -- so that the weights from the target particles (wts) are combined 
    % -- with the weights of the human operated robot (wwts)
    % -- wts = 1-(1-p(Xi|Theta))*(1-p(Xi|Xh))
%     wts = wts + wwts - wts.*wwts;
    % -- p(Xi|Theta)*p(Xi|Xh)
    wts = wts.*wwts;
end

% -- whether or not to share rb_information between robots
% if share_info && robot ~= 1 && sum(row1) > 0
%     wts_share = sharing_info(p, agents, Z, Zr, hfun, robot, row1);
%     wts = wts.*wts_share;
% %     wts = wts + wts_share - wts.*wts_share;
% end

function wts=lfn_MI(Z, Zsup, p, hfun, robot, bin_map, param)

% -- number of particles 
np = size(p,2);

% -- initialize a set of particle weights to one
wts = ones(1,np);

% -- get the map dimensions
X_limit = size(bin_map,1);
Y_limit = size(bin_map,2);

for kk = 1:np
    % -- covert particle from meters to pixels for the map
    Xpos = round(p(4,kk).*(X_limit/param.L(1)));
    Ypos = round(p(5,kk).*(Y_limit/param.L(2)));

    % -- check if the particle is within the outer boundary of the map
    if Xpos > 0 && Ypos > 0 && Xpos < X_limit && Ypos < Y_limit

        % -- check if the particle representing the target position 
        % -- is within a usable portion of the map
        if bin_map(Xpos,Ypos)
            wts(kk) = 1/param.N;
        else
            wts(kk) = 0;
        end
    else
        % -- if outside the boundary, wts = 0
        wts(kk) = 0; 
    end
end

% -- get a measurement of the human robot from the autonomous robot
% -- using the autonomous robot's particle estimate
% -- non-linear measurement model
Zh = hfun(p);

% -- compare the Zsup value passed in, to the calculated Zh measurement
% -- calculated from the particles passed in
for ii=1:size(param.eta,1)
    wts=wts.*normpdf(Zh(ii,:), Zsup(ii,:), param.eta(robot));
end

function X = rt2d(X, v, omega, dt)
% robot
X(3,1) = X(3,1) + omega*dt;
X(1,1) = X(1,1) + v*cos(X(3,1))*dt;
X(2,1) = X(2,1) + v*sin(X(3,1))*dt;

% target
X(4,1) = X(4,1);
X(5,1) = X(5,1);

% -- the human teleoperated robot
X(6,1) = X(6,1);
X(7,1) = X(7,1);

function z = get_measurements(X)

z(1,:)=wrapToPi(atan2((X(5,:)-X(2,:)), (X(4,:)-X(1,:)))-X(3,:));
% z(1,:)=atan2((X(end,:)-X(2,:)), (X(end-1,:)-X(1,:)))-X(3,:);