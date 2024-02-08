function traj_plot(simdata, X0, N, nsim, eta, Num_target_found, time, dt, ...
                   img, agents, location, share_info, share_info_MI, target_locations, alpha)

rb_color = ["bs", "rs", "gs"]; % -- store the robot color for its beginning position
rp_color = ["b.", "r.", "g."]; % -- store the robot color for its beginning position
re_color = ["xb","xr","xg"]; % -- store the robot color for its end position
tp_color = ["g.", "m."]; % -- store hte target particle color state
te_color = ["+b", "+r", "+g"]; % -- store the target estimate color state

% -- plot the map of the simulated environment
figure(1); gcf; clf;
imagesc([0 30],[0 30],img); hold on;
set(gca,'ydir','normal');

success = find(Num_target_found == 1); % -- only get the successful runs of the simulation
times = time(success); % -- get the times that correspond to the successfull sim
more = 0;

% -- loop for every agent that is in the simulation
for robot = 1:agents
    % -- check if we even succeded in finding the target
    if sum(success) == 0
        disp("No Successful Run :(");
        break
    end
    % -- plot the location of the simulated target, true target location!
    [xt, yt] = draw_circle(X0(4:5,1,1,1), 1); % -- we want a target same size as robot
    patch(xt, yt, 'k', 'facealpha', .5, 'edgecolor', 'none'); % -- simulated target location
    
    % -- loop for every successful simulation completed
    for ii = 1:size(success,1)
        % -- get the sim number that was successful
        jj = success(ii);
        
        % -- plot the estimated robot trajectory, and the robot simulated trajectory
        plot(simdata(jj).Xs(1,:,1,robot),simdata(jj).Xs(2,:,1,robot), 'k');
%         plot(simdata(1).Xh(1,:,1,1), simdata(1).Xh(2,:,1,1), 'color', [0 0 0]+(ones(1,3)-[0 0 0])*0.5);
        hold on; plot(simdata(jj).Xh(1,:,1,robot), simdata(jj).Xh(2,:,1,robot), rp_color(robot), 'markersize',3);
        
        % -- mark down where the robots began and ended, their estimates!
        hold on; plot(simdata(jj).Xh(1,1,1,robot),simdata(jj).Xh(2,1,1,robot), rb_color(robot), 'markersize', 2);
        hold on; plot(simdata(jj).Xh(1,end,1,robot),simdata(jj).Xh(2,end,1,robot), re_color(robot), 'linewidth', 5, 'markersize', 20);
        
        % -- plot the possible hding locations of the distressed animal
        for j = 1:size(target_locations,1)
           plot(target_locations(j,1), target_locations(j,2), 'sm', 'markersize', ...
                16, 'linewidth', 2); 
        end
        
        if more
            % -- plot the estimated target position for every robot
            plot(simdata(ii).Xh(4,:,1,robot), simdata(ii).Xh(5,:,1,robot), te_color(robot),'linewidth', 1, 'markersize', 16);
        end
    end
    hold on;
end

% -- create a grid and square the map to make it look clean
grid on; grid minor;
axis([-5 35 -5 35]); axis square;

% -- title the trajectory map
title(sprintf('Number of sim: %d, Avg time: %.2f +- %.2f steps, dt: %d s, alpha: %.1f',...
        length(success), mean(times), std(times), dt, alpha));

% -- Save the figure only if there was atleast one successful run!
% -- In the image title, state the type of environment, number of particles, 
% -- number of successful simulations, and number of agents in simulation.
if success
    saveas(gcf,append(location, sprintf('_p=%d_nsim=%d_agents=%d_SI=%d_a=%.1f.png', ...
           N, size(success,1), agents, share_info, alpha)));
end

end

