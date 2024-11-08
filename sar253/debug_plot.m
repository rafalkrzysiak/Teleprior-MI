function debug_plot(Xs, Xh, k, p, param, bin_map, fiducial, ...
                    jj, X0, saveFrames, target_locations, I, ...
                    alpha, TotalDist, pDxMxT, pDyMyT, ConfigSetup)
% This function is to be used as a debugging tool
% a figure is displayed to show what is happening while the mutual
% information code with a particle filter is running

% -- flag to determine whether or not to show additional information
more = 1;

% -- the number of rows and columns depending on the number of robots
row = param.agents;
col = 2;

% -- colors to be used for plotting
self_color = ["r", "g", "b", "c", "m"]; % -- store the robot color for its beginning and end position
self_l_color = ["-r", "-g", "-b", "-c", "-m"]; 
self_MI_norm = ["--r", "--g", "--b", "--c", "m"];
self_p_color = ["r.","g.", "b.", "c.", "m."]; % -- store the robot particle position color
self_ref_color = ["rx","gx", "bx", "cx", "mx"]; % -- store the robot particle position color
tp_color = ["ro", "go", "bo", "k.", "b."]; % -- store the target particle color state
t_color = ["r+", "g+", "b+", "+k", "+b"]; % -- store the target estimate color state

% -- for each of the robot, display the particle histogram
% for kk = 1:agents
%     figure(10); gcf;
%     subplot(row,col,2*kk-1);
%     hist(p(4,:,1,kk), 0:1:L);
%     title(sprintf('Robot %d Predicted X Target',kk));
%     subplot(row,col,2*kk);
%     hist(p(5,:,1,kk), 0:1:L);
%     title(sprintf('Robot %d Predicted Y Target',kk));
% end

% -- plot the MI values in real time
for agent = 1:param.agents
    
    % -- only at the first timestep, clear the figure
    if k == 1
       figure(10); clf; 
    end
    
    % -- create the figure and subplots for each robot
    figure(10); gcf;
    subplot(1, param.agents, agent);
    
    % -- plot the MI value without normalizing it of the target
    plot(1:1:k, I(4,1:k,1,agent), self_l_color(agent), 'linewidth', 2);
    hold on;
    
    % -- plot the following of the human teleoperated robot
    if agent ~= 1

        % -- plot the MI value after normalizing of the human robot
        plot(1:1:k, I(5,1:k,1,agent), self_MI_norm(agent), 'linewidth', 2);
        hold on;
    end
    
    axis image; axis square; axis([0 k 0 2]);
    title(sprintf("Mutual Information of robot: %d", agent), 'fontsize', 18);
    xlabel("Time (s)", 'fontsize', 18);
    ylabel("MI (bits)", 'fontsize', 18);
end

% -- plot the environment and make it look nice
figure(1); gcf; clf;
hold on;
imagesc([0 param.L(1)],[0 param.L(2)],bin_map); 
% set(gca,'ydir','normal');
set(gca,'ydir','reverse');
axis image;
axis([-1 param.L(1)+1 -1 param.L(2)+1]); 

% -- loop through every robot in the simulation
for kk = 1:param.agents
    
     % -- for getting the circles that represent the robot, FOV, and target
     if kk ~= 1
         [xc, yc] = draw_circle(Xs(:,:,1,kk), param.r_visible(kk), param.r_FOV(kk)); % -- robot field of view
     else
         xc = [Xs(1,:,1,kk), Xs(1,:,1,kk)+param.r_visible(kk)*cos(Xs(3,:,1,kk)-param.r_FOV(kk)/2), ...
               Xs(1,:,1,kk)+param.r_visible(kk)*cos(Xs(3,:,1,kk)+param.r_FOV(kk)/2)];
         yc = [Xs(2,:,1,kk), Xs(2,:,1,kk)+param.r_visible(kk)*sin(Xs(3,:,1,kk)-param.r_FOV(kk)/2), ...
               Xs(2,:,1,kk)+param.r_visible(kk)*sin(Xs(3,:,1,kk)+param.r_FOV(kk)/2)];
     end
    [xr, yr] = draw_circle(Xs(:,:,1,kk), 2*param.r_robot, 0); % -- robot size
    [xt, yt] = draw_circle(Xs(4:5,:,1,kk), 1, 0); % -- we want a target same size as robot
    
    % -- only show the simulated target position once
    if kk == 1
        patch(xt,yt,'k','facealpha',.5, 'edgecolor', 'none'); % simulated target location
        
        % -- plot the possible hiding locations of the animal in distress
        for j = 1:size(target_locations,1)
           plot(target_locations(j,1), target_locations(j,2), 'sm', 'markersize', ...
                16, 'linewidth', 2); 
        end
    end
    
    % -- plot the target particle estimates and the target estimate position
    plot(p(4,:,1,kk), p(5,:,1,kk), tp_color(kk)); % particle state of target
    plot(Xh(4,:,1,kk), Xh(5,:,1,kk), t_color(kk), 'linewidth', 4, 'markersize', 18);
    
    if kk ~= 1
       plot(p(6,:,1,kk), p(7,:,1,kk), self_ref_color(kk), 'markersize', 6, 'linewidth', 1); % particle state of robot 
%        plot(Xh(6,1,1,kk), Xh(7,1,1,kk), 'xc', 'markersize', 12, 'linewidth', 2);
%        plot(Xh(4,1,1,kk), Xh(5,1,1,kk), t_color(kk), 'linewidth', 4,'markersize', 20); % estimate of target
    end
    
    % -- plot the robot information
    patch(xc,yc,'k','facealpha',.15, 'edgecolor', 'none'); % -- robot field of view face
    patch(xr,yr,self_color(kk),'facealpha',.5, 'edgecolor', 'none'); % visualization of robot body
    
    % -- show the orientation of the robot
%     quiver(Xh(1,1,1,kk), Xh(2,1,1,kk),Xh(1,1,1,kk)*cos(Xh(3,1,1,kk)), Xh(2,1,1,kk)*sin(Xh(3,1,1,kk)),0);
    line([Xs(1,1,1,kk), Xs(1,1,1,kk)+3*cos(Xs(3,1,1,kk))],...
         [Xs(2,1,1,kk), Xs(2,1,1,kk)+3*sin(Xs(3,1,1,kk))],...
         'color',self_color(kk),'linestyle','-')
    
    % -- flag if we want additional information to be displayed
    if more
        % -- plotting information for the robots in the simulation
        plot(X0(1,1,1,kk),X0(2,1,1,kk), self_color(kk), 'markersize', 20, 'linewidth', 2); % -- plot the initial position of the robots
%         plot(p(1,:,1,kk), p(2,:,1,kk), 'k.'); % -- particle state of robot
        plot(Xh(1,1,1,kk), Xh(2,1,1,kk), self_color(kk), 'markersize', 20); % estimate of robot
        plot(p(1,:,1,kk), p(2,:,1,kk), self_p_color(kk)); % particle state of robot
        plot(Xs(1,1,1,kk), Xs(2,1,1,kk), 'ks', 'markersize', 16); % simulated robot
        if kk~=1
            plot(Xh(6,1,1,kk), Xh(7,1,1,kk), self_ref_color(kk), 'markersize', 12, 'linewidth', 2);
        end
        plot(Xh(4,1,1,kk), Xh(5,1,1,kk), t_color(kk), 'linewidth', 4,'markersize', 20); % estimate of target
    end
    

    %title('MI= \alpha I(Z_t|X_i) + (1-\alpha) I(Z_h|X_i)', 'Fontsize', 16);

end
% -- display the time step number on the x-axis, and refresh the plot
xlabel(k);

% -- plot the alpha over time
figure(3);gcf; clf;
hold on;
plot(1:1:k, alpha(1:k,1), 'k-', 'linewidth', 2);
title(sprintf('Alpha: %0.5f', alpha(k,1)), 'FontSize', 16);
xlabel('Time (s)', 'FontSize', 16);
ylabel('Alpha', 'FontSize', 16);
ylim([0 1]);

figure(40); gcf; clf;
subplot(1,3,1);
for kk = 1:param.agents

    if kk ~=1
        
        hold on;
        plot(Xs(1,1,1,1),  Xs(2,1,1,1), 'k.',  'markersize', 28); hold on;
        plot(Xh(6,1,1,kk), Xh(7,1,1,kk), self_ref_color(kk),  'markersize', 26, 'linewidth', 3)
        xlabel('X location (m)', 'FontSize', 16);
        ylabel('Y location (m)', 'FontSize', 16);
        title('True position vs estimate');
    
    end
    axis square;
end

subplot(1,3,2);
for kk = 1:param.agents
    if kk ~=1
        
        difx(k,kk) = Xh(6,1,1,kk) - Xs(1,1,1,1);
        dify(k,kk) = Xh(7,1,1,kk) - Xs(2,1,1,1);
        hold on; plot(0, 0, 'ko', 'markersize', 26, 'linewidth', 3)
        hold on; plot(difx(k,kk), dify(k,kk), self_ref_color(kk), 'markersize', 26, 'linewidth', 3);
        axis([-7 7 -7 7]);
        xlabel('X Position diff', 'FontSize', 16);
        ylabel('Y Position diff', 'FontSize', 16); 
        title(sprintf("Timestep: %d", k), 'fontsize', 18);
        legend('Ideal');
        axis square;
    end
end

if ConfigSetup == "alpha_t/TotalDist"

    subplot(1,3,3);
    for kk = 1:param.agents
        if kk ~=1
            hold on;
            plot(1:1:k, TotalDist(1:k,1,kk), self_l_color(kk), 'LineWidth', 2)
            xlabel('Time (s)', 'FontSize', 16);
            ylabel('Total distance', 'FontSize', 16); 
            title('Total distance', 'fontsize', 18);
            axis square;
        end
    end
    legend('robot 1', 'robot 2');

end

if ConfigSetup == "alpha_t/FreezeTime"
    
end

figure(50); gcf; clf;
plot(1:1:k, pDxMxT(1:k,1), 'k-', 'LineWidth', 2);
hold on; plot(1:1:k, pDyMyT(1:k,1), 'r-', 'LineWidth', 2);
legend('pDxMxT', 'pDyMyT', 'FontSize', 16);
xlabel('Timestep', 'FontSize', 16);
ylabel('p', 'FontSize', 16);
title(sprintf("pDxMxT: %0.5f, pDyMyT:%0.5f", pDxMxT(k,1), pDyMyT(k,1)), 'fontsize', 16);
axis square;
axis([0 k 0 1]);

drawnow;
% -- save the figures as a frame to later create videos
saveas(gcf,strcat(saveFrames, sprintf('%02d%04d.png', jj, k)));

end

