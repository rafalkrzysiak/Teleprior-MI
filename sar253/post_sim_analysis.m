function post_sim_analysis

% -- this function will serve for analysis of the simulation data

% -- clear  everything before running any function
clear all; clc;

sim_plot(); % -- plot the trajectories of the robot
% sim_animation(); % -- looking at the simulation over time
% sim_time();
% sim_bias();
% alpha_v_dist();
% dist_over_time();
% alpha_v_dist_auto();
% MI_data();
% sim_speed();

end

function sim_plot()
% -- read the simulation data
% -- by varying the parameters
% agents = ["1", "2", "3"];
% bias = ["1"];
% alpha = ["0", "0.25", "0.5", "0.75", "1"];
% target = ["2", "3"];

% agents = ["1", "2", "3"];
% bias = ["1"];
% alpha = ["0", "0.25", "0.5", "0.75", "1"];
% target = ["1", "2", "3"];

agents = ["2"];
bias = ["1"];
alpha = ["0", "0.25", "0.5", "0.75", "1"];
target = ["3"];

% -- personalized colors for the robots
r_color = ['b', "r-.", "r-."];
rs_color = ["bs", "ro", "ro"];
re_color = ["bx", "rx", "rx"];

fignum = 1;
for tloc = 1:size(target,2)
    for agent = 1:size(agents,2)
        for b = 1:size(bias,2)
            step = 1;
            for a = 1:size(alpha,2)
                
                % -- specify the directory of the .mat files
                % -- using the specified items in the nested loops
                % -- for ther simulation with omnly the human, the alpha
                % -- value is 1 (MI optimized for the target)
                if agents(agent) == "1" && str2double(alpha(a)) < 1
                    continue
                else
                    dir = append('data/plain_map/',agents(agent),'robots/alpha_',alpha(a),'/bias_',bias(b),...
                                 '/target_position',target(tloc),'/plain_map_p=600_nsim=25_agents=',agents(agent),'.mat');

                    % -- load the .mat file using the specified directory
                    load(dir);
                    IMG = imread('plain_map_new.jpg');
                    
                    % -- get the total time it took to run the saimulation
                    % -- for every part of the simulation
                    % -- the variable will hold all times of the simulation
                    % -- per condition
                    time = cat(1, simdata.time);
                    success = cat(1,simdata.success);
                    size(time(success ~= 2));
                    % -- get the mean and standard deviation of time
                    mean_time = mean(time);
                    std_time = std(time);
                    
                    % -- get the number of simulations ran for the condition
                    NumberOfSim = size(simdata,2);
                    loop_step = 1;
                    % -- loop through each individual sim within a condition
                    for sim = 1:NumberOfSim
                        % -- Create a figure that corresponds to the target location number
                        % -- subplots for each figure are 5x5, with the
                        % -- columns being the alpha value and the rows are
                        % -- the sim number
                        if simdata(sim).success ~= 2
%                             if loop_step == 11
%                                 break;
%                             end
                            figure(sim);
                            axes = subplot(1,size(alpha,2),a);
                            imagesc([0 simdata(sim).param.L(1)],[0 simdata(sim).param.L(2)],IMG); hold on;
                            set(gca,'ydir','normal');
                            axis([0 simdata(sim).param.L(1) 0 simdata(sim).param.L(2)]); axis square;

                            % -- get the MI values for each of the robots in
                            % -- that condition simulated
                            Xs_vals = simdata(sim).Xs(:,1:time(sim),:,:); 

                            % -- get the size of the MI_vals dimension
                            % -- we want to make sure it matches the agents
                            % -- number for that condition. When there are more
                            % -- robots in the simulation , the dimension of the
                            % -- array is 4-D
                            size_MI = size(Xs_vals,4);

                            % -- plot the target location on the map
                            plot(simdata(sim).tloc(1), simdata(sim).tloc(2), ...
                                 'kd', 'markersize', 8, 'linewidth', 2); 
                            hold on;

                            % -- looping through for every robot in the sim
                            for rob = 1:size_MI
                                % -- the individual trajectories of each robot
                                plot(simdata(sim).Xs(1,1:time(sim),1,rob), simdata(sim).Xs(2,1:time(sim),1,rob),...
                                     r_color(rob), 'markersize', 1, 'linewidth', 2); hold on;

                                % -- plot the starting and ending points of the robots
                                plot(simdata(sim).Xs(1,1,1,rob), simdata(sim).Xs(2,1,1,rob),...
                                     rs_color(rob), 'markersize', 8, 'linewidth', 2); hold on;

                                plot(simdata(sim).Xs(1,time(sim),1,rob), simdata(sim).Xs(2,time(sim),1,rob),...
                                     re_color(rob), 'markersize', 8, 'linewidth', 2); hold on;
                            end
%                             title(sprintf("t: %d, a: %.2f, Sim: %d, Rf: %d", tloc, alpha(a), sim, simdata(sim).Which_robot(sim)), 'fontsize', 10)
                            val = alpha(a);
                            title(sprintf('\\alpha=%.2f', str2double(val)), 'fontsize', 24, 'fontweight', 'normal');
                            ylabel("Y (m)", 'fontsize', 24);
                            xlabel("X (m)", 'fontsize', 24)
                            axes.FontSize = 24;
                            step = step + 1;
                            loop_step = loop_step + 1;
                        end
                    end
                end
            end
            fignum = fignum + 1;
        end
    end
end
    
end

function sim_animation()

% -- read the simulation data
% -- by varying the parameters
agents = ["1", "2", "3"];
bias = ["1"];
alpha = ["0", "0.25", "0.5", "0.75", "1"];
% target = ["1", "2", "3", "4"];
target = ["1", "2", "3"];

% -- personalized colors for the robots
r_color = ['b', 'r', 'g'];
start_r_color = ["bs", "rs", "gs"];

for agent = 3:size(agents,2)
    for a = 5:size(alpha,2)
        for b = 1:size(bias,2)
            for tloc = 3:size(target,2)
                
                % -- specify the directory of the .mat files
                % -- using the specified items in the nested loops
                % -- for ther simulation with omnly the human, the alpha
                % -- value is 1 (MI optimized for the target)
                if agent == 1 && str2double(alpha(a)) < 1
                    continue
                else
                    dir = append('data/plain_map/',agents(agent),'robots/alpha_',alpha(a),'/bias_',bias(b),...
                                 '/target_position',target(tloc),'/plain_map_p=600_nsim=25_agents=',agents(agent),'.mat');

                    % -- load the .mat file using the specified directory
                    load(dir);
                    
                    % -- get the total time it took to run the saimulation
                    % -- for every part of the simulation
                    % -- the variable will hold all times of the simulation
                    % -- per condition
                    time = cat(1, simdata.time);
                    
                    % -- get the mean and standard deviation of time
                    mean_time = mean(time);
                    std_time = std(time);
                    
                    % -- hold the successes into a single variable
                    success = cat(1, simdata.success);
                    
                    % -- begin animating the simulation
                    for sim_num = 1:size(simdata,2) % -- one sim at a time per condition
                        
                        if simdata(sim_num).success ~= 2
                            % -- place the map of the environment
                            figure(1); gcf; clf;
                            imagesc([0 simdata(sim_num).param.L(1)],[0 simdata(sim_num).param.L(2)],img); hold on;
                            set(gca,'ydir','normal');
                            axis([-5 simdata(sim_num).param.L(1)+5 -5 simdata(sim_num).param.L(2)+5]); axis square;

                            % -- plot the target location
                            [xt, yt] = draw_circle(simdata(sim_num).Xs(4:5,1,1,1), 1, 0); % -- we want a target same size as robot
                            patch(xt,yt,'k','facealpha',.5, 'edgecolor', 'none'); % simulated target location

                            % -- plot the starting points of the robots
                            for r = 1:str2double(agents(agent))
                               plot(simdata(sim_num).Xs(1,1,1,r), simdata(sim_num).Xs(2,1,1,r),...
                                    start_r_color(r), 'markersize', 16, 'linewidth', 3);
                               hold on;
                            end

                            title(sprintf('Sim: %d, tloc: %d, Total time: %d, alpha: %.2f', ...
                                  sim_num, str2double(target(tloc)), time(sim_num), str2double(alpha(a))));

                            for k = 1:time(sim_num) % -- until the target is found
                                for robot = 1:str2double(agents(agent)) % -- for every robot in the simulation
                                    [xr, yr] = draw_circle(simdata(sim_num).Xs(:,k,1,robot), .3, 0); % -- robot size

                                    patch(xr,yr,r_color(robot),'facealpha',.5, 'edgecolor', 'none'); % visualization of robot body

                                    drawnow;

                                end 
                            end
                        end
                    end
                    
                end
                
            end
        end
    end
end

end

function sim_time()
% -- read the simulation data
% -- by varying the parameters
agents = ["1", "2", "3"];
bias = ["1"];
alpha = ["0", "0.25", "0.5", "0.75", "1"];
target = ["1", "2", "3"];

% -- personalized colors for the robots
r_color = ['b', 'r', 'g'];

for agent = 1:size(agents,2)
    for a = 1:size(alpha,2)
        for b = 1:size(bias,2)
            for tloc = 1:size(target,2)
                
                % -- specify the directory of the .mat files
                % -- using the specified items in the nested loops
                % -- for ther simulation with omnly the human, the alpha
                % -- value is 1 (MI optimized for the target)
                if agents(agent) == "1" && str2double(alpha(a)) < 1
                    continue
                else
                    dir = append('data/plain_map/',agents(agent),'robots/alpha_',alpha(a),'/bias_',bias(b),...
                                 '/target_position',target(tloc),'/plain_map_p=600_nsim=10_agents=',agents(agent),'.mat');

                    % -- load the .mat file using the specified directory
                    load(dir);
                    
                    % -- get the total time it took to run the saimulation
                    % -- for every part of the simulation
                    % -- the variable will hold all times of the simulation
                    % -- per condition
                    time = cat(1, simdata.time);
                    
                    % -- hold the successes into a single variable
                    success = cat(1, simdata.success);
                    
                    time = time(success == 1).*simdata(1).param.dt;
                    
                    % -- get the mean and standard deviation of time
                    mean_time(tloc) = mean(time);
                    std_time(tloc) = std(time);
                    
                end
            end
            % -- create figure that corresponds to total robot number
%             figure(agent);
            figure(1);
            if agents(agent) == "1" && str2double(alpha(a)) == 1
%                 errorbar([1 2 3], mean_time, std_time, 'linewidth', 2);
%                 confplot([1 2 3], mean_time, std_time);
                r1_mean_time = mean_time;
                r1_std_time = std_time;
                
%                 shadedErrorBar([1 2 3], mean_time, std_time, 'lineProps',r_color(b));
%                 hold on;
            elseif agents(agent) ~= "1"
                subplot(1,size(alpha,2),a);
%                 errorbar([1 2 3], mean_time, std_time, 'linewidth', 2);
%                 confplot([1 2 3], mean_time, std_time);
                shadedErrorBar([1 2 3], r1_mean_time, r1_std_time, 'lineProps',r_color(1));
                hold on;
                shadedErrorBar([1 2 3], mean_time, std_time, 'lineProps',r_color(agent));
                hold on;
            end
        end
        axis image; axis square; axis([1 3 0 1000]);
        legend('1 robots with bias','2 robots with bias','3 robots with bias',...
               'fontsize', 12);
        xlabel('Target position', 'fontsize', 18); ylabel('Time (s)', 'fontsize', 18);
        title(sprintf('agents: %d, alpha: %.1f, dt: %.1f', agent, str2double(alpha(a)), .2), 'fontsize', 18);
        grid on; grid minor;
    end
end
    
end

function sim_bias()
% -- read the simulation data
% -- by varying the parameters
% agents = ["1", "2", "3"];
% bias = ["1"];
% alpha = ["0", "0.25", "0.5", "0.75", "1"];
% target = ["1", "2", "3"];

agents = ["1", "2", "3"];
bias = ["1"];
alpha = ["0", "0.25", "0.5", "0.75", "1"];
target = ["1", "2", "3"];

% -- personalized colors for the robots
r_color = ['b', 'r', 'g'];
rm_color = ["b", "--r*", "--go"];

mean_time = zeros(size(agents,2), size(alpha,2));
std_time = mean_time;

% -- create an empty matrix that will contain all numbers of the sim
sim_info = zeros(23,size(agents,2)*size(alpha,2)*size(target,2));
column = 1;

for tloc = 1:size(target,2)
    for b = 1:size(bias,2)
        for agent = 1:size(agents,2)
            for a = 1:size(alpha,2)
                
                % -- specify the directory of the .mat files
                % -- using the specified items in the nested loops
                % -- for ther simulation with omnly the human, the alpha
                % -- value is 1 (MI optimized for the target)
                if agents(agent) == "1" && str2double(alpha(a)) < 1
                    continue
                else
                    dir = append('data/plain_map/',agents(agent),'robots/alpha_',alpha(a),'/bias_',bias(b),...
                             '/target_position',target(tloc),'/plain_map_p=600_nsim=25_agents=',agents(agent),'.mat');
                    
                    % -- load the .mat file using the specified directory
                    load(dir);
                    
                    % -- get the total time it took to run the saimulation
                    % -- for every part of the simulation
                    % -- the variable will hold all times of the simulation
                    % -- per condition
                    time = cat(1, simdata.time);
                    
                    % -- hold the successes into a single variable
                    success = cat(1, simdata.success);
                    
                    time = time(success ~= 2).*simdata(1).param.dt;
                    time = time(1:20);
                    % -- get the mean and standard deviation of time
                    mean_time(agent,a) = mean(time);
                    
%                     if agent == 1 && a == size(alpha,2)
%                         Rtime = mean_time(agent,a);
%                     end
                    
                    mean_time(agent,a) = mean_time(agent,a);%./Rtime;
                    std_time(agent,a) = std(time);
                    
                    % -- hold the successes into a single variable
                    success = cat(1, simdata.success);
                    
                    sim_info(:,column) = [str2double(target(tloc));
                                          str2double(agents(agent));
                                          str2double(alpha(a));
                                          time];
                    column = column + 1;
                end
            end
        end
    end
    % -- create figure that corresponds to total robot number
    figure(2);
    
    for kk = 1:size(agents,2)
        axes = subplot(1,size(target,2),tloc);
        
        if kk == 1
%             confplot([0, .25, .5, .75, 1], ones(1,size(alpha,2))*mean_time(kk,end), ones(1,size(alpha,2))*std_time(kk,end));
              plot(linspace(0,1,5), ones(1,size(alpha,2))*mean_time(kk,end),...
                  'k--', 'linewidth', 2); hold on;
              plot(linspace(0,1,5), ones(1,size(alpha,2))*(mean_time(1,end)+std_time(1,end)),'k:', 'linewidth', 1); hold on;
              plot(linspace(0,1,5), ones(1,size(alpha,2))*(mean_time(1,end)-std_time(1,end)),'k:', 'linewidth', 1); hold on;
              
%               shadedErrorBar([0, .25, .5, .75, 1], ones(1,size(alpha,2))*mean_time(kk,end), ones(1,size(alpha,2))*std_time(kk,end), 'lineProps', r_color(b));
        else
%             confplot(str2double(alpha), mean_time(kk,:), std_time(kk,:));
              shadedErrorBar(linspace(0,1,5), mean_time(kk,:), std_time(kk,:),...
                  'lineProps', r_color(kk)); hold on;
              plot(linspace(0,1,5),mean_time(kk,:), rm_color(kk), 'markersize', 12, 'linewidth', 3); hold on;
        end
%         errorbar(str2double(alpha), mean_time(kk,:), std_time(kk,:), 'linewidth', 2);
        hold on;
    end
    axis image; axis square; axis([0 1 0 simdata(1).param.T]);
    f=get(axes,'Children');
    legend([f(7),f(3),f(1)],'1 robot','2 robots','3 robots','fontsize', 20); hold on;
    set(axes, 'XTick', 0:.25:1, 'XTicklabel', alpha);
    title(sprintf('Target location: %d', tloc), 'fontsize', 22, 'fontweight', 'normal');
    axes.FontSize = 28; xlabel('\alpha', 'fontsize', 40); ylabel('Time (s)', 'fontsize', 28);
    grid on;
end
    
end

function alpha_v_dist()
% -- read the simulation data
% -- by varying the parameters
% agents = ["1", "2", "3"];
% bias = ["1"];
% alpha = ["0", "0.25", "0.5", "0.75", "1"];
% target = ["1", "2", "3"];

agents = ["1", "2", "3"];
bias = ["1"];
alpha = ["0", "0.25", "0.5", "0.75", "1"];
target = ["1", "2", "3"];

% -- personalized colors for the robots
r_color = ['b', 'r', 'g'];
rm_color = ["b", "--r*", "--go"];

mean_dist = zeros(size(agents,2), size(alpha,2));
std_dist = mean_dist;

for tloc = 1:size(target,2)
    for b = 1:size(bias,2)
        for agent = 1:size(agents,2)
            for a = 1:size(alpha,2)
                
                % -- specify the directory of the .mat files
                % -- using the specified items in the nested loops
                % -- for ther simulation with omnly the human, the alpha
                % -- value is 1 (MI optimized for the target)
                if agents(agent) == "1"
                    continue
                else
                    dir = append('data/plain_map/',agents(agent),'robots/alpha_',alpha(a),'/bias_',bias(b),...
                                 '/target_position',target(tloc),'/plain_map_p=600_nsim=25_agents=',agents(agent),'.mat');

                    % -- load the .mat file using the specified directory
                    load(dir);
                    
                    % -- get the total time it took to run the saimulation
                    % -- for every part of the simulation
                    % -- the variable will hold all times of the simulation
                    % -- per condition
                    time = cat(1, simdata.time);
                    % -- hold the successes into a single variable
                    success = cat(1, simdata.success);
                    
                    time = time(success ~= 2).*simdata(1).param.dt;
                    time = time(1:20);
                    mean_time = mean(time);
                    loop_step = 1;
                    
                    for jj = 1:size(simdata,2)
                        if simdata(jj).success ~= 2
                            dist1(jj,:) = sqrt((simdata(jj).Xs(1,1:simdata(jj).param.T,1,1)-simdata(jj).Xs(1,1:simdata(jj).param.T,1,str2double(agents(agent)))).^2 +...
                                              (simdata(jj).Xs(2,1:simdata(jj).param.T,1,1)-simdata(jj).Xs(2,1:simdata(jj).param.T,1,str2double(agents(agent)))).^2);
                        else
                            dist1(jj,:) = zeros(1, simdata(jj).param.T);
                        end
                    end
                    
                    mean_d1 = mean(dist1);
                    
                    % -- get the mean and standard deviation of time
                    mean_dist1(agent,a) = mean(mean_d1(1:floor(mean_time)));
                    std_dist1(agent,a) = std(mean_d1(1:floor(mean_time)));
                    
                    
                    % -- hold the successes into a single variable
                    success = cat(1, simdata.success);
                    
                end
            end
            
            figure(1); hold on;
    
            if agents(agent) == "2" || agents(agent) == "3"
                axes = subplot(1,size(target,2),tloc);
                shadedErrorBar(str2double(alpha), mean_dist1(agent,:), std_dist1(agent,:), 'lineProps', r_color(agent));
                hold on; plot(linspace(0,1,5), mean_dist1(agent,:), rm_color(agent), 'markersize', 12, 'linewidth', 3); hold on;
                
                f = get(axes,'Children');
                axis image; axis square; axis([0 1 0 20]);
%                 
                
                set(axes, 'XTick', 0:.25:1, 'XTicklabel', alpha);
                title(sprintf('Target location: %d', tloc), 'fontsize', 22, 'fontweight', 'normal');
                axes.FontSize = 28; xlabel('\alpha', 'fontsize', 40); ylabel('Avg distance to teleoperated robot (m)', 'fontsize', 28);
                grid on;
            end
        end
    end
    % -- create figure legend
    legend([f(3), f(1)],'2 robots', '3 robots', 'fontsize', 22);
end
    
end

function dist_over_time()
% -- read the simulation data
% -- by varying the parameters
agents = ["2"];
bias = ["1"];
alpha = ["0", "0.5", "1"];
target = ["1", "2", "3"];

% -- personalized colors for the robots
r_color = ['b', 'r', 'g'];

mean_dist = zeros(size(agents,2), size(alpha,2));
std_dist = mean_dist;

% -- increment the figure number to match the number of parameters
% -- initialize the figure number to 1
fig_num = 1;

for tloc = 1:size(target,2)
    for b = 1:size(bias,2)
        for agent = 1:size(agents,2)
            for a = 1:size(alpha,2)
                
                % -- specify the directory of the .mat files
                % -- using the specified items in the nested loops
                % -- for ther simulation with omnly the human, the alpha
                % -- value is 1 (MI optimized for the target)
                if agents(agent) == "1"
                    continue
                else
                    dir = append('data/plain_map/',agents(agent),'robots/alpha_',alpha(a),'/bias_',bias(b),...
                                 '/target_position',target(tloc),'/plain_map_p=800_nsim=10_agents=',agents(agent),'.mat');

                    % -- load the .mat file using the specified directory
                    load(dir);
                    
                    % -- get the total time it took to run the saimulation
                    % -- for every part of the simulation
                    % -- the variable will hold all times of the simulation
                    % -- per condition
                    time = cat(1, simdata.time);
                    mean_time = mean(time);
                    
                    for jj = 1:size(simdata,2)
                        % -- get the distance between the human robot and
                        % -- the autonomous one
                        dist(jj,:) = sqrt((simdata(jj).Xs(1,1:simdata(jj).T,1,1)-simdata(jj).Xs(1,1:simdata(jj).T,1,str2double(agents(agent)))).^2 +...
                                          (simdata(jj).Xs(2,1:simdata(jj).T,1,1)-simdata(jj).Xs(2,1:simdata(jj).T,1,str2double(agents(agent)))).^2);
                        
                        % -- get the moving mean of the distance (smoother)              
                        dist1 = movmean(dist(jj,:), 10);              
                        
                        % -- plot everything
                        figure(fig_num); gcf; % -- create a figure for each alpha value
                        subplot(2,5,jj); % -- create multiple subplots, one subplot for each sim
                        plot(1:time(jj), dist(jj,1:time(jj)), 'k', 'linewidth', 2); % -- the distance
                        hold on; plot(1:time(jj), dist1(1:time(jj)), 'b', 'linewidth', 2); % -- plot the moving mean dist
                        xlabel('Timestep'); ylabel('Distance (m)'); % -- axis labels
                        title(sprintf('alpha:%.2f, sim:%d, bias:%d, tloc:%d', ...
                              str2double(alpha(a)), jj, str2double(bias(b)), tloc)); % -- title
                        axis image; axis square; axis([0 time(jj) 0 30]); grid on; grid minor; % -- figure formatting
                        
                    end
                    fig_num = fig_num + 1;
                end
            end
        end
    end
end

end

function alpha_v_dist_auto()
% -- read the simulation data
% -- by varying the parameters
agents = ["3"];
bias = ["1"];
alpha = ["0", "0.25", "0.5", "0.75", "1"];
target = ["1", "2", "3"];

% -- personalized colors for the robots
r_color = ['b', 'r', 'g'];

mean_dist = zeros(size(agents,2), size(alpha,2));
std_dist = mean_dist;

for tloc = 1:size(target,2)
    for b = 1:size(bias,2)
        for agent = 1:size(agents,2)
            for a = 1:size(alpha,2)
                
                % -- specify the directory of the .mat files
                % -- using the specified items in the nested loops
                % -- for ther simulation with omnly the human, the alpha
                % -- value is 1 (MI optimized for the target)
                if agents(agent) == "1"
                    continue
                else
                    dir = append('data/plain_map/',agents(agent),'robots/alpha_',alpha(a),'/bias_',bias(b),...
                                 '/target_position',target(tloc),'/plain_map_p=600_nsim=10_agents=',agents(agent),'.mat');

                    % -- load the .mat file using the specified directory
                    load(dir);
                    
                    % -- get the total time it took to run the saimulation
                    % -- for every part of the simulation
                    % -- the variable will hold all times of the simulation
                    % -- per condition
                    time = cat(1, simdata.time);
                    % -- hold the successes into a single variable
                    success = cat(1, simdata.success);
                    
                    time = time(success == 1).*simdata(1).dt;
                    mean_time = mean(time);
                    
                    for jj = 1:size(simdata,2)
                        dist1(jj,:) = sqrt((simdata(jj).Xs(1,1:simdata(jj).T,1,2)-simdata(jj).Xs(1,1:simdata(jj).T,1,str2double(agents(agent)))).^2 +...
                                          (simdata(jj).Xs(2,1:simdata(jj).T,1,2)-simdata(jj).Xs(2,1:simdata(jj).T,1,str2double(agents(agent)))).^2);
                                     
                    end
                    
                    mean_d1 = mean(dist1);
                    
                    % -- get the mean and standard deviation of time
                    mean_dist1(agent,a) = mean(mean_d1(1:floor(mean_time)));
                    std_dist1(agent,a) = std(mean_d1(1:floor(mean_time)));
                    
                    
                    % -- hold the successes into a single variable
                    success = cat(1, simdata.success);
                    
                end
            end
            
            figure(1); gcf; hold on;
    
            if agents(agent) == "3"
%                 for kk = 1:(size(agents,2)-1)
                    subplot(1,size(target,2),tloc);
%                     errorbar(str2double(alpha), mean_dist1(kk,:), std_dist1(kk,:), 'linewidth', 2);
%                     confplot(str2double(alpha), mean_dist1(kk,:), std_dist1(kk,:));
                    shadedErrorBar(str2double(alpha), mean_dist1(agent,:), std_dist1(agent,:), 'lineProps',r_color(agent+1));
                    hold on;
%                 end
%                 if agent == 1
                    axis image; axis square; axis([0 1 0 35]);
                    legend('Distance between A1 and A2', 'fontsize', 12);
                    xlabel('Alpha Value', 'fontsize', 18); ylabel('Dist (m)', 'fontsize', 18);
                    title(sprintf('Target: %d', tloc), 'fontsize', 18);
                    grid on; grid minor;

            end
        end
    end
    % -- create figure that corresponds to total robot number
end
    
end

function MI_data()
% -- read the simulation data
% -- by varying the parameters
% agents = ["1", "2", "3"];
% bias = ["1"];
% alpha = ["0", "0.25", "0.5", "0.75", "1"];
% target = ["2", "3"];

agents = ["1", "2", "3"];
bias = ["1"];
alpha = ["0", "0.25", "0.5", "0.75", "1"];
target = ["1", "2", "3"];

% -- personalized colors for the robots
rt_color = ["-b", "-r", "-g"];
rr_color = ["--b", "--r", "--g"];

fignum = 1;
for tloc = 1:size(target,2)
    for agent = 1:size(agents,2)
        for b = 1:size(bias,2)
            step = 1;
            for a = 1:size(alpha,2)
                
                % -- specify the directory of the .mat files
                % -- using the specified items in the nested loops
                % -- for ther simulation with omnly the human, the alpha
                % -- value is 1 (MI optimized for the target)
                if agents(agent) == "1" && str2double(alpha(a)) < 1
                    continue
                else
                    dir = append('data/plain_map/',agents(agent),'robots/alpha_',alpha(a),'/bias_',bias(b),...
                                 '/target_position',target(tloc),'/plain_map_p=600_nsim=12_agents=',agents(agent),'.mat');

                    % -- load the .mat file using the specified directory
                    load(dir);
                    
                    % -- get the total time it took to run the saimulation
                    % -- for every part of the simulation
                    % -- the variable will hold all times of the simulation
                    % -- per condition
                    time = cat(1, simdata.time);
                    
                    % -- get the number of simulations ran for the condition
                    NumberOfSim = size(simdata,2);
                    loop_step = 1;
                    
                    % -- loop through each individual sim within a condition
                    for sim = 1:NumberOfSim
                        
                        if simdata(sim).success ~= 2
                            if loop_step == 11
                                break;
                            end
                            % -- Create a figure that corresponds to the target location number
                            % -- subplots for each figure are 5x5, with the
                            % -- columns being the alpha value and the rows are
                            % -- the sim number
                            figure(fignum);
                            subplot(size(alpha,2), NumberOfSim-2, step);

                            % -- get the MI values for each of the robots in
                            % -- that condition simulated
                            MI_vals = simdata(sim).I(:,1:time(sim),:,:); 

                            % -- get the size of the MI_vals dimension
                            % -- we want to make sure it matches the agents
                            % -- number for that condition. When there are more
                            % -- robots in the simulation , the dimension of the
                            % -- array is 4-D
                            size_MI = size(MI_vals,4);

                            % -- plot the MI information over time for each sim
                            % -- for every condition
                            % -- the overall MI after weighting with alpha
                            plot(1:time(sim), MI_vals(1,1:time(sim),1,1), '-k', 'linewidth', 2); 
                            hold on;

                            % -- looping through for every robot in the sim
                            for rob = 1:size_MI
                                % -- the individual MI vals before weighting of
                                % -- the target
                                plot(1:time(sim), MI_vals(4,1:time(sim),1,rob), rt_color(rob),...
                                     'linewidth', 2); hold on;

                                if rob ~= 1
                                    % -- the individual MI vals before
                                    % -- weighting with alpha of the human robot
                                    plot(1:time(sim), MI_vals(5,1:time(sim),1,rob), rr_color(rob),...
                                         'linewidth', 2); hold on;
                                end
                            end
                            axis image; axis square; axis([0 time(sim) 0 2])
                            title(sprintf("tloc: %d, alpha: %.2f, Sim: %d", tloc, alpha(a), sim), 'fontsize', 8)
                            ylabel("MI", 'fontsize', 12);
                            xlabel("Time (s)", 'fontsize', 12)
                            step = step + 1;
                            loop_step = loop_step + 1;
                        end
                    end
                end
            end
            fignum = fignum + 1;
        end
    end
end
    
end

function sim_speed()
% -- read the simulation data
% -- by varying the parameters
agents = ["1", "2", "3"];
bias = ["1"];
alpha = ["0", "0.25", "0.5", "0.75", "1"];
target = ["1", "2", "3"];

% -- personalized colors for the robots
r_color = ["-b", "-r", "-g"];

fignum = 1;
for tloc = 1:size(target,2)
    for agent = 1:size(agents,2)
        for b = 1:size(bias,2)
            step = 1;
            for a = 1:size(alpha,2)
                
                % -- specify the directory of the .mat files
                % -- using the specified items in the nested loops
                % -- for ther simulation with omnly the human, the alpha
                % -- value is 1 (MI optimized for the target)
                if agents(agent) == "1" && str2double(alpha(a)) < 1
                    continue
                else
                    dir = append('data/plain_map/',agents(agent),'robots/alpha_',alpha(a),'/bias_',bias(b),...
                                 '/target_position',target(tloc),'/plain_map_p=600_nsim=10_agents=',agents(agent),'.mat');

                    % -- load the .mat file using the specified directory
                    load(dir);
                    
                    % -- get the total time it took to run the saimulation
                    % -- for every part of the simulation
                    % -- the variable will hold all times of the simulation
                    % -- per condition
                    time = cat(1, simdata.time);
                    
                    % -- get the number of simulations ran for the condition
                    NumberOfSim = size(simdata,2);
                    
                    % -- loop through each individual sim within a condition
                    for sim = 1:NumberOfSim
                        % -- Create a figure that corresponds to the target location number
                        % -- subplots for each figure are 5x5, with the
                        % -- columns being the alpha value and the rows are
                        % -- the sim number
                        figure(fignum);
                        subplot(size(alpha,2), NumberOfSim,step);
                        
                        % -- get the velocity and omega values of the robots
                        vel = simdata(sim).vel(:,1:time(sim),:,:); 
                        omega = simdata(sim).omega(:,1:time(sim),:,:); 
                        
                        % -- get the size of the MI_vals dimension
                        % -- we want to make sure it matches the agents
                        % -- number for that condition. When there are more
                        % -- robots in the simulation , the dimension of the
                        % -- array is 4-D
                        size_MI = size(vel,4);

                        % -- looping through for every robot in the sim
                        for rob = 1:size_MI
                            % -- the individual MI vals before weighting of
                            % -- the target
                            plot(1:time(sim), vel(1,1:time(sim),1,rob), r_color(rob),...
                                 'linewidth', 1); hold on;
                        end
                        
                        axis image; axis square; axis ([0 time(sim) 0 .9]);
                        title(sprintf("tloc: %d, alpha: %.2f, Sim: %d", tloc, alpha(a), sim), 'fontsize', 12)
                        ylabel("velocity (m/s)", 'fontsize', 12);
                        xlabel("Time (s)", 'fontsize', 12)
                        
                        figure(9+fignum);
                        subplot(size(alpha,2), NumberOfSim, step);
                        % -- looping through for every robot in the sim
                        for rob = 1:size_MI
                            % -- the individual MI vals before weighting of
                            % -- the target
                            plot(1:time(sim), omega(1,1:time(sim),1,rob), r_color(rob),...
                                 'linewidth', 1); hold on;
                        end
                        
                        axis image; axis square; axis ([0 time(sim) -1 1]);
                        title(sprintf("tloc: %d, alpha: %.2f, Sim: %d", tloc, alpha(a), sim), 'fontsize', 12)
                        ylabel("Omega (rad/s)", 'fontsize', 12);
                        xlabel("Time (s)", 'fontsize', 12)
                        
                        step = step + 1;
                    end
                end
            end
            fignum = fignum + 1;
        end
    end
end
    
end