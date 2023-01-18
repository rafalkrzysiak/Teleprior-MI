sim_num = 2;
time_end = [188 151 238 324 136 203 133 108];
jj_num = [1 1 6 1 2 1 1 1];
robots = 2;
rb_color = ["bs", "rs", "gs"]; % -- store the robot color for its beginning position
rp_color = ["b.", "r.", "g."]; % -- store the robot color for its beginning position
re_color = ["xb","xr","xg"]; % -- store the robot color for its end position
tp_color = ["g.", "m."]; % -- store hte target particle color state
te_color = ["+b", "+r", "+g"]; % -- store the target estimate color state

figure(1); clf; gcf; 
imagesc([0 20],[0 20],img); hold on;
set(gca,'ydir','normal');

for robot = 1:robots
    jj = jj_num(sim_num);
    t_end = time_end(sim_num);
    % -- plot the estimated robot trajectory, and the robot simulated trajectory
    plot(simdata(jj).Xs(1,:,1,robot),simdata(jj).Xs(2,:,1,robot), 'k');
    hold on; plot(simdata(jj).Xh(1,1:t_end,1,robot), simdata(jj).Xh(2,1:t_end,1,robot), rp_color(robot), 'markersize', 5);

    % -- mark down where the robots began and ended, their estimates!
    hold on; plot(simdata(jj).Xh(1,1,1,robot),simdata(jj).Xh(2,1,1,robot), rb_color(robot), 'linewidth', 5,'markersize', 10);
    hold on; plot(simdata(jj).Xs(1,end,1,robot),simdata(jj).Xs(2,end,1,robot), re_color(robot), 'linewidth', 5, 'markersize', 20);
    hold on; plot(simdata(jj).tloc(1,1), simdata(jj).tloc(1,2), 'sm', 'markersize', 16, 'linewidth', 2);
    
    axis image; axis square;

end

clear; clc;