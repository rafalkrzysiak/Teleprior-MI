load('data_12_11_19/closedloop=1_particles=500_etar=10.00_etath=0.0175.mat');

xdiff=Xh(1,:)-Xs(1,1:end-1);
ydiff=Xh(2,:)-Xs(2,1:end-1);

sr=2; % number of rows in the figure
sc=4; % number of columns in the figure
closed_loop = 1;
 
% plots the results of the estimated state of both
% the robot and the target in the inertial frame
figure(1); gcf; clf;
subplot(sr,sc,1);
plot(xdiff, ydiff, 'k.', 'markersize', 10);
grid on; grid minor; axis ([-2 1 -.5 1.5]); axis square;
ylabel('Y Diff'); xlabel('X Diff');
title('Estimated minus Simulated robot position');

subplot(sr,sc,2);
polarplot(wrapToPi(Xh(3,:)),tspan);
title('Oreintation vs time');
grid minor;

for ii=4:5
    subplot(sr,sc,ii-1); gca;
    errorbar(tspan, Xh(ii,:), squeeze(sqrt(P(ii,ii,:))), 'linewidth', 1);
    hold on;
    
    if ii == 4
        hold on;
        plot(tspan, Xs(ii,1:end-1), 'r'); % plot the true X position of the target 
        title('Simulated Target X position with estimate');
        set(gca, 'fontname', 'times', 'fontsize', 10);
        grid on; grid minor; axis image; axis square;
        ylabel('x_j'); xlabel('time (s)');
    end
    
    if ii == 5
        hold on;
        plot(tspan, Xs(ii,1:end-1), 'r'); % plot the true Y position of the target
        title('Simulated Target Y position with estimate');
        set(gca, 'fontname', 'times', 'fontsize', 10);
        grid on; grid minor; axis image; axis square;
        ylabel('y_j'); xlabel('time (s)');
    end
end

% plot the robots estimated path along with 
% it's time step position, start and end point and target position
subplot(sr,sc,5);
plot(Xs(1,:), Xs(2,:), 'r'); % Simulated
hold on; plot(Xh(1,:), Xh(2,:), 'b'); % Estimated 
hold on; plot(Xh(1,:), Xh(2,:), 'co', 'Markersize', 8); % Time step position of estimate 
hold on; plot(Xh(1,1), Xh(2,1), '^k', 'linewidth', 2, 'markersize', 16); % beginning point
hold on; plot(Xh(1,end), Xh(2,end), 'k.', 'linewidth', 5, 'markersize', 25); % stopping point
hold on; plot(Xs(4,:), Xs(5,:), 'sg', 'linewidth', 2, 'markersize', 10); % Target Position
hold on; plot(Xh(4,:), Xh(5,:), 'k.', 'markersize', 10); % estimated state of the target
axis image; axis square; xlabel('x_1'); ylabel('x_2');
grid on; grid minor;
set(gca, 'fontname', 'times', 'fontsize', 10);
legend('Simulated','Estimate','Time step position estimate',...
       'Robot Start', 'Robot End', 'Target Location','Estimated Target',...
       'location', 'southeast');
   
% plot the original target position and 
% the estimated state of the target position
subplot(sr,sc,6);
plot(Xs(4,:), Xs(5,:), 'sg', 'linewidth', 2, 'markersize', 12); % Target Position
hold on; plot(Xh(4,:), Xh(5,:), 'k.', 'markersize', 10); % estimated state of the target
axis ([4 7 4.5 5]); axis square; xlabel('x_j'); ylabel('y_j');
grid on; grid minor;
legend('Target True position', 'Estimated Target position');

% subplot(sr,sc,8);
% surf(v,omega,avg_mutual)
% xlabel('Velocity (m/s)'); ylabel('Omega (rad/s)');
% title('Avg Mutual Info'); axis image; axis square;
% colorbar; view(0,90);