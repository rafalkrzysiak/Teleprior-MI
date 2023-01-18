figure(1); gcf; clf;
X = table2array(X1);
Xh = table2array(Xh1);

wp = [30,30;
      152,30;
      152,213;
      30,213];
  
f = [30,0;
     182,30;
     152,240;
     0,212];
 
r_robot = 15;

i = 0; % -- counter to enter the if condition in the for loop
j = 1; % -- frame number for live feed
figure(1); gcf;
subplot(2,4,7:8);



for n = 1:2120
    figure(1); gcf;
    
    % -- display the overhead camera frames
    img = imread(sprintf('frames/%04d.jpg',n));
    subplot(2,4,5:6);
    imshow(img);
    
    % -- display the live view from robots perspective
    % -- and the estimate of the robot location
    if rand > .65
        
        % -- display the robot's live feed
        if j <= 743
           live = imread(sprintf('live_feed/%04d.jpg',j));
           subplot(2,4,2:3);
           imshow(live);
        end
        
        % -- plot the estimate of the robots position
        subplot(2,4,7:8);
        map = ones(202,260,3);
        binmap = imbinarize(map);
        imagesc([-10 192],[-10 250],binmap);
        plot(wp(:,1), wp(:,2), 'b+', 'markersize', 10, 'linewidth', 2); hold on;
        plot(f(:,1), f(:,2), 'sk', 'markersize', 10, 'linewidth', 2);
        [xr, yr] = draw_circle(Xh(j,:), r_robot); % -- robot size
        patch(xr,yr,'r','facealpha',.5, 'edgecolor', 'none'); % visualization of robot body
        line([Xh(j,1), Xh(j,1)+20*cos(Xh(j,3))],...
             [Xh(j,2), Xh(j,2)+20*sin(Xh(j,3))],...
             'color','r','linestyle','-')
        plot(X(1:j,1), X(1:j,2), 'k.', 'markersize', 5);
        plot(Xh(1:j,1), Xh(1:j,2), 'r.', 'markersize', 5)
        set(gca,'xdir','reverse')
        axis image; axis([-10 192 -10 250])
        xlabel('X(cm)', 'Fontsize', 16); ylabel('Y(cm)', 'Fontsize', 16)
        view([-90 90]);
        j = j+1; % -- move to the next frame of the live feed
        %i = 0;
    end
    drawnow;
    thisfile = sprintf('images/%04d.jpg', n);
    saveas(gcf, thisfile);
end