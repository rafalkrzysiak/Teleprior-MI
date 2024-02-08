for k = 1:2
    figure(1); gcf; clf;
%     for i = 1:size(p,3)-1
    for i = 1:300
        imagesc([0 20],[0 20],img); hold on;
        
        plot(simdata(k).Xs(1,i,1,1),simdata(k).Xs(2,i,1,1), 'k');
        hold on; plot(simdata(k).Xs(1,i,1,2),simdata(k).Xs(2,i,1,2), 'k');
        
        plot(simdata(k).Xs(4,i),simdata(k).Xs(5,i),'sr',...
                'linewidth', 2, 'markersize', 20);
            
            
        plot(simdata(k).Xs(1,1,1,1),simdata(k).Xs(2,1,1,1),'sb',...
            'linewidth', 2, 'markersize', 20);
        plot(simdata(k).Xs(1,1,1,2),simdata(k).Xs(2,1,1,2),'sb',...
            'linewidth', 2, 'markersize', 20);

        plot(simdata(k).p(1,:,i,1), simdata(k).p(2,:,i,1), 'b.', 'markersize', 12); % particle state of robot
        plot(simdata(k).p(1,:,i,2), simdata(k).p(2,:,i,2), 'b.', 'markersize', 12); % particle state of robot
        
        plot(simdata(k).p(4,:,i,1), simdata(k).p(5,:,i,1), 'r.', 'markersize', 12); % particle state of target
        plot(simdata(k).p(4,:,i,2), simdata(k).p(5,:,i,2), 'm.', 'markersize', 12); % particle state of target
%         plot(simdata(k).Xh(4,i), simdata(k).Xh(5,i), '+g', 'linewidth', 4,'markersize', 20); % estimate of target

        [xc1, yc1] = draw_circle(simdata(k).Xs(1:2,i,1,1), r_visible);
        [xc2, yc2] = draw_circle(simdata(k).Xs(1:2,i,1,2), r_visible);
        
        [xr1, yr1] = draw_circle(simdata(k).Xs(1:2,i,1), .3);
        [xr2, yr2] = draw_circle(simdata(k).Xs(1:2,i,2), .3);
        
        plot(simdata(k).Xs(1,i,1,1), simdata(k).Xs(2,i,1,1), 'ks', 'markersize', 16); % simulated robot
        plot(simdata(k).Xs(1,i,1,2), simdata(k).Xs(2,i,1,2), 'ks', 'markersize', 16); % simulated robot
        
        plot(simdata(k).Xs(4,i,1,1), simdata(k).Xs(5,i,1,1), 'rs', 'markersize', 16); % simulated target
        plot(simdata(k).Xs(4,i,1,2), simdata(k).Xs(5,i,1,2), 'rs', 'markersize', 16); % simulated target
        
        plot(simdata(k).Xh(1,i,1,1), simdata(k).Xh(2,i,1,1), 'ks', 'markersize', 20); % estimate of robot
        plot(simdata(k).Xh(1,i,1,2), simdata(k).Xh(2,i,1,2), 'ks', 'markersize', 20); % estimate of robot
        
        patch(xc1,yc1,'k','facealpha',.15, 'edgecolor', 'none');
        patch(xc2,yc2,'k','facealpha',.15, 'edgecolor', 'none');
        
        plot(xr1, yr1, 'c', 'linewidth', 3); % visualization of robot visual range
        plot(xr2, yr2, 'c', 'linewidth', 3); % visualization of robot visual range
        
        pause(.25); axis image; axis ([0 20 0 20]);
        xlabel('X(m)','FontSize',24); ylabel('Y(m)','FontSize',24);
        set(gca,'xticklabel',[]); set(gca,'yticklabel',[]);
        disp(sprintf('time step: %02d%03d', k, i));
        
        saveas(gcf,sprintf('images/film/%02d%03d.png', k, i));
        clf; 
        if k == 1 && i == 215
            break
        end
        
        if k == 2 && i == 85
            break
        end
    end
end

function [xc, yc] = draw_circle(X, r)

beta = 0:pi/16:2*pi;
xc = r*cos(beta) + X(1);
yc = r*sin(beta) + X(2);
end