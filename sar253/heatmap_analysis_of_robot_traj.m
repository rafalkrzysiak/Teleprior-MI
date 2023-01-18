
figure(1); gcf; clf;
hist2d(simdata.Xh(1,:),simdata.Xh(2,:),5,'countdensity')
axis image; axis square;
xlabel('X(m)'); ylabel('Y(m)');
colorbar;