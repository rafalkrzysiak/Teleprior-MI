function [pdata, x]=calc_pdf(dstr, xlbl, conditions)
% this function calculates the pdf and KL divergence

alldata=[dstr{:}];
edges=linspace(min(alldata), max(alldata), 10);

% calculate probabilities p(feature|knowledge)
for k=1:4
    data = dstr{k};
    [n,x]=histcounts(data, edges);
    pdata(:,k)= n/sum(n);
end

if nargin >2 % verbose option
    % normalize so that all values sum up to one
    load colorblind_colormap.mat
    % plot everything
    gca; cla;
    for k=1:4
        plot(x(1:end-1),pdata(:,k), 'color', ...
            colorblind(k,:), 'linewidth', 2);
        hold on;
    end
    grid on;
%     set(gca,'ylim',[0,1])
    set(gca, 'fontsize', 16);
    xlabel(xlbl)
    ylabel('p')
    legend(conditions)
    drawnow;
end

x=x(1:end-1); % because we need equal edges and p values
end