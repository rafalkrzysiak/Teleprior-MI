function [pdata, x]=calc_pdf(dstr, xlbl, conditions)
% this function calculates the pdf and KL divergence

alldata=[dstr{:}];
edges=linspace(min(alldata), max(alldata), 10);

% calculate probabilities
for k=1:4
    data = dstr{k};
    [n,x]=histcounts(data, edges);
    pdata(:,k)= n/sum(n);
end

if nargin >2 % quiet option
    % plot everything
    gca; cla;
    for k=[1,4]
        plot(x(1:end-1),pdata(:,k), 'linewidth', 2);
        hold on;
    end
    grid on;
    set(gca,'ylim',[0,1])
    set(gca, 'fontsize', 16);
    xlabel(xlbl)
    ylabel('p')
    legend(conditions([1,4]))
    drawnow;
end

x=x(1:end-1); % because we need equal edges and p values
end