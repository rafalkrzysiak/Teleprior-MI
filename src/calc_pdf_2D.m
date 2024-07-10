function [pdata, xe, ye]=calc_pdf_2D(dstr1, dstr2, xlbl, conditions)
% this function calculates the pdf and KL divergence

alldata=[dstr1{:}; dstr2{:}];
Xedges=linspace(min(alldata(1,:)), max(alldata(1,:)), 10);
Yedges=linspace(min(alldata(2,:)), max(alldata(2,:)), 10);

% calculate probabilities p(feature|knowledge)
for k=1:4
    [n,xe,ye]=histcounts2(dstr1{k}, dstr2{k}, Xedges, Yedges);
    pdata(:,:,k)= n/sum(n(:));
end

if nargin >2 % verbose option
    % plot everything
    gca; cla;
    for k=1:4
        subplot(2,2,k);
        imagesc(xe(1:end-1), ye(1:end-1), pdata(:,:,k));
        title(conditions(k));   
        grid on;

        set(gca, 'fontsize', 16);
        xlabel('distance')
        ylabel('freezing time')
    end
    drawnow;
end

xe=xe(1:end-1); % because we need equal edges and p values
ye=ye(1:end-1);
end