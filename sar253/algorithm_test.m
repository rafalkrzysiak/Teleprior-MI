% this script tests

% read the map from the folder
img = imread('basic_map.jpg');
% img = imbinarize(img);
thresh = graythresh(img); % use if using matlab 2016 or lower
img=im2bw(img,thresh); % use if using matlab 2016 or lower

% X represents combined state
% robot location/orientation and target location
X0 = [5 18 3*pi/2 18 6]'; 
dt=.5; % time step
T=25; % total simulation time
r_visible = 3; % visible range of the robot
eta=.5; % ######### Changing eta value 
N=500; % Number of particles
nsim=1; % number of simulations when debug mode isn't enabled
closedloop=1; % enable or disable opimization of mutual information
debug=0; % enable or disable debug mode
w0=.35;

p(:,:,1)=[X0(1)+randn(1,N)*w0;
          X0(2)+randn(1,N)*w0;
          X0(3)+randn(1,N)*w0;
          rand(1,N)*20;
          rand(1,N)*20]; 

XX = 5:.5:8;
for k = 1:numel(XX)
    [xGrid,yGrid] = meshgrid(1:1:410,1:1:410);
    BW = ones(410,410,1);
    BW(:,:,1) = sqrt((xGrid - XX(k)*(410/20)).^2 + (yGrid - XX(k)*(410/20)).^2) <= r_visible*(410/20);
    BW(:,:,1) = ~BW(:,:,1)';
    bin_map = BW(:,:,1).*img(:,:,1);

    particle_plane = zeros(size(bin_map,1),size(bin_map,2));
    % >> convert from actual value to Z

    % ^^ noise values these should be changed depending on the measurement
    % model above
    wts=ones(1,size(p,2));
    wts_h = zeros(1,size(p,2));

    % get the particle layout within the image plane (meter to pixels)
    p_hold = round(p(end-1:end,:,1).*(size(particle_plane,1)/20));

    % place a 1 for every particle position of the target
    for kk = 1:size(p_hold,2)
        aa = abs(p_hold(1,kk));
        bb = abs(p_hold(2,kk));

        if aa < 10
            aa = 10;
        end

        if aa > 400
            aa = 400;
        end

        if bb < 10
            bb = 10;
        end

        if bb > 400
            bb = 400;
        end

        particle_plane(aa,bb) = 1;
    end

    p_location = particle_plane.*bin_map;
    [px, py] = find(p_location(:,:,1) == 1);
    combined_p = [px, py]';

    for prt = 1:size(p,2)
       for cp = 1:size(combined_p,2)
           if p_hold(:,prt) == combined_p(:,cp)
               wts_h(1,prt) = 1;
           end
       end
    end

    wts(1,:) = wts(1,:).*wts_h(1,:);

    p(end-1,:,1) = p(end-1,:,1).*wts(1,:);
    p(end,:,1) = p(end,:,1).*wts(1,:);

    figure(1); gcf; clf;
    subplot(1,2,1);
    imagesc([0 20],[0 20],bin_map); hold on;
    plot(p(end-1,:,1),p(end,:,1), 'r.', 'Markersize', 10);
    grid on; grid minor;
    set(gca,'ydir','normal');
    axis image;

    subplot(1,2,2);
    plot(p(end-1,:,1),p(end,:,1), 'r.', 'Markersize', 10);
    axis image;
    pause(1);
end