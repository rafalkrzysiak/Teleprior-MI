function [X0, L] = robot_position(maps, target_loc)

% -- X represents combined state
% -- robot location/orientation and target location

if maps == "basic_map"
    L= [30 30]; % -- length, width of domain size (m),
    X0(:,:,1,1) = [5 36 -pi/2 target_loc(1) target_loc(2)]'; % basic map
    X0(:,:,1,2) = [3 18 -pi/4 target_loc(1) target_loc(2)]'; % basic map
    X0(:,:,1,3) = [7 18 -pi/2 target_loc(1) target_loc(2)]'; % basic map
elseif maps == "plain_map"
    L= [30 30]; % -- length, width of domain size (m),
    X0(:,:,1,1) = [4 5 0*pi/180 target_loc(1) target_loc(2)]'; % basic map
    X0(:,:,1,2) = [2 3 0 target_loc(1) target_loc(2)]'; % basic map
    X0(:,:,1,3) = [2 6 pi/4 target_loc(1) target_loc(2)]'; % basic map
elseif maps == "u_map"
    L= [30 30]; % -- length, width of domain size (m),
    X0(:,:,1,1) = [3 17 3*pi/2 target_loc(1) target_loc(2)]'; % u map
    X0(:,:,1,2) = [5 18 3*pi/2 target_loc(1) target_loc(2)]'; % u map
    X0(:,:,1,3) = [15 5 -pi/2 target_loc(1) target_loc(2)]'; % u map
elseif maps == "complex_map"
    L= [30 30]; % -- length, width of domain size (m),
    X0(:,:,1,1) = [4 17 0 target_loc(1) target_loc(2)]'; % complex_map
    X0(:,:,1,2) = [3 16 0 target_loc(1) target_loc(2)]'; % complex_map
    X0(:,:,1,3) = [2 18 0 target_loc(1) target_loc(2)]'; % complex_map
elseif maps == "OmronLab"
    L= [18 9]; % -- length, width of domain size (m), WIDTH ALWAYS LESS!
%     X0(:,:,1,1) = [10 4 pi/2 target_loc(1) target_loc(2)]'; % complex_map
     X0(:,:,1,1) = [8 4 -pi+rand*2*pi target_loc(1) target_loc(2)]'; % complex_map
     X0(:,:,1,2) = [7 4 -pi+rand*2*pi target_loc(1) target_loc(2)]'; % complex_map
     X0(:,:,1,3) = [9 4 -pi+rand*2*pi target_loc(1) target_loc(2)]'; % complex_map
     X0(:,:,1,4) = [12 4 -pi+rand*2*pi target_loc(1) target_loc(2)]'; % complex_map
     X0(:,:,1,5) = [8 4 -pi+rand*2*pi target_loc(1) target_loc(2)]'; % complex_map
end

end

