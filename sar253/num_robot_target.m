function X0 = num_robot_target(num_robots, num_targets)
X = zeros(3,1); Xt = zeros(2,1); % temporary place holders for robot and target
rx = 1; rth = 3; % initial positions of the robot state space
eta = [0 10];

% placing the ith robot into the state space
for i = 1:num_robots
    for j = 1:3
       X(j) = eta(i) + randn; 
    end
    X0(rx:rth,1) = X(1:3,1);
    X0(3*i,1) = randn;
    rx = rx + 3;
    rth = rth + 3; 
end

% X(3) = randn;
% X(6) = randn;

% placing the kth target into the state space after the robots
r_size = length(X0);
tx = r_size+1; ty = r_size+2;
for k = 1:num_targets
   for kk = 1:2
       Xt(kk) = 5+randn;
   end
   X0(tx:ty,1) = Xt(1:2,1);
   tx = tx + 2;
   ty = ty + 2;
end

