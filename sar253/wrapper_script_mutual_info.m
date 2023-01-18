function wrapper_script_mutual_info
% This function will serve as a more higher level code
% for the robot_nav_pf_mutual_info code.
% The function will accept arguements that will modify the following:
% 1. The number of particles in the estimate
% 2. The noise in the measurement, more specifrically range and bearing
% 3. whether or not the code is a closed or open loop system
% the Function will accept the arguements in the order stated above

particles = [500]; % number of particles to predict where robot is
etaz_r=[10]; % noise to add onto the measurement of the range
etaz_th=[pi/180]; % noise to add onto the measurement of the bearing
closed_loop = [1]; % boolean value for using the closed loop
dt = 1/2; % time step 
Tfinal = 25; % overall simulation time
sim = 0; % the simulation number
num_robots = [2]; % the number of robots present in the simulation
num_targets = [1]; % the number of targets present in the simulation
w = 1; % the disturbance in the motion model
omega = -1:.1:1;
v = 0:.1:1;
b = [1 5 10 15 20 25 30 35 40 45 50];
l = [1 5 10 15 20 25 30 35 40 45 50];

% begin running the simulation for every type of combination
for i = 1:numel(closed_loop)
    for j = 1:numel(particles)
       for k = 1:numel(etaz_r)
          for ii = 1:numel(etaz_th)
              for kk = 1:numel(b)
                  for jj = 1:numel(l)
                     % simulation number
                      sim = sim + 1

                      % append the noise of the range and bearing together
                      etaz = [etaz_r(k) etaz_th(ii)];

                      % putting everything into the simulation code
%                       robot_nav_pf_mutual_info(particles(j), etaz, closed_loop(i), dt, Tfinal)

                      % simulate the multi robot code
                      robot_nav_pf_multi_robot(particles(j), etaz, closed_loop(i), dt, Tfinal,...
                                               num_robots, num_targets, w, omega, v, b(kk), l(jj)) 
                  end
              end
          end
       end
    end
end

end

