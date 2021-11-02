% this will take raw trajectory data, filter it, store secondary measures
% as follows:
% divide filtered condition 4 data into 4a (prior to u turn) and 4b
% likely we'll have 15 4b trials with actual data the rest could be empty
% 1) EKFSpeed
% 2) EKFOmega
% 3) comSpeed
% 4) comOmega
% 5) timeToFind (no scaling)
% 6) pathLength (add successive differences in positions)
% 7) timeStayingInPlace (add the total number of timesteps where speed was
% less than 0.1 m/s)
% 8) 
% such as speed, turn rates, times, etc. into separate files, 
% all in the subject's folder for later analysis
