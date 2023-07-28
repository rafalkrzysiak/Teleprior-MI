function [pdstr, xdstr]=calc_pdf_feature(tau, ids_train)
dtTrack=1/2;
dtCommand=1/10;
% make sure the feature here matches the one we are computing in
% infoseek2Db_multi_robot.m before calling updateAlpha
dstr_dist=extract_dist_data(tau, ids_train, ...
        dtTrack, dtCommand); % distance

[pdstr, xdstr]=calc_pdf(dstr_dist); % pass one argument only 