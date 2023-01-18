function [xc, yc] = draw_circle(X, r, r_FOV)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

beta = 0:pi/16:2*pi;
xc = r*cos(beta) + X(1);
yc = r*sin(beta) + X(2);

end

