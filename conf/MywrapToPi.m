function z = MywrapToPi(bearing)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
z = bearing - 2*pi*floor((bearing+pi)/(2*pi));
end

