function [ A ] = defineGeometry(alphad,r)
% return A matrix for geometry of curved surface gripper 
% alphad is angle contact points make with COM (defined by Elliot's FBD)
% ALPHA IS IN DEGREES
% r is radius of object

% Set up problem
t1 = [cosd(alphad); sind(alphad); - r];
t2 = [-cosd(alphad); sind(alphad); r];
c1 = [sind(alphad); -cosd(alphad); 0];
c2 = [-sind(alphad); -cosd(alphad); 0];
A =  [t1 t2 c1 c2];

end