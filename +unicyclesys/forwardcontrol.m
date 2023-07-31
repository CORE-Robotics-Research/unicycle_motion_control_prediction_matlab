% [V, F, vol] = forwardcontrol(Pose, goal, gains) 
% returns the linear and angular velocity control inputs
% for the unicycle robot with 2D Unicycle kinematic 
% robot motion controlled via forward position control. 
%
% Usage:
%   import unicyclesys.forwardcontrol
%   [v, w] = forwardcontrol(Pose, goal, gains)
% Input:
%   Pose: Unicycle robot pose, (2+1) x 1 array
%   goal: Goal position for the unicycle robot, 2 x 1 array
%   gains: Control input gains for the forward motion control
% Output:
%   v: linear velocity control input, float
%   w: angular velocity control input, float

function [v, w] = forwardcontrol(Pose, goal, gains)


    Pose = reshape(Pose, [], 1);
    goal = reshape(goal, [], 1);

    x = Pose(1:2);
    theta = Pose(3);


    v = gains(1) * max(0, [cos(theta) sin(theta)]*(goal-x));
    w = gains(2) * atan2( [-sin(theta) cos(theta)]*(goal-x) , [cos(theta) sin(theta)]*(goal-x) );
    
end