% [V, F, vol] = forwardcontrolSimulated(Pose, gains, deltax, stopCond) returns the 
% simulated trajectory as a motion prediction polygon with 0 volume
% for the unicycle robot with 2D Unicycle kinematic 
% robot motion controlled via forward position control. 
%
% Usage:
%   import unicyclesys.forwardcontrolSimulated
%   [V, F, vol] = forwardcontrolForwardSimulated(Pose, gains, deltax, stopCond))
% Input:
%   Pose: Unicycle robot pose, (2+1) x 1 matrix
%   gains: Control input gains for the forward motion control
%   dx: Simulation spatial step size
%   dth: Simulation angular step size
% Output:
%   V: Vertices of the finite sized subset of the motion prediction
%   polygon.
%   F: Faces of the motion prediction polygon.
%   vol: Volume of the motion prediction



function [V, F, vol] = forwardcontrolSimulated(Pose, gains, dx, dth)
% Author: Aykut Isleyen, a.isleyen@tue.nl
% Created: February 14, 2023

    vol = 0;
    Pose = reshape(Pose, [], 3);
    goal = [0 0];

    [T, X] = unicyclesys.forwardcontrolSimulatedtraj(Pose, goal, gains, dx, dth);

    V = [X(:,1:2); flipud(X(1:end-1,1:2))];
    F = [1:(numel(V)/2)];

end