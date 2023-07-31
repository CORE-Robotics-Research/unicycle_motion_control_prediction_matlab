% [X, K, vol] = forwardcontrolCone(Pose, phi, res) returns the finite
% subset of the motion prediction cone with infinite volume for the
% unicycle robot with 2D Unicycle kinematic robot motion
% controlled via forward position control. 
%
% Usage:
%   import unicyclesys.forwardcontrolCone
%   [K, X, vol] = forwardcontrolCone(P,res)
% Input:
%   Pose: Unicycle robot pose, (2+1) x 1 matrix
%   res: Number of samples of the circles, integer
% Output:
%   X: Vertices of the finite sized subset of the motion prediction
%   polygon.
%   K: Faces of the motion prediction polygon.
%   vol: Volume of the motion prediction



function [X, K, vol] = forwardcontrolCone(Pose, res)
% Author: Aykut Isleyen, isleyenaykut@gmail.com
% Created: June 15, 2022
% Modified:Aug 08, 2022


%   h: height of the cone (in principle that should be infinite
    h = 50;
    phi = atan2(  ( -sin(Pose(3))*(0  - Pose(1)) +...
                     cos(Pose(3))*(0  - Pose(2))) ,...
                    (cos(Pose(3))*(0 - Pose(1)) +...
                     sin(Pose(3))*(0  - Pose(2))));


    if phi >= pi/2 || phi <= -pi/2
        [X, K, vol] = unicyclesys.forwardcontrolCircular(Pose, res);
    else
        % Determine critical points
        X = zeros(3,2);
        X(1,:) = reshape(Pose(1:2),1,2);
        X(2,:) = X(1,:) + h*[cos(Pose(3)),sin(Pose(3))];
        X(3,:) = X(1,:) + h*[cos(Pose(3) + 2*phi),sin(Pose(3) + 2*phi)];
        [K, ~] = geom.convhulln(X);
        K = K(:,1)';
        vol = Inf;
    end
        
end