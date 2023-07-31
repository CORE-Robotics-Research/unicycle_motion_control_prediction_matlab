% [X, K, vol] = forwardcontrolCircular(Pose, res) returns the critical boundary
% points and their convex hull that contains a 2D Unicycle kinematic robot motion
% controlled via forward position control. 
% Please refer to the reference page of convhull to learn more about K.
%
% Usage:
%   import unicyclesys.forwardcontrolCircular
%   [K, X, vol] = forwardcontrolCircular(Pose,res)
% Input:
%   Pose: Unicycle robot pose, (2+1) x 1 matrix
%   res: Number of samples of the circles, integer
% Output:
%   X: Lyapunov Circle Boundary points, (res) x 2 matrix
%   K: Boundary point indicies defining their convex hull, 1 x mline vector or mtri x 3 matrix 
%   vol: Convex hull volume 


function [X, K, vol] = forwardcontrolCircular(Pose, res)
% Author: Aykut Isleyen, isleyenaykut@gmail.com
% Created: June 15, 2022
% Modified:Aug 08, 2022

    [X, K, vol] =  geom.shape.circleConvhull(norm(Pose(1:2)), res);
    K = K(:,1)';
end