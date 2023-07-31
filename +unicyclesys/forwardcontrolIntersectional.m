% [X, K, vol] = forwardcontrolIntersectional(Pose, res) returns the critical boundary
% points and their convex hull that contains a 2D Unicycle kinematic robot motion
% controlled via forward position control. 
% Please refer to the reference page of convhull to learn more about K.
%
% Usage:
%   import unicyclesys.forwardcontrolIntersectional
%   [K, X, vol] = forwardcontrolIntersectional(Pose,res)
% Input:
%   Pose: Unicycle robot pose, (2+1) x 1 matrix
%   res: Number of samples of the circles, integer
% Output:
%   X: Lyapunov Circle Boundary points, (res) x 2 matrix
%   K: Boundary point indicies defining their convex hull, 1 x mline vector or mtri x 3 matrix 
%   vol: Convex hull volume 


function [X, K, vol] = forwardcontrolIntersectional(Pose, res)
% Author: Aykut Isleyen, isleyenaykut@gmail.com
% Created: June 15, 2022
% Modified:Aug 08, 2022

    phi = atan2(  ( -sin(Pose(3))*(0  - Pose(1)) +...
                     cos(Pose(3))*(0  - Pose(2))) ,...
                    (cos(Pose(3))*(0 - Pose(1)) +...
                     sin(Pose(3))*(0  - Pose(2))));


    if phi >= pi/2 || phi <= -pi/2
        [X, K, vol] = unicyclesys.forwardcontrolCircular(Pose, res);
    else
        % Determine critical points
        R = norm(Pose(1:2));
        th = linspace(Pose(3)-phi,Pose(3)+3*phi,res);
        arc = R*[cos(th') sin(th')];
        X = [arc; Pose(1), Pose(2)];
        [K, ~] = geom.convhulln(X);
        K = K(:,1)';
        vol = Inf;
    end

end