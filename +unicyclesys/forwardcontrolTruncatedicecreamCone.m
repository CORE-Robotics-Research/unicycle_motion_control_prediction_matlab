% [X, K, vol] = forwardcontrolTruncatedicecreamCone(Pose, phi, res) returns the critical boundary
% points and their convex hull that contains a 2D Unicycle kinematic robot motion
% controlled via forward position control. 
% Please refer to the reference page of convhull to learn more about K.
%
% Usage:
%   import unicyclesys.forwardcontrolTruncatedicecreamCone
%   [K, X, vol] = forwardcontrolTruncatedicecreamCone(P,res)
% Input:
%   Pose: Unicycle robot pose, (2+1) x 1 matrix
%   res: Number of samples of the circles, integer
% Output:
%   P: returns a polyshape object whose regions are the truncated ice cream
%   cone motion prediction. 


function [X, K, vol] = forwardcontrolTruncatedicecreamCone(Pose, res)
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
        % Determine critical boundary points
        XCone = zeros(3,2);
        XCone(2,:) = reshape(Pose(1:2),1,2);
        XCone(3,:) = XCone(2,:) + norm(Pose(1:2))*cos(phi)*[cos(Pose(3)),sin(Pose(3))];
        [KCone, vol] = convhulln(XCone);
        KCone = KCone(:,1)';
     
        [XCircle, KCircle, volCircle] = geom.shape.circleConvhull(abs(norm(Pose(1:2))*sin(phi)),...
            res);
        KCircle = KCircle + max(KCone);
        XCircle = reshape(XCircle,res,2);
        KCircle = KCircle(:,1)';

        KCone(size(KCone,2)+1:size(KCircle,2)) = nan;
        K = [KCone; KCircle];
        X = [XCone; XCircle];
        
        vol = vol + volCircle*(2*pi-pi/2+abs(phi))/(2*pi);
    end

end