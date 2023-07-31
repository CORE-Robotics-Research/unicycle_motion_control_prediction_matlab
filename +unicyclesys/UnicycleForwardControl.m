% Unicycle Forward Kinematic Control Class 
%
%   v = gain_0 max(0, \norm{\goal - \pos} \cos \phi)
%   w = gain_1 \phi   
%
%\phi = \angle (\begin{bmatrix}\cos\theta\\\sin\theta \end{bmatrix}, -x)
%
% UnicycleForwardControl.Gains = [gain_0, gain_1]
% Default Gains : [1 1]


classdef UnicycleForwardControl <  matlab.mixin.Copyable
% Authors: Aykut Isleyen, isleyenaykut@gmail.com, Omur Arslan, omurarslan.phd@gmail.com
% Created: June 15, 2022
% Modified: Aug 7, 2022


    % Object Properties
    properties
       Gains
    end

    % Private Object Properties
    properties(SetAccess='private')
       Dimension
       Order % System Order
    end

    properties
       MotionPolygonMethod 
       MotionPolygonResolution
    end
    
    
    methods 

        function obj = UnicycleForwardControl(Gains)
            switch nargin
                case 0
                    Gains = [1 1];
                otherwise 
                    % Do nothing
            end
            obj.Gains = Gains;
            obj.Dimension = 3; % 2 position + 1 orientation
            obj.Order = 1;
            obj.MotionPolygonMethod = "IceCreamCone";
            obj.MotionPolygonResolution = 60;
        end
    end


    % Object Functions for Motion Planning
    methods

        function  x = mat2vec(obj, X)
            x = reshape(X', [obj.Dimension, 1]);
        end
        
        function X = vec2mat(obj, x)
            X = reshape(x', [obj.Dimension,1])';
        end
    end

    % Object Functions - Trajectory and Motion Polygon Functions
    methods
        function dX = vectorfield(obj, x, goal)
            
            if (nargin < 3)
                goal = zeros(2,1);
            end
            [v, w] = unicyclesys.forwardcontrol(x, goal, obj.Gains);
            dx = v*cos(x(3));
            dy = v*sin(x(3));
            dtheta = w;
            dX = [dx;dy;dtheta];

        end
        
        function [T, X] = traj(obj, tspan, x0, options)
            % Numerically solves the system state trajectory using ode45
            
            if nargin < 4
                options = odeset();
            end
            [T, X] = ode45(@(t, x) obj.vectorfield(x), tspan, x0, options); 
        end
             
        function [T, X] = simulated_traj(obj, x0, dx, dth)
            % Solves the system trajectory using integration stepsizes
            [T, X] = unicyclesys.forwardcontrolSimulatedtraj(x0, [0 0], obj.Gains, dx, dth);

        end

        function P = motionpolygon(obj, x0, goal, varargin)
            
            if nargin < 3
                goal = zeros(1,2);
            end
            x0 = reshape(x0,1,3);
            goal = reshape(goal,1,2);

            x0(3) = x0(3) + 1e-8*(rand); %% UGLY Correction for convexhull
            x0(1:2) = x0(1:2) + 1e-8*(rand(size(x0(1:2)))); %% UGLY Correction for convexhull         
            x0(1:2) = x0(1:2) - goal;
            
            switch lower(obj.MotionPolygonMethod)
                case 'circular'
                    [V, F] = unicyclesys.forwardcontrolCircular(x0,obj.MotionPolygonResolution);
                    V = V + goal(1,:);
                    P = V(F(1,:),:);
                case 'simulated'
                    [V, F] = unicyclesys.forwardcontrolSimulated(x0, obj.Gains, 0.01, 0.01);
                    V = V + goal(1,:);
                    P = V(F(1,:),:);
                case 'intersectional'
                    [V, F] = unicyclesys.forwardcontrolIntersectional(x0,obj.MotionPolygonResolution);
                    V = V + goal(1,:);
                    P = V(F(1,:),:);
                case 'cone'
                    [V, F] = unicyclesys.forwardcontrolCone(x0, obj.MotionPolygonResolution);
                    V = V + goal(1,:);
                    P = V(F(1,:),:);
                case 'icecreamcone'
                    [V, F] = unicyclesys.forwardcontrolIcecreamCone(x0,obj.MotionPolygonResolution);
                    V = V + goal(1,:);
                    F = convhulln(V);
                    P = V(F(:,1),:);
                case 'truncatedicecreamcone'
                    [V, F] = unicyclesys.forwardcontrolTruncatedicecreamCone(x0,obj.MotionPolygonResolution);
                    V = V + goal(1,:);
                    for k = 1:size(F,1)
                        polyvec(k) = polyshape(V(F(k,~isnan(F(k,:))),:));
                    end
                        pout = union(polyvec);
                        P = pout.Vertices;
                            if isempty(P)
                                P = V(1,:);
                                warning('Union of polyshapes returns empty set!')
                            end
                otherwise
                    error('Unknown motion prediction method!');
            end
        end

        function P = augmentedmotionpolygon(obj, x0, RobotPolygon, goal)
            
            if nargin < 4
                goal = zeros(1,2);
            end
            x0 = reshape(x0,1,3);
            goal = reshape(goal,1,2);
            R = RobotPolygon;

            pos0 = x0(1:2);
            theta0= x0(3);
            theta0 = theta0 + 1e-8*(rand(size(theta0))); %% UGLY Correction for convexhull
            pos0 = pos0 + 1e-8*(rand(size(pos0))); %% UGLY Correction for convexhull

            phi0 = atan2(  (-sin(theta0)*(goal(1) - pos0(1)) +...
                cos(theta0)*(goal(2) - pos0(2))) ,...
               (cos(theta0)*(goal(1) - pos0(1)) +...
                sin(theta0)*(goal(2) - pos0(2))));
            
            pos0 = pos0 - goal;
            p = [pos0,theta0];
            switch lower(obj.MotionPolygonMethod)
                case 'circular'
                    [V, F] = unicyclesys.forwardcontrolCircular(p,obj.MotionPolygonResolution);
                    V = V + goal(1,:);
                    P = V(F(1,:),:);
                    [P1, P2] = geom.cvxpolysum(P(:,1), P(:,2), R(:,1), R(:,2));
                    P = [P1(:), P2(:)];
                case 'simulated'
                    [V, F] = unicyclesys.forwardcontrolSimulated(p, obj.Gains, 0.01, 0.01);
                    V = V + goal(1,:);
                    P = V(F(1,:),:);
                    for k = 1:size(F,2)
                        Polytemp = [P(k,1)+R(:,1), P(k,2)+R(:,2)];
                        polyvec(k) = polyshape(Polytemp);
                    end
                        pout = union(polyvec);
                        P = pout.Vertices;
                case 'intersectional'
                    [V, F] = unicyclesys.forwardcontrolIntersectional(p,obj.MotionPolygonResolution);
                    V = V + goal(1,:);
                    P = V(F(1,:),:);
                    [P1, P2] = geom.cvxpolysum(P(:,1), P(:,2), R(:,1), R(:,2));
                    P = [P1(:), P2(:)];
                case 'cone'
                    [V, F] = unicyclesys.forwardcontrolCone(p, obj.MotionPolygonResolution);
                    V = V + goal(1,:);
                    P = V(F(1,:),:);
                    [P1, P2] = geom.cvxpolysum(P(:,1), P(:,2), R(:,1), R(:,2));
                    P = [P1(:), P2(:)];
                case 'icecreamcone'
                    [V, F] = unicyclesys.forwardcontrolIcecreamCone(p, obj.MotionPolygonResolution);
                    V = V + goal(1,:);
                    P = V(F(1,:),:);
                    [P1, P2] = geom.cvxpolysum(P(:,1), P(:,2), R(:,1), R(:,2));
                    P = [P1(:), P2(:)];
                case 'truncatedicecreamcone'
                    [V, F] = unicyclesys.forwardcontrolTruncatedicecreamCone(p,obj.MotionPolygonResolution);
                    V = V + goal(1,:);
                    for k = 1:size(F,1)
                        P = V(F(k,~isnan(F(k,:))),:);
                        [P1, P2] = geom.cvxpolysum(P(:,1), P(:,2), R(:,1), R(:,2));
                        P = [P1(:), P2(:)];
                        polyvec(k) = polyshape(P);
                    end
                        pout = union(polyvec);
                        P = pout.Vertices;
                otherwise
                    error('Unknown motion prediction method!');
            end
        end
        
    end
    
    
end





