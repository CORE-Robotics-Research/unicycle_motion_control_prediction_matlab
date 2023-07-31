classdef GovernedReferencePlanner <  matlab.mixin.Copyable
    
    properties
        Robot
        ReferencePlanner
        GovernorGain
    end
    
    methods
        function obj = GovernedReferencePlanner(Robot, Planner)
            obj.Robot = Robot;
            obj.ReferencePlanner = Planner;
            obj.GovernorGain = 1;
        end
        
        function [dx, dy] = vectorfield(obj, x, y)
            x = obj.Robot.Control.vec2mat(x); 
            goal = [reshape(y, 1, []); zeros(obj.Robot.Control.Order-1, obj.Robot.Control.Dimension)];
            
            dx = obj.Robot.Control.vectorfield(x, goal);
            
            s = max(obj.safetylevel(x, y), 0);
            dy = obj.ReferencePlanner.vectorfield(y');

            if norm(dy) > s
                dy = s*dy/norm(dy);
            end
            dy = obj.GovernorGain * dy;
             
        end
        
        function dxy = vectorfield2(obj, xy)
            x = xy(1:(obj.Robot.Order*obj.Robot.Dimension));
            y = xy((obj.Robot.Order*obj.Robot.Dimension + 1):end);
            [dx, dy] = obj.vectorfield(x, y);
            dxy = [obj.Robot.Control.mat2vec(dx); dy(:)];
        end
        
        function [T, X, Y] = ode45(obj, tspan, x0, y0, varargin)
            x0 = obj.Robot.Control.mat2vec(x0);
            y0 = y0(:);
            xy0 = [x0; y0];
            [T, XY] = ode45(@(t, xy) obj.vectorfield2(xy), tspan, xy0, varargin{:});
            X = XY(:, 1:(obj.Robot.Order*obj.Robot.Dimension));
            Y = XY(:,(obj.Robot.Order*obj.Robot.Dimension + 1):end);
        end
        
        function P = motionpolygon(obj, x, y)
            x = obj.Robot.Control.vec2mat(x); 
            goal = [reshape(y, 1, []); zeros(obj.Robot.Control.Order-1, obj.Robot.Control.Dimension)];
            
            P = obj.Robot.Control.motionpolygon(x, goal);
        end
        
        function P = augmentedmotionpolygon(obj, x, y)
            x = obj.Robot.Control.vec2mat(x); 
            goal = [reshape(y, 1, []); zeros(obj.Robot.Control.Order-1, obj.Robot.Control.Dimension)];
            R = obj.Robot.BodyPolygon;
            P = obj.Robot.Control.augmentedmotionpolygon(x,R,goal);

        end
        
        function s = safetylevel(obj, x, y)
   
            x = obj.Robot.Control.vec2mat(x); 
            goal = [reshape(y, 1, []); zeros(obj.Robot.Control.Order-1, obj.Robot.Control.Dimension)];
            
            P = obj.Robot.Control.motionpolygon(x, goal);
            s = obj.ReferencePlanner.Workspace.polydist2coll(P) - obj.Robot.BodyRadius;
            
        end
        
        
    end
    
    
end