classdef PathPursuitPlanner <  matlab.mixin.Copyable
    
    properties
       Workspace
       Path
       SafetyMargin = 0;
       ControlGain = 1.0;
    end
    
    % Visualization Options
    properties
        PatchResolution = 200;
        QuiverResolution = 60;
    end
    
    methods
        function obj = PathPursuitPlanner(Workspace, Path)
           obj.Workspace = Workspace;
           obj.Path = Path;
        end
        
        function I = indomain(obj, X)
            
            collDist = obj.Workspace.dist2coll(X) - obj.SafetyMargin;
            pathDist = obj.Path.dist(X);
            I = (pathDist <= collDist);
            
        end


        function Y = goal(obj, X)
            
            r = max(obj.Workspace.dist2coll(X) - obj.SafetyMargin, 0);
            Y = obj.Path.goal(X, r);                    
            
        end
        
        function dX = vectorfield(obj, X)
            
            Y = obj.goal(X);
            dX = -obj.ControlGain *(X - Y); 
            
        end
        
        function [T, X] = ode45(obj, tspan, x0, varargin)
            [T, X] = ode45(@(t, x) obj.vectorfield(x')', tspan, x0, varargin{:});
        end
        
        
        function h = plotPath(obj, varargin)
            h = plot(obj.Path, [], varargin{:});
        end
        
        function h = plotDomain(obj, varargin)
            XLim = obj.Workspace.XWorldLimits;
            YLim = obj.Workspace.YWorldLimits; 
            [X, Y] = meshgrid(linspace(XLim(1), XLim(2), obj.PatchResolution),...
                               linspace(YLim(1), YLim(2), obj.PatchResolution));
            XY = [X(:), Y(:)];
            IN = obj.indomain(XY); 
            IN = reshape(IN, size(X));
            [B, L] = bwboundaries(IN, 8, 'noholes');
            I = sub2ind(size(X), B{1}(:,1), B{1}(:,2));
            h = patch('XData', X(I), 'YData', Y(I), varargin{:});
        end
        
        function h = plotField(obj, varargin)
            XLim = obj.Workspace.XWorldLimits;
            YLim = obj.Workspace.YWorldLimits; 

           resolution = max(diff(XLim), diff(YLim))/obj.QuiverResolution;            
            [X, Y] = meshgrid(linspace(XLim(1), XLim(2), round(diff(XLim)/resolution)),...
                             linspace(YLim(1), YLim(2),  round(diff(YLim)/resolution)));
            XY = [X(:), Y(:)];
            IN = obj.indomain(XY); 
            XY = XY(IN,:);

            UV = obj.vectorfield(XY);
            UV = UV./sqrt(sum(UV.^2,2));

            h = quiver(XY(:,1), XY(:,2), UV(:,1), UV(:,2), varargin{:});
        end
        
        
        function h = plot(obj, varargin)
            
            % Default Settings
            quiverResolution = 60;
            patchResolution = 300;
            plotOptions = {'Color', 'b', 'LineWidth', 1};
            quiverOptions = {0.5 , 'Color', 'k', 'LineWidth', 2.5};
            patchOptions = {'FaceColor', 'b', 'FaceAlpha', 0.5, 'EdgeColor', 'none'};
            for k = 1:2:length(varargin)
                switch lower(varargin{k})
                    case 'quiverresolution'
                        quiverResolution = varargin{k+1};
                    case 'patchresolution'
                        patchResolution = varargin{k+1};
                    case 'plotoptions'
                        plotOptions = varargin{k+1};
                    case 'quiveroptions'
                        quiverOptions = varargin{k+1};
                    case 'patchoptions'
                        patchOptions = varargin{k+1};
                    otherwise
                        error('Unknown plot option');
                end
            end

%            resolution = max(diff(obj.XLimits), diff(obj.YLimits))/resolution;
%            domain_resolution = max(diff(obj.XLimits), diff(obj.YLimits))/domain_resolution;
           
           XLim = obj.Workspace.XWorldLimits;
           YLim = obj.Workspace.YWorldLimits; 
           %XLim = obj.XLimits + (inflationRadius + 1e-6*diff(obj.XLimits))*[1, -1];
           %YLim = obj.YLimits + (inflationRadius + 1e-6*diff(obj.YLimits))*[1, -1];
           
           [X, Y] = meshgrid(linspace(XLim(1), XLim(2), quiverResolution),...
                             linspace(YLim(1), YLim(2), quiverResolution));
           XY = [X(:), Y(:)];
           IN = obj.indomain(XY); 
           XY = XY(IN,:);

           UV = obj.vectorfield(XY);
           UV = UV./sqrt(sum(UV.^2,2));
           
           h.quiver = quiver(XY(:,1), XY(:,2), UV(:,1), UV(:,2), quiverOptions{:});
            
           [X2, Y2] = meshgrid(linspace(XLim(1), XLim(2), patchResolution),...
                               linspace(YLim(1), YLim(2), patchResolution));
           XY2 = [X2(:), Y2(:)];
           IN = obj.indomain(XY2); 
           IN = reshape(IN, size(X2));
           [B, L] = bwboundaries(IN, 8, 'noholes');
           I = sub2ind(size(X2), B{1}(:,1), B{1}(:,2));
           h.patch = patch('XData', X2(I), 'YData', Y2(I), patchOptions{:});
           h.plot = plot(obj.Path, plotOptions{:});
   
        end
    end
    
    
    
    
    
end