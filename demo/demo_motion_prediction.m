% Demo for unicycle feedback motion predictions that bound the closed-loop 
% unicycle motion trajectory towards a given goal position: motion ball, 
% bounded motion cone, unbounded motion cone, ice-cream motion cone, 
% and truncated ice-cream motion cone.

% Change settings.predictionMethod to test different motion prediction
% methods:
% 0: Forward-Simulation, 1: Circle, 2: Bounded Cone
% 3: Unbounded Cone, 4: Icecream Cone, 5: Truncated Icecream Cone

%% Start  a clean simulation environment
close all % Close all figures
clear variables % Clear all imports and variables
rng(0); % Fix random number generator seed

%% Scenario Settings

% 0: Forward-Simulation, 1: Circle, 2: Bounded Cone
% 3: Unbounded Cone, 4: Icecream Cone, 5: Truncated Icecream Cone
settings.predictionMethod = 1;

% Augment the motion prediction with the robot shape
augmented = false;

saveFigure = false;


settings = advanced_settings(settings);
myrobot = unicyclesys.UnicycleForwardControl();
myrobot.Gains = settings.gain;
myrobot.MotionPolygonMethod = settings.MotionPolygonMethod;
myrobot.MotionPolygonResolution = settings.MotionPolygonResolution;
x0  = settings.x0;
goal = settings.goal;
dx = 0.1;
dth = pi/60;


%% Demo Settings
tspan = [0 10]; 
odeOptions = odeset('MaxStep', 0.1, 'RelTol', 1e-3, 'AbsTol', 1e-6);
[T, X] = myrobot.traj(tspan, x0, odeOptions);



P = myrobot.motionpolygon(x0, goal);
if augmented
    P = myrobot.augmentedmotionpolygon(x0, settings.RobotPolygon, goal);
end
%% Motion Visualization
h.figure = figure(settings.figure{:});
axes(settings.axes{:});

patch('XData', P(:,1), 'YData',P(:,2), settings.motionPatch{:});
plot(X(:,1), X(:,2), settings.pathPlot{:});
scatter(goal(1), goal(2), settings.scatter{:});

plot([X(1,1) goal(1)] ,[X(1,2) goal(1)], settings.dirPlot{:});


% Robot Visualization
th = linspace(0,2*pi,100); % Angle samples for the visualization of robot body 
thwl = linspace(5*pi/6, pi/6, 60); % Angle samples for the visualization of the left robot wheel
thwr = linspace(7*pi/6, 11*pi/6, 60); % Angle sample for the visualization of the right robot wheel
R = settings.RobotRadius ;
patch('XData', X(1,1) + R*cos(th),'YData', X(1,2) + R*sin(th), 'FaceColor',  [0 1 1]);
patch('XData', X(1,1) + R*cos(thwl+X(1,3)), 'YData',  X(1,2) + R*sin(thwl+X(1,3)), 'FaceColor', 'k');
patch('XData', X(1,1) + R*cos(thwr+X(1,3)), 'YData',  X(1,2) + R*sin(thwr+X(1,3)), 'FaceColor', 'k');
thick = settings.DirectionThickness;
thdir = [ pi/2, -pi/2, linspace(-asin(thick/R), asin(thick/R), 60)];
mdir = [repmat(thick, 1, 2), repmat(R, 1, 60)];
patch('XData', X(1,1)+mdir.*cos(thdir+X(1,3)),'YData', X(1,2) + mdir.*sin(thdir+X(1,3)), 'FaceColor', 'k');


%% Saving Options
if saveFigure
  set(gcf, 'renderer', 'painters')
%   saveas(gcf, settings.filename, 'epsc');
  saveas(gcf, settings.filename, 'png');
  saveas(gcf, settings.filename, 'svg');
  set(h.figure, 'PaperPositionMode', 'auto');
  disp('Simulation figure is saved!!!');
end


%% Advanced Settings
function settings = advanced_settings(settings)
    
    % Visualization Options
    settings.filename = sprintf('temp/motionprediction/S_%d', settings.predictionMethod);
    screenSize = get(0, "ScreenSize");
    screenAspectRatio =screenSize(3)/screenSize(4); 
    settings.figure = {'Units', 'normalized', 'Position', [0.25, 0.25, 0.20, 0.20*screenAspectRatio], 'Color', [1 1 1]};
    settings.axes = {'NextPlot', 'add', 'DataAspectRatio', [1, 1, 1],...
        'LineWidth', 1.0, 'Position', [0 0 1 1], 'Box', 'on',...
        'XGrid', 'on', 'YGrid', 'on', 'Layer', 'top' ...
        'XLim', [-5, 5], 'XTick', -5:5, 'XTickLabels', [],...
        'YLim', [-5, 5], 'YTick', -5:5, 'YTickLabels', []};
    settings.map.patch = {'FaceColor', 'k', 'EdgeColor', 'k', 'LineWidth', 1};

    settings.RobotRadius = 0.5;
    settings.RobotResolution = 60;
    settings.DirectionThickness = 0.05;

    settings.pathPlot = {'Color', [0.3 0.3 0.6], 'LineStyle', '-', 'LineWidth', 2};
    settings.dirPlot = {'Color', [0.3 0.3 0.6], 'LineStyle', '--', 'LineWidth', 1};
    settings.scatter = {50, 'r', 'filled', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r'};
    settings.motionPatch = {'FaceColor', [0.9686, 0.8627, 0.6039], 'EdgeColor', 'none', 'FaceAlpha', 1};

    angles = linspace(0, 2*pi, settings.RobotResolution)';
    settings.RobotPolygon = settings.RobotRadius*[cos(angles), sin(angles)];

    settings.x0 = [4.75*[cos(3*pi/4) sin(3*pi/4)] -pi/8];
    settings.goal = [0 0];
    settings.gain = [1 1.5];
    settings.MotionPolygonResolution = 60;
    if settings.predictionMethod == 0
        settings.MotionPolygonMethod = 'Simulated';
    elseif settings.predictionMethod == 1
        settings.MotionPolygonMethod = 'Circular';
    elseif settings.predictionMethod == 2
        settings.MotionPolygonMethod = 'intersectional';
    elseif settings.predictionMethod == 3
        settings.MotionPolygonMethod = 'cone';
    elseif settings.predictionMethod == 4
        settings.MotionPolygonMethod = 'icecreamcone';
    elseif settings.predictionMethod == 5
        settings.MotionPolygonMethod = 'truncatedicecreamcone';
    end
end
