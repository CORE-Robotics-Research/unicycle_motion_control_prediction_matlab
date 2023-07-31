% Demonstration of positive inclusion property of the motion prediction 
% methods.

% Change settings.predictionMethod to test different motion prediction
% methods:
% 0: Forward-Simulation, 1: Circle, 2: Bounded Cone
% 3: Unbounded Cone, 4: Icecream Cone, 5: Truncated Icecream Cone

%% Start  a clean simulation environment
close all % Close all figures
clear variables % Clear all imports and variables
rng(0); % Fix random number generator seed

%% Scenario Selection


%% Demo Settings
% 0: Simulated Trajectory, 1: Circular, 2: Bounded Cone
% 3: Unbounded Cone, 4: Icecream Cone, 5: Truncated Icecream Cone
settings.predictionMethod = 1;



saveFigure = false;

settings = advanced_settings(settings);
% Map Visualization
h.figure = figure(settings.figure{:});
axes(settings.axes{:});
hold on
%% Demo Settings
myRobot = settings.myRobot;

tspan = [0 70];
goal = [5 5];

alpha = 3*pi/4;
pos0  = 4.75*[cos(alpha) sin(alpha)];
theta0 = -pi/8;


pose0 = [pos0 theta0];
odeoptions = odeset('MaxStep', 0.1, 'RelTol', 1e-3, 'AbsTol', 1e-6);



[T, X] = myRobot.Control.traj(tspan, pose0, odeoptions);
X(:,1:3) = [goal+X(:,1:2) , X(:,3)];



%% Motion Prediction Visualization
c = 1;
PL = sum(sqrt(sum(diff(X(:,1:2)).^2,2))); % Path length
dx = PL/(settings.numsample); % Update position rate

if settings.predictionMethod == 0
    myRobot.Control.MotionPolygonMethod = 'simulated';
elseif settings.predictionMethod == 1
    myRobot.Control.MotionPolygonMethod = 'circular';
elseif settings.predictionMethod == 2
    myRobot.Control.MotionPolygonMethod = 'intersectional';
elseif settings.predictionMethod == 3
    myRobot.Control.MotionPolygonMethod = 'cone';
elseif settings.predictionMethod == 4
    myRobot.Control.MotionPolygonMethod = 'icecreamcone';
elseif settings.predictionMethod == 5
    myRobot.Control.MotionPolygonMethod = 'truncatedicecreamcone';
end


P = myRobot.Control.motionpolygon(X(1,1:3), goal);
h.motion.patch = patch('XData', P(:,1), 'YData', P(:,2), settings.motionPatch{:}, 'FaceColor', settings.colors(c,:));
xpre = X(1,1:2);
for k=1:length(T)
    if norm(X(k,1:2) - xpre) <= dx
        continue;
    end
    P = myRobot.Control.motionpolygon(X(k,1:3), goal);
    h.motion.patch = patch('XData', P(:,1), 'YData', P(:,2), settings.motionPatch{:}, 'FaceColor', settings.colors(c,:));
    h.path.scatter{k} = scatter(X(k,1)+goal(1),X(k,2)+goal(2),settings.path.scatter{:},'MarkerFaceColor',settings.scattercolors(c,:));
    xpre = X(k,1:2);
end


% Trajectory Visualization
plot(X(:,1), X(:,2), settings.pathPlot{:});
scatter(goal(1),goal(2), settings.scatter{:},'MarkerFaceColor','r');
for k = 1:numel(h.path.scatter)
    uistack(h.path.scatter{k},'top')
end

% Robot Visualization
R     = settings.myRobot.BodyRadius;
th = linspace(0,2*pi,100); % Angle samples for the visualization of robot body 
thwl = linspace(5*pi/6, pi/6, 60); % Angle samples for the visualization of the left robot wheel
thwr = linspace(7*pi/6, 11*pi/6, 60); % Angle sample for the visualization of the right robot wheel
patch('XData', X(1,1) + R*cos(th),'YData', X(1,2) + R*sin(th), 'FaceColor', [0.3010, 0.7450, 0.9330]);
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
    

    % Save Options
    settings.filename = sprintf('temp/positiveinclusion/demo_positiveinclusion_%d_gain1-2', settings.predictionMethod);
    screenSize = get(0, "ScreenSize");
    screenAspectRatio =screenSize(3)/screenSize(4); 
    settings.figure = {'Units', 'normalized', 'Position', [0.25, 0.25, 0.2, 0.2*screenAspectRatio], 'Color', [1 1 1]};
    
    settings.axes = {'NextPlot', 'add', 'DataAspectRatio', [1, 1, 1],...
        'LineWidth', 1.0, 'Position', [0 0 1 1], 'Box', 'on',...
        'XGrid', 'on', 'YGrid', 'on', 'Layer', 'top', 'GridAlpha', 0.15,...
        'XLim', [0, 10], 'XTick', 0:10, 'XTickLabels', [],...
        'YLim', [0, 10], 'YTick', 0:10, 'YTickLabels', []};


    RobotRadius = 0.5;
    settings.DirectionThickness = 0.05;

    myControl.Gains = [1 2];
    myControl = unicyclesys.UnicycleForwardControl(myControl.Gains);
    settings.myRobot = unicyclerobot.UnicycleRobot(1, 3, RobotRadius);
    settings.myRobot.Control = myControl;
    settings.myRobot.BodyResolution = 60;

    settings.pathPlot = {'Color', 0.3*[1, 1, 1], 'LineStyle', '-', 'LineWidth', 2};
    settings.scatter = {50, 'filled', 'MarkerEdgeColor', 'none'};
    settings.path.scatter = {[], 'k', 'filled', 'MarkerEdgeColor', 'k'};

    settings.numsample = 6; % Number of positively included motion polygons
    settings.motionPatch = {'EdgeColor', [0, 0.4470, 0.7410], 'EdgeAlpha', 1	, 'FaceAlpha', 0.22};
    settings.colors = flipud(autumn(ceil(settings.numsample)))*0+ 1*[0, 0.4470, 0.7410];
    settings.scattercolors = flipud(autumn(settings.numsample+1))*0+ 1*[0 1 1];


end