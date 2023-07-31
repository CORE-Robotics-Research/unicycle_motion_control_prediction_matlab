% Demo for safe unicycle robot navigation via feedback motion prediction 
% and safety assessment.

% Change Scenario ID to test the general motion framework with
% different motion prediction methods, mapping methods, 
% example environments, and reference governor gain:

% Scenario ID:
%  First Digit : 1:PolygonMap  2:BinaryOccupancyGrid Mappng
%  Second Digit: 1:Office Environment  2:Cluttered Env  3:Circular-Like Env
%  Third Digit : 1:Circular 2:Intersectional 3:IceCreamCone
%                4:TruncatedIceCone 5:ForwardSimulated
%  Fourth Digit: Governor Gain

% Example: 
% The Scenario ID: 2134 demonstrates a safe navigation application of the
% unicycle robot in an office-like environment mapped with Binary Occupancy
% Grid Mapping. The feedback motion prediction is selected as icecream cone
% and the reference governor gain is 4.
% Note: Each environment comes with a pre-determined user-defined path
% plan. See the advanced_simulation_settings() function at the end of the
% code for more detail.

%% Start  a clean simulation environment
clear variables
close all
clc

settings.saveVideo = false;
saveFigure = false;

%% Simulation Settings

% Scenario ID:
%  First Digit : 1:PolygonMap  2:BinaryOccupancyGrid
%  Second Digit: 1:Office Environment  2:Cluttered Env  3:Circular-Like Env
%  Third Digit : 1:Circular 2:Intersectional 3:IceCreamCone
%                4:TruncatedIceCone 5:ForwardSimulated
%  Fourth Digit: Governor Gain
ScenarioID = 1134;


settings.ScenarioID = ScenarioID;

settings.tspan = [0, 60];
settings = advanced_simulation_settings(settings);

%% Simulation Computation

myRobot = settings.Robot;
myPlanner = settings.ReferencePlanner; 
myGovernedPlanner = planner.GovernedReferencePlanner(myRobot, myPlanner);
myGovernedPlanner.GovernorGain = settings.GovernorGain;
tspan = settings.tspan;
x0 = settings.x0;
y0 = settings.y0;
odeoptions = odeset('MaxStep', 0.1, 'RelTol', 1e-3, 'AbsTol', 1e-6);

[T, X, Y] = myGovernedPlanner.ode45(tspan, x0, y0, odeoptions);
fprintf("Simulation Computed\n")
%% Visualization


% Map Visualization
figure(settings.workspace.figure{:});
axes(settings.workspace.axes{:});

switch floor(settings.ScenarioID/1000)
    case 1
        plotConfSpace(settings.ReferencePlanner.Workspace, myRobot.BodyRadius,...
            settings.workspace.patchConfSpace{:});
        plot(settings.ReferencePlanner.Workspace, settings.workspace.patch{:});
    case 2
        % Due to surf function properties, choose either one of those
        % h.image = plot(myPlanner.Workspace);
        h.image = plotConfSpace(settings.ReferencePlanner.Workspace, myRobot.BodyRadius,...
                settings.workspace.patchConfSpace{:});
        axis equal; axis tight
end


myPlanner.plotDomain(settings.planner.patch{:});
myPlanner.plotField(settings.planner.quiver{:});
myPlanner.plotPath(settings.planner.path{:});

th = linspace(0,2*pi,100); % Angle samples for the visualization of robot body 
thwl = linspace(5*pi/6, pi/6, 60); % Angle samples for the visualization of the left robot wheel
thwr = linspace(7*pi/6, 11*pi/6, 60); % Angle sample for the visualization of the right robot wheel
R = myRobot.BodyRadius;




h.governor.path = plot(Y(1,1), Y(1,2), settings.governor.path{:});

h.robot.path = plot(X(1,1), X(1,2), settings.robot.path{:});



h.robot.start{1} = patch('XData', myPlanner.Path.Points(1,1) + myRobot.BodyPolygon(:,1),...
      'YData', myPlanner.Path.Points(1,2) + myRobot.BodyPolygon(:,2),...
      settings.start.patch{:});
h.robot.start{2} = patch('XData', X(1,1) + R*cos(thwl+X(1,3)), 'YData',  X(1,2) + R*sin(thwl+X(1,3)), 'FaceColor', 'k');
h.robot.start{3} = patch('XData', X(1,1) + R*cos(thwr+X(1,3)), 'YData',  X(1,2) + R*sin(thwr+X(1,3)), 'FaceColor', 'k');
h.robot.end = patch('XData', myPlanner.Path.Points(end,1) + myRobot.BodyPolygon(:,1),...
      'YData', myPlanner.Path.Points(end,2) + myRobot.BodyPolygon(:,2),...
      settings.end.patch{:});

thick = settings.DirectionThickness;
thdir = [ pi/2, -pi/2, linspace(-asin(thick/R), asin(thick/R), 60)];
mdir = [repmat(thick, 1, 2), repmat(R, 1, 60)];
h.robot.start{4} = patch('XData', X(1,1)+mdir.*cos(thdir+X(1,3)),'YData', X(1,2) + mdir.*sin(thdir+X(1,3)), 'FaceColor', 'k'); 


P = myGovernedPlanner.motionpolygon(X(1,:), Y(1,:));
h.motion.patch = patch('XData', P(:,1), 'YData', P(:,2), settings.motion.patch{:});


% Plot Governor
h.governor.patch = patch('XData', Y(1,1) + myRobot.BodyPolygon(:,1),...
                         'YData', Y(1,2) + myRobot.BodyPolygon(:,2),...
                         settings.governor.patch{:});
% Plot Robot
h.robot.patch = patch('XData', X(1,1) + myRobot.BodyPolygon(:,1),...
                      'YData', X(1,2) + myRobot.BodyPolygon(:,2),...
                      settings.robot.patch{:});
h.robotwl.patch = patch('XData', X(1,1) + R*cos(thwl+X(1,3)), 'YData',  X(1,2) + R*sin(thwl+X(1,3)), 'FaceColor', 'k');
h.robotwr.patch = patch('XData', X(1,1) + R*cos(thwr+X(1,3)), 'YData',  X(1,2) + R*sin(thwr+X(1,3)), 'FaceColor', 'k');
h.robotdir.patch= patch('XData', X(1,1)+mdir.*cos(thdir+X(1,3)),'YData', X(1,2) + mdir.*sin(thdir+X(1,3)), 'FaceColor', 'k');


if settings.saveVideo
   
    if ispc
        VideoProfile = 'MPEG-4';
    else
        VideoProfile = 'Uncompressed AVI';
    end
    h.writerobj = VideoWriter(settings.filename, VideoProfile);
    h.writerobj.FrameRate = settings.FrameRate;
    open(h.writerobj);
    for k = 1:2*h.writerobj.FrameRate
       writeVideo(h.writerobj,getframe(gcf));
    end
end   


tpre = T(1);
xpre = X(1, 1:2);
V = [];

% Simulation
for k = 1:length(T)
    if (T(k) - tpre) <= 1/settings.FrameRate
        continue;
    end
    tpre = T(k);
    set(h.governor.patch, 'XData', Y(k,1) + myRobot.BodyPolygon(:,1),...
                          'YData', Y(k,2) + myRobot.BodyPolygon(:,2));
    set(h.robot.patch, 'XData', X(k,1) + myRobot.BodyPolygon(:,1),...
                          'YData', X(k,2) + myRobot.BodyPolygon(:,2));


    set(h.robotwl.patch, 'XData', X(k,1) + R*cos(thwl+X(k,3)),...
                         'YData', X(k,2) + R*sin(thwl+X(k,3)), 'FaceColor', 'k');
    set(h.robotwr.patch, 'XData', X(k,1) + R*cos(thwr+X(k,3)),...
                         'YData', X(k,2) + R*sin(thwr+X(k,3)), 'FaceColor', 'k');
    set(h.robotdir.patch,'XData', X(k,1)+mdir.*cos(thdir+X(k,3)),...
                         'YData', X(k,2) + mdir.*sin(thdir+X(k,3)), 'FaceColor', 'k');
    
    set(h.robot.path, 'XData', X(1:k,1), 'YData', X(1:k, 2));
    set(h.governor.path, 'XData', Y(1:k,1), 'YData', Y(1:k,2));
    P = myGovernedPlanner.augmentedmotionpolygon(X(k,:), Y(k,:));
    set(h.motion.patch, 'XData', P(:,1), 'YData', P(:,2));

        xpos = X(k,1:2);
        govpos = Y(k,1:2);
        theta = X(k,3);
        phi = atan2(  (-sin(theta)*(govpos(1) - xpos(1)) +...
                        cos(theta)*(govpos(2) - xpos(2))) ,...
                       (cos(theta)*(govpos(1) - xpos(1)) +...
                        sin(theta)*(govpos(2) - xpos(2))));
        v = settings.Robot.Control.Gains(1) * max(0, norm(govpos - xpos) * cos(phi) );
        V = [V;T(k) v];

    if norm(X(k,1:2) - xpre) > settings.PathSpacing
        xpre = X(k,1:2);
        vbar = plot([X(k,1), X(k,1)-settings.vscale*v*sin(theta)], [X(k,2), X(k,2)+settings.vscale*v*cos(theta)], settings.robot.velocity{:});
    end
    drawnow();              
    pause(1/settings.FrameRate);
    
    if settings.saveVideo
      writeVideo(h.writerobj,getframe(gcf));
    end
    if norm(Y(k,1:2)-X(k,1:2)) < 1e-2
       break 
    end
end


if settings.saveVideo
    for k = 1:2*h.writerobj.FrameRate
     writeVideo(h.writerobj,getframe(gcf));
    end
    close(h.writerobj);
    disp('Simulation video is saved!!!');    
end


%%

if saveFigure
  delete(h.governor.patch);
  delete(h.robot.patch);
  delete(h.motion.patch);
  delete(h.robotdir.patch);
  delete(h.robotwl.patch);
  delete(h.robotwr.patch);
  uistack(h.robot.start{1},'top');
  uistack(h.robot.start{2},'top');
  uistack(h.robot.start{3},'top');
  uistack(h.robot.start{4},'top');
  uistack(h.robot.end,'top');
  set(gcf, 'Renderer', 'painters')
  saveas(gcf, settings.filename, 'epsc');
  saveas(gcf, settings.filename, 'png');
  disp('Simulation figure is saved!!!');
  save(settings.filename,'V')
end




%% Advanced Simulation Settings
function settings = advanced_simulation_settings(settings)
    


    if settings.saveVideo == 1
    settings.filename = sprintf("temp/governedplanner/videos/demo_governedplanner_%d_lingain1_anggain2_govgain4",...
                                settings.ScenarioID);
    end
    settings.FrameRate = 10;
    settings.PathSpacing = 0.3;
    
    screenSize = get(0, "ScreenSize");
    screenAspectRatio =screenSize(3)/screenSize(4); 

    switch floor(settings.ScenarioID/1000)
        case 1
            settings.workspace.patch = {'FaceColor', 'k', 'EdgeColor', 'k', 'LineWidth', 1};
            settings.workspace.patchConfSpace = {'FaceColor', 0.8*[1 1 1], 'EdgeColor', 'none'};
        case 2
            settings.workspace.patch = {'FaceColor', 'k', 'EdgeColor', 0.5*[1 1 1],...
                        'LineWidth', 1};
            settings.workspace.patchConfSpace = {'EdgeColor', 'none'};
    end
    
    settings.robot.patch = {'FaceColor', [77/255, 190/255, 238/255], 'EdgeColor', 'none', 'FaceAlpha', 1};
    settings.robot.path = {'Color', [0.0, 0.4, 1.0], 'LineWidth', 2};
    settings.governor.patch = {'FaceColor', [0, 0.75, 0], 'EdgeColor', 'none', 'FaceAlpha', 1};
    settings.governor.path = {'Color', [0, 0.75, 0], 'LineWidth', 3};
    settings.robot.velocity  = {'Color', [0.0, 0.4, 1.0], 'LineWidth', 2};
    settings.planner.quiver = {'Color', [1.0, 0.2, 0.2], 'LineWidth', 1, 'AutoScaleFactor', 0.35, 'MaxHeadSize', 0.35};
    settings.planner.patch = {'FaceColor', [1.0, 0, 0], 'FaceAlpha', 0.15, 'EdgeColor', 'none'};
    settings.planner.path = {'Color', [0.8, 0.0, 0.0], 'LineWidth', 3}; 
    settings.start.patch = {'FaceColor', [77/255, 190/255, 238/255], 'EdgeColor', 'none'};
    settings.end.patch = {'FaceColor', [0.8, 0.0, 0.0], 'EdgeColor', 'none'};
    settings.motion.patch = {'FaceColor', [0.9290, 0.6940, 0.1250], 'EdgeColor', 'none', 'FaceAlpha', 0.6};    
    settings.robot.velocity  = {'Color', [0.0, 0.4, 1.0], 'LineWidth', 2};

    
    SpaceDimension = 2;
    RobotRadius = 0.35; 
    settings.DirectionThickness = 0.05;



    switch floor(rem(settings.ScenarioID,100)/10)
        case 1
            settings.MotionPolygonMethod = "Circular";
            settings.filename = sprintf("temp/governedplanner/demo_governedplanner_%d_circular_lingain1_anggain15_govgain4",...
                                settings.ScenarioID);
        case 2
            settings.MotionPolygonMethod = "Intersectional";
            settings.filename = sprintf("temp/governedplanner/demo_governedplanner_%d_boundedconic_lingain1_anggain15_govgain4",...
                                settings.ScenarioID);
        case 3
            settings.MotionPolygonMethod = "IceCreamCone";
            settings.filename = sprintf("temp/governedplanner/demo_governedplanner_%d_icecreamcone_lingain1_anggain15_govgain4",...
                                settings.ScenarioID);
        case 4
            settings.MotionPolygonMethod = "TruncatedIceCreamCone";
            settings.filename = sprintf("temp/governedplanner/demo_governedplanner_%d_truncatedicecreamcone_lingain1_anggain15_govgain4",...
                                settings.ScenarioID);
        case 5
            settings.MotionPolygonMethod = "Simulated";
            settings.filename = sprintf("temp/governedplanner/demo_governedplanner_%d_forwardsimulated_lingain1_anggain15_govgain4",...
                                settings.ScenarioID);
    end

    settings.GovernorGain = settings.ScenarioID - floor(settings.ScenarioID/10)*10;

    switch rem(floor(settings.ScenarioID/100),10)
        case 1
            PathPoints = [3 9; 3 8; 1.125 8; 1.125 4.5; 3 4.5; 3 1.125; 5.75 1.125; 5.75 3; 8 4; 8 6; 6 8; 6 9; 7 9];
            Workspace.Boundary = [0 0; 0 10; 10 10; 10 0];
            Workspace.Obstacle{1} = [0 0; 0 10; 0.25 10; 0.25 0];
            Workspace.Obstacle{2} = [0 10; 10 10; 10 9.75; 0 9.75];
            Workspace.Obstacle{3} = [10 10; 10 0; 9.75 0; 9.75 10];
            Workspace.Obstacle{4} = [0 0; 0 0.25; 10 0.25; 10 0];
            Workspace.Obstacle{5} = [4 10; 4.5 10; 4.5 8; 4 8];
            Workspace.Obstacle{6} = [8 8; 10 8; 10 7.5; 8 7.5];
            Workspace.Obstacle{7} = [4 2; 4 6; 4.5 6; 4.5 2];
            Workspace.Obstacle{8} = [7 2; 7 2.5; 10 2.5; 10 2];
            Workspace.Obstacle{9} = [4.5 4; 4.5 4.5; 6 4.5; 6 4];
            Workspace.Obstacle{10} = [0 2; 0 2.5; 2 2.5; 2 2];
            Workspace.Obstacle{11} = [2 6; 2 6.5; 4.5 6.5; 4.5 6];
            
            settings.vscale = 1;
            settings.workspace.figure = {'Units', 'normalized', 'Position', [0.25, 0.25, 0.2, 0.2*screenAspectRatio], 'Color', [1 1 1]};

            settings.workspace.axes = {'NextPlot', 'add', 'DataAspectRatio', [1, 1, 1],...
                'Position', [0 0 1 1], 'Box', 'on', 'XGrid', 'on', 'YGrid', 'on', ...
                'XLim', [0, 10], 'XTick', 0:0.5:10, 'XTickLabels', [],...
                'YLim', [0, 10], 'YTick', 0:0.5:10, 'YTickLabels', []};
            
        case 2
            PathPoints = [1.5 9; 1.5 8; 6 8; 7 7; 14 7; 14 8.55; 18 8.55; 18 5.5; 16.5 4; 16.5 2; 18.5 1.5];
            Workspace.Boundary = [0 0; 0 10; 20 10; 20 0];
            % obstacles
            Workspace.Obstacle{1} = [4.25 10; 5.75 10; 5.75 9.25; 4.25 9.25];
            Workspace.Obstacle{2} = [10.25 10; 11.75 10; 11.75 8.5; 10.25 8.5];
            Workspace.Obstacle{3} = [15.5 6.0; 15.5 7.75; 16 7.75; 16 6.0];
            Workspace.Obstacle{4} = [3.75 4; 3.75 7; 4.25 7; 4.25 4];
            Workspace.Obstacle{5} = [4.25 5.3; 4.25 5.7; 12 5.7; 12 5.3];
            Workspace.Obstacle{6} = [12 5.7; 12.5 5.7; 12.5 4.0; 12 4.0];
            Workspace.Obstacle{7} = [12 4.0; 15 4.0; 15 3.5; 12 3.5];
            Workspace.Obstacle{8} = [18 3.5; 18 4.0; 20 4.0; 20 3.5];
            Workspace.Obstacle{9} = [4.25 0; 4.25 1; 6.75 1; 6.75 0];
            Workspace.Obstacle{10} = [10 0; 10 1; 12 1; 12 0];
            Workspace.Obstacle{11} = [0 0; 0 10; 0.25 10; 0.25 0];
            Workspace.Obstacle{12} = [0 10; 20 10; 20 9.75; 0 9.75];
            Workspace.Obstacle{13} = [20 10; 20 0; 19.75 0; 19.75 10];
            Workspace.Obstacle{14} = [0 0; 0 0.25; 20 0.25; 20 0];

            settings.vscale = 1;
            settings.workspace.figure = {'Units', 'normalized', 'Position', [0.25, 0.25, 0.4, 0.2*screenAspectRatio], 'Color', [1 1 1]};
            settings.workspace.axes = {'NextPlot', 'add', 'DataAspectRatio', [1, 1, 1],...
                'Position', [0 0 1 1], 'Box', 'on', 'XGrid', 'on', 'YGrid', 'on', ...
                'XLim', [0, 20], 'XTick', 0:20, 'XTickLabels', [],...
                'YLim', [0, 10], 'YTick', 0:10, 'YTickLabels', []};
        case 3
            PathPoints = [4 1.5; 1.1 1.5; 1.1 5.5; 8.65 5.5; 8.65 1.5; 6 1.5];
            Workspace.Boundary = [0 0; 0 7; 10 7; 10 0];
            Workspace.Obstacle{1} = [1.95 2.75; 7.55 2.75; 7.55 4.25; 1.95 4.25];
            Workspace.Obstacle{2} = [0 0; 0 7; 0.25 7; 0.25 0];
            Workspace.Obstacle{3} = [0 0; 0 0.25; 10 0.25; 10 0];
            Workspace.Obstacle{4} = [9.75 0; 10 0; 10 7; 9.75 7];
            Workspace.Obstacle{5} = [0 6.75; 0 10; 10 10; 10 6.75];

            settings.vscale = 0.45;
            settings.workspace.figure = {'Units', 'normalized', 'Position', [2.25, 0.25, 0.25, 0.175*screenAspectRatio], 'Color', [1 1 1]};
                if settings.saveVideo == 1
                    settings.workspace.figure = {'Units', 'normalized', 'Position', [0.25, 0.1, 0.5, 0.35*screenAspectRatio], 'Color', [1 1 1]};
                end
            settings.workspace.axes = {'NextPlot', 'add', 'DataAspectRatio', [1, 1, 1],...
                'Position', [0 0 1 1], 'Box', 'on', 'XGrid', 'on', 'YGrid', 'on', ...
                'XLim', [0, 10], 'XTick', 0:1:10, 'XTickLabels', [],...
                'YLim', [0, 7], 'YTick', 0:1:7, 'YTickLabels', []};
    end

    %Map
    myWorkspace = world.PolygonMap2D(Workspace.Boundary,Workspace.Obstacle);
    if floor(settings.ScenarioID/1000) == 2
        settings.resolution = 50;
        myWorkspace = myWorkspace.binaryOccupancyGrid(settings.resolution);
    end

    % Reference Path
    myPath = path.Path(PathPoints);

    % Reference Motion Planner
    settings.ReferencePlanner = planner.PathPursuitPlanner(myWorkspace, myPath);
    settings.ReferencePlanner.ControlGain = 1;
    settings.ReferencePlanner.SafetyMargin = RobotRadius;
    settings.ReferencePlanner.PatchResolution = 400;
    settings.ReferencePlanner.QuiverResolution = 50;

    myControl.Gains = [1 1.5];
    myControl = unicyclesys.UnicycleForwardControl(myControl.Gains);
    
    myControl.MotionPolygonMethod = settings.MotionPolygonMethod;
    myControl.MotionPolygonResolution = 60;

    settings.Robot = unicyclerobot.UnicycleRobot(1, 3, RobotRadius);
    settings.Robot.Control = myControl;
    settings.Robot.BodyResolution = 60;

    % Initial Pose
    settings.initialOrientation = atan2(myPath.Points(2,2) - myPath.Points(1,2),...
                                        myPath.Points(2,1) - myPath.Points(1,1));
    settings.x0 = [myPath.Points(1,:), settings.initialOrientation+1e-2];
    settings.y0 = myPath.Points(1,:);


end