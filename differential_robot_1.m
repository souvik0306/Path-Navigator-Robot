clc 
clear all 
% Probabilistic Roadmap (PRM) path planner
% The imported maps are : simpleMap, complexMap and ternaryMap.
% Example Maps for Planning a Path
load exampleMaps.mat 
map = binaryOccupancyMap(simpleMap,2)
show(map)
% To ensure that the robot does not collide with any 
% obstacles, you should inflate the map by the
% dimension of the robot before supplying it to the PRM path planner
robotRadius = 0.1
%load a more complex map
mapInflated = copy(map)
inflate(mapInflated, robotRadius);
show(mapInflated)
prm = mobileRobotPRM;
prm.Map = mapInflated;
prm.NumNodes = 100;
prm.ConnectionDistance = 10;
startLocation = [2 1];
endLocation = [12 2];
%final path 
path = findpath(prm, startLocation, endLocation)
show(prm)
path = [2.00 1.00;
 1.25 1.75;
 5.25 8.25;
 7.25 8.75;
 11.75 10.75;
 12.00 10.00];

%set initial robot location and goal 
robotInitialLocation = path(1,:);
robotGoal = path(end,:);

%the robot orientation is the angle between the robot heading
%and the positive X-axis, measured counterclockwise
initialOrientation = 0;

%Define the current pose for the robot [x y theta]
robotCurrentPose = [robotInitialLocation initialOrientation]';

% differentialDriveKinematics creates a differential-drive vehicle model to
% simulate simplified vehicle dynamics. This model approximates a vehicle 
% with a single fixed axle and wheels separated by a specified track width.
% The wheels can be driven independently. Vehicle speed and heading is
% defined from the axle center. The state of the vehicle is defined as a
% three-element vector, [x y theta], with a global xy-position, specified 
% in meters, and a vehicle heading, theta, specified in radians. 
robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");

%Visualize the desired path
figure
plot(path(:,1), path(:,2),'k--d')
xlim([0 14])
ylim([0 14])

% Based on the path defined above and a robot motion model, you need a path following controller to
% drive the robot along the path.
controller = controllerPurePursuit;
%set desired waypoints
controller.Waypoints = path

%Set the path following controller parameters. The desired linear velocity is set to 0.6 meters/second
%for this example.
controller.DesiredLinearVelocity = 2;
controller. MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.3;

goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);

% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);
% Initialize the figure
figure
% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/0.8;
while( distanceToGoal > goalRadius )

 % Compute the controller outputs, i.e., the inputs to the robot
 [v, omega] = controller(robotCurrentPose);

 % Get the robot's velocity using controller inputs
 vel = derivative(robot, robotCurrentPose, [v omega]);

 % Update the current pose
 robotCurrentPose = robotCurrentPose + vel*sampleTime;

 % Re-compute the distance to the goal
 distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));

 % Update the plot
 hold off

 % Plot path each instance so that it stays persistent while robot mesh
 % moves
 plot(path(:,1), path(:,2),"k--d")
 hold all

 % Plot the path of the robot as a set of transforms
 plotTrVec = [robotCurrentPose(1:2); 0];
 plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
 plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent",  gca,"View",'2D', "FrameSize", frameSize);
 light;
 xlim([0 15])
 ylim([0 15])

 waitfor(vizRate);
end












