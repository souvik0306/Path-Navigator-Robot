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
 robotRadius = 0.2;
% PRM does not account for the dimension of the robot, and hence providing an
%inflated map to the PRM takes into account the robot dimension
map = binaryOccupancyMap(complexMap,1)
show(map)

mapInflated = copy(map);
inflate(mapInflated,robotRadius);
show(mapInflated);   
%to define a path planner
prm = mobileRobotPRM;
%Assign the inflated map to the PRM object
prm.Map = mapInflated;
%define nodes or points over the map, more 
% nodes, greater the complexity of the map and 
% higher computation time
prm.NumNodes = 50;
% these denotes the Define the maximum allowed distance
% between two connected nodes on the map. PRM connects all
% nodes separated by this distance (or less) on the map
prm.ConnectionDistance = 10;
%define start location
startLocation = [3 1]
%define end locationk
endLocation = [45 35]
%final path 
path = findpath(prm, startLocation, endLocation)
show(prm)
% For complex maps, there may not be a feasible
% path for a given number of nodes (returns an empty path).
path = findpath(prm, startLocation, endLocation);
while isempty(path)
 % No feasible path found yet, increase the number of nodes
 prm.NumNodes = prm.NumNodes + 10;

 % Use the |update| function to re-create the PRM roadmap with the changed
 % attribute
 update(prm);

 % Search for a feasible path with the updated PRM
 path = findpath(prm, startLocation, endLocation);
end
show(prm)
path = [2.00 1.00;
 1.25 1.75;
 5.25 8.25;
 9.25 7.75;
 9.50  15.00;
 15.75 5.00;
 25.00 5.00;
 27.00 6.00;
 29.00 15.00;
 25.00 45.00;
 29.00 35.00];

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
 show(map);
 hold all
 % Plot path each instance so that it stays persistent while robot mesh
 % moves
 plot(path(:,1), path(:,2),"k--d")

 % Plot the path of the robot as a set of transforms
 plotTrVec = [robotCurrentPose(1:2); 0];
 plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
 plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent",  gca,"View",'2D', "FrameSize", frameSize);
 light;
 xlim([0 27])
 ylim([0 26])

 waitfor(vizRate);
end