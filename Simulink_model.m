clc 
clear all
load exampleMaps.mat
startLoc = [5 5];
goalLoc = [20 20];
open_system('pathPlanningSimulinkModel.slx')
map = binaryOccupancyMap(simpleMap);
simulation = sim('pathPlanningSimulinkModel.slx');
robotPose = simulation.Pose;
thetaIdx = 3;
% Translation
xyz = robotPose;
xyz(:, thetaIdx) = 0;
% Rotation in XYZ euler angles
theta = robotPose(:,thetaIdx);
thetaEuler = zeros(size(robotPose, 1), 3 * size(theta, 2));
thetaEuler(:, end) = theta;
% Plot the robot poses at every 10th step.
for k = 1:10:size(xyz, 1)
 show(map)
 hold on;

 % Plot the start location.
 plotTransforms([startLoc, 0], eul2quat([0, 0, 0]))
 text(startLoc(1), startLoc(2), 2, 'Start');

 % Plot the goal location.
 plotTransforms([goalLoc, 0], eul2quat([0, 0, 0]))
 text(goalLoc(1), goalLoc(2), 2, 'Goal');

 % Plot the xy-locations.
 plot(robotPose(:, 1), robotPose(:, 2), '-b')

 % Plot the robot pose as it traverses the path.
 quat = eul2quat(thetaEuler(k, :), 'xyz');
 plotTransforms(xyz(k,:), quat, 'MeshFilePath',...
 'groundvehicle.stl');
 light;
 drawnow;
 hold off;
end
