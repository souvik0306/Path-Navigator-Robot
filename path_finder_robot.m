clc 
clear all 
%e Probabilistic Roadmap (PRM) path planner
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
prm.NumNodes = 20;
% these denotes the Define the maximum allowed distance
% between two connected nodes on the map. PRM connects all
% nodes separated by this distance (or less) on the map
prm.ConnectionDistance = 10;
%define start location
startLocation = [2 1]
%define end locationk
endLocation = [8 2]
path = findpath(prm, startLocation, endLocation)
show(prm)