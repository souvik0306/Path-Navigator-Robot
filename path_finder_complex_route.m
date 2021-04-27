clc 
clear all 
% Probabilistic Roadmap (PRM) path planner
% The imported maps are : simpleMap, complexMap and ternaryMap.
% Example Maps for Planning a Path
load exampleMaps.mat 
map = binaryOccupancyMap(simpleMap,2)
show(map)
robotRadius = 0.2
%load a more complex map
map = binaryOccupancyMap(complexMap,1)
show(map)
mapInflated = copy(map)
inflate(mapInflated, robotRadius);
show(mapInflated)
% prm = mobileRobotPRM;
% prm.Map = mapInflated;
% prm.NumNodes = 50;
% prm.ConnectionDistance = 10;
% startLocation = [3 1]
% endLocation = [45 35]
% %final path 
% path = findpath(prm, startLocation, endLocation)
% show(prm)
% % For complex maps, there may not be a feasible
% % path for a given number of nodes (returns an empty path).
% path = findpath(prm, startLocation, endLocation);
% while isempty(path)
%  % No feasible path found yet, increase the number of nodes
%  prm.NumNodes = prm.NumNodes + 10;
% 
%  % Use the |update| function to re-create the PRM roadmap with the changed
%  % attribute
%  update(prm);
% 
%  % Search for a feasible path with the updated PRM
%  path = findpath(prm, startLocation, endLocation);
% end
% show(prm)








