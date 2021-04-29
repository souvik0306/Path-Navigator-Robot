clc 
clear all 
% Probabilistic Roadmap (PRM) path planner
% The imported maps are : simpleMap, complexMap and ternaryMap.
% Example Maps for Planning a Path
load exampleMaps.mat 
map = binaryOccupancyMap(simpleMap,2)
show(map)