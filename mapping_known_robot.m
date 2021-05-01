clc 
clear all
load exampleMaps.mat
% Create the reference binary occupancy map using simpleMap with a resolution of 1. Show the figure
% and save the handle of the figure.
refMap = binaryOccupancyMap(simpleMap,1);
refFigure = figure('Name','SimpleMap');
show(refMap);
