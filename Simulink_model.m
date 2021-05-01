clc 
clear all 
load exampleMaps.mat
startLoc = [5 5];
goalLoc = [20 20];
open_system('pathPlanningSimulinkModel.slx')
