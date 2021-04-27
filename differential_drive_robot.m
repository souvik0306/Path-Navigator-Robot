clc
clear all
path = [2.00 1.00;
 1.25 1.75;
 5.25 8.25;
 7.25 8.75;
 11.75 10.75;
 12.00 10.00];
robotInitialLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = 0;
robotCurrentPose = [robotInitialLocation initialOrientation]';
robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");
%Visualize the desired path
figure
plot(path(:,1), path(:,2),'k--d')
xlim([0 13])
ylim([0 13])
