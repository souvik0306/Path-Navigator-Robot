## Path-Navigator-Robot
With this project, I demonstrate how to compute an obstacle-free path between two locations on a given
map using the Probabilistic Roadmap (PRM) path planner. PRM path planner constructs a roadmap in
the free space of a given map using randomly sampled nodes in the free space and connecting them
with each other. Once the roadmap has been constructed, you can query for a path from a given start location to a given end location on the map.

(*Important* - Please make sure you have **Robotics System Toolbox** Add-On installed on your MATLAB)

<!--[![Build Status](https://travis-ci.com/petercorke/Path-Navigator-Robot.svg?branch=master)](https://travis-ci.com/petercorke/Path-Navigator-Robot)-->

The following project has been executed in **MATLAB R2021a** version 

**Results-**

<img src="https://github.com/souvik0306/Path-Navigator-Robot/blob/master/Media/Route_Map_Solved.jpeg" width="650" height="600">

**Simulation for Differential Drive Robot-**

<img src="https://github.com/souvik0306/Path-Navigator-Robot/blob/master/Media/Simulation_1.gif" width="600" height="300">

<img src="https://github.com/souvik0306/Path-Navigator-Robot/blob/master/Media/Simulation_2.gif" width="600" height="300">

**Unknown Mapping Simulation-**
(*IMPORTANT*: Ensure that ***<code>MappingWithKnownPosesDiffDriveExample</code>*** is added to your MATLAB file path, for convinience I have added that file with the other commits under the header "Unknown Mapping")

<img src="https://github.com/souvik0306/Path-Navigator-Robot/blob/master/Media/Simulation_3.gif" width="600" height="300">

**Simulink Simulation-** 

<img src="https://github.com/souvik0306/Path-Navigator-Robot/blob/master/Media/Simulation_4_Simulink.gif" width="600" height="300">

