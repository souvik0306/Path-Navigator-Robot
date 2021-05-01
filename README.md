## Path-Navigator-Robot
With this project, I demonstrate how to compute an obstacle-free path between two locations on a given
map using the Probabilistic Roadmap (PRM) path planner. PRM path planner constructs a roadmap in
the free space of a given map using randomly sampled nodes in the free space and connecting them
with each other. Once the roadmap has been constructed, you can query for a path from a given start location to a given end location on the map.

(*Important* - Please make sure you have **Robotics System Toolbox** Add-On installed on your MATLAB)

The following project has been executed in **MATLAB R2021a** version 

**Results-**

<img src="https://github.com/souvik0306/Path-Navigator-Robot/blob/master/Route_Map_Solved.jpeg" width="650" height="600">

**Simulation for Differential Drive Robot-**

<img src="https://github.com/souvik0306/Path-Navigator-Robot/blob/master/Simulation_1.gif" width="600" height="300">

<img src="https://github.com/souvik0306/Path-Navigator-Robot/blob/master/Simulation_2.gif" width="600" height="300">

**Unknown Mapping Simulation-**
(*IMPORTANT*: Ensure that ***MappingWithKnownPosesDiffDriveExample*** is added to your MATLAB file path, for your convinience I have added that file with the other commits under the head of 'Unknown Mapping'.)

<img src="https://github.com/souvik0306/Path-Navigator-Robot/blob/master/Simulation_3.gif" width="600" height="300">


