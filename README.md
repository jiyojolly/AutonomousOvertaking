# Autonomous overtaking maneuver under complex driving conditions

## Abstract
While existing solutions attempt multi-lane overtaking involving simple and
static scenarios, the focus here is on single lane overtaking which require minimal intrusion
on to the adjacent lane in dynamically changing conditions.The method proposed utilizes
a heuristic rule-based strategy to select optimal maneuvers and then uses a combination of
safe and reachable sets to iteratively generate intermediate reference targets based on the
desired maneuver. A nonlinear model predictive controller then plans dynamically feasible
trajectories to these intermediate reference targets that avoid collisions. The proposed
method was implemented and tested under 7 different scenarios that cover many complex
lane-keeping and overtaking scenarios using the CARLA simulation engine with ROS
(Robotic Operating System) framework for inter-component communication with model
predictive controller developed using MATLAB. In every tested scenario, the proposed
planning and control paradigm was able to select the best course of action (maneuver)
and execute the same without collisions with other nearby vehicles.

## Video Demo
[![Watch the video](https://img.youtube.com/vi/-dNw-KPgBoM/0.jpg)](https://youtu.be/-dNw-KPgBoM)

## Results
![Alt text](relative/path/to/img.jpg?raw=true "Title")
## Project Structure
```bash
├───Functions     // All MATLAB functions that are used for planning and control scheme 
├───refs    // All referenced Vehicle dynamics models
├───Scenario_definitions  // All definition files that describe different overtaking scenarios, postions of other actors, lane configurations etc.
├───Scripts    // Folder containing the scripts that initialse parameters and necessary structures in the base workspace.
    ├───init_mpc.m  // Contains parameters pertaining to MPC formulation. 
    └───initAll.m   // Vehicle model and other related parameters
├───System   // Containing Main model files.
    └───Overtaking_Integrated.slx  // Main Simulink model file
└───Work
```   
 ##

##Setup Instructions

1. Install ROS Melodic (requires Ubuntu 18.04)
    Follow instructions from the following link.
    [ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu)

2. Install Carla Simulator (<=0.9.9)
    Follow instructions from the following link.
    [Carla Simulator](https://carla.readthedocs.io/en/latest/start_quickstart/)


3. Install MATLAB 2019b
    * Install required toolboxes
        - MPC Toolbox
        - ROS Support toolbox - [link](https://www.mathworks.com/products/ros.html)
        - Custom Message support [link](https://www.mathworks.com/matlabcentral/fileexchange/49810-ros-toolbox-interface-for-ros-custom-messages)

4. Create a parent directory and clone repo from Aalto Git
    * [Git Repo Link](https://version.aalto.fi/gitlab/palattj1/autonomous-overtaking)
    * Update Submodules 
        - `$ git submodule update --init --recursive`

5. Create a catkin workspace in the parent directory 

6. Create a symlink to the git repo in the src folder of the catkin workspace
    * `$ ln -s /path/to/repo /path/to/src`

7. Install required python packages using pip (pip compatible with python 2 -- Source code is written in Python 2, hence python 2 compatible packages are required)
    
    *  `$ pip install -r pip_requirements.txt`

8. Install required ros-dependencies
   
    *   `$ rosdep update`
    *   `$ rosdep install --from-paths src --ignore-src -r`

9. Build using catkin build
    * `$ catkin build`


10. Source devel files 
    * `$ source devel/setup.bash`

11. Open MATLAB
    *  Build custom msgs for ROS communication.
    *  
        - Check if geometry msgs folder exists or get from [Geometry Msgs](https://github.com/ros/common_msgs/tree/noetic-devel/geometry_msgs) 
        - Execute `gen_rosmsgs.m`
        - Follow instructions to update javapath and MATLAB path
    *  Run init_mpc.m
12. Run Carla
13. Run roslaunch
