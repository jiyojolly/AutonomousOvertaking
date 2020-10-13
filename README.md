# Autonomous Overtaking

A look into the possibility of autonomous overtaking.

Setup Instructions

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
