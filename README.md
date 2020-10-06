# Autonomous Overtaking

A look into the possibility of autonomous overtaking.

Setup Instructions

1. Install ROS Melodic (requires Ubuntu 18.04)
    Follow instructions from the following link.
    [ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu)

2. Install Carla Simulator
    Follow instructions from the following link.
    [Carla Simulator](https://carla.readthedocs.io/en/latest/start_quickstart/)
3.Install MATLAB 2019b
    *Install required toolboxes
    **ROS Support toolbox - [link](https://www.mathworks.com/products/ros.html)
    **Custom Message support [link](https://www.mathworks.com/matlabcentral/fileexchange/49810-ros-toolbox-interface-for-ros-custom-messages)


4. Create a parent directory and clone repo from Aalto Git
    [Git Repo Link](https://version.aalto.fi/gitlab/palattj1/autonomous-overtaking)
5. Create a catkin workspace in a the parent directory 
6. Create a symlink to the git repo in the src folder of the catkin workspace
    `$ ln -s /path/to/repo /path/to/src`
7. Install required ros-dependencies
   
    *   `$ rosdep update`
   
    *   `$ rosdep install --from-paths src --ignore-src -r`
8. Install required python packages using pip ( Source code is written in Python 2, hence python 2 packages are required)
    
    *  Numpy - `$ pip2 install numpy`
    
    *  Shapely - `$ pip2 install Shapely`
   
    *   Matplotlib - `$ pip2 install matplotlib`
   
    *   Joblib - `$ pip install joblib`
9. Build using catkin build
    `$ catkin build`
10. Build custom msgs for ROS communication. 
    
11. Source devel files 
    `$ source devel/setup.bash`

