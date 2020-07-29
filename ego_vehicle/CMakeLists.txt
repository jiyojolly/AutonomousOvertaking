cmake_minimum_required(VERSION 2.8.3)
project(ego_vehicle)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslaunch
  carla_ego_vehicle
)

catkin_python_setup()

roslaunch_add_file_check(launch)

catkin_package(
  CATKIN_DEPENDS
  rospy
  carla_ego_vehicle
)

catkin_install_python(PROGRAMS
  src/ego_vehicle/ego_vehicle.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
