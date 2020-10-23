cmake_minimum_required(VERSION 2.8.3)
project(ego_vehicle)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslaunch
  carla_ego_vehicle
  ros_np_multiarray
)

catkin_python_setup()
catkin_package()

roslaunch_add_file_check(launch)

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS src/ego_vehicle/vehicle.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)


