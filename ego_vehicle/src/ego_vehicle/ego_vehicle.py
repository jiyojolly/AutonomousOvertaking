#!/usr/bin/env python
#
# Copyright (c) 2020 
# Jiyo Jolly Palatti
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
base class for spawning a Ego Vehicle in ROS

Two modes are available:
- spawn at random Carla Spawnpoint
- spawn at the pose read from ROS topic /initialpose

Whenever a pose is received via /initialpose, the vehicle gets respawned at that
position. If no /initialpose is set at startup, a random spawnpoint is used.

/initialpose might be published via RVIZ '2D Pose Estimate" button.
"""

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
import os,sys
import carla
import rospy
import numpy as np
import json
from carla_ego_vehicle.carla_ego_vehicle import CarlaEgoVehicle
from carla_msgs.msg import CarlaWorldInfo

class EgoVehicle(CarlaEgoVehicle):

    """
    Handles the spawning of the ego vehicle and its sensors

    """

    def __init__(self):
        super(EgoVehicle,self).__init__()

        self.param_definition_file = rospy.get_param('~param_definition_file')
        # Read params from file
        if not os.path.exists(self.param_definition_file):
            raise RuntimeError(
                "Could not read param-definition from {}".format(self.param_definition_file))
        json_params = None
        with open(self.param_definition_file) as handle:
            json_params = json.loads(handle.read())




    def pot(self):
        
        sub_vehicles = self.get_subvehicles()
        rospy.loginfo("DEBUG || "+ str(sub_vehicles))



        str_val = "DEBUG || "+ str(sub_vehicles[0].get_location())
        rospy.loginfo(str_val)

        leadv_loc = self.get_nploc(self.player.get_location())
        egov_loc = self.get_nploc(sub_vehicles[0].get_location())

        str_val = "DEBUG || "+ str(leadv_loc)
        rospy.loginfo(str_val)

        str_val = "DEBUG || "+ str(egov_loc)
        rospy.loginfo(str_val)

        





    def get_nploc(self,loc):
        return np.array([(loc.x,loc.y)])

    def get_subvehicles(self):
        vehicles_list = self.world.get_actors().filter('vehicle.*')
        vehicles_list = [x for x in vehicles_list if x.id != self.player.id]
        return vehicles_list



        

    def run(self):
        """
        main loop
        """
        # wait for ros-bridge to set up CARLA world
        rospy.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
        try:
            rospy.wait_for_message("/carla/world_info", CarlaWorldInfo, timeout=10.0)
        except rospy.ROSException:
            rospy.logerr("Timeout while waiting for world info!")
            sys.exit(1)

        rospy.loginfo("CARLA world available. Spawn ego vehicle...")

        client = carla.Client(self.host, self.port)
        client.set_timeout(self.timeout)
        self.world = client.get_world()
        self.restart()
        self.pot()

        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass    


                    




# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    ego_vehicle = EgoVehicle()
    try:
        ego_vehicle.run()
    finally:
        if ego_vehicle is not None:
            ego_vehicle.destroy()


if __name__ == '__main__':
    main()



