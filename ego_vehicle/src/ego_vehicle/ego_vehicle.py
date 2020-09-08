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
import math
import numpy as np
import json
from carla_ego_vehicle.carla_ego_vehicle import CarlaEgoVehicle
from carla_msgs.msg import CarlaWorldInfo
from shapely.geometry import Polygon, LineString, Point, box
import shapely.ops
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


import artf_pot_funcs
from artf_pot_funcs import CarPotential, LanePotential
import Utils

# You probably won't need this if you're embedding things in a tkinter plot...
plt.ion()

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
        self.json_params = None
        with open(self.param_definition_file) as handle:
            self.json_params = json.loads(handle.read())

        # Ego car postion Point(x,y) at [0,0,0]
        self.ego_zero = np.array([0.0, 0.0, 0.0])

        #State Variables as Numpy Arrays in World frame
        self.lane_edges = None

        #Risk Map variables
        # Create mesh for potential calculation
        x = np.arange(self.json_params["X_limit"][0],self.json_params["X_limit"][1], self.json_params["grid_res"])
        y = np.arange(self.json_params["Y_limit"][0],self.json_params["Y_limit"][1], self.json_params["grid_res"])
        self.pos_meshgrid = np.meshgrid(x, y, sparse=True)

        # Create car potential class
        self.car_potential = CarPotential()
        self.lane_potential = LanePotential()

        # Plotting variables
        # Add fig and axes for contour and 3d surface plots
        self.fig_3d = plt.figure()
        self.ax_3d = self.fig_3d.add_subplot(111, projection='3d')
        self.ax_3d.set(xlabel="x", ylabel="y", zlabel="f(x, y)", title="3D Surface plot of Risk Map")

        self.fig_2d = plt.figure()
        self.ax_2d = self.fig_2d.add_subplot(111)
        self.surf = None
        # # Define X, Y, Z limits for plots
        # xlims = np.array([-50,50])
        # ylims = np.array([-50,50])
        # zlims = np.array([0,30])

    def update_state(self):

        #Get surrounding obstacle vehicles 
        self.sub_cars = self.get_subvehicles()
    
        rospy.logwarn("Ego Vehicle Transform: {:s}".format(self.player.get_transform()))
       
        #Update car potential state and polygons
        self.car_potential.update_state(self.player, self.sub_cars, self.json_params)

        #Update Lane edges
        curr_waypoint = self.world.get_map().get_waypoint(self.player.get_location(), project_to_road=True)
        self.lane_potential.update_state(curr_waypoint)
        # self.lane_edges = Utils.getLaneEdges(curr_waypoint) 
        
         


    def update_pot(self):
         
        #Update State 
        self.update_state()
        #Initialize meshgrid with zeroes
        z = np.zeros(self.pos_meshgrid[0].shape)

        # #Update Car Obstacle potentials   
        z = z + self.car_potential.update(self.pos_meshgrid, self.json_params)
        # #Update Road, Lane potentials   
        z = z + self.lane_potential.update(self.pos_meshgrid, self.json_params)

        z = np.clip(z,-20,20) 
        # Update plot ...
        self.plot_all(z)

    # def select_target(self):
        

        

    def plot_all(self, z):

        #Clear previous plots
        if self.surf is not None: self.surf.remove()
        if self.ax_2d is not None: self.ax_2d.clear()
        
        # Plot lane markings
        # curr_waypoint = self.world.get_map().get_waypoint(self.player.get_location(), project_to_road=True)
        # self.lane_edges = Utils.getLaneEdges(curr_waypoint)
        # self.ax_2d.plot(np.asarray(self.lane_edges)[:,0]-self.ego_car_loc.x, np.asarray(self.lane_edges)[:,1]-self.ego_car_loc.y, marker='*')

        #Plot ego vehicle and its polygon
        self.ax_2d.plot(self.car_potential.ego_car_location[0], self.car_potential.ego_car_location[1], marker='x')
        self.ax_2d.plot(np.asarray(self.car_potential.ego_plgn)[:,0], np.asarray(self.car_potential.ego_plgn)[:,1])
        

        # Plot obstacle vehicle and its polygons
        [self.ax_2d.plot(obst_car[0], obst_car[1], marker='x') for obst_car in self.car_potential.obstcl_vehicles_locs]
        [self.ax_2d.plot(np.asarray(obst_car_plgn)[:,0],np.asarray(obst_car_plgn)[:,1]) for obst_car_plgn in self.car_potential.obstcl_vehicles_plgns]
        
        # Plot Nearest point to obstable
        [self.ax_2d.plot([shapely.ops.nearest_points(Point(self.car_potential.ego_car_location), obst_car)[0].x, 
                            shapely.ops.nearest_points(Point(self.car_potential.ego_car_location), obst_car)[1].x],
                         [shapely.ops.nearest_points(Point(self.car_potential.ego_car_location), obst_car)[0].y, 
                            shapely.ops.nearest_points(Point(self.car_potential.ego_car_location), obst_car)[1].y]) for obst_car in self.car_potential.obstcl_vehicles_plgns]

       
        # Plot potential field as a 3d surface plot
        self.surf = self.ax_3d.plot_surface(self.pos_meshgrid[0], self.pos_meshgrid[1], z, cmap=plt.cm.coolwarm, antialiased=True, linewidth=0, rstride=1, cstride=1)
        # Print min max of pot field
        print("Details of Mesh grid values: Shape={:s}, Min z value={:.2f}, Max z value={:.2f}".format(z.shape, np.amin(z), np.amax(z)))

        # #Set axes limits of all plots
        # self.ax_2d.set_xlim(-30,30)
        # self.ax_2d.set_ylim(-30,30)
        # ax_3d.set_xlim(xlims[0],xlims[1])
        # ax_3d.set_ylim(ylims[0],ylims[1])
        # ax_3d.set_zlim(zlims[0],zlims[1])
            
        
        # self.fig_3d.canvas.draw()
        self.fig_2d.canvas.draw()
        raw_input("Press Enter to continue...")


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
            rospy.logerr("Timeout while waiting for world_infod info!")
            sys.exit(1)

        rospy.loginfo("CARLA world available. Spawn ego vehicle...")

        client = carla.Client(self.host, self.port)
        client.set_timeout(self.timeout)
        self.world = client.get_world()
        self.restart()
        # self.update_pot()

        try:
            rate = rospy.Rate(30) # 10hz
            while not rospy.is_shutdown():
                self.update_pot()
                rate.sleep()
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


