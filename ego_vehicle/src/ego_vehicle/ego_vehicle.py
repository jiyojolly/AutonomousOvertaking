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
from shapely.geometry import Polygon, LineString, Point, box, asPoint
import shapely.ops
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


import artf_pot_funcs
from artf_pot_funcs import CarPotential

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

        self.car_pot = None
        self.pos_meshgrid = None

        # Plot variables
        self.fig_3d = None
        self.fig_2d = None
        self.ax_3d = None
        self.ax_2d = None
        self.surf = None


    def pot(self):
        
        # Create car potential class
        self.car_pot = CarPotential()

        # Create mesh for potential calculation
        x = np.arange(self.json_params["X_limit"][0],self.json_params["X_limit"][1], 1)
        y = np.arange(self.json_params["Y_limit"][0],self.json_params["Y_limit"][1], 1)
        self.pos_meshgrid = np.meshgrid(x, y, sparse=False)

        # Add fig and axes for contour and 3d surface plots
        self.fig_3d = plt.figure()
        self.ax_3d = self.fig_3d.add_subplot(111, projection='3d')

        self.fig_2d = plt.figure()
        self.ax_2d = self.fig_2d.add_subplot(111)
        # Define X, Y, Z limits for plots
        xlims = np.array([-50,50])
        ylims = np.array([-50,50])
        zlims = np.array([0,30])

        self.update_pot()


    def update_pot(self):
        ego_car_loc = Point(self.player.get_location().x, self.player.get_location().y) 
        sub_cars = self.get_subvehicles()
        sub_car_locs = [ Point(self.trans2inertial(sub_v.get_location())) for sub_v in sub_cars]  
       
        self.car_pot.update_obst_plgn(sub_car_locs, self.json_params)   
        z = self.car_pot.update_car_pot(self.pos_meshgrid, self.json_params)  

        # Plot everything...
        self.ax_3d.set(xlabel="x", ylabel="y", zlabel="f(x, y)", title="3D Surface plot of Risk Map")
        self.plot_all(z)
        
        # Print min max of pot field
        # print("Details of Mesh grid values: Shape={:s}, Min z value={:.2f}, Max z value={:.2f}".format(z.shape, np.amin(z), np.amax(z)))


    def plot_all(self, z):
        # Ego car postion Point(x,y)
        ego_zero = Point(0,0)

        if self.surf is not None: self.surf.remove()
        if self.ax_2d is not None: self.ax_2d.clear()
        self.ax_2d.plot(np.asarray(ego_zero)[0], np.asarray(ego_zero)[1], marker='x')

        # Plot obstacle polygons
        [self.ax_2d.plot(np.asarray(plgn.boundary)[:,0],np.asarray(plgn.boundary)[:,1]) for plgn in self.car_pot.plgn_list]
        
        # Plot Nearest point to obstable
        [self.ax_2d.plot([shapely.ops.nearest_points(ego_zero,plgn)[0].x,shapely.ops.nearest_points(ego_zero,poly)[1].x],
                [shapely.ops.nearest_points(ego_zero,plgn)[0].y,shapely.ops.nearest_points(ego_zero,poly)[1].y]) for poly in self.car_pot.plgn_list]

       
        # Plot potential field as a 3d surface plot
        # print("Shape of Meshgrid: {:s} - {:s} - {:s}".format(self.pos_meshgrid[0].shape,self.pos_meshgrid[1].shape,z.shape))
        self.surf = self.ax_3d.plot_surface(self.pos_meshgrid[0], self.pos_meshgrid[1], z, cmap=plt.cm.coolwarm, antialiased=True, linewidth=0, rstride=1, cstride=1)
        # self.fig_3d.colorbar(self.surf, shrink=0.5, aspect=5)

        #Set axes limits of all plots
        # ax_2d.set_xlim(xlims[0],xlims[1])
        # ax_2d.set_ylim(ylims[0],ylims[1])
        # ax_3d.set_xlim(xlims[0],xlims[1])
        # ax_3d.set_ylim(ylims[0],ylims[1])
        # ax_3d.set_zlim(zlims[0],zlims[1])
            
        
        self.fig_3d.canvas.draw()
        self.fig_2d.canvas.draw()



    def trans2inertial(self, loc):

        egov_loc = self.get_nploc(self.player.get_location())
        loc_np = self.get_nploc(loc)

        return loc_np - egov_loc


    def get_nploc(self,loc):
        return np.array([loc.x,loc.y])

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



