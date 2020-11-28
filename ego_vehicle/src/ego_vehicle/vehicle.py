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
from shapely.geometry import Polygon, LineString, Point, box, LinearRing
import shapely.ops
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from std_msgs.msg import Bool

from ego_vehicle.overtake_algs import CarPotential, LanePotential, ReachableSet, TargetStateSelection

import utils
#Message definitions
from custom_msgs.msg import Float64Arr4
from std_msgs.msg import Float32MultiArray


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


        self.carla_reset_scenario_subscriber = rospy.Subscriber(
            "/carla/reset_scenario", Bool, self.reset_scenario_callback)


        # Ego car postion Point(x,y) at [0,0,0]
        self.ego_zero = np.array([0.0, 0.0, 0.0])

        #State Variables as Numpy Arrays in World frame
        self.lane_edges = None

        #Risk Map variables
        # Create mesh for potential calculation
        x = np.arange(self.json_params["X_limit"][0],self.json_params["X_limit"][1], self.json_params["grid_res"])
        y = np.arange(self.json_params["Y_limit"][0],self.json_params["Y_limit"][1], self.json_params["grid_res"])
        self.pos_meshgrid = np.meshgrid(x, y, sparse=False)

        # Create car potential class
        self.car_potential = CarPotential()
        self.lane_potential = LanePotential()
        self.reach_set = ReachableSet(self.json_params)
        self.target_selection = TargetStateSelection(self.json_params)

        # Sets 
        self.L_safe = []
        self.L_safe_reach = []
        self.L_safe_reach_max = []

        # Plotting variables
        # Add fig and axes for contour and 3d surface plots
        self.plot_3d = False
        if self.plot_3d == True:
            self.fig_3d = plt.figure()
            self.ax_3d = self.fig_3d.add_subplot(111, projection='3d')
            self.ax_3d.set(xlabel="x", ylabel="y", zlabel="f(x, y)", title="3D Surface plot of Combined Risk Map")

        self.fig_2d = plt.figure()
        self.ax_2d = self.fig_2d.add_subplot(111)
        self.ax_2d.set(xlabel="x (Ego Vehicle Frame)", ylabel="y (Ego Vehicle Frame)", title="View from the top")
        self.surf = None
        # # Define X, Y, Z limits for plots
        # xlims = np.array([-50,50])
        # ylims = np.array([-50,50])
        # zlims = np.array([0,30])

        #Initialize publisher
        self.pub_obstacle = rospy.Publisher('Obstacle', Float32MultiArray, queue_size = 2)
        self.pub_ego = rospy.Publisher('Ego_car', Float32MultiArray, queue_size = 2)
        #Initialize publisher
        self.pub_xref = rospy.Publisher('X_Ref', Float64Arr4, queue_size = 2)
        self.pub_pdes = rospy.Publisher('P_des', Float64Arr4, queue_size = 2)


    def publish_ego_data(self):
        #Publish ego car data to MATLAB
        data_ego = np.asarray(self.car_potential.ego_plgn_world.coords)
        data_ego = np.expand_dims(data_ego,axis=0)
        # print(data_ego)
        # print(data_ego.shape)
        data_to_send = utils.numpy_to_multiarray(Float32MultiArray, data_ego)
        self.pub_ego.publish(data_to_send)

        #Publish obstacle car data to MATLAB
        # data_np = np.asarray(self.car_potential.obstcl_vehicles_plgns_world[0].coords)[2:,:]
        # data_np = np.expand_dims(data_np,axis=0)
        # data_MArr = utils.numpy_to_multiarray(Float32MultiArray, data_np)
        # data_to_send = data_MArr # assign the array with the value you want to send
        # self.pub_obstacle.publish(data_to_send)
    def publish_obstcl_data(self):
        #Publish obstacle car data to MATLAB newww
        if self.target_selection.nearest_car:
            obstacl_car = self.target_selection.nearest_car
            obstacl_loc = np.array([obstacl_car.get_transform().location.x, obstacl_car.get_transform().location.y, obstacl_car.get_transform().location.z])
            get_boundingbox_transformed = lambda vehicle, transform : [utils.transform_location(np.array([vertex.x, vertex.y, vertex.z]),transform)
                                                             for i,vertex in enumerate(vehicle.bounding_box.get_world_vertices(vehicle.get_transform())) if (i % 2) == 0]
            obstacl_car_box_world = get_boundingbox_transformed(obstacl_car, carla.Transform(location=carla.Location(0.0,0.0,0.0), rotation=carla.Rotation(0.0,0.0,0.0)))
            obstacl_car_plgn_world = LinearRing([(obstacl_car_box_world[0][0:2]), (obstacl_car_box_world[1][0:2]), (obstacl_car_box_world[3][0:2]), (obstacl_car_box_world[2][0:2]), (obstacl_car_box_world[2][0:2])])
            data_np = np.asarray(obstacl_car_plgn_world.coords)
            data_np = np.expand_dims(data_np,axis=0)
            data_MArr = utils.numpy_to_multiarray(Float32MultiArray, data_np)
            data_to_send = data_MArr # assign the array with the value you want to send
            self.pub_obstacle.publish(data_to_send)



    def publish_MPC_ref(self):
        data_to_send = Float64Arr4()  # the data to be sent, initialise the array
        data_to_send.data = self.target_selection.X_ref_target_world # assign the array with the value you want to send
        self.pub_xref.publish(data_to_send)
        data_to_send = Float64Arr4()  # the data to be sent, initialise the array
        data_to_send.data = np.append(self.target_selection.final_ref_target_world,0.0) # assign the array with the value you want to send
        self.pub_pdes.publish(data_to_send)

    def reset_scenario_callback(self,data):
        rospy.logwarn("Resetting scenario.... {}".format(data))
        self.restart()

    def get_subvehicles(self):
        vehicles_list = self.world.get_actors().filter('vehicle.*')
        vehicles_list = [x for x in vehicles_list if x.id != self.player.id]
        return vehicles_list

    def update_state(self):
       
        #Update state of obstacle car risk calculation
        self.car_potential.update_state(self.player, self.sub_cars, self.json_params)

        #Update state of Road & Lane risk calculation 
        curr_waypoint = self.world.get_map().get_waypoint(self.player.get_location(), project_to_road=True)
        self.lane_potential.update_state(curr_waypoint, self.player.get_transform())

    def select_target(self):
        mask_safe = self.z < 8
        # print("Safe mask shape: {:s}").format(mask_safe.shape)
        self.L_safe = [self.pos_meshgrid[0][mask_safe], self.pos_meshgrid[1][mask_safe]]
        # print("L Safe shape: {:s}").format(self.L_safe[0].shape)
        func = lambda x,y : Polygon(self.reach_set.reach_set).contains(Point(x, y))
        vfunc = np.vectorize(func) 
        mask_safe_reach = vfunc(self.L_safe[0], self.L_safe[1])
        # print("Safe, Reachable mask shape: {:s}").format(mask_safe_reach.shape)
        self.L_safe_reach = [self.L_safe[0][mask_safe_reach], self.L_safe[1][mask_safe_reach]]

        #Publish reference state vector that maximizes longitudinal distance travel
        self.target_selection.update_state(self.player, self.sub_cars, self.L_safe_reach)
        self.target_selection.update_ref(self.world.get_map())

        #Publish obstacle data and ref point
        self.publish_obstcl_data()
        self.publish_MPC_ref()


    def update_pot(self):
         
        #Get surrounding obstacle vehicles 
        self.sub_cars = self.get_subvehicles()
        # rospy.logwarn("Ego Vehicle Transform: {:s}".format(self.player.get_transform()))
        
        if self.player is not None:
            #Update State
            self.update_state()
            #Publish state data to MATLAB
            self.publish_ego_data()
            #Initialize meshgrid with zeroes
            z = np.zeros(self.pos_meshgrid[0].shape)

            #Update Car Obstacle potentials
            z = z + self.car_potential.update(self.pos_meshgrid, self.json_params)

            #Update Road, Lane potentials   
            z = z + self.lane_potential.update(self.pos_meshgrid, self.json_params)

            self.z = np.clip(z,-20,20) 
            
            #Update Reachability and safe set
            self.reach_set.update(self.player)
            self.select_target()

            # Update plot ...
            # self.plot_all(self.z, pause=True)
            self.plot_all(self.z, pause=False)

    def plot_all(self, z, pause=False):

        #Clear previous plots
        if self.ax_2d is not None: self.ax_2d.clear()
        self.ax_2d.set(xlabel="x", ylabel="y", title="Bird's eye view (Ego car frame)")

        # Plot lane markings
        #For label
        self.ax_2d.plot(self.lane_potential.lane_edges_transformed[0][0], self.lane_potential.lane_edges_transformed[0][1], marker='_', label='Lane Edges', color='#ffcccb')
        [self.ax_2d.plot(lane[0], lane[1], marker='_', color='#ffcccb') for lane in self.lane_potential.lane_edges_transformed]

        #Plot ego vehicle and its polygon
        self.ax_2d.plot(self.car_potential.ego_car_location[0], self.car_potential.ego_car_location[1], marker='x', color='#0000ff' )
        self.ax_2d.plot(np.asarray(self.car_potential.ego_plgn)[:,0], np.asarray(self.car_potential.ego_plgn)[:,1], label='Ego Car', color='#0000ff')
        

        # Plot obstacle vehicle and its polygons
        if self.car_potential.obstcl_vehicles_locs:
            #For label
            self.ax_2d.plot(self.car_potential.obstcl_vehicles_locs[0][0], self.car_potential.obstcl_vehicles_locs[0][1], marker='x',label='Obstacle Car',color='r')
            [self.ax_2d.plot(obst_car[0], obst_car[1], marker='x',color='r') for obst_car in self.car_potential.obstcl_vehicles_locs ]
            [self.ax_2d.plot(np.asarray(obst_car_plgn)[:,0],np.asarray(obst_car_plgn)[:,1], color='r') for obst_car_plgn in self.car_potential.obstcl_vehicles_plgns]
        
            # Plot Nearest point to obstable
            # [self.ax_2d.plot([shapely.ops.nearest_points(Point(self.car_potential.ego_car_location), obst_car)[0].x, 
            #                     shapely.ops.nearest_points(Point(self.car_potential.ego_car_location), obst_car)[1].x],
            #                  [shapely.ops.nearest_points(Point(self.car_potential.ego_car_location), obst_car)[0].y, 
            #                     shapely.ops.nearest_points(Point(self.car_potential.ego_car_location), obst_car)[1].y]) for obst_car in self.car_potential.obstcl_vehicles_plgns]

        #Plot Reachable / safe set
        if self.L_safe: self.ax_2d.scatter(self.L_safe[0], self.L_safe[1], marker='.', color='#D3D3D3', label='Safe Set')

        self.ax_2d.plot(np.asarray(self.reach_set.reach_set)[:,0],np.asarray(self.reach_set.reach_set)[:,1], color='#ffa500', label='Reach Set')

        if self.L_safe_reach:self.ax_2d.scatter(self.L_safe_reach[0], self.L_safe_reach[1], marker='.', color='#4dff4d', label='Safe, Reach Set')

        self.ax_2d.scatter(self.target_selection.x_ref_target_ego[0] , self.target_selection.x_ref_target_ego[1], marker='x', color='#cdb332', label='Intd. MPC Target')
        self.ax_2d.scatter(self.target_selection.final_ref_target_ego[0], self.target_selection.final_ref_target_ego[1], marker='x', color='#008000', label='Final Target')


        # Plot potential field as a 3d surface plot
        if self.plot_3d == True:
            #Clear previous plots
            if self.surf is not None: self.surf.remove()
            self.surf = self.ax_3d.plot_surface(self.pos_meshgrid[0], self.pos_meshgrid[1], z, cmap=plt.cm.coolwarm, antialiased=True, linewidth=0, rstride=1, cstride=1)
        # Print min max of pot field
        # print("Details of Mesh grid values: Shape={:s}, Min z value={:.2f}, Max z value={:.2f}".format(z.shape, np.amin(z), np.amax(z)))

        #Set axes limits of all plots
        self.ax_2d.set_xlim(-10,20)
        self.ax_2d.set_ylim(-20,20)
        #Set legend
        self.ax_2d.legend(prop={'size': 12})  
        
        if self.plot_3d == True:
            self.ax_3d.set_xlim(-20,20)
            self.ax_3d.set_ylim(-20,20)
            self.ax_3d.set_zlim(-5,40)
          
        
        # self.fig_3d.canvas.draw()
        self.fig_2d.canvas.draw()
        if pause:
            input("Press Enter to continue...")
        

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
        self.world.set_weather(carla.WeatherParameters.CloudySunset)
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



