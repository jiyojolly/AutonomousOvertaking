#!/usr/bin/env python
#
# Copyright (c) 2020 
# Jiyo Jolly Palatti
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Functions for generating artificial potential fields

"""
import numpy as np
from scipy import integrate
from shapely.geometry import LineString, Polygon, LinearRing, Point, box, asPoint
import shapely.ops
from joblib import Parallel, delayed
import Utils
import math
import rospy
from kinematic_bicycle_model import bicycle_model
import carla

# @profile
def yukawa_pot(json_params, Kd):
    """
    Returns the Yukawa potential value corresponding 
    to distance Kd from he obstacle
    Args:
        json_params (json List): Parameter list
        Kd (float): Pseudo-Distance 
    Returns:
        (float): Yukawa potential field value at Kd distance

    """
    U = np.multiply(json_params["Yukawa_Acar"],np.divide(np.exp(-1*json_params["Yukawa_alpha"]*Kd),Kd))
    return U

def lane_potential(json_params, y_lane, y):
    """
    Returns the Yukawa potential value corresponding 
    to distance Kd from he obstacle
    Args:
        json_params (json List): Parameter list
        Kd (float): Pseudo-Distance 
    Returns:
        (float): Yukawa potential field value at Kd distance

    """
    U = np.zeros()
    U = np.multiply(json_params["Lane_Alane"],np.exp(-np.divide((y-y_lane[0])**2, (2*json_params["Lane_widthfactor"]*2))))
    return U



def min_euclid_dist(B, pose):
    """
    Returns the minimum euclidean distance between 
    obstacle (collection of points) to a single point in 
    the exterior of the obstacle
    NOT USED
    Args:
        B (numpy array(2,n)): Point array of obstacle
        pose (numpy array(2,1)): Single position   
    Returns:
        (float): Potential field value at Kd distance

    """
    
    if B.shape[1] != 2:raise Exception("Wrong Obstacle point array shape")
    pose_arr = np.tile(pose,(B.shape[0],1))
    
    return np.amin(np.linalg.norm(B-pose_arr,axis=(1)))


class CarPotential(object):
    """docstring for CarPotential"""
    def __init__(self):
        super(CarPotential, self).__init__()
        self.ego_car_location = None
        self.ego_car_vel = None
        self.ego_plgn = None
        self.obstcl_vehicles_locs = []
        self.obstcl_vehicles_vels = []
        self.obstcl_vehicles_plgns = []




    # @profile
    def update_state(self, ego_car, obstcl_vehicles, json_params):
        """
        Create/Update Obstacle polygons
        Args:
            x (float): x coordinate of a position
            y (float): x coordinate of a position
            poly (Shapely Polygon Object): Polygon object representing obstacle
            json_params (JSON Dictionary Object): Parameters Object   
        Returns:
            (float): Potential field value due to obstacle
        """
        ## function that generates bounding box using Carla
        get_boundingbox_transformed = lambda vehicle, transform : [Utils.transform_location_R(np.array([vertex.x, vertex.y, vertex.z]),transform)
                                                         for i,vertex in enumerate(vehicle.bounding_box.get_world_vertices(vehicle.get_transform())) if (i % 2) == 0]

        get_boundingbox_local = lambda vehicle : [np.array([vertex.x, vertex.y, vertex.z])
                                                         for i,vertex in enumerate(vehicle.bounding_box.get_local_vertices()) if (i % 2) == 0]


        #Calculate velocity scaling
        def vel_scale(vel_ego, vel_obstcar):

            if vel_ego >= json_params["d_zero"]/json_params["Tf"]: psi_zero = json_params["d_zero"]/(json_params["Tf"]*vel_ego)
            else: psi_zero = 1.0
            # print("Psi value:{:f}".format(psi_zero))
            if vel_ego > vel_obstcar: psi = psi_zero * np.exp(-json_params["beta"]*(vel_ego-vel_obstcar))
            else: psi = psi_zero 
            # print("Psi value:{:f}".format(psi))
            return psi

        ##Update State variables
        #Ego Vehicle
        # frame_of_ref = carla.Transform(location=carla.Location(0.0,0.0,0.0), rotation=carla.Rotation(0.0,0.0,0.0))
        frame_of_ref = ego_car.get_transform()

        self.ego_car_location = Utils.transform_location_R(np.array([ego_car.get_transform().location.x, ego_car.get_transform().location.y, ego_car.get_transform().location.z]),frame_of_ref)
        v = ego_car.get_velocity()
        self.ego_car_vel = math.sqrt(v.x**2 + v.y**2 + v.z**2)
        ego_box = get_boundingbox_transformed(ego_car, frame_of_ref)
        self.ego_plgn = LinearRing([(ego_box[0][0:2]), (ego_box[1][0:2]), (ego_box[3][0:2]), (ego_box[2][0:2])])

        #Obstacle Vehicles in Ego car frame
        self.obstcl_vehicles_locs = [Utils.transform_location_R(np.array([vehicle.get_location().x, vehicle.get_location().y, vehicle.get_location().z]), 
                                                               frame_of_ref) for vehicle in obstcl_vehicles]
        self.obstcl_vehicles_vels = [math.sqrt(vehicle.get_velocity().x**2 + vehicle.get_velocity().y**2 + vehicle.get_velocity().z**2) for vehicle in obstcl_vehicles]
        obstcl_vehicles_boxes_carla = [get_boundingbox_local(vehicle) for vehicle in obstcl_vehicles]

        obstcl_vehicles_boxes_carla_T = []
        for i,vehicle in enumerate(obstcl_vehicles_boxes_carla):
            triang_vertex = np.array([vehicle[0][0]+(json_params["delta_vertex"]*(np.clip(float(1.0)/vel_scale(self.ego_car_vel, self.obstcl_vehicles_vels[i]),1,20))), 
                                vehicle[0][1]+((vehicle[1][1]-vehicle[0][1])/2.00), 0.0])

            vehicle.insert(1,triang_vertex)
            # print(ego_car.get_transform())
            # print(obstcl_vehicles[i].get_transform())
            # T = Utils.relative_transform(obstcl_vehicles[i].get_transform(), frame_of_ref)
            # print(T)
            # print("Old Vehicle: {:s}".format(vehicle))
            vehicle = [Utils.transform_location_R(coord,obstcl_vehicles[i].get_transform(), inv = True) for coord in vehicle]
            vehicle = [Utils.transform_location_R(coord, frame_of_ref) for coord in vehicle]
            obstcl_vehicles_boxes_carla_T.append(vehicle)
            # print("New Vehicle: {:s}".format(vehicle))
                
            


        self.obstcl_vehicles_plgns = [LinearRing([(box[0][0:2]), (box[1][0:2]), (box[2][0:2]), (box[4][0:2]), (box[3][0:2])]) for box in obstcl_vehicles_boxes_carla_T]
    

    # @profile
    def update(self, pos_meshgrid, json_params):
        """
        Calculate car potential for a single (obstacle) 
        Args:
            x (float): x coordinate of a position
            y (float): x coordinate of a position
            poly (Shapely Polygon Object): Polygon object representing obstacle
            json_params (JSON Dictionary Object): Parameters Object   
        Returns:
            (float): Potential field value due to obstable
        """

        def eval_car_pot(x, y, obstcl_vehicle):
            # Check if point inside obstacle polygon then return max pot
            if Polygon(obstcl_vehicle).contains(Point(x,y)):
                return 20.0
            else:
                # Check point behind vehicle
                Kd = obstcl_vehicle.distance(Point(x,y))
                return yukawa_pot(json_params,Kd)


        # Vectorize and evaluate potential for mesh grid
        func = lambda obstcl_vehicle : np.vectorize(eval_car_pot, excluded=['obstcl_vehicle'])(x=pos_meshgrid[0], y=pos_meshgrid[1], obstcl_vehicle=obstcl_vehicle)
        car_z = [func(obstcl_vehicle) for obstcl_vehicle in self.obstcl_vehicles_plgns]
        # car_z = Parallel(n_jobs=4)(delayed(func)(obstcl_vehicle) for obstcl_vehicle in self.obstcl_vehicles_plgns)
        # temp = self.plgn_list[0]
        # print("What is this?? : {:s}".format(np.asarray(temp)[0]))
        return np.array(car_z).sum(axis=0)


class LanePotential(object):
    """docstring for LanePotential"""
    def __init__(self):
        super(LanePotential, self).__init__()
        self.lane_edges_transformed = []
        self.curr_waypoint = None
        self.ego_transform = None

    def update_state(self, curr_waypoint, ego_transform):
        self.curr_waypoint = curr_waypoint
        self.ego_transform = ego_transform
        lane_edges = Utils.getLaneEdges(curr_waypoint)
        self.lane_edges_transformed = [Utils.transform_location_R(np.array([lane_edge[0], lane_edge[1], 0]),
                                        ego_transform)[:-1]  for lane_edge in lane_edges]
        # print("Lane Edges: {:s}").format(lane_edges)
        # print("Lane Edges Transformed: {:s}").format(self.lane_edges_transformed)

    def update(self, pos_meshgrid, json_params):
        """
        Calculate car potential for a single (obstacle) 
        Args:
            x (float): x coordinate of a position
            y (float): x coordinate of a position
            poly (Shapely Polygon Object): Polygon object representing obstacle
            json_params (JSON Dictionary Object): Parameters Object   
        Returns:
            (float): Potential field value due to obstable
        """

        # print(self.lane_edges_transformed)
        U = np.zeros(pos_meshgrid[1].shape)  
        # print("Ego yaw: {:f}").format(Utils.deg360(self.ego_transform.rotation.yaw))
        # print("Waypoint yaw: {:f}").format(Utils.deg360(self.curr_waypoint.transform.rotation.yaw))
        theta = -(Utils.deg360(-self.ego_transform.rotation.yaw) - Utils.deg360(-self.curr_waypoint.transform.rotation.yaw))*np.pi/180
        # theta = 0.0
        # print("The angle between: {:f}").format(theta*180/np.pi)
        #Calculate Road potential 
        # if theta < (np.pi/2) and theta > -(np.pi/2): 
        U = U + np.multiply(0.5*json_params["Road_scale_factor"], 
                            np.divide(1.0,np.square((np.sin(theta)*(pos_meshgrid[0] - self.lane_edges_transformed[0][0])+
                                                    (np.cos(theta)*(pos_meshgrid[1] - self.lane_edges_transformed[0][1]))))))

        U = U + np.multiply(0.5*json_params["Road_scale_factor"], 
                            np.divide(1.0,np.square((np.sin(theta)*(pos_meshgrid[0] - self.lane_edges_transformed[-1][0])+
                                                    (np.cos(theta)*(pos_meshgrid[1] - self.lane_edges_transformed[-1][1]))))))
        mask1 =  ((np.sin(theta)*pos_meshgrid[0])+(np.cos(theta)*pos_meshgrid[1]) > (np.sin(theta)*self.lane_edges_transformed[0][0] + np.cos(theta)*self.lane_edges_transformed[0][1])) 
        mask2 =  ((np.sin(theta)*pos_meshgrid[0])+(np.cos(theta)*pos_meshgrid[1]) < (np.sin(theta)*self.lane_edges_transformed[-1][0] + np.cos(theta)*self.lane_edges_transformed[-1][1]))
        # print(mask1.shape)
        # print(mask2.shape)
        mask = mask1&mask2
        U = np.where(mask, U, 20)
        
        #Calculate lane potential
        for i,lane in enumerate(self.lane_edges_transformed[1:-1]):
            U = U + np.multiply(json_params["Lane_Alane"], 
                                np.exp(-np.divide(((np.sin(theta)*pos_meshgrid[1])+(np.cos(theta)*pos_meshgrid[0])-(np.sin(theta)*lane[0] + np.cos(theta)*lane[1]))**2, 
                                (2 * (json_params["Lane_widthfactor"]*abs((np.sin(theta)*self.lane_edges_transformed[i+1][0] + np.cos(theta)*self.lane_edges_transformed[i][1])-
                                                                            (np.sin(theta)*self.lane_edges_transformed[i][0] + np.cos(theta)*self.lane_edges_transformed[i][1])))**2))))

        return U

class ReachableSet(object):
    """docstring for ReachableSet"""
    def __init__(self, json_params):
        super(ReachableSet, self).__init__()
        self.ax_max = json_params["ax_max"]
        self.delta_max = json_params["delta_max"]
        self.v_des = json_params["v_des"]/3.6     # Convert to m/s 
        self.delta_t = json_params["delta_t"] # time step size (seconds)
        self.t_max = json_params["t_reach_horizon"] # max sim time (seconds)
        self.t = np.linspace(0, self.t_max, self.t_max/self.delta_t)
        self.reach_set = None

        
    def update(self, ego_car):

        def generate_reachset(t, x_init, a_max, delta_max, v_des = 5.0, v_des_enable = True):

            dx_vals_int_1 = integrate.odeint(bicycle_model, x_init, t, args=(np.array([a_max, delta_max]),))
            dx_vals_int_2 = integrate.odeint(bicycle_model, x_init, t, args=(np.array([a_max, -delta_max]),))
        
            # Find Points where velocity exceeds
            idx = np.where(dx_vals_int_2[:,3]>v_des)
            # print(idx)
            if idx[0].size == 0 or (not v_des_enable):
                v_max_idx = None
            else: 
                v_max_idx = idx[0][0]

            edge_points = []
            for x in list(np.linspace(-1,1,18)):
                dx_vals_int = integrate.odeint(bicycle_model, x_init, t, args=(np.array([a_max, x*delta_max]),))
                if v_max_idx is None:
                    edge_points.append((dx_vals_int[-1,0],dx_vals_int[-1,1]))
                else:
                    edge_points.append((dx_vals_int[v_max_idx,0],dx_vals_int[v_max_idx,1]))
      
            
            # print(v_max_idx)
            reach_set_tup1 = zip(dx_vals_int_1[:v_max_idx, 0],dx_vals_int_1[:v_max_idx, 1])
            reach_set_tup2 = zip(dx_vals_int_2[:v_max_idx, 0],dx_vals_int_2[:v_max_idx, 1])
            reach_set_tup1.reverse()
            reach_set_tup1.extend(reach_set_tup2)
            reach_set_tup1.extend(edge_points)

            if v_max_idx == 0:
                rospy.logwarn("Warning!! No reachable states that obey velocity limits. Desired Speed: {:f} km/h".format(v_des*3.6))
                print(reach_set_tup1)

            return reach_set_tup1

        v = ego_car.get_velocity()
        self.ego_car_vel = math.sqrt(v.x**2 + v.y**2 + v.z**2)
        x_0 = np.array([0.0,0.0,0.0, self.ego_car_vel])
        
        
        
        reach_set_tup1 = generate_reachset(self.t, x_0, self.ax_max, self.delta_max, v_des=self.v_des)
        self.reach_set = LinearRing(reach_set_tup1)







        

