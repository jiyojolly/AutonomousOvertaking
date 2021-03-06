# Copyright (c) 2020 
# Jiyo Jolly Palatti
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Classes and associated methods that perform risk map calculations, reachable set and decision making. 
"""
#Std packages
import rospy
import numpy as np
import math
from scipy import integrate
import carla

#3rd party packages
from shapely.geometry import LineString, Polygon, LinearRing, Point, box, asPoint
import shapely.ops
from joblib import Parallel, delayed
from statemachine import StateMachine, State

#custom packages
import utils
from ego_vehicle.kinematic_bicycle_model import bicycle_model

#Message definitions
from custom_msgs.msg import Float64Arr4
from std_msgs.msg import Float32MultiArray


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
        self.ego_plgn_world = None
        self.obstcl_vehicles_locs = []
        self.obstcl_vehicles_vels = []
        self.obstcl_vehicles_plgns = []
        self.obstcl_vehicles_plgns_world = []

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
        get_boundingbox_transformed = lambda vehicle, transform : [utils.transform_location(np.array([vertex.x, vertex.y, vertex.z]),transform)
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

        def vel_scale_front(vel_ego, vel_obstcar):

            if vel_obstcar >= json_params["d_zero"]/json_params["Tf"]: psi_zero = json_params["d_zero"]/(json_params["Tf"]*vel_obstcar)
            else: psi_zero = 1.0
            # print("Psi value:{:f}".format(psi_zero))
            # if vel_ego > vel_obstcar: psi = psi_zero * np.exp(-json_params["beta"]*(vel_ego-vel_obstcar))
            # else: psi = psi_zero 
            # print("Psi value:{:f}".format(psi))
            return psi_zero

        ##Update State variables
        #Ego Vehicle
        # frame_of_ref = carla.Transform(location=carla.Location(0.0,0.0,0.0), rotation=carla.Rotation(0.0,0.0,0.0))
        frame_of_ref = ego_car.get_transform()

        self.ego_car_location = utils.transform_location(np.array([ego_car.get_transform().location.x, ego_car.get_transform().location.y, ego_car.get_transform().location.z]),frame_of_ref)
        v = ego_car.get_velocity()
        self.ego_car_vel = math.sqrt(v.x**2 + v.y**2 + v.z**2)
        ego_box = get_boundingbox_transformed(ego_car, frame_of_ref)
        ego_box_world = get_boundingbox_transformed(ego_car, carla.Transform(location=carla.Location(0.0,0.0,0.0), rotation=carla.Rotation(0.0,0.0,0.0)))
        self.ego_plgn = LinearRing([(ego_box[0][0:2]), (ego_box[1][0:2]), (ego_box[3][0:2]), (ego_box[2][0:2])])
        self.ego_plgn_world = LinearRing([(ego_box_world[0][0:2]), (ego_box_world[1][0:2]), (ego_box_world[3][0:2]), (ego_box_world[2][0:2])])

        if obstcl_vehicles:
            #Obstacle Vehicles in Ego car frame
            self.obstcl_vehicles_locs = [utils.transform_location(np.array([vehicle.get_location().x, vehicle.get_location().y, vehicle.get_location().z]), 
                                                                   frame_of_ref) for vehicle in obstcl_vehicles]
            self.obstcl_vehicles_vels = [math.sqrt(vehicle.get_velocity().x**2 + vehicle.get_velocity().y**2 + vehicle.get_velocity().z**2) for vehicle in obstcl_vehicles]
            obstcl_vehicles_boxes_carla = [get_boundingbox_local(vehicle) for vehicle in obstcl_vehicles]

            obstcl_vehicles_boxes_carla_ego = []
            obstcl_vehicles_boxes_carla_world = []
            for i,vehicle in enumerate(obstcl_vehicles_boxes_carla):
                triang_vertex_b = np.array([vehicle[0][0]-(json_params["delta_vertex"]*(np.clip(float(1.0)/vel_scale(self.ego_car_vel, self.obstcl_vehicles_vels[i]),1, json_params["d_zero"]))), 
                                    vehicle[0][1]+((vehicle[1][1]-vehicle[0][1])/2.00), 0.0])
                triang_vertex_i = np.array([vehicle[2][0]+(json_params["delta_vertex"]*(np.clip(float(1.0)/vel_scale_front(self.ego_car_vel, self.obstcl_vehicles_vels[i]),1, json_params["d_zero"]))), 
                                    vehicle[0][1]+((vehicle[1][1]-vehicle[0][1])/2.00), 0.0])

                vehicle.insert(1,triang_vertex_b)
                vehicle.insert(4,triang_vertex_i)
                # print(ego_car.get_transform())
                # print(obstcl_vehicles[i].get_transform())
                # T = utils.relative_transform(obstcl_vehicles[i].get_transform(), frame_of_ref)
                # print(T)
                # print("Old Vehicle: {:s}".format(vehicle))
                vehicle = [utils.transform_location(coord,obstcl_vehicles[i].get_transform(), inv = True) for coord in vehicle]
                obstcl_vehicles_boxes_carla_world.append(vehicle)
                vehicle = [utils.transform_location(coord, frame_of_ref, loc_CS = 'R') for coord in vehicle]
                obstcl_vehicles_boxes_carla_ego.append(vehicle)
                # print("New Vehicle: {:s}".format(vehicle))
                    
                


            # print(obstcl_vehicles_boxes_carla_T)
            self.obstcl_vehicles_plgns = [LinearRing([(box[0][0:2]), (box[1][0:2]), (box[2][0:2]), (box[5][0:2]), (box[4][0:2]), (box[3][0:2])]) for box in obstcl_vehicles_boxes_carla_ego]
            self.obstcl_vehicles_plgns_world = [LinearRing([(box[0][0:2]), (box[1][0:2]), (box[2][0:2]), (box[5][0:2]), (box[4][0:2]), (box[3][0:2])]) for box in obstcl_vehicles_boxes_carla_world]
    

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
        car_z = np.zeros(pos_meshgrid[0].shape)
        if self.obstcl_vehicles_plgns:
            func = lambda obstcl_vehicle : np.vectorize(eval_car_pot, excluded=['obstcl_vehicle'])(x=pos_meshgrid[0], y=pos_meshgrid[1], obstcl_vehicle=obstcl_vehicle)
            car_z = [func(obstcl_vehicle) for obstcl_vehicle in self.obstcl_vehicles_plgns]
            # car_z = Parallel(n_jobs=4)(delayed(func)(obstcl_vehicle) for obstcl_vehicle in self.obstcl_vehicles_plgns)
            # temp = self.plgn_list[0]
            # print("What is this?? : {:s}".format(np.asarray(temp)[0]))
            return np.array(car_z).sum(axis=0)
        else:
            return car_z


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
        lane_edges = utils.getLaneEdges(curr_waypoint)
        self.lane_edges_transformed = [utils.transform_location(np.array([lane_edge[0], lane_edge[1], 0]),
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
        U = np.zeros(pos_meshgrid[1].shape)  
        # print("Ego yaw: {:f}").format(utils.deg360(-self.ego_transform.rotation.yaw))
        # print("Waypoint yaw: {:f}").format(utils.deg360(-self.curr_waypoint.transform.rotation.yaw))
        theta = utils.deg360(utils.deg360(-self.curr_waypoint.transform.rotation.yaw) - utils.deg360(-self.ego_transform.rotation.yaw) )*np.pi/180

        #Calculate Road potential
        '''
        Theta is taken as negative due to the nature of 
        applying rotation on functions applied on meshgrid
        '''
        theta = -theta
        U = U + np.multiply(0.5*json_params["Road_scale_factor"], 
                                np.divide(1.0,np.square((np.sin(theta)*(pos_meshgrid[0] - self.lane_edges_transformed[0][0])+
                                                        (np.cos(theta)*(pos_meshgrid[1] - self.lane_edges_transformed[0][1]))))))

        U = U + np.multiply(0.5*json_params["Road_scale_factor"], 
                            np.divide(1.0,np.square((np.sin(theta)*(pos_meshgrid[0] - self.lane_edges_transformed[-1][0])+
                                                    (np.cos(theta)*(pos_meshgrid[1] - self.lane_edges_transformed[-1][1]))))))
        
        # if theta < (np.pi/2) or theta > (3*np.pi/2): 
        mask1 =  ((np.sin(theta)*pos_meshgrid[0])+(np.cos(theta)*pos_meshgrid[1]) < (np.sin(theta)*self.lane_edges_transformed[0][0] +
                                                                                 np.cos(theta)*self.lane_edges_transformed[0][1])) 
        mask2 =  ((np.sin(theta)*pos_meshgrid[0])+(np.cos(theta)*pos_meshgrid[1]) > (np.sin(theta)*self.lane_edges_transformed[-1][0] +
                                                                                 np.cos(theta)*self.lane_edges_transformed[-1][1]))
        mask = mask1&mask2
        U = np.where(mask, U, 20)

        #Calculate lane potential
        for i,lane in enumerate(self.lane_edges_transformed[1:-1]):
            U = U + np.multiply(json_params["Lane_Alane"], 
                                np.exp(-np.divide((np.sin(theta)*(pos_meshgrid[0] - lane[0]) + np.cos(theta)*(pos_meshgrid[1] - lane[1]))**2, 
                                (2 * (json_params["Lane_widthfactor"]*abs(np.sin(theta)*(self.lane_edges_transformed[i+1][0] - self.lane_edges_transformed[i][0]) + 
                                                                           np.cos(theta)*(self.lane_edges_transformed[i+1][1] - self.lane_edges_transformed[i][1])))**2))))

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
            reach_set_tup1 = list(zip(dx_vals_int_1[:v_max_idx, 0],dx_vals_int_1[:v_max_idx, 1]))
            reach_set_tup2 = list(zip(dx_vals_int_2[:v_max_idx, 0],dx_vals_int_2[:v_max_idx, 1]))
            reach_set_tup1.reverse()
            reach_set_tup1.extend(reach_set_tup2)
            reach_set_tup1.extend(edge_points)

            if v_max_idx == 0:
                rospy.logwarn("Warning!! No reachable states that obey velocity limits. Desired Speed: {:f} km/h".format(v_des*3.6))
                print(reach_set_tup1)

            return reach_set_tup1

        v = ego_car.get_velocity()
        # self.ego_car_vel = math.sqrt(v.x**2 + v.y**2 + v.z**2)
        # x_0 = np.array([0.0,0.0,0.0, self.ego_car_vel])
        x_0 = np.array([0.0,0.0,0.0, self.v_des])
        
        
        
        reach_set_tup1 = generate_reachset(self.t, x_0, self.ax_max, self.delta_max, v_des=self.v_des)
        self.reach_set = LinearRing(reach_set_tup1)


class TargetStateSelection(object):
    """Class that holds the functions to select the required target reference state for 
        MPC. Takes into consideration the nearby cars to choose optimal reference target to 
        minimize distance to final reference point which could be a 
        1. Waypoint ahead that car has to follow
        2. Safe waypoint ahead of the obstacle car infront 
        3. Safe waypoint behind the obstacle car incase of overtaking maneuver cancellation
        4. others

        Publishes the optimal(safe, reachable) reference target via ROS topic

    """
    def __init__(self, json_params):
        super(TargetStateSelection, self).__init__()

        self.d_safe_overtake = json_params["d_safe_overtake"]
        self.d_safe_cruise = json_params["d_safe_cruise"]
        self.v_des = json_params["v_des"]/3.6      
        self.disable_overtake = False
        #Instance variables
        self.ego_car = None
        self.obstcl_cars = []
        self.set_safe_reach_np = None
        self.nearest_car = None
        self.nearest_car_loc_ego = np.array([0,0])
        
        #Final Targets
        self.final_ref_target_ego = np.zeros(2)
        self.final_ref_target_world = np.zeros(3)
        self.x_ref_target_ego = np.zeros(2)
        self.X_ref_target_world = np.zeros(4)

        #Behaviour Selection
        class BehaviourSM(StateMachine):
            lane_keeping = State('Lane Keeping', initial=True)
            overtaking = State('Overtaking')
            
            overtake = lane_keeping.to(overtaking)
            lane_keep = overtaking.to(lane_keeping)

        self.behaviour = BehaviourSM()

    def update_state(self, ego_car, obstcl_cars, set_safe_reach):
        self.ego_car = ego_car
        self.obstcl_cars = obstcl_cars
        self.set_safe_reach_np = np.transpose(np.array(set_safe_reach))
        
        self.get_nearest_car()


    def get_nearest_car(self):
        """ 
        The logic will be expanded to check for nearest 
        lead car in case of many car scenario
        """
        nearest_lead_car = None
        for car in self.obstcl_cars:
            obstcl_car_loc_w = car.get_location()
            obstcl_car_loc_ego = utils.transform_location(np.array([obstcl_car_loc_w.x, obstcl_car_loc_w.y, obstcl_car_loc_w.z]),
                                                         self.ego_car.get_transform(), loc_CS = 'L')
            # d = np.linalg.norm(obstcl_car_loc_ego)
            if obstcl_car_loc_ego[0] > -5 and obstcl_car_loc_ego[0] < 10: 
                if self.nearest_car is None:
                    nearest_lead_car  = car        
                elif (self.nearest_car.id is not car.id) and  obstcl_car_loc_ego[0] > self.nearest_car_loc_ego[0]:
                    nearest_lead_car  = car
                else:
                    nearest_lead_car = self.nearest_car
                
        if nearest_lead_car is not None:
            nearest_lead_car_loc_w = nearest_lead_car.get_location()
            nearest_lead_car_loc_ego = utils.transform_location(np.array([nearest_lead_car_loc_w.x, nearest_lead_car_loc_w.y, nearest_lead_car_loc_w.z]),
                                                         self.ego_car.get_transform(), loc_CS = 'L')
            self.nearest_car = nearest_lead_car 
            self.nearest_car_loc_ego  = nearest_lead_car_loc_ego   
        else:
            self.nearest_car = None
            self.nearest_car_loc_ego = np.array([0,0,0])    

        rospy.logwarn(f"Distance to nearest lead car: {self.nearest_car}, {self.nearest_car_loc_ego}")



        
    def check_overtake_complete(self):

        d = np.linalg.norm(self.final_ref_target_ego)
        if d<2.0:
            return True
        else:
            return False


    def decide_behavior(self):

        if self.nearest_car and not self.disable_overtake: 
            if self.nearest_car_loc_ego[0] > 0: 
                    if not(self.behaviour.is_overtaking):
                        self.behaviour.overtake()
                    else: pass 
            elif self.check_overtake_complete():
                    if not(self.behaviour.is_lane_keeping):
                        self.behaviour.lane_keep()
                    else: pass
            else:
                pass

        # else if self.behaviour.is_overtaking : 
        else:
            if not(self.behaviour.is_lane_keeping):
               self.behaviour.lane_keep()
            else: pass
   
        rospy.logwarn(f"Current behaviour : {self.behaviour.current_state}")

    def update_ref(self, world_map):
        '''
        Decide whether to overtake and calculate reference based on waypoint
        #To overtake or not to
        #overtake_mode = 0  -- cruise
        #overtake_mode = 1  -- overtake
        #overtake_mode = 2  -- cancel overtake
        ''' 
        self.decide_behavior()

        #Lane keeping mode    
        if self.behaviour.is_lane_keeping:
            target_loc_obscl_frame =  np.array([self.d_safe_cruise, 0, 0])
            loc_world = utils.transform_location(target_loc_obscl_frame, self.ego_car.get_transform(), inv = True, loc_CS = 'R')
            waypoint = world_map.get_waypoint(carla.Location(loc_world[0],
                                                             -loc_world[1],
                                                             loc_world[2]), project_to_road=True)
            self.final_ref_target_world = np.array([waypoint.transform.location.x, -waypoint.transform.location.y, waypoint.transform.location.z])
            rospy.logwarn(f"Final Target Ref world frame: {self.final_ref_target_world}")
            # print(final_ref_target_world)

        #Overtake mode
        elif self.behaviour.is_overtaking:
            target_loc_obscl_frame =  np.array([self.d_safe_overtake, 0, 0])
            loc_world = utils.transform_location(target_loc_obscl_frame, self.nearest_car.get_transform(), inv = True, loc_CS = 'R')
            waypoint = world_map.get_waypoint(carla.Location(loc_world[0],
                                                             -loc_world[1],
                                                             loc_world[2]), project_to_road=True)
            self.final_ref_target_world = np.array([waypoint.transform.location.x, -waypoint.transform.location.y, waypoint.transform.location.z])
            rospy.logwarn(f"Final Target Ref world frame: {self.final_ref_target_world}")
            # print(final_ref_target_world)

        else:
            pass



        #Transform ego final frame
        frame_of_ref = self.ego_car.get_transform()        
        final_ref_target_ego_xyz = utils.transform_location(self.final_ref_target_world, frame_of_ref, loc_CS = 'R')
        self.final_ref_target_ego = final_ref_target_ego_xyz[:2]
        
        #Find nearest point from reach, safe set that minimizes the distance to final target state
        # if False:
        if self.set_safe_reach_np.size != 0:
            ref_loc_index =  np.argmin(np.linalg.norm(self.final_ref_target_ego[:2] - self.set_safe_reach_np, axis = 1))
            # print(self.set_safe_reach_np.shape)
            ref_target_safe_reach =  self.set_safe_reach_np[ref_loc_index,:]
            # print(self.ref_target_safe_reach) 
            rospy.logwarn(f"Intd. MPC Ref in Ego frame: {ref_target_safe_reach}")

            # Transform to world frame before publishing..
            self.x_ref_target_ego = ref_target_safe_reach
            x_ref_target_world = utils.transform_location( np.array((ref_target_safe_reach[0],ref_target_safe_reach[1],0.0)), frame_of_ref, inv = True, loc_CS = 'R')
            x_ref_target_world = x_ref_target_world[:2]
            self.X_ref_target_world = np.append(x_ref_target_world, [1.571, self.v_des])
        else:
            ref_target_safe_reach = np.array([0.0, 0.0])
            self.x_ref_target_ego = ref_target_safe_reach
            # Transform to world frame before publishing..
            x_ref_target_world = utils.transform_location( np.array((ref_target_safe_reach[0],ref_target_safe_reach[1],0.0)), frame_of_ref, inv = True, loc_CS = 'R')
            x_ref_target_world = x_ref_target_world[:2]
            self.X_ref_target_world = np.append(x_ref_target_world, [1.571, 0.0])

        rospy.logwarn(f"Target Ref for MPC in world frame: {self.X_ref_target_world}")
       








        

