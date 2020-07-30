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
from shapely.geometry import Polygon, LineString, Point, box, asPoint
import shapely.ops
from joblib import Parallel, delayed

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
        self.plgn_list = None


    def update_obst_plgn(self, sub_car_locs, json_params):
        """
        Create/Update Obstacle polygons
        Args:
            x (float): x coordinate of a position
            y (float): x coordinate of a position
            poly (Shapely Polygon Object): Polygon object representing obstacle
            json_params (JSON Dictionary Object): Parameters Object   
        Returns:
            (float): Potential field value due to obstable
        """
        self.plgn_list = [box(np.asarray(loc)[0]-(json_params["Obstacle_W"]/2.0),
                np.asarray(loc)[1]-(json_params["Obstacle_L"]/2.0),
                np.asarray(loc)[0]+(json_params["Obstacle_W"]/2.0),
                np.asarray(loc)[1]+(json_params["Obstacle_L"]/2.0)) for loc in sub_car_locs ]

    def eval_car_pot(self, x, y, plgn, json_params):
        """
        Calculate car potential for a single (obstacle) 
        Args:
            x (float): x coordinate of a position
            y (float): x coordinate of a position
            plgn (Shapely Polygon Object): Polygon object representing obstacle
            json_params (JSON Dictionary Object): Parameters Object   
        Returns:
            (float): Potential field value due to obstable
        """
        # Check if point inside obstacle polygon then return max pot

        # print("X Y Bounds: {:s} Min Y: {:f}".format(plgn.bounds, plgn.bounds[1]))
        if y >= plgn.bounds[1]:     
            if plgn.contains(Point(x,y)):
                return 20.0
            else:
                Kd = plgn.boundary.distance(Point(x,y))
                return yukawa_pot(json_params,Kd)
        else:
            # print("Polygon exterior coords: {:s}".format(list(plgn.exterior.coords)))     
            triang_vertex = (plgn.bounds[0]+1, plgn.bounds[1]-10)    
            new_plgn = list(plgn.exterior.coords)
            new_plgn.insert(-1,triang_vertex)
            plgn = Polygon(new_plgn)
            
            if plgn.contains(Point(x,y)):
                return 20.0
            else:
                Kd = plgn.boundary.distance(Point(x,y))
                return yukawa_pot(json_params,Kd)

    def update_car_pot(self, pos_meshgrid, json_params):
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

        # Vectorize and evaluate potential for mesh grid
        func = lambda plgn : np.vectorize(self.eval_car_pot)(pos_meshgrid[0], pos_meshgrid[1], plgn, json_params)
        car_z = Parallel(n_jobs=4)(delayed(func)(plgn) for plgn in self.plgn_list)

        return np.clip(np.array(car_z).sum(axis=0),-20,20)
