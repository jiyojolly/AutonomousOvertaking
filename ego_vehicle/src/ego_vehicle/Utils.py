#!/usr/bin/env python
#
# Copyright (c) 2020 
# Jiyo Jolly Palatti
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Helper functions for Carla


"""

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
import math
import carla
import rospy
import numpy as np
import transforms3d



def getLaneEdges(waypoint):

    curr_lane = waypoint
    left_lanes = []
    right_lanes = []

    left_lane_edges = []
    curr_lane_edges = []
    right_lane_edges = []
    
    # Find all left and right lanes recursively
    if (curr_lane.get_left_lane() is not None and 
        curr_lane.get_left_lane().lane_type is carla.LaneType.Driving):
        left_lanes = [curr_lane.get_left_lane()]
    if (curr_lane.get_right_lane() is not None and 
        curr_lane.get_right_lane().lane_type is carla.LaneType.Driving):
        right_lanes = [curr_lane.get_right_lane()]

    while left_lanes and not(left_lanes[-1].get_left_lane() is None or 
                left_lanes[-1].get_right_lane() is None or 
                left_lanes[-1].get_left_lane().lane_type is carla.LaneType.Driving or 
                left_lanes[-1].get_right_lane().lane_type is carla.LaneType.Driving):

            if curr_lane.lane_id*left_lanes[-1].lane_id > 0: left_lanes.append(left_lanes[-1].get_left_lane())
            else: left_lanes.append(left_lanes[-1].get_right_lane())

    while right_lanes and not(right_lanes[-1].get_left_lane() is None or 
                right_lanes[-1].get_right_lane() is None or 
                right_lanes[-1].get_left_lane().lane_type is carla.LaneType.Driving or 
                right_lanes[-1].get_right_lane().lane_type is carla.LaneType.Driving): 
        
        if curr_lane.lane_id*right_lanes[-1].lane_id > 0: right_lanes.append(right_lanes[-1].get_right_lane()) 
        else: right_lanes.append(right_lanes[-1].get_left_lane())

    # Add current lane to both lists
    left_lanes.insert(0, curr_lane)
    right_lanes.insert(0, curr_lane)
         
    #find lane edges from lane mids
    for lane_mid in left_lanes:
        if left_lanes[0].transform.rotation.yaw > 180:
            if lane_mid.transform.rotation.yaw <= 180:
                xx = lane_mid.transform.location.x + (lane_mid.lane_width/2.0)*(math.cos(lane_mid.transform.rotation.yaw*0.01745329 - (0.5*math.pi)))
                yy = lane_mid.transform.location.y + (lane_mid.lane_width/2.0)*(math.sin(lane_mid.transform.rotation.yaw*0.01745329 - (0.5*math.pi)))
            else:
                xx = lane_mid.transform.location.x + (lane_mid.lane_width/2.0)*(math.cos(lane_mid.transform.rotation.yaw*0.01745329 + (0.5*math.pi)))
                yy = lane_mid.transform.location.y + (lane_mid.lane_width/2.0)*(math.sin(lane_mid.transform.rotation.yaw*0.01745329 + (0.5*math.pi)))
        else:
            if lane_mid.transform.rotation.yaw > 180:
                xx = lane_mid.transform.location.x + (lane_mid.lane_width/2.0)*(math.cos(lane_mid.transform.rotation.yaw*0.01745329 - (0.5*math.pi)))
                yy = lane_mid.transform.location.y + (lane_mid.lane_width/2.0)*(math.sin(lane_mid.transform.rotation.yaw*0.01745329 - (0.5*math.pi)))
            else:
                xx = lane_mid.transform.location.x + (lane_mid.lane_width/2.0)*(math.cos(lane_mid.transform.rotation.yaw*0.01745329 + (0.5*math.pi)))
                yy = lane_mid.transform.location.y + (lane_mid.lane_width/2.0)*(math.sin(lane_mid.transform.rotation.yaw*0.01745329 + (0.5*math.pi)))
        left_lane_edges.append((xx,yy))
        if (left_lanes.index(lane_mid) == len(left_lanes)-1):
            if left_lanes[0].transform.rotation.yaw > 180:
                if lane_mid.transform.rotation.yaw <= 180:
                    xx = lane_mid.transform.location.x + (lane_mid.lane_width/2.0)*(math.cos(lane_mid.transform.rotation.yaw*0.01745329 + (0.5*math.pi)))
                    yy = lane_mid.transform.location.y + (lane_mid.lane_width/2.0)*(math.sin(lane_mid.transform.rotation.yaw*0.01745329 + (0.5*math.pi)))
                else:
                    xx = lane_mid.transform.location.x + (lane_mid.lane_width/2.0)*(math.cos(lane_mid.transform.rotation.yaw*0.01745329 - (0.5*math.pi)))
                    yy = lane_mid.transform.location.y + (lane_mid.lane_width/2.0)*(math.sin(lane_mid.transform.rotation.yaw*0.01745329 - (0.5*math.pi)))
            else:
                if lane_mid.transform.rotation.yaw > 180:
                    xx = lane_mid.transform.location.x + (lane_mid.lane_width/2.0)*(math.cos(lane_mid.transform.rotation.yaw*0.01745329 + (0.5*math.pi)))
                    yy = lane_mid.transform.location.y + (lane_mid.lane_width/2.0)*(math.sin(lane_mid.transform.rotation.yaw*0.01745329 + (0.5*math.pi)))
                else:
                    xx = lane_mid.transform.location.x + (lane_mid.lane_width/2.0)*(math.cos(lane_mid.transform.rotation.yaw*0.01745329 - (0.5*math.pi)))
                    yy = lane_mid.transform.location.y + (lane_mid.lane_width/2.0)*(math.sin(lane_mid.transform.rotation.yaw*0.01745329 - (0.5*math.pi)))

            left_lane_edges.append((xx,yy))

    for lane_mid in right_lanes:
        if right_lanes[0].transform.rotation.yaw > 180:
            if lane_mid.transform.rotation.yaw <= 180:
                xx = lane_mid.transform.location.x + (lane_mid.lane_width/2.0)*(math.cos(lane_mid.transform.rotation.yaw*0.01745329 + (0.5*math.pi)))
                yy = lane_mid.transform.location.y + (lane_mid.lane_width/2.0)*(math.sin(lane_mid.transform.rotation.yaw*0.01745329 + (0.5*math.pi)))
            else:
                xx = lane_mid.transform.location.x + (lane_mid.lane_width/2.0)*(math.cos(lane_mid.transform.rotation.yaw*0.01745329 - (0.5*math.pi)))
                yy = lane_mid.transform.location.y + (lane_mid.lane_width/2.0)*(math.sin(lane_mid.transform.rotation.yaw*0.01745329 - (0.5*math.pi)))
        else:
            if lane_mid.transform.rotation.yaw > 180:
                xx = lane_mid.transform.location.x + (lane_mid.lane_width/2.0)*(math.cos(lane_mid.transform.rotation.yaw*0.01745329 + (0.5*math.pi)))
                yy = lane_mid.transform.location.y + (lane_mid.lane_width/2.0)*(math.sin(lane_mid.transform.rotation.yaw*0.01745329 + (0.5*math.pi)))
            else:
                xx = lane_mid.transform.location.x + (lane_mid.lane_width/2.0)*(math.cos(lane_mid.transform.rotation.yaw*0.01745329 - (0.5*math.pi)))
                yy = lane_mid.transform.location.y + (lane_mid.lane_width/2.0)*(math.sin(lane_mid.transform.rotation.yaw*0.01745329 - (0.5*math.pi)))
        right_lane_edges.append((xx,yy))
        if (right_lanes.index(lane_mid) == len(right_lanes)-1):
            if right_lanes[0].transform.rotation.yaw > 180:
                if lane_mid.transform.rotation.yaw <= 180:
                    xx = lane_mid.transform.location.x + (lane_mid.lane_width/2.0)*(math.cos(lane_mid.transform.rotation.yaw*0.01745329 - (0.5*math.pi)))
                    yy = lane_mid.transform.location.y + (lane_mid.lane_width/2.0)*(math.sin(lane_mid.transform.rotation.yaw*0.01745329 - (0.5*math.pi)))
                else:
                    xx = lane_mid.transform.location.x + (lane_mid.lane_width/2.0)*(math.cos(lane_mid.transform.rotation.yaw*0.01745329 + (0.5*math.pi)))
                    yy = lane_mid.transform.location.y + (lane_mid.lane_width/2.0)*(math.sin(lane_mid.transform.rotation.yaw*0.01745329 + (0.5*math.pi)))
            else:
                if lane_mid.transform.rotation.yaw > 180:
                    xx = lane_mid.transform.location.x + (lane_mid.lane_width/2.0)*(math.cos(lane_mid.transform.rotation.yaw*0.01745329 - (0.5*math.pi)))
                    yy = lane_mid.transform.location.y + (lane_mid.lane_width/2.0)*(math.sin(lane_mid.transform.rotation.yaw*0.01745329 - (0.5*math.pi)))
                else:
                    xx = lane_mid.transform.location.x + (lane_mid.lane_width/2.0)*(math.cos(lane_mid.transform.rotation.yaw*0.01745329 + (0.5*math.pi)))
                    yy = lane_mid.transform.location.y + (lane_mid.lane_width/2.0)*(math.sin(lane_mid.transform.rotation.yaw*0.01745329 + (0.5*math.pi)))
            right_lane_edges.append((xx,yy))

    # rospy.logwarn("Left Lanes Edges: {:s}".format(left_lane_edges))
    # rospy.logwarn("Right Lanes Edges: {:s}".format(right_lane_edges))

    # Reverse and join to get all lane edges
    left_lane_edges.reverse()
    if len(left_lane_edges) <= 2:
        final_lane_edges = right_lane_edges
    elif len(right_lane_edges) <= 2:
        final_lane_edges = left_lane_edges
    else:
        left_lane_edges.extend(right_lane_edges[2:-1])
        final_lane_edges  = left_lane_edges

    # rospy.logwarn("Combined Lanes Edges: {:s}".format(final_lane_edges))
    return final_lane_edges



def carla_vec2numpy(vec):
    return np.array([vec.x, vec.y, vec.z])


def transform2mat(t):
    #
    xyz = carla_vec2numpy(t.location)
    xyz = np.array([xyz[0], -xyz[1], xyz[2]])
    roll = -t.rotation.roll
    pitch = t.rotation.pitch    
    yaw = -t.rotation.yaw
    R = transforms3d.taitbryan.euler2mat( 
        np.deg2rad(yaw), np.deg2rad(pitch), np.deg2rad(roll))

    M = np.zeros((4, 4))
    M[0:3, 0:3] = R
    M[0:3, 3] = xyz
    M[3, 3] = 1

    return M


def mat2transform(M):

    pitch, roll, yaw = transforms3d.taitbryan.mat2euler(M[0:3, 0:3])
    roll = np.rad2deg(roll)
    pitch = np.rad2deg(pitch)
    yaw = np.rad2deg(yaw)

    T = carla.Transform(
        carla.Location(x=M[0, 3], y=-M[1, 3], z=M[2, 3]),
        carla.Rotation(pitch=pitch, yaw=-yaw, roll=-roll),
    )

    return T


def relative_transform(source, target):
    source_t = transform2mat(source)
    target_t = transform2mat(target)
    target_t_inv = np.linalg.inv(target_t)

    relative_transform_mat = np.dot(source_t, target_t_inv)

    relative_transform_transform = mat2transform(relative_transform_mat)

    return relative_transform_transform

def switch_CoordinateSystem(xyz):
    xyz[1] = -xyz[1]
    return xyz

def transform_location(loc, target_transform, inv = False, loc_CS = 'L', transformloc_CS ='R'):
    """
    Takes a location in the Carla left 
    """

    if loc_CS is 'L':
        loc = switch_CoordinateSystem(np.append(loc,1))
    elif loc_CS is 'R':
        loc = np.append(loc,1)
    else:
        raise ValueError("Unrecognised parameters for location transformation")


    T = transform2mat(target_transform)
    if inv is True:
        tr_loc = np.dot(T, loc)
    else:    
        tr_loc = np.dot(np.linalg.inv(T), loc)

    if transformloc_CS is 'L':
        return switch_CoordinateSystem(tr_loc)[:-1]
    elif transformloc_CS is 'R':
        return tr_loc[:-1]
    else:
        raise ValueError("Unrecognised parameters for location transformation")



def deg360(angle):
    return abs(angle%360)



# def get_nearestcar():

#     print(self.sub_cars[0].get_location())

