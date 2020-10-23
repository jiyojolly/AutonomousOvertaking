import artf_pot_funcs
from artf_pot_funcs import CarPotential
import os
import json
import numpy as np
from shapely.geometry import Polygon, LineString, Point, box, asPoint
import shapely.ops
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import carla
import Utils

# You probably won't need this if you're embedding things in a tkinter plot...
plt.ion()


def main():
    # Read params from file
    param_definition_file = "/home/jiyo/workspace/autonomous-overtaking/autonomous-overtaking/ego_vehicle/config/params.json"
    if not os.path.exists(param_definition_file):
        raise RuntimeError(
            "Could not read param-definition from {}".format(param_definition_file))
    json_params = None
    with open(param_definition_file) as handle:
        json_params = json.loads(handle.read())

    # Add fig and axes for contour and 3d surface plots
    fig_3d = plt.figure()
    ax_3d = fig_3d.add_subplot(111, projection='3d')

    fig_2d = plt.figure()
    ax_2d = fig_2d.add_subplot(111)

    # Define X, Y, Z limits for plots
    xlims = np.array(np.array([-50,50]))
    ylims = np.array([-50,50])
    zlims = np.array([0,30])

    # Define X, Y limits for area under consideration
    x_vision_limit = np.array(json_params["X_limit"])
    y_vision_limit = np.array(json_params["Y_limit"])


    
    #TO DO Convert position (x,y) to Point objects

    # Dummy ego car postion Point(x,y)
    ego_car_loc = carla.Transform(carla.Location(x=-197.0, y=-85.0, z=0.0), carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))


    # Dummy subject car #1 postion  
    # sub_car_locs = [np.array([-0.0,-10.0,0.0]), np.array([0.0,20.0,0.0]), np.array([-5,0.0,0.0])]
    sub_car_locs = [np.array([-198.0,-95.0,0.0]), np.array([-198.0,-75.0,0.0]), np.array([-198,-65.0,0.0])]
    sub_car_vel = [4.2, 5.3, 3.2]

    # Create car potential class
    car_potential = CarPotential()
    
    car_potential.update_state(ego_car_loc, 5.0, sub_car_locs, sub_car_vel, json_params)   

    # Create mesh for potential calculation
    x = np.arange(x_vision_limit[0],x_vision_limit[1], 0.5)
    y = np.arange(y_vision_limit[0],y_vision_limit[1], 0.5)
    pos_meshgrid = np.meshgrid(x, y, sparse=True, copy=True)

    z = car_potential.update(pos_meshgrid,json_params)  
    
    # Print min max of pot field
    print("Details of Mesh grid values: Shape={:s}, Min z value={:.2f}, Max z value={:.2f}".format(z.shape, np.amin(z), np.amax(z)))
   

    def plot_all(z):
        # Ego car postion Point(x,y)
        ego_zero = np.array([0.0, 0.0, 0.0])

        # Clear previous axis data
        # if surf is not None: surf.remove()
        # if ax_2d is not None: ax_2d.clear()
        
        ax_2d.plot(ego_zero[0], ego_zero[1], marker='x')
        # ax_2d.plot(lane_edges[:,0]-ego_car_loc.x, np.asarray(lane_edges)[:,1]-ego_car_loc.y, marker='*')

        # Plot obstacle cars
        [ax_2d.plot(Utils.transform_location_R(obst_car, ego_car_loc)[0], Utils.transform_location_R(obst_car, ego_car_loc)[1], marker='x') for obst_car in sub_car_locs]

        # Plot obstacle polygons
        [ax_2d.plot(np.asarray(obst_car[0])[:,0],np.asarray(obst_car[0])[:,1]) for obst_car in car_potential.obst_car_list]
        
        # Plot Nearest point to obstable
        [ax_2d.plot([shapely.ops.nearest_points(Point(ego_zero), obst_car[0])[0].x, shapely.ops.nearest_points(Point(ego_zero), obst_car[0])[1].x],
                         [shapely.ops.nearest_points(Point(ego_zero), obst_car[0])[0].y, shapely.ops.nearest_points(Point(ego_zero), obst_car[0])[1].y]) for obst_car in car_potential.obst_car_list]

       
        # Plot potential field as a 3d surface plot
        # print("Shape of Meshgrid: {:s} - {:s} - {:s}".format(pos_meshgrid[0].shape,pos_meshgrid[1].shape,z.shape))
        surf = ax_3d.plot_surface(pos_meshgrid[0], pos_meshgrid[1], z, cmap=plt.cm.coolwarm, antialiased=True, linewidth=0, rstride=1, cstride=1)
        # Print min max of pot field
        # print("Details of Mesh grid values: Shape={:s}, Min z value={:.2f}, Max z value={:.2f}".format(z.shape, np.amin(z), np.amax(z)))

        # #Set axes limits of all plots
        # ax_2d.set_xlim(xlims[0],xlims[1])
        # ax_2d.set_ylim(ylims[0],ylims[1])
        # ax_3d.set_xlim(xlims[0],xlims[1])
        # ax_3d.set_ylim(ylims[0],ylims[1])
        # ax_3d.set_zlim(zlims[0],zlims[1])
            
        
        fig_3d.canvas.draw()
        fig_2d.canvas.draw()
        raw_input("Press Enter to continue...")


    plot_all(z)
    # raw_input("Press Enter to continue...")



if __name__ == '__main__':
    main()


