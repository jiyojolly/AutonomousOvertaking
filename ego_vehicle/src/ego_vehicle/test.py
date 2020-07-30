import artf_pot_funcs
from artf_pot_funcs import CarPotential
import os
import json
import numpy as np
from shapely.geometry import Polygon, LineString, Point, box, asPoint
import shapely.ops
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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
    xlims = np.array([-50,50])
    ylims = np.array([-50,50])
    zlims = np.array([0,30])

    # Define X, Y limits for area under consideration
    x_vision_limit = np.array(json_params["X_limit"])
    y_vision_limit = np.array(json_params["Y_limit"])


    
    #TO DO Convert position (x,y) to Point objects

    # Dummy ego car postion Point(x,y)
    ego_car_loc = Point(-5,-10)
    print(ego_car_loc.x)
    ax_2d.plot(np.asarray(ego_car_loc)[0], np.asarray(ego_car_loc)[1], marker='x')

    # Dummy subject car #1 postion  
    sub_car_locs = [Point(-10,-1), Point(0,3), Point(10,8)]
    # sub_car_locs = [Point(5,1)]
    [ax_2d.plot(np.asarray(loc)[0], np.asarray(loc)[1], marker='o') for loc in sub_car_locs]


    # Create car potential class
    car_pot = CarPotential()
    
    car_pot.update_obst_plgn(sub_car_locs, json_params)    
    

    # Plot obstacle polygons
    [ax_2d.plot(np.asarray(plgn.boundary)[:,0],np.asarray(plgn.boundary)[:,1]) for plgn in car_pot.plgn_list]
    
    # Plot Nearest point to obstable
    [ax_2d.plot([shapely.ops.nearest_points(ego_car_loc,plgn)[0].x,shapely.ops.nearest_points(ego_car_loc,poly)[1].x],
            [shapely.ops.nearest_points(ego_car_loc,plgn)[0].y,shapely.ops.nearest_points(ego_car_loc,poly)[1].y]) for poly in car_pot.plgn_list]

    # Create mesh for potential calculation
    x = np.arange(x_vision_limit[0],x_vision_limit[1], 0.5)
    y = np.arange(y_vision_limit[0],y_vision_limit[1], 0.5)
    pos_meshgrid = np.meshgrid(x, y, sparse=False)

    z = car_pot.update_car_pot(pos_meshgrid,json_params) 
    # + road_pot.update_road_pot()
    
    # Print min max of pot field
    print("Details of Mesh grid values: Shape={:s}, Min z value={:.2f}, Max z value={:.2f}".format(z.shape, np.amin(z), np.amax(z)))

    # Plot potential field as a 3d surface plot
    print("Shape of Meshgrid: {:s} - {:s} - {:s}".format(pos_meshgrid[0].shape,pos_meshgrid[1].shape,z.shape))
    surf = ax_3d.plot_surface(pos_meshgrid[0], pos_meshgrid[1], z, cmap=plt.cm.coolwarm, antialiased=True, linewidth=0, rstride=1, cstride=1)
    fig_3d.colorbar(surf, shrink=0.5, aspect=5)

    ax_3d.set(xlabel="x", ylabel="y", zlabel="f(x, y)", title=None)

    #Set axes limits of all plots
    ax_2d.set_xlim(xlims[0],xlims[1])
    ax_2d.set_ylim(ylims[0],ylims[1])
    ax_3d.set_xlim(xlims[0],xlims[1])
    ax_3d.set_ylim(ylims[0],ylims[1])
    ax_3d.set_zlim(zlims[0],zlims[1])

        
    # print("Update plot with latest values")
    # for delta in np.linspace(0, 10, 50):
    #     surf.remove()
    #     ax_2d.clear()
    #     print("Point : {:s}".format(sub_car_locs[0]))
    #     sub_car_locs[0] = Point(sub_car_locs[0].x + delta, sub_car_locs[0].y)
    #     car_pot.update_obst_plgn(sub_car_locs, json_params)
    #     z = car_pot.update_car_pot(pos_meshgrid,json_params)
    #     surf = ax_3d.plot_surface(pos_meshgrid[0], pos_meshgrid[1], z, cmap=plt.cm.coolwarm, antialiased=True, linewidth=0)
    #     # Plot obstacle polygons
    #     [ax_2d.plot(np.asarray(loc)[1],np.asarray(loc)[0],marker='o') for loc in sub_car_locs]
    #     [ax_2d.plot(np.asarray(plgn.boundary)[:,0],np.asarray(plgn.boundary)[:,1]) for plgn in car_pot.plgn_list]

    #     fig_3d.canvas.draw()
    #     fig_2d.canvas.draw()
    #     ax_2d.set_xlim(xlims[0],xlims[1])
    #     ax_2d.set_ylim(ylims[0],ylims[1])
        
    
    fig_3d.canvas.draw()
    fig_2d.canvas.draw()
    raw_input("Press Enter to continue...")


if __name__ == '__main__':
    main()


