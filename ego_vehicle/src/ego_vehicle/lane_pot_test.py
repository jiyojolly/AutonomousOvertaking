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

    # fig_2d = plt.figure()
    # ax_2d = fig_2d.add_subplot(111)

    # Add fig and axes for contour and 3d surface plots
    fig_3d = plt.figure()
    ax_3d = fig_3d.add_subplot(111, projection='3d')


    # Define X, Y limits for area under consideration
    x_vision_limit = np.array(json_params["X_limit"])
    y_vision_limit = np.array(json_params["Y_limit"])

    # Create mesh for potential calculation
    x = np.arange(x_vision_limit[0],x_vision_limit[1], 1)
    y = np.arange(y_vision_limit[0],y_vision_limit[1], 1)
    xx, yy = np.meshgrid(x, y, sparse=False, copy=True)
    # z = np.sin(xx**2 + yy**2) / (xx**2 + yy**2)
    # print(yy.shape)
    # print(z.shape)


    lanes = [(1,-3.5),(1,3.5),(1,10.5),(1,17.5)]
    lane_width = 7.0
    # for i,lane in enumerate(lanes[1:-1]):
    #     print(lane,i+1)
    #     print(lanes[i+1][1]-lanes[i][1])
    # lanes = [-2.0,2.0,6.0]
    # lane_width = 4.0

    # result = [artf_pot_funcs.lane_potential(json_params, lanes, y_coor) for y_coor in y]
    U = np.zeros(yy.shape)

    #Calculate Road potential 
    U = U + np.multiply(0.5*json_params["Road_scale_factor"], 
                            np.divide(1.0,(yy-lanes[0][1])**2))
    U = U + np.multiply(0.5*json_params["Road_scale_factor"], 
                            np.divide(1.0,(yy-lanes[-1][1])**2))

    #Calculate lane potential
    for i,lane in enumerate(lanes[1:-1]):
        U = U + np.multiply(json_params["Lane_Alane"], 
                            np.exp(-np.divide((yy-lane[1])**2, 
                            (2 * (json_params["Lane_widthfactor"]*abs(lanes[i+1][1]-lanes[i][1]))**2))))

     
    surf = ax_3d.plot_surface(xx,yy, U,
                                cmap=plt.cm.coolwarm, 
                                antialiased=True, 
                                linewidth=0, 
                                rstride=1, cstride=1)

    # ax_2d.set_xlim(-6,12)
    # ax_2d.plot(y, U)
    raw_input("Press Enter to continue...")




if __name__ == '__main__':
    main()

