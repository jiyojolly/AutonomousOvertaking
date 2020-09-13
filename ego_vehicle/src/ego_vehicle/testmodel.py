import artf_pot_funcs
import os
import json
import numpy as np
from scipy import integrate
from shapely.geometry import Polygon, LinearRing, Point, box, asPoint
import shapely.ops
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import carla
import Utils
from kinematic_bicycle_model import bicycle_model


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
    ax_3d.set(xlabel="x", ylabel="y", zlabel="f(x, y)", title="3D Surface plot of Risk Map")

    fig_2d = plt.figure()
    ax_2d = fig_2d.add_subplot(111)

    # Define X, Y, Z limits for plots
    xlims = np.array(np.array([-50,50]))
    ylims = np.array([-50,50])
    zlims = np.array([0,30])

    # Define X, Y limits for area under consideration
    x_vision_limit = np.array(json_params["X_limit"])
    y_vision_limit = np.array(json_params["Y_limit"])

    # Create mesh
    x = np.arange(x_vision_limit[0],x_vision_limit[1], 0.5)
    y = np.arange(y_vision_limit[0],y_vision_limit[1], 0.5)
    pos_meshgrid = np.meshgrid(x, y, sparse=False, copy=True)
    
    
    lanes = [(1,-3.5),(1,3.5),(1,10.5),(1,17.5)]


    theta = 45*np.pi/180
    z = np.multiply(0.5*json_params["Road_scale_factor"], 
                                np.divide(1.0,np.square((np.sin(theta)*pos_meshgrid[1])+(np.cos(theta)*pos_meshgrid[0])-lanes[0][1])))
    z = np.where((np.sin(theta)*pos_meshgrid[1])+(np.cos(theta)*pos_meshgrid[0])>lanes[0][1], z, 20)
    # z = np.square(pos_meshgrid[0])
    # z = 1/np.square((np.sin(theta)*pos_meshgrid[0])+(np.cos(theta)*pos_meshgrid[1]))

    z = np.clip(z,-20,20)
    # Plot potential field as a 3d surface plot
    surf = ax_3d.plot_surface(pos_meshgrid[0],
                                pos_meshgrid[1], z, 
                                cmap=plt.cm.coolwarm, 
                                antialiased=True, 
                                linewidth=0, 
                                rstride=1, cstride=1)
    print(z)
    print(z.shape)

    l = np.where(z>4)

    print(l)
    # print(z[l])

    # x_0 = np.array([0.0,0.0,0.0,5.0])
    # u_r1 = np.array([1.0, 0.1])
    # u_r2 = np.array([1.0, -0.1])
    
    # # Get timesteps
    # delta_t = 0.1 # time step size (seconds)
    # t_max = 5 # max sim time (seconds)
    # t = np.linspace(0, t_max, t_max/delta_t)

    # dx_vals_int_1 = integrate.odeint(bicycle_model, x_0, t, args=(u_r1,))

    # dx_vals_int_2 = integrate.odeint(bicycle_model, x_0, t, args=(u_r2,))     

    # reach_set_tup1 = zip(dx_vals_int_1[:, 0],dx_vals_int_1[:, 1])
    # reach_set_tup2 = zip(dx_vals_int_2[:, 0],dx_vals_int_2[:, 1])
    # reach_set_tup1.reverse()
    # reach_set_tup1.extend(reach_set_tup2)
    # reach_set = LinearRing(reach_set_tup1)

    # reach_safe_set = 

    # plt.plot(np.asarray(reach_set)[:,0],np.asarray(reach_set)[:,1])


    # plt.legend(loc='best')
    # plt.xlabel('t')
    # plt.grid()
    # plt.show()
    raw_input("Press Enter to continue...")






if __name__ == '__main__':
    main()


