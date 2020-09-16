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
    # fig_3d = plt.figure()
    # ax_3d = fig_3d.add_subplot(111, projection='3d')
    # ax_3d.set(xlabel="x", ylabel="y", zlabel="f(x, y)", title="3D Surface plot of Risk Map")

    fig_2d = plt.figure()
    ax_2d = fig_2d.add_subplot(111)

    # # Define X, Y, Z limits for plots
    # xlims = np.array(np.array([-50,50]))
    # ylims = np.array([-50,50])
    # zlims = np.array([0,30])

    # # Define X, Y limits for area under consideration
    # x_vision_limit = np.array(json_params["X_limit"])
    # y_vision_limit = np.array(json_params["Y_limit"])

    # # Create mesh
    # x = np.arange(x_vision_limit[0],x_vision_limit[1], 0.5)
    # y = np.arange(y_vision_limit[0],y_vision_limit[1], 0.5)
    # pos_meshgrid = np.meshgrid(x, y, sparse=False, copy=True)
    
    
    # lanes = [(1,-3.5),(1,3.5),(1,10.5),(1,17.5)]


    # theta = 45*np.pi/180
    # z = np.multiply(0.5*json_params["Road_scale_factor"], 
    #                             np.divide(1.0,np.square((np.sin(theta)*pos_meshgrid[1])+(np.cos(theta)*pos_meshgrid[0])-lanes[0][1])))
    # z = np.where((np.sin(theta)*pos_meshgrid[1])+(np.cos(theta)*pos_meshgrid[0])>lanes[0][1], z, 20)
    # # z = np.square(pos_meshgrid[0])
    # # z = 1/np.square((np.sin(theta)*pos_meshgrid[0])+(np.cos(theta)*pos_meshgrid[1]))

    # z = np.clip(z,-20,20)
    # # Plot potential field as a 3d surface plot
    # surf = ax_3d.plot_surface(pos_meshgrid[0],
    #                             pos_meshgrid[1], z, 
    #                             cmap=plt.cm.coolwarm, 
    #                             antialiased=True, 
    #                             linewidth=0, 
    #                             rstride=1, cstride=1)
    # print(z)
    # print(z.shape)

    # l = np.where(z>4)

    # print(l)
    # print(z[l])

    # Get timesteps
    delta_t = 0.1 # time step size (seconds)
    t_max = 3 # max sim time (seconds)
    t = np.linspace(0, t_max, t_max/delta_t)

    def sim_full(a_max, delta_max, x_init, v_des_enable = True, color='c'):

        dx_vals_int_1 = integrate.odeint(bicycle_model, x_init, t, args=(np.array([a_max, delta_max]),))
        dx_vals_int_2 = integrate.odeint(bicycle_model, x_init, t, args=(np.array([a_max, -delta_max]),))
        
        # Find Points where velocity exceeds
        idx = np.where(dx_vals_int_2[:,3]>v_des)
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

        # print(reach_set_tup1)
        reach_set = LinearRing(reach_set_tup1)

        plt.plot(np.asarray(reach_set)[:,0], np.asarray(reach_set)[:,1], color=color)
        return reach_set
    

    def sim(x_init, u,c):
        dx_vals_int = integrate.odeint(bicycle_model, x_init, t, args=(u,))
        plt.plot(dx_vals_int[:,0], dx_vals_int[:,1],color=c)
    

    v_des = 10.0
    a_max = 2.0
    delta_max = 0.3
    x_0 = np.array([0.0,0.0,0.0, 8.0])

    reach1 = Polygon(sim_full(a_max, delta_max, x_0))
    reach1 = Polygon(sim_full(a_max, delta_max, x_0, False, 'r'))
    # reach2 = Polygon(sim_full(-a_max, delta_max, x_0))

    # sim(x_0, np.array([a_max, delta_max/4]), 'g' )
    # sim(x_0, np.array([a_max, delta_max/2]), 'g' )
    # sim(x_0, np.array([a_max, 3*delta_max/4]), 'g' )

    # v_des = 10.0
    # a_max = 1.0
    # delta_max = 0.5
    # x_0 = np.array([0.0,0.0,0.0, 0.0])

    

    # sim(np.array([0.0,0.0,0.0, 5.0]), np.array([0.0, delta_max]), 'g' )
    # sim(np.array([0.0,0.0,0.0, 5.0]), np.array([-a_max, delta_max]), 'r' )
    # sim(np.array([0.0,0.0,0.0, 5.0]), np.array([-a_max, -delta_max]), 'r' )
    # sim(np.array([0.0,0.0,0.0, 5.0]), np.array([-a_max, 0.0]), 'r' )
    
    # y_p = np.linspace(0, 50, 1/0.01)
    # x_p = np.sqrt(np.square(25)-np.square(y_p-5)) + 12  
    # print(y_p)
    # print(x_p)
    # plt.plot(x_p, y_p)
    # plt.plot(-x_p, y_p)


    # plt.plot(dx_vals_int_4[:,0], dx_vals_int_4[:,1 ])
    # plt.plot(t[:v_max_idx], dx_vals_int_3[:v_max_idx,3])



    plt.legend(loc='best')
    plt.xlabel('t')
    plt.grid()
    plt.show()
    raw_input("Press Enter to continue...")






if __name__ == '__main__':
    main()


