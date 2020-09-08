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
	x_0 = np.array([0.0,0.0,0.0,5.0])
	u_r1 = np.array([1.0, 0.1])
	u_r2 = np.array([1.0, -0.1])
	
	# Get timesteps
	delta_t = 0.1 # time step size (seconds)
	t_max = 5 # max sim time (seconds)
	t = np.linspace(0, t_max, t_max/delta_t)

	dx_vals_int_1 = integrate.odeint(bicycle_model, x_0, t, args=(u_r1,))

	dx_vals_int_2 = integrate.odeint(bicycle_model, x_0, t, args=(u_r2,))		

	plt.plot(dx_vals_int_1[:, 0], dx_vals_int_1[:, 1], 'r', label='xy plot(t)')
	plt.plot(dx_vals_int_2[:, 0], dx_vals_int_2[:, 1], 'r', label='xy plot(t)')

	reach_set_tup1 = zip(dx_vals_int_1[:, 0],dx_vals_int_1[:, 1])
	reach_set_tup2 = zip(dx_vals_int_2[:, 0],dx_vals_int_2[:, 1])
	reach_set_tup1.reverse()
	reach_set_tup1.extend(reach_set_tup2)
	reach_set = LinearRing(reach_set_tup1)


	plt.plot(np.asarray(reach_set)[:,0],np.asarray(reach_set)[:,1])


	plt.legend(loc='best')
	plt.xlabel('t')
	plt.grid()
	plt.show()
	raw_input("Press Enter to continue...")






if __name__ == '__main__':
    main()


