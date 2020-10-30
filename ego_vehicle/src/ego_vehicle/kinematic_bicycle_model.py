import numpy as np


def bicycle_model(x, t, u):
	L = 3.705
	L_F = 1.2525
	L_R = L - L_F

	#Initialize dx (derivatives) with zeroes
	dx = np.zeros((4))
	
	#Calculate slip angle
	beta = np.arctan2(L_R*np.tan(u[1]), L_R+L_F)

	#Calculate all angles
	cpsibeta = np.cos(x[2]+beta)
	spsibeta = np.sin(x[2]+beta)
	cbeta = np.cos(beta)
	tdelta = np.tan(u[1])

	#Calculate all derivatives
	dx[0] = x[3]*cpsibeta
	dx[1] = x[3]*spsibeta
	dx[2] = (x[3]*cbeta*tdelta) / (L_R+L_F)
	dx[3] = u[0]

	return dx



