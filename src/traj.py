import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import time
import pandas as pd
from matplotlib.animation import FuncAnimation




# load points in the world coords
points = np.loadtxt('/home/aun/Documents/visionalgorithms/exercise2/pnp/data/p_W_corners.txt',delimiter = ',')

# load camera extrinsic
data = np.loadtxt('test.txt')

# get number of batches/ number of frames
batches= data.shape[0]/3

# converts into a list of numpy arrays 
data = np.split(data, int(batches))


# set fig
fig = plt.figure(figsize = (50,50))
ax  = fig.gca(projection = '3d')

# angles = [a for a in range(0,360,int(360/batches))]

# counter = 0
for d in data:


	# convert from world to camera -> camera to world 
	r_c_w = d[0:3,0:3]
	t_c_w = d[0:3,3]

	rot_mat = r_c_w.T

	temp = -1 *rot_mat
	pos  = temp.dot(t_c_w)

	


	# 3d coords of the points in the WCS
	xs = points[:,0]
	ys = points[:,1]
	zs = points[:,2]

	# plot points
	ax.scatter(xs,ys,zs, marker= 'o')
	plt.axis("equal")


	#plot axes
	ax.quiver(pos[0],pos[1],pos[2],
		      rot_mat[0,0],
		      rot_mat[1,0],
		      rot_mat[2,0],length = 5,color = 'red')

	ax.quiver(pos[0],pos[1],pos[2],
		      rot_mat[0,1],
		      rot_mat[1,1],
		      rot_mat[2,1],length = 5,color = 'green')


	ax.quiver(pos[0],pos[1],pos[2],
		      rot_mat[0,2],
		      rot_mat[1,2],
		      rot_mat[2,2],length = 5 ,color = 'blue')



	# # for rotations and saving
	# ax.view_init(elev=-40., azim=angles[counter])
	# plt.savefig("movie%d.png" % counter)

	# plt.pause(0.05)
	# counter+=1


plt.show()




