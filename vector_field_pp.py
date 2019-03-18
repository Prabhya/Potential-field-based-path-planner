######################################
##### Vector field based planner #####
######################################

##################################################################
# This planner was written based on potential field based path planning (ppbfp)
# This program does not follow to theory faithfully and is a different outlook on the same path planning
# The ppbfp will be later implemented using this program style as the basis
# A similar program will be written in python version for learning and understanding
# -Prabhjeet Singh Arora
##################################################################

import matplotlib.pyplot as plt
import numpy as np
from numpy.random import seed
from numpy.random import rand

dx = 0.5 # step size

plt.figure(1)

############# Workspace ##############
plt.gca().add_patch(plt.Rectangle((-50,-50),100,100, fill=False, edgecolor='k', linewidth=3))
######################################

############# Start point ############
start = np.array([-40,-40])
plt.plot(start[0],start[1],'bo')
plt.text(start[0]+1,start[1]+1, r'START')
plt.grid(color='k', linestyle='-', linewidth=0.1)
plt.axis([-55,55,-55,55])
plt.xlabel('x-position')
plt.ylabel('y-position')
plt.title('Path Planning')
######################################

############# Goal point #############
final = np.array([40,40])
plt.plot(final[0],final[1],'ko')
plt.text(final[0]+1,final[1]+1, r'GOAL')
######################################

############# Point Obstacles #########################
thres = 10          #threshold radius
objr = 5
numobs = 10
obstacle = np.array(-40 + (30+40)*rand(2,numobs)) #random position of point obstacles
plt.plot(obstacle[0],obstacle[1],'ro')
for i in range(0,numobs):
    plt.gca().add_patch(plt.Circle((obstacle[0][i],obstacle[1][i]),thres, fill=False, edgecolor='r', linewidth=1))
    plt.text(obstacle[0][i]+1,obstacle[1][i]+1, r'obs')
  #  plt.gca().add_patch(plt.Circle((obstacle[0][i],obstacle[1][i]),objr+2*dx, fill=False, edgecolor='r', edgestyle ='--', linewidth=0.5))
  #  plt.gca().add_patch(plt.Circle((obstacle[0][i],obstacle[1][i]),objr, fill=True, color = 'r',alpha =0.3))
obstacle = np.transpose(obstacle)
#######################################################

############# Planner ################
pos = start # current position
rep = 0
while np.linalg.norm(final-pos) >dx:
    repnet = 0
    att = np.linalg.norm((final - pos))
    for i in range(0,numobs):
        rep = np.linalg.norm(obstacle[i]-pos)
        if rep >thres:
            rep = 0
        else:
            if rep > (objr+2*dx):
                repnet = repnet + obstacle[i]-pos   # net repulsive vector
    if np.linalg.norm(repnet) >0:
     #   repnet = repnet/np.linalg.norm(repnet)
        movector = -repnet*(1/np.linalg.norm(repnet)) + (final - pos)/att    #total movement vector
    else:
        movector = (final - pos)/att
    movector = movector/np.linalg.norm(movector)    # unit movement vector
    pos = pos + dx*movector                         # propagation along movement vector
    plt.plot(pos[0],pos[1],'bo')
    plt.pause(0.05)

plt.show()