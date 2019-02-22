#!/usr/bin/env python
import rospy
import numpy as np
import math
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

rospy.init_node('AStar', anonymous=True)
importedMap =[0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
              0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
              0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
              1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
              0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
              0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
              0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
              0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,
              0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,
              0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,
              0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,
              0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,
              0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,
              0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,
              0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
              0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,
              0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0,
              0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0,
              0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,
              0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]

def discretize(xCor,yCor):
       return [20-(int(yCor)+10)-1, int(xCor)+9]

def heuristics(cell,goalCell):
       euclidian = math.sqrt((goalCell[1] - cell[1])**2 + (goalCell[0] - cell[0])**2)
       return euclidian

map = np.reshape(importedMap,(20,18))
startPoint = [-8.0,-2.0]
startPoint = discretize(startPoint[0],startPoint[1])
goal = [rospy.get_param('goalx'),rospy.get_param('goaly')]
goal = discretize(goal[0],goal[1])

parent = np.zeros(np.shape(map),dtype=object)
infinity = 5000
spaceX,spaceY = np.where(map == 0)
fMatrix = np.full(np.shape(map),infinity,dtype=np.float64)
fMatrix[startPoint[0],startPoint[1]] = 0
gMatrix = np.copy(fMatrix)
openList = []
closeList = []
openList.append(startPoint)
while(len(openList) != 0):
       Fs = []
       for item in openList:
              Fs.append(fMatrix[item[0],item[1]])
       node = openList.pop(Fs.index(min(Fs)))
       if(node == goal):
              print("Goal reached.")
              break
       closeList.append(node)
       for i in range(-1,2):
              for j in range(-1,2):
                     neighbor = [node[0]+i, node[1]+j]
                     if(neighbor == node):
                            continue
                     if(map[neighbor[0],neighbor[1]] == 1):
                            continue
                     if(map[neighbor[0]+1,neighbor[1]]==1 and map[neighbor[0],neighbor[1]-1]==1):
                            continue
                     if(neighbor in closeList):
                            continue

                     if((node[0] == neighbor[0]) or (node[1] == neighbor[1])):
                            gScore = gMatrix[node[0],node[1]] + 1.0
                     else:
                            gScore = gMatrix[node[0], node[1]] + 1.4

                     if (neighbor not in openList):
                            openList.append(neighbor)
                     if(gScore >= gMatrix[neighbor[0], neighbor[1]]):
                            continue
                     parent[neighbor[0],neighbor[1]] = node
                     gMatrix[neighbor[0], neighbor[1]] = gScore
                     hScore = heuristics(neighbor,goal)
                     fScore = gScore + hScore
                     fMatrix[neighbor[0], neighbor[1]] = fScore
current = goal
path = []
while(current != startPoint):
       path.append(parent[current[0],current[1]])
       current = parent[current[0],current[1]]
path = path[::-1]
print("Calculated path is: "+str(path))

p1 = path[0]
p2 = path[1]

vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
current_angle = (1,1,1)
pos = (0,0)
def perform_movement(Odometry):
    global current_angle
    global pos
    pos = (Odometry.pose.pose.position.x,Odometry.pose.pose.position.y)
    current_angle = tf.transformations.euler_from_quaternion([Odometry.pose.pose.orientation.x,
                                                       Odometry.pose.pose.orientation.y,
                                                       Odometry.pose.pose.orientation.z,Odometry.pose.pose.orientation.w])


rospy.Subscriber('/base_pose_ground_truth',
                     Odometry, perform_movement)
idx = 0

rate = rospy.Rate(20)
velocity = Twist()

while(not rospy.is_shutdown()):
    p1 = path[idx]
    if (idx + 1 == len(path)):
        print("Destination arrived.")
        break
    p2 = path[idx + 1]
    theta = math.atan2(p1[0] - p2[0], p2[1] - p1[1])
    deltaTheta = theta - current_angle[2]
    velocity.linear.x = 0.2
    velocity.angular.z = deltaTheta

    vel_pub.publish(velocity)
    state = discretize(pos[0],pos[1])
    print(state)
    print(p2)
    pythagoras = math.sqrt(((state[0]+0.5)-(p2[0]+0.5))**2 + ((state[1]+0.5) - (p2[1]+0.5))**2)
    print(pythagoras)
    if(pythagoras <= 1.0):
      print("taking next point")
      idx = idx + 1
    rate.sleep()


