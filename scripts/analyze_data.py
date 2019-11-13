#!/usr/bin/env python
import rosbag
import rospy
import numpy as np
import math
import matplotlib.pyplot as plt
from sphero_formation.msg import collision_detection

# average the positions of all robots at every sample
def average_positions(arr,total_num_of_robots):
    # find shortest array because some messages are lost in rosbag
    min_len = len(arr[0])
    for robot in range(1,total_num_of_robots):
        if len(arr[robot])<min_len:
            min_len=len(arr[robot])
    # find average at each time point and store it in an array
    avg = []
    for i in range(0,min_len):
        temp_list = [arr[robot][i] for robot in range(0,total_num_of_robots)]
        avg.append(np.mean(np.array(temp_list)))
    return avg

# match the size of array to the smallest one
def trim(arr,n):
    if(len(arr)!=n):
        return arr[:n]
    else:
        return arr

# calculate RMSE
def RMSE(arr,avg):
    rmse = math.sqrt((np.square(np.array(trim(arr,len(avg))) - np.array(avg))).mean())
    return rmse

# average RMSE values
def average_RMSE(arr, avg, total_num_of_robots):
    rmse = []
    for robot in range(0,total_num_of_robots):
        rmse.append(RMSE(arr[robot],avg))
    return np.mean(rmse)

def norm(x1,x2,y1,y2):
    dist = []
    for i in range(0,min(len(x1),len(x2))):
        dist.append(math.sqrt(pow(x1[i]-x2[i], 2) + pow(y1[i]-y2[i], 2)))
    return dist 

def calculate_distance(xpos,avg_xpos,ypos,avg_ypos,total_num_of_robots):
    distance = []
    for robot in range(0,total_num_of_robots):
        distance.append(norm(xpos[robot],avg_xpos,ypos[robot],avg_ypos))
    return distance


# read bag file
# TODO: make the name of the file a parameter
bag = rosbag.Bag('2019-11-12-17-03-09.bag') 
start_time = bag.get_start_time()
end_time = bag.get_end_time()
total_time = end_time - start_time
print(total_time)
total_num_of_robots = 7 #rospy.get_param("/num_of_robots", 20)
boids = [0 for i in range(total_num_of_robots)]
ypos = [0 for i in range(total_num_of_robots)]
xpos = [0 for i in range(total_num_of_robots)]
collision_mask = [[] for i in range(total_num_of_robots)]
total_agent_collisions = 0
total_wall_collisions = 0

# extract data from bag
for robot in range(0, total_num_of_robots): 
    boids[robot] = []
    xpos[robot] = []
    ypos[robot] = []
    for topic, msg, t in bag.read_messages("/robot_" + str(robot) + "/collisions"):
        collision_mask[robot].append(msg.agent_collision.data or msg.wall_collision.data)
        if(msg.agent_collision.data):
            total_agent_collisions+=1
        if(msg.wall_collision.data):
            total_wall_collisions+=1

    for topic, msg, t in bag.read_messages("/robot_" + str(robot) + "/odom"):
        boid_pose = np.zeros(2)  
        boid_pose[0] = msg.pose.pose.position.x
        boid_pose[1] = msg.pose.pose.position.y
        boids[robot].append(boid_pose)
        xpos[robot].append(boid_pose[0])
        ypos[robot].append(boid_pose[1])
    #print(collision_mask[robot])
avg_xpos = average_positions(xpos,total_num_of_robots) 
avg_ypos = average_positions(ypos,total_num_of_robots)
pos = calculate_distance(xpos,avg_xpos,ypos,avg_ypos,total_num_of_robots)
avg_pos = average_positions(pos,total_num_of_robots)
RMSE = average_RMSE(pos,avg_pos,total_num_of_robots)
print("RMSE of flock distances %f"%RMSE)
print("agent collisions %d" %total_agent_collisions)
print("wall collisions %d" %total_wall_collisions)
# for robot_idx in range(0, total_num_of_robots): 
#     for robot2_idx in range(0, total_num_of_robots):
#         if(robot_idx!=robot2_idx):
#             A = np.array(x_axis[robot_idx])
#             B = np.array(x_axis[robot_idx])
#             print((A==B).all())
#             #print(set(x_axis[robot_idx]).intersection(x_axis[robot2_idx])) # does not work 

#generate graphs
plt.figure()
labels = []

for robot in range(0, total_num_of_robots): 
    fs = len(ypos[robot])/total_time # get sampling frequency
    t = np.arange(0,total_time,1/fs) # create time step array
    print(len(t))
    print(len(ypos[robot]))
    if(len(t)==len(ypos[robot])):# temporary fix, remove this in the future
        plt.plot( t,ypos[robot])
        labels.append(r'robot %s' % (robot))
# plt.plot(t,avg_ypos)
# labels.append("average")
plt.legend(labels, ncol=4, loc='upper center', 
           bbox_to_anchor=[0.5, 1.1], 
           columnspacing=1.0, labelspacing=0.0,
           handletextpad=0.0, handlelength=1.5,
           fancybox=True, shadow=True)
plt.show()
bag.close()