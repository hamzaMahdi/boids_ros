#!/usr/bin/env python
import rosbag
import rospy
import numpy as np
import matplotlib.pyplot as plt
bag = rosbag.Bag('example.bag')  # Read bag# The data between start_time and end_time will be analyzed
start_time = bag.get_start_time()
end_time = bag.get_end_time()
total_time = end_time - start_time
print(total_time)
total_num_of_robots = 7 #rospy.get_param("/num_of_robots", 20)topics = bag.get_type_and_topic_info()[1].keys()  # All topics in rosbagboids = {}  # Dictionary for all boids poses through time
boids = [0 for i in range(total_num_of_robots)]
y_axis = [0 for i in range(total_num_of_robots)]
x_axis = [0 for i in range(total_num_of_robots)]
for robot_idx in range(0, total_num_of_robots):  # start from robot_1
   boids[robot_idx] = []
   x_axis[robot_idx] = []
   y_axis[robot_idx] = []
   for topic, msg, t in bag.read_messages("/robot_" + str(robot_idx) + "/odom"):
       boid_pose = np.zeros(2)  # Leader pose at specific time
       boid_pose[0] = msg.pose.pose.position.x
       boid_pose[1] = msg.pose.pose.position.y
       boids[robot_idx].append(boid_pose)
       x_axis[robot_idx].append(boid_pose[0])
       y_axis[robot_idx].append(boid_pose[1])
plt.figure()
labels = []

for robot_idx in range(0, total_num_of_robots): 
    fs = len(y_axis[robot_idx])/total_time
    t = np.arange(0,total_time,1/fs)
    print(len(t))
    print(len(y_axis[robot_idx]))
    if(len(t)==len(y_axis[robot_idx])):
        plt.plot( t,y_axis[robot_idx])
        labels.append(r'robot %s' % (robot_idx))
plt.legend(labels, ncol=4, loc='upper center', 
           bbox_to_anchor=[0.5, 1.1], 
           columnspacing=1.0, labelspacing=0.0,
           handletextpad=0.0, handlelength=1.5,
           fancybox=True, shadow=True)
plt.show()
bag.close()