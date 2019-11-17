#!/usr/bin/env python
import rosbag
import rospy
import sys
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import get_test_data
from mpl_toolkits.mplot3d import Axes3D  
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

def get_heading(xheading,yheading,total_num_of_robots):
    heading = [[] for i in range(total_num_of_robots)]
    for robot in range(0,total_num_of_robots):
        for i in range(len(yheading[robot])):
            heading[robot].append(math.degrees(math.atan2(yheading[robot][i], xheading[robot][i]))  )
    return heading

# data extractor
def extract_data(bag,total_num_of_robots):
    boids = [0 for i in range(total_num_of_robots)]
    ypos = [0 for i in range(total_num_of_robots)]
    xpos = [0 for i in range(total_num_of_robots)]
    yheading = [[] for i in range(total_num_of_robots)]
    xheading = [[] for i in range(total_num_of_robots)]
    collision_mask = [[] for i in range(total_num_of_robots)]
    total_agent_collisions = 0
    total_wall_collisions = 0
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
        for topic, msg, t in bag.read_messages("/robot_" + str(robot) + "/cmd_vel"):
            xheading[robot].append(msg.linear.x)
            yheading[robot].append(msg.linear.y)
        for topic, msg, t in bag.read_messages("/robot_" + str(robot) + "/odom"):
            boid_pose = np.zeros(2)  
            boid_pose[0] = msg.pose.pose.position.x
            boid_pose[1] = msg.pose.pose.position.y
            boids[robot].append(boid_pose)
            xpos[robot].append(boid_pose[0])
            ypos[robot].append(boid_pose[1])
    return xpos,ypos,total_agent_collisions,total_wall_collisions,collision_mask,xheading,yheading

def get_output_name (file):
    return (file.split('/')[-1]).split('.')[0] 

# Read bag file
# The dictionary refers to how many robots are in each test case.
# The keys are the test cases and the values are the robot numbers
num_robot_dict = {1:6, 2:6, 3:6, 4:6, 5:6, 6:12, 7:12, 8:12, 9:12, 10:12, 11:20,
 12:20, 13:20, 14:20, 15:20, 16:6, 17:6, 18:6, 19:12, 20:12, 21:12, 
 22:20, 23:20, 24:20, 25:6, 26:6, 27:6, 28:12, 29:12, 30:12, 31:20, 32:20, 33:20} 
for i in range(16,34):
    #file = str(sys.argv[1])
    file = '../data/test_'+str(i)+'.bag'
    bag = rosbag.Bag(file)
    f = open("../results/"+get_output_name(file)+'.txt', "w")
    start_time = bag.get_start_time()
    end_time = bag.get_end_time()
    total_time = end_time - start_time
    #total_num_of_robots = int(input("please enter the number of robots for test %d\n"%i))#7 #rospy.get_param("/num_of_robots", 20)
    total_num_of_robots = num_robot_dict[i]
    print(total_num_of_robots)
    try:
        # extract data from bag file
        xpos,ypos,total_agent_collisions,total_wall_collisions,collision_mask,xheading,yheading = extract_data(bag,total_num_of_robots)

        avg_xpos = average_positions(xpos,total_num_of_robots) 
        avg_ypos = average_positions(ypos,total_num_of_robots)
        pos = calculate_distance(xpos,avg_xpos,ypos,avg_ypos,total_num_of_robots)
        avg_pos = average_positions(pos,total_num_of_robots)
        avg_RMSE = average_RMSE(pos,avg_pos,total_num_of_robots)
        heading = get_heading(xheading,yheading,total_num_of_robots)
        print("Total run time (s): %f"%total_time)
        print("RMSE of flock distances %f"%avg_RMSE)
        print("agent collisions %d" %total_agent_collisions)
        print("wall collisions %d" %total_wall_collisions)
        f.write("Total run time (s): %f\n"%total_time)
        f.write("Number of robots: %d\n"%total_num_of_robots)
        f.write("RMSE of flock distances %f\n"%avg_RMSE)
        f.write("agent collisions %d\n" %total_agent_collisions)
        f.write("wall collisions %d\n" %total_wall_collisions)
        f.close()
        # for robot_idx in range(0, total_num_of_robots): 
        #     for robot2_idx in range(0, total_num_of_robots):
        #         if(robot_idx!=robot2_idx):
        #             A = np.array(x_axis[robot_idx])
        #             B = np.array(x_axis[robot_idx])
        #             print((A==B).all())
        #             #print(set(x_axis[robot_idx]).intersection(x_axis[robot2_idx])) # does not work 

        #generate graphs
        fig = plt.figure(figsize=plt.figaspect(0.5))
        ax = fig.add_subplot(2, 1, 1, projection='3d')
        labels = []

        for robot in range(0, total_num_of_robots): 
            fs = len(ypos[robot])/total_time # get sampling frequency
            t = np.arange(0,total_time,1/fs) # create time step array
            check = len(ypos[robot])<len(t)
            if(check):
                t = trim(t,len(ypos[robot]))
            else:
                ypos[robot] = trim(ypos[robot],len(t))
                xpos[robot] = trim(xpos[robot],len(t))

            #if(len(t)==len(ypos[robot])):# temporary fix, remove this in the future
                #plt.plot( xpos[robot],ypos[robot])
            ax.plot3D(t,xpos[robot], ypos[robot])
            ax.view_init(30, -50)
            labels.append(r'robot %s' % (robot))
        ax.set_xlabel('time (s)')
        ax.set_ylabel('x position')
        ax.set_zlabel('y position')
        plt.legend(labels, ncol=10   , loc='lower center',
                bbox_to_anchor=[0.5, 1.1], 
                columnspacing=1.0, labelspacing=0.0,
                handletextpad=0.0, handlelength=1.5,
                fancybox=True, shadow=True)
        plt.title("Robot Position Over Time for "+get_output_name(file))

        fig.add_subplot(2, 1, 2)
        labels = []

        for robot in range(0, total_num_of_robots): 
            fs = len(heading[robot])/total_time # get sampling frequency
            t = np.arange(0,total_time,1/fs) # create time step array
            check = len(heading[robot])<len(t)
            if(check):
                t = trim(t,len(heading[robot]))
            else:
                heading[robot] = trim(heading[robot],len(t))

            if(len(t)==len(heading[robot])):# temporary fix, remove this in the future
                plt.plot( t,heading[robot])
                #labels.append(r'robot %s' % (robot))
        plt.xlabel('time (s)')
        plt.ylabel('heading (degrees)')
        # plt.legend(labels, ncol=4   , loc='center right',
        #            bbox_to_anchor=[0.5, 1.1], 
        #            columnspacing=1.0, labelspacing=0.0,
        #            handletextpad=0.0, handlelength=1.5,
        #            fancybox=True, shadow=True)
        plt.title("Robot Heading Over Time for "+get_output_name(file))
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        #plt.show()
        plt.savefig("../results/"+get_output_name(file)+'.png')
    except Exception as e: print(e)

bag.close()