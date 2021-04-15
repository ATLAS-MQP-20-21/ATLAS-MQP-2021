#!/usr/bin/env python

# Created by emily on 2/9/21.

import numpy as np
import rospy
import roslib
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import OccupancyGrid
#include "FrontierExplorerNode.h"

#include <iostream>
#include "ros/ros.h" 
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Transform.h"
#include "sensor_msgs/PointCloud2.h"    
#include "nav_msgs/OccupancyGrid.h"
#include <vector>
#include <deque>
#include <array>

#include "tough_common/robot_state.h"

#find all includes (sensor messages as well)


#initialize variables
# geometry_msgs::Pose robot_pose;
# RobotStateInformer* current_state;
# RobotDescription* rd_;

#set up Global Publishers?
# ros::Publisher pointcloud_publisher;
# ros::Subscriber occupancy_grid_listener;
occupancy_grid_listener = 0
robot_pose_listener = 0 
point_publisher = 0
robot_pose = 0

#done
def createOccupancyArray(data, rowLength):
    # make sure data can be divided by rosLength evenly
    if(len(data) % rowLength != 0):
        rospy.loginfo("WARNING: Data can't be evenly distributed into array wtih given row length")

    array = np.full((rowLength, len(data)/rowLength), -1)
    for i in range(len(data)):
        array[i%rowLength][i/rowLength] = data[i]
    return array

#done
def isFrontier(grid, x, y):
    #if square is unknown, it isn't a frontier
    if(grid[x][y] < 0):
        return False

    #check squares in 8-neighborhood to see if any are unknown
    for a in [-1,0,1]:
        for b in [-1,0,1]:
            if(not (a == 0 and b == 0)):
                if(grid[a][b] == -1):
                    return True
    return False

#done
def addNeighbors(toExplore, x, y, grid):
    for a in [-1,0,1]:
        for b in [-1,0,1]:
            if(not (a == 0 and b == 0) and grid[x+a][y+b] <= 100):
                toExplore.append([x+a, y+b])
                grid[x+a, y+b] += 100

#done
def findFrontiers(grid):
    #
    # std::vector<std::array<int, 2>> newFrontierCluster = {};
    # std::vector<std::array<int, 2>> toExplore = {};
    returnVector = []#np.array([[[0,0]]])
    for i in range(len(grid)):
        for j in range(len(grid[i])):
            newFrontierCluster = []#np.array([[0,0]])
            toExplore = []
            toExplore.append([i,j])
            #while there is something to explore, explore
            while len(toExplore) > 0:
                x = toExplore[0][0]
                y = toExplore[0][1]
                # rospy.loginfo("checking point (%d, %d) with value %d" % (x,y, grid[x][y]))
                #if it isn't explored and it is a frontier, check its neighbors
                if (not (grid[x][y] > 100) and isFrontier(grid, x, y)):
                    rospy.loginfo("checking point (%d, %d)" % (x,y))
                    #add as part of frontier
                    #np.vstack((newFrontierCluster, toExplore[0]))
                    newFrontierCluster.append(toExplore[0])
                    #add neighbors to list to be epxlored
                    addNeighbors(toExplore, x, y, grid)
                    #remember that square is checked
                    if grid[x][y] <= 100:
                        grid[x][y] += 100
                #erase the first element
                toExplore.pop(0)

            if(len(newFrontierCluster) > 0):
                rospy.loginfo("Found new frontier cluster")
                #remove initial [0,0] from vector before putting it in return vector
                returnVector.append(newFrontierCluster)
                #np.vstack((returnVector, newFrontierCluster [1:]))

    return returnVector

#done
def getCentroids(frontiers):
    #std::vector<std::array<int,2>> centroids;

    centroids = []#np.array([]) #how do this?

    #numFronts = frontiers.size();
    
    #go through all frontiers
    #for(int i = 0; i < numFronts; i++){
    for frontier in frontiers:
        centroid = [0, 0]
        xTotal = 0
        yTotal = 0
        #go through each point in a frontier
        #for(int j = 0 j < frontiers[i].size(); j++){
        for frontierPoint in frontier:
            #xTotal += frontiers[i][j][0];
            #yTotal += frontiers[i][j][1];
            xTotal += frontierPoint[0] 
            yTotal += frontierPoint[1] 

        #create current centroid and update list of centroids
        #centroid[0] = xTotal / frontiers[i].size();
        #centroid[1] = yTotal / frontiers[i].size();
        #centroids[i] = centroid;

        centroid[0] = xTotal / len(frontier)
        centroid[1] = yTotal / len(frontier)
        centroids.append(centroid)
        #np.vstack((centroids, centroid)) #almost definately wrong
    
    return centroids

#done
def distance(centroid):
    x1 = centroid[0]
    y1 = centroid[1]
    x2 = robot_pose.position.x
    y2 = robot_pose.position.y

    return np.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));

#kinda done
def getCurrentRobotPose(pose):
    rospy.loginfo("Getting current robot pose")
    #rd_->getPelvisFrame()
    # rospy.loginfo("Seg fault here?");
    # std::string temp = "Hello World!";//rd_->getPelvisFrame();
    # rospy.loginfo("or here?");
    # current_state->getCurrentPose(temp, robot_pose);

#done
def chooseFrontier(occupancyList):
    global robot_pose
    
    rospy.loginfo("Get current robot pose (TODO: fix this)");
    # getCurrentRobotPose();
    robot_pose = occupancyList.info.origin

    #turn list into array
    rospy.loginfo("Turn list into array, hopefully not transpose")
    rowLength = occupancyList.info.width; #this might be height...
    
    #make occupancyList.data into array
    occupancyArray = createOccupancyArray(occupancyList.data, rowLength)

    #cluster the frontiers
    rospy.loginfo("Cluster the frontier")
    frontiers = findFrontiers(occupancyArray)

    #find centroids of frontiers
    rospy.loginfo("Get centroids of frontiers")
    centroids = getCentroids(frontiers)
    
    #choose frontier
    rospy.loginfo("Choose frontiers")
    closest = centroids[0]
    smallestDistance = distance(centroids[0])
    for c in centroids:
        d = distance(c)
        if(d < smallestDistance):
            smallestDistance = d
            closest = c

    ideal_goal = Point() #iDeAl gooooooooooooooooooooooooooooooooooooal
    ideal_goal.x = closest[0]
    ideal_goal.y = closest[1]
    ideal_goal.z = 0 #might need to change???
    #publish centroid of chosen frontier
    rospy.loginfo("Publishing")
    point_publisher.publish(ideal_goal)

#done?
if __name__ == '__main__':
    rospy.loginfo("PythonFrontierExplorerNode")
    
    point_publisher = rospy.Publisher('ideal_goal', Point)
    rospy.init_node('frontier_explorer_node')
    rate = rospy.Rate(10)

    occupancy_grid_listener = rospy.Subscriber("projected_map", OccupancyGrid, chooseFrontier)

    robot_pose_listener = rospy.Subscriber("robot_pose", Pose, getCurrentRobotPose)

    rospy.loginfo("Starting Spin")
    rospy.spin()