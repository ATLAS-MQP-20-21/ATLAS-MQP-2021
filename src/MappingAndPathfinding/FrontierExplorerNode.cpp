//
// Created by emily on 2/9/21.
//

#include "FrontierExplorerNode.h"

#include <iostream>
#include "ros/ros.h" 
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Transform.h"
#include "sensor_msgs/PointCloud2.h"
#include <vector>

//find all includes (sensor messages as well)


//initialize variables
geometry_msgs::Pose robot_pose;

//set up Global Publishers?
ros::Publisher pointcloud_publisher;
ros::Subscriber occupancy_grid_listener;
ros::Subscriber robot_pose_listener;

void chooseFrontier(const nav_msgs::OcccupancyGrid& ocupancyList) {
    //turn list into array
    rowLength = occupancyList.info.width; //this might be height...
    int occupancyArray[][] = createOccupancyArray(occupancyList.data, rowLength)

    //cluster the frontiers
    std::vector<std::vector<std::array<int, 2>>> frontiers = findFrontiers(occupancyArray);

    //find centroids of frontiers
    std::vector<std::array<int, 2>> centroids = getCentroids(frontiers)

    //choose frontier
    std::array<int, 2> closest = centroids[0];
    float smallestDistance = distance(centroids[0]);
    for(c: centroids){
        float d = distance(c);
        if(d < smallestDistance){
            smallestDistance = d;
            closest = c;
        }
    }

    //publish centroid of chosen frontier
    ROS_INFO_STREAM("Publishing");
    pointcloud_publisher.publish(closest);
    
}

int[][] createOccupancyArray(int[] data, int rowLength){
    // make sure data can be divided by rosLength evenly
    if(data.size()%rowLength != 0){
        ROS_INFO_STREAM("WARNING: Data can't be evenly distributed into array wtih given row length");
    }
    //
    int array[rowLength][data.size/rowLength];
    for(int i = 0; i < data.size(); i++){
        array[i%rowLength][i/rowLength] = data[i];
    }
    return array;
}

std::vector<std::vector<std::array<int, 2>>> findFrontiers(int[][] grid){
    // std::vector<std::array<int, 2>> newFrontierCluster = {};
    // std::vector<std::array<int, 2>> toExplore = {};
    std::vector<std::vector<std::array<int, 2>>> returnVector = {};
    for(int i = 0; i < grid.size(); i++){
        for(int j = 0; j < grid[i].size(); j++){
            std::vector<std::array<int, 2>> newFrontierCluster = {};
            std::vector<std::array<int, 2>> toExplore = {};
            toExplore.push_back([i,j]);
            //while there is something to explore, explore
            while(toExplore.size() > 0){
                //if it isn't explored and it is a frontier, check its neighbors
                if(!(toExplore[0] > 100) && isFrontier(grid, toExplore[0])){
                    //add as part of frontier
                    newFrontierCluster.push_back(toExplore[0]);
                    //add neighbors to list to be epxlored
                    addNeighbors(toExplore());
                    //remember that square is checked
                    grid[toExplore[0][0]][toExplore[0][1]] += 100;
                    //erase the first element
                    toExplore.erase(0);
                }
            }
            if(newFrontierCluster.size > 0){
                returnVector.push_back(newFrontierCluster);
            }
        }
    }
    return returnVector;
}

bool isFrontier(int[][] grid, int x, int y){
    //if square is unknown, it isn't a frontier
    if(grid[i][j] < 0){
        return false;
    }
    //check squares in 8-neighborhood to see if any are unknown
    for (int a = -1; a <= 1; a++){
        for(int b = -1; b <= 1; b++){
            if(!(a == 0 && b == 0)){
                if(grid[x][y] == -1) return true;
            }
        }
    }
    return false;
}

std::vector<std::array<int, 2>> getCentroids(std::vector<std::vector<std::array<int, 2>>> frontiers){
    std::vector<std::array<int,2>> centroids;

    int xTotal = 0;
    int yTotal = 0;
    int numFronts = frontiers.size();
    for(int i = 0; i < frontiers.size(); i++){
        for(int j = 0; j < i.size(); j++){
            xTotal += frontiers[i][j][0];
            yTotal += frontiers[i][j][1];
        }
        //add in functionality to account for all centroids
    }

    // std::vector< std::vector<int> >::const_iterator row; 
    // std::vector<int>::const_iterator col;
    // int xTotal;
    // int yTotal;

    // for (row = frontiers.begin(); row != frontiers.end(); ++row)
    // { 
    //      for (col = row->begin(); col != row->end(); ++col)
    //      { 
            
    //      } 
    // } 

    return 0;
}

float distance(std::array<int, 2> centroid){
    float x1 = centroid[0];
    float y1 = centroid[1];
    float x2 = robot_pose[0];
    float y2 = robot_pose[1];

    return Math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))
}

void remember_pose(geometry_msgs::Pose pose){
    robot_pose = pose;
}


int main(int argc, char *argv[])
{
    ROS_INFO_STREAM("FrontierExplorerNode");
    

    ros::init(argc, argv, "frontier_explorer_node");

    ros::NodeHandle n;
    //pointcloud_publisher = n.advertise<sensor_msgs::PointCloud2>("ideal_goal", 1000); 

    occupancy_grid_listener = n.subscribe("projected_map", 1000, chooseFrontier);
    robot_pose_listener = n.subscribe("???", 1000, remember_pose)

    ROS_INFO_STREAM("Starting Spin");
    ros::spin();
}