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


//set up Global Publishers?
ros::Publisher pointcloud_publisher;
ros::Subscriber occupancy_grid_listener;

void chooseFrontier(const nav_msgs::OcccupancyGrid& ocupancyList) {
    //turn list into array
    rowLength = occupancyList.info.width; //this might be height...
    int occupancyArray[rowLength][occupancyList.data.size/rowLength] = createOccupancyArray(occupancyList.data, rowLength)

    //cluster the frontiers
    std::vector<std::vector<std::array<int, 2>>> = findFrontiers(occupancyArray);

    //find centroids of frontiers

    //choose frontier

    //publish centroid of chosen frontier
    ROS_INFO_STREAM("Publishing");
    pointcloud_publisher.publish(newcloud);
    
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
    return 0;
}


int main(int argc, char *argv[])
{
    ROS_INFO_STREAM("FrontierExplorerNode");
    

    ros::init(argc, argv, "frontier_explorer_node");

    ros::NodeHandle n;
    //pointcloud_publisher = n.advertise<sensor_msgs::PointCloud2>("ideal_goal", 1000); 

    occupancy_grid_listener = n.subscribe("projected_map", 1000, chooseFrontier);

    ROS_INFO_STREAM("Starting Spin");
    ros::spin();
}