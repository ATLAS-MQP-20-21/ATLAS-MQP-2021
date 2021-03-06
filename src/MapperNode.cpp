//
// Created by emily on 2/9/21.
//

#include "MapperNode.h"

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
ros::Subscriber pointcloud_listener;



void addClouds(const sensor_msgs::PointCloud2& newcloud) {
    // do stuff when you get info about robot movement
    pointcloud_publisher.publish(newcloud);
    
}


int main(int argc, char *argv[])
{
    ROS_INFO_STREAM("MapperNode");

    ros::init(argc, argv, "mapper_node");

//number one victory royale 
//yeah fortnite we bout to get down (get down!)

    ros::NodeHandle n;
    pointcloud_publisher = n.advertise<sensor_msgs::PointCloud2>("cloud_in", 1000); 

    pointcloud_listener = n.subscribe("transformed_cloud", 1000, addClouds);

    ROS_INFO_STREAM("Starting Spin");
    ros::spin();
}