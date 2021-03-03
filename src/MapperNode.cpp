//
// Created by emily on 2/9/21.
//

#include "MapperNode.h"

#include <iostream>
#include "ros/ros.h" 
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Transform.h"
//find all includes (sensor messages as well)


//initialize variables


//set up Global Publishers?
ros::Publisher pointcloud_publisher;
ros::Subscriber pointcloud_listener;



void combineClouds(const sensor_msgs::PointCloud2& newcloud) {
    // do stuff when you get info about robot movement
    
}


int main(int argc, char *argv[])
{
    ROS_INFO_STREAM("MapperNode");

    ros::init(argc, argv, "pointcloud_transformer");

    ros::NodeHandle n;
    //pointcloud_publisher = n.advertise<sensor_msgs::PointCloud2>("transformed_cloud", 1000); 

    pointcloud_listener = n.subscribe("transformed_cloud", 1000, combineClouds);

    ROS_INFO_STREAM("Starting Spin");
    ros::spin();
}