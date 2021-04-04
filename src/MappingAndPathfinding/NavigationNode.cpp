//
// Created by emily on 2/9/21.
//

#include "NavigationNode.h"

#include <iostream>
#include "ros/ros.h" 
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Transform.h"
#include "sensor_msgs/PointCloud2.h"
#include <vector>

//find all includes (sensor messages as well)


//initialize variables


//set up Global Publishers?
ros::Publisher navigation_publisher;
ros::Subscriber navigation_listener;



void add_navigation_pose(const sensor_msgs::PointCloud2& new_pose) {
    // do stuff when you get info about robot movement
    ROS_INFO_STREAM("Publishing");
    navigation_publisher.publish(new_pose);
    
}


int main(int argc, char *argv[])
{
    ROS_INFO_STREAM("NavigationNode");
    

    ros::init(argc, argv, "navigation_node");

    ros::NodeHandle n;
    navigation_publisher = n.advertise<sensor_msgs::nav_goal or PoseStamped>("goTo_Pose", 1000); 

    navigation_listener = n.subscribe("ideal_goal", 1000, addClouds);
    c_space_listener = n.subscribe("c_space", 1000, addPadding)

    ROS_INFO_STREAM("Starting Spin");
    ros::spin();
}