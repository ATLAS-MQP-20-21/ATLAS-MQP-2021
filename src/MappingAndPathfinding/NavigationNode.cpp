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
//#include "home/anir/catkin_ws/src/tough/tough_common/include/tough_common/robot_state.h"


//find all includes (sensor messages as well)


//initialize variables
nav_msgs::OccupancyGrid padded_map;

//set up Global Publishers
ros::Publisher navigation_publisher;
ros::Subscriber navigation_listener;
ros::Subscriber c_space_listener;

// find goal and send to footstep planner
void navigate(const sensor_msgs::PointCloud2& new_pose) {
    
    //Find closest walkable spot to given pose
    //get footstep planner to 

    ROS_INFO_STREAM("Publishing");
    navigation_publisher.publish(new_pose);
    
}

// store c-space map
void rememberMap(const nav_msgs::OccupancyGrid& map){
    padded_map = map;
}


int main(int argc, char *argv[])
{
    ROS_INFO_STREAM("NavigationNode");
    

    ros::init(argc, argv, "navigation_node");

    ros::NodeHandle n;
    navigation_publisher = n.advertise<sensor_msgs::nav_goal or PoseStamped>("goTo_Pose", 1000); 

    navigation_listener = n.subscribe("ideal_goal", 1000, navigate);
    c_space_listener = n.subscribe("c_space", 1000, rememberMap)

    ROS_INFO_STREAM("Starting Spin");
    ros::spin();
}