//
// Created by emily on 2/9/21.
//

#include "CSpaceNode.h"

#include <iostream>
#include "ros/ros.h" 
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Transform.h"
#include "sensor_msgs/PointCloud2.h"
#include <vector>
#include "home/anir/catkin_ws/src/tough/tough_common/include/tough_common/robot_state.h"


//find all includes (sensor messages as well)


//initialize variables


//set up Global Publishers?
ros::Publisher c_space_publisher;
ros::Subscriber occupancy_grid_listener;



void add_c_space(const sensor_msgs::PointCloud2& newcloud) {
    // do stuff when you get info about robot movement
    ROS_INFO_STREAM("Publishing");
    pointcloud_publisher.publish(newcloud);
    
}


int main(int argc, char *argv[])
{
    ROS_INFO_STREAM("C-Space Node");
    

    ros::init(argc, argv, "C-Space Node");

    ros::NodeHandle n;
    c_space_publisher = n.advertise<sensor_msgs::???>("c_space", 1000); 

    occupancy_grid_listener = n.subscribe("projected_map", 1000, add_c_space);

    ROS_INFO_STREAM("Starting Spin");
    ros::spin();
}