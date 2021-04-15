//
// Created by Snake Boy on 04/11/21.
//

//#include "CSpaceNode.h"

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
ros::Publisher pose_publisher;
ros::Subscriber pose_listener;



void publish_pose(const sensor_msgs::PointCloud2& newcloud) {
    // do stuff when you get info about robot movement
    ROS_INFO_STREAM("Publishing");
    pointcloud_publisher.publish(newcloud);
    
}


int main(int argc, char *argv[])
{
    ROS_INFO_STREAM("C-Space Node");
    

    ros::init(argc, argv, "C-Space Node");

    ros::NodeHandle n;
    pose_publisher = n.advertise<sensor_msgs::???>("current_robot_pose", 1000); 

    pose_listener = n.subscribe("projected_map", 1000, publish_pose);

    ROS_INFO_STREAM("Starting Spin");
    ros::spin();
}