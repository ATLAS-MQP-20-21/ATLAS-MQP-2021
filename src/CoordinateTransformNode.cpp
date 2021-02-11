//
// Created by emily & anir & ryan on 2/9/21.
// (^_^)b
//

#include <iostream>
#include "ros/ros.h" 
#include "CoordinateTransformNode.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Transform.h"
//find all includes (sensor messages as well)


//initialize variables


//set up Global Publishers?


ros::NodeHandle n;




void callback() {

}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "location change");
    ros::Subscriber location_Change = n.subscribe("geometry_msgs/PoseStamped", 1000, callback;
    ros::Publisher wrt_Orign = n.publish()
    ros::spin();
}