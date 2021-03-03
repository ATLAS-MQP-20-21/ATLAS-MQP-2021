//
// Created by emily on 2/9/21.
//

#include "PointCloudHandlerNode.h"

#include <iostream>
#include "ros/ros.h" 
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Transform.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Float64MultiArray.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <Eigen/Dense>
//find all includes (sensor messages as well)


//set up Global Publishers/subscribers
ros::Publisher pointcloud_publisher;
ros::Subscriber pointcloud_listener;
ros::Subscriber matrix_listener;
sensor_msgs::PointCloud2 cloudout;

//initialize variables
std_msgs::Float64MultiArray transformationMatrix;
bool doingStuff = false;
bool haveTMatrix = false;



void dostuff(const sensor_msgs::PointCloud2& cloudin) {
    
    // do stuff when you get point cloud
    ROS_INFO_STREAM("Recieved Point Cloud message");

    // pcl::PCLPointCloud2 cloudin;
    // pcl_conversions::toPCL(msg, cloudin);
    // pcl::PCLPointCloud2 cloudout;

    //sensor_msgs::PointCloud2 cloudout; //initialize this?

    if(!haveTMatrix){
        ROS_INFO_STREAM("No transformation matrix. Skipping this point cloud.");
        return;
    }

    Eigen::Matrix4f tmatrix = Eigen::Matrix4f::Identity();

    //ROS_INFO_STREAM("1");

    doingStuff = true;
    int i;

    //dies here before printing 3

    for(i = 0; i < 16; i++){
        ROS_INFO_STREAM("2");
        tmatrix (i/4,i%4) = transformationMatrix.data[i]; 
        ROS_INFO_STREAM("3");
    }

    //ROS_INFO_STREAM("4");
    doingStuff = false;

    //ROS_INFO_STREAM("5");c
    pcl_ros::transformPointCloud(tmatrix, cloudin, cloudout);

    //ROS_INFO_STREAM("6");
    pointcloud_publisher.publish(cloudout);
    //ROS_INFO_STREAM("6.1");

}

//save transformation matrix in global variable
void setTransformationMatrix(const std_msgs::Float64MultiArray& msg) {

    //ROS_INFO_STREAM("ryan is really cool guy still");
    haveTMatrix = true;
    //ROS_INFO_STREAM("di da di da didi dodo");
    if(!doingStuff){
        transformationMatrix = msg;
        //ROS_INFO_STREAM("Gennert with rockstar hair");
    }
}

int main(int argc, char *argv[])
{
    ROS_INFO_STREAM("PointCloudHandlerNode");

    ros::init(argc, argv, "pointcloud_transformer");

    ros::NodeHandle n;
    pointcloud_publisher = n.advertise<sensor_msgs::PointCloud2>("transformed_cloud", 1000); 

    pointcloud_listener = n.subscribe("/multisense/image_points2", 1000, dostuff);
    matrix_listener = n.subscribe("transformation_matrix", 1000, setTransformationMatrix);

    ROS_INFO_STREAM("Starting Spin");
    ros::spin();
}