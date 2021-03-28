//The structure of a point cloud is this:
    //the fields variable describes the variables contained in your
    //point cloud (ie x,y,z or RGB for color). If your fields contains
    //x,y,z,rgb, the data variable will be arranged as a 1D array in the 
    //form [x,y,z,rgb,x,y,z,rgb,x,y,z,rgb, ... ]
    //each point describes a point the lidar sees wrt the camera

#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Transform.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/PointCloud2.h"

#include <iostream>
#include <string>
#include <vector>
#include <std_msgs/Float64.h>

ros::Publisher pointcloud_publisher;
ros::Subscriber pointcloud_listener;
ros::Subscriber matrix_listener;

std_msgs::Float64MultiArray tm;
const int X = 0, Y = 1, Z = 2, RGB = 3; //constants for readability
bool haveMatrix = false;
int filterThreshold = 0; //don't know what these units are


void filterFunction(const sensor_msgs::PointCloud2& cloud_in){
    //Do the z-filtering
    ROS_INFO_STREAM("Starting Z-filtering");

    sensor_msgs::PointCloud2 z_filtered_pc = cloud_in;

    // int pointCloudSize = cloud_in.data.length;
    // z_filtered_pc.data.resize(pointCloudSize);

    // int i;
    // for (i = 0; i < pointCloudSize; i++){
    //     if(cloud_in.data[i].z > 3){
    //         z_filtered_pc.data[i] = cloud_int.data[i];
    //     }
    //     else{
            
    //     }
    // }


    /**
     * Streams the characteristics of the incoming pointcloud
     * Uses the fields variable of the point cloud object to get the values of the parameters
     * The breakdown of cloud_in.fields is as follows:
     *   string name      # Name of field
     *   uint32 offset    # Offset from start of point struct
     *   uint8  datatype  # Datatype enumeration, see above
     *   uint32 count     # How many elements in the field
     */
    
    // int i;
    // for(int i = 0; i < cloud_in.fields.size(); i++) {        
    //     std::string strdata = "";
    //     strdata += "Name: ";
    //     strdata += cloud_in.fields[i].name;
    //     strdata += "Offset: ";
    //     strdata += std::to_string(cloud_in.fields[i].offset);
    //     strdata += "Datatype: ";
    //     strdata += std::to_string(cloud_in.fields[i].datatype);
    //     strdata += "Count: ";
    //     strdata += std::to_string(cloud_in.fields[i].count);
        // ROS_INFO_STREAM(strdata);    }

    std::vector<int> newData = {};
    if (haveMatrix){
        int i;
        int numFields = cloud_in.fields.size();
        for(i = 0; i < cloud_in.data.size()/numFields; i += numFields){
            //get points
            double point_x = (double)(cloud_in.data[i+X]);
            double point_y = (double)(cloud_in.data[i+Y]);
            double point_z = (double)(cloud_in.data[i+Z]);
            double point_rgb = (double)(cloud_in.data[i+RGB]);

            //transform points
            //multiply the transformation matrix by [x,y,z,1]
            int new_point_x = (int)(tm.data[0]*point_x + tm.data[1]*point_y + tm.data[2]*point_z + tm.data[3]);
            int new_point_y = (int)(tm.data[4]*point_x + tm.data[5]*point_y + tm.data[6]*point_z + tm.data[7]);
            int new_point_z = (int)(tm.data[8]*point_x + tm.data[9]*point_y + tm.data[10]*point_z + tm.data[11]);

            //determine if points are less than z filter value
            if (new_point_z < filterThreshold){
                //if less than threshold, don't add to new point cloud, otherwise do add
                newData.push_back(new_point_x);
                newData.push_back(new_point_y);
                newData.push_back(new_point_z);
                newData.push_back(point_rgb);
            }
        }

        //convert vector to int array and store in new pc
        int newDataArr[newData.size()];
        for(int i = 0; i < newData.size(); i ++){
            newDataArr[i] = newData[i];
            ROS_INFO_STREAM(std::to_string(newData[i]));
        }
        //z_filtered_pc.data = newData;
        //ROS_INFO_STREAM(newData);

    }

    // ROS_INFO_STREAM(std::to_string(cloud_in.data.size()));

    //pointcloud_publisher.publish(z_filtered_pc);

    ROS_INFO_STREAM("Published z-filtered point cloud");
}

//save transformation matrix in global variable
void setTransformationMatrix(const std_msgs::Float64MultiArray& msg) {

    haveMatrix = true;
    tm = msg;
}


int main(int argc, char *argv[])
{
    ROS_INFO_STREAM("Z_Filter");

    ros::init(argc, argv, "Z_Filter");

    ros::NodeHandle n;
    pointcloud_publisher = n.advertise<sensor_msgs::PointCloud2>("z_filtered_pc", 1000); 

    pointcloud_listener = n.subscribe("/multisense/image_points2", 1000, filterFunction);
    matrix_listener = n.subscribe("transformation_matrix", 1000, setTransformationMatrix);

    ROS_INFO_STREAM("Starting Spin");
    ros::spin();
}