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
double filterThreshold = 0; //don't know what these units are


void filterFunction(const sensor_msgs::PointCloud2& cloud_in){
    //Do the z-filtering
    ROS_INFO_STREAM("Starting Z-filtering");

    sensor_msgs::PointCloud2 z_filtered_pc = cloud_in;
    std::vector<unsigned char> newData = {};
    if (haveMatrix){
        ROS_INFO_STREAM("haveMatrix is true");
        int i;
        int numFields = cloud_in.fields.size();
        for(i = 0; i < cloud_in.data.size(); i += numFields){
            //get points
            unsigned char point_x = cloud_in.data[i+X];
            unsigned char point_y = cloud_in.data[i+Y];
            unsigned char point_z = cloud_in.data[i+Z];
            unsigned char point_rgb = cloud_in.data[i+RGB];

            if (point_z > .1){
                //transform points
                //multiply the transformation matrix by [x,y,z,1]
                double new_point_x = tm.data[0]*point_x + tm.data[1]*point_y + tm.data[2]*point_z + tm.data[3];
                double new_point_y = tm.data[4]*point_x + tm.data[5]*point_y + tm.data[6]*point_z + tm.data[7];
                double new_point_z = tm.data[8]*point_x + tm.data[9]*point_y + tm.data[10]*point_z + tm.data[11];

                //Print some useful info
                // ROS_INFO_STREAM("point_z:");
                // ROS_INFO_STREAM(std::to_string(point_z));
                // ROS_INFO_STREAM("new_point_z:");
                // ROS_INFO_STREAM(std::to_string(new_point_z));
            
            

                //If z is less than threshold, replace z point with ground
                if (new_point_z < filterThreshold){
                    double ground = 0; //the desired new z value (wrt origin)
                    double replaced_x = (tm.data[0]*new_point_x + tm.data[4]*new_point_y + tm.data[8]*(ground - tm.data[3]));
                    double replaced_y = (tm.data[1]*new_point_x + tm.data[5]*new_point_y + tm.data[9]*(ground - tm.data[7]));
                    double replaced_z = (tm.data[2]*new_point_x + tm.data[6]*new_point_y + tm.data[10]*(ground - tm.data[11]));
                    //ROS_INFO_STREAM("Put transformed xyz");
                    newData.push_back((unsigned char)replaced_x);
                    newData.push_back((unsigned char)replaced_y);
                    newData.push_back((unsigned char)replaced_z);
                    // ROS_INFO_STREAM("replaced_z:");
                    // ROS_INFO_STREAM(std::to_string(replaced_z));
                }
                //otherwise, keep original z
                else {
                    //ROS_INFO_STREAM("Put original xyz");
                    newData.push_back(point_x);
                    newData.push_back(point_y);
                    newData.push_back(point_z);
                }
            }
            else {
                //ROS_INFO_STREAM("Put original xyz");
                newData.push_back(point_x);
                newData.push_back(point_y);
                newData.push_back(point_z);
            }

            //ROS_INFO_STREAM("Put RGB");
            newData.push_back(point_rgb);


        }

        if (newData.size() == 0){
            ROS_INFO_STREAM("WARNING: no points in filtered cloud");
        }

        ROS_INFO_STREAM("Original length of data:");
        ROS_INFO_STREAM(std::to_string(cloud_in.data.size()));
        ROS_INFO_STREAM("New length of data:");
        ROS_INFO_STREAM(std::to_string(newData.size()));
        
        ROS_INFO_STREAM("About to set data");
        z_filtered_pc.data = newData;
       

    }

    // ROS_INFO_STREAM(std::to_string(cloud_in.data.size()));

    pointcloud_publisher.publish(z_filtered_pc);

    ROS_INFO_STREAM("Published z-filtered point cloud");
}

//save transformation matrix in global variable
void setTransformationMatrix(const std_msgs::Float64MultiArray& msg) {

    haveMatrix = true;
    tm = msg;
    ROS_INFO_STREAM("Got transformation matrix and stored it");
}


int main(int argc, char *argv[])
{
    ROS_INFO_STREAM("Z_Filter");

    ros::init(argc, argv, "Z_Filter");

    ros::NodeHandle n;
    pointcloud_publisher = n.advertise<sensor_msgs::PointCloud2>("filtered_cloud2", 1000); 

    pointcloud_listener = n.subscribe("/multisense/image_points2", 1000, filterFunction);
    matrix_listener = n.subscribe("transformation_matrix", 1000, setTransformationMatrix);

    ROS_INFO_STREAM("Starting Spin");
    ros::spin();
}