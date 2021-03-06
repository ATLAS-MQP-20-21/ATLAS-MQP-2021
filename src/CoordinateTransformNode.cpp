
#include <iostream>
#include "ros/ros.h"
#include "CoordinateTransformNode.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Transform.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"

#include <vector>
#include <std_msgs/Float64.h>

//#include "std/Vector.h"


//find all includes (sensor messages as well)


//initialize variables
//double rotMat[4][4];

//set up Global Publishers?



ros::Publisher location_highlyillogical;
ros::Subscriber location_Change;


void makeRotMat(const geometry_msgs::PoseStamped& msg) {
    // do stuff when you get info about robot movement
    // extract x,y,z, and w (theta?) and create this matrix for multiplication: [cosw -sinw 0 x; sinw cosw 0 y; 0 0 1 z; 0 0 0 1]

    ROS_INFO_STREAM("Recieved PoseStamped Message");

    std_msgs::Float64 x, y, z, q0, q1, q2, q3;
    
    x.data = msg.pose.position.x;
    y.data = msg.pose.position.y;
    z.data = msg.pose.position.z;

    q0.data = msg.pose.orientation.x;
    q1.data = msg.pose.orientation.y;
    q2.data = msg.pose.orientation.z;
    q3.data = msg.pose.orientation.w;

    float r00, r01, r02, r10, r11, r12, r20, r21, r22;
    
    //# First row of the rotation matrix
    r00 = 2 * (q0.data * q0.data + q1.data * q1.data) - 1;
    r01 = 2 * (q1.data * q2.data - q0.data * q3.data);
    r02 = 2 * (q1.data * q3.data + q0.data * q2.data);
     
    //# Second row of the rotation matrix
    r10 = 2 * (q1.data * q2.data + q0.data * q3.data);
    r11 = 2 * (q0.data * q0.data + q2.data * q2.data) - 1;
    r12 = 2 * (q2.data * q3.data - q0.data * q1.data);
     
    //#.data Third row of the rotation matrix
    r20 = 2 * (q1.data * q3.data - q0.data * q2.data);
    r21 = 2 * (q2.data * q3.data + q0.data * q1.data);
    r22 = 2 * (q0.data * q0.data + q3.data * q3.data) - 1;


    // //# Example of how it was before
    // r20 = 2 * (q1 * q3 - q0 * q2);
    // r21 = 2 * (q2 * q3 + q0 * q1);
    // r22 = 2 * (q0 * q0 + q3 * q3) - 1;

    //double rotMat[4][4] = {{r00,r01, r02, x}, {r10, r11,r12, y}, {r20, r21, r22, z}, {0.0, 0.0, 0.0, 1.0}};
    //was float, but couldn't do std::vector double to float[16]
    
    //std::vector<double> rotMat[16] = {r00,r01, r02, x, r10, r11,r12, y, r20, r21, r22, z, 0, 0, 0, 1};
    
    float rotMat[16] = {-r00, r01, r02, x.data, 
                        r10, -r11, r12, y.data, 
                        r20, r21, r22, z.data, 
                        0, 0, 0, 1};
    //didadiadididdodo didi da didi do dididididi da didi do 
    std_msgs::Float64MultiArray sentMatrix;
    sentMatrix.data.resize(16);
 
    
    int i = 0;
    for(i = 0; i < 16; i++){
        sentMatrix.data[i] = rotMat[i];
    }

    //float rotMat = 1.0;
    //std_msgs sentMatrix;
    // sentMatrix::Float64MultiArray.data = rotMat;


    // std_msgs::Float64MultiArray sentMatrix;
    //sentMatrix.data = rotMat;
    
    location_highlyillogical.publish(sentMatrix);

    

}


int main(int argc, char *argv[])
{
    ROS_INFO_STREAM("CoordinateTransformNode");
    ros::init(argc, argv, "location_change");

    ros::NodeHandle n;
    location_highlyillogical = n.advertise<std_msgs::Float64MultiArray>("transformation_matrix", 1000);

    location_Change = n.subscribe("stereo_odometer/pose", 1000, makeRotMat);
    //ros::Publisher wrt_Orign = n.publish();

    ROS_INFO_STREAM("Starting Spin");
    ros::spin();
    
}

