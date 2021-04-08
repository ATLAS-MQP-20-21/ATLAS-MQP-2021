//
// Created by emily on 2/9/21.
//

#include "FrontierExplorerNode.h"

#include <iostream>
#include "ros/ros.h" 
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Transform.h"
#include "sensor_msgs/PointCloud2.h"    
#include "nav_msgs/OccupancyGrid.h"
#include <vector>
#include <deque>
#include <array>

#include "tough_common/robot_state.h"

//find all includes (sensor messages as well)


//initialize variables
geometry_msgs::Pose robot_pose;
RobotStateInformer* current_state;
RobotDescription* rd_;

//set up Global Publishers?
ros::Publisher pointcloud_publisher;
ros::Subscriber occupancy_grid_listener;

// std::vector<std::vector<int>>> createOccupancyArray(int data[], int rowLength){
//     // make sure data can be divided by rosLength evenly
//     int dataSize = sizeof(data)/sizeof(*data);
//     if(dataSize % rowLength != 0){
//         ROS_INFO_STREAM("WARNING: Data can't be evenly distributed into array wtih given row length");
//     }
//     //

//     //int a[7];
//     //std::cout << "Length of array = " << (sizeof(a)/sizeof(*a)) << std::endl;
//     int *array = new int[rowLength][dataSize/rowLength];
//     // int array[rowLength][dataSize/rowLength];
//     for(int i = 0; i < dataSize; i++){
//         array[i%rowLength][i/rowLength] = data[i];
//     }
//     return array;

int arrayToVectorCoordinates(int x, int y){

}

bool isFrontier(std::vector<std::vector<int>> grid, int x, int y){
    //if square is unknown, it isn't a frontier
    if(grid[x][y] < 0){
        return false;
    }
    //check squares in 8-neighborhood to see if any are unknown
    for (int a = -1; a <= 1; a++){
        for(int b = -1; b <= 1; b++){
            if(!(a == 0 && b == 0)){
                if(grid[a][b] == -1) return true;
            }
        }
    }
    return false;
}

void addNeighbors(std::deque<std::array<int, 2>> toExplore, int x, int y){
    for (int a = -1; a <= 1; a++){
        for(int b = -1; b <= 1; b++){
            if(!(a == 0 && b == 0)){
                toExplore.push_back({x+a, y+b});
            }
        }
    }
}

std::vector<std::vector<std::array<int, 2>>> findFrontiers(std::vector<std::vector<int>> grid){
    // std::vector<std::array<int, 2>> newFrontierCluster = {};
    // std::vector<std::array<int, 2>> toExplore = {};
    std::vector<std::vector<std::array<int, 2>>> returnVector = {};
    for(int i = 0; i < grid.size(); i++){
        for(int j = 0; j < grid[i].size(); j++){
            std::vector<std::array<int, 2>> newFrontierCluster = {};
            std::deque<std::array<int, 2>> toExplore = {};
            toExplore.push_back({i,j});
            //while there is something to explore, explore
            while(toExplore.size() > 0){
                int x = toExplore[0][0];
                int y = toExplore[0][1];
                //if it isn't explored and it is a frontier, check its neighbors
                if(!(grid[x][y] > 100) && isFrontier(grid, x, y)){
                    //add as part of frontier
                    newFrontierCluster.push_back(toExplore[0]);
                    //add neighbors to list to be epxlored
                    addNeighbors(toExplore, x, y);
                    //remember that square is checked
                    grid[x][y] += 100;
                    //erase the first element
                    toExplore.pop_front();
                }
            }
            if(newFrontierCluster.size() > 0){
                ROS_INFO_STREAM("Found new frontier cluster");
                returnVector.push_back(newFrontierCluster);
            }
        }
    }
    return returnVector;
}


std::vector<std::array<int, 2>> getCentroids(std::vector<std::vector<std::array<int, 2>>> frontiers){
    std::vector<std::array<int,2>> centroids;

    int xTotal = 0;
    int yTotal = 0;
    int numFronts = frontiers.size();
    //go through all frontiers
    for(int i = 0; i < numFronts; i++){
        std::array<int,2> centroid;
        xTotal = 0;
        yTotal = 0;
        //go through each point in a frontier
        for(int j = 0; j < frontiers[i].size(); j++){
            xTotal += frontiers[i][j][0];
            yTotal += frontiers[i][j][1];
        }
        //create current centroid and update list of centroids
        centroid[0] = xTotal / frontiers[i].size();
        centroid[1] = yTotal / frontiers[i].size();
        centroids[i] = centroid;
    }
    
    return centroids;
}

float distance(std::array<int, 2> centroid){
    float x1 = centroid[0];
    float y1 = centroid[1];
    float x2 = robot_pose.position.x;
    float y2 = robot_pose.position.y;

    return std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}


void getCurrentRobotPose(){
    //rd_->getPelvisFrame()
    // ROS_INFO_STREAM("Seg fault here?");
    // std::string temp = "Hello World!";//rd_->getPelvisFrame();
    // ROS_INFO_STREAM("or here?");
    // current_state->getCurrentPose(temp, robot_pose);
}

void chooseFrontier(const nav_msgs::OccupancyGrid& occupancyList) {
    ROS_INFO_STREAM("Getting current robot pose");
    // getCurrentRobotPose();
    robot_pose = occupancyList.info.origin;

    //turn list into array
    ROS_INFO_STREAM("Turn list into array, hopefully not transpose");
    int rowLength = occupancyList.info.width; //this might be height...
    
    //make occupancyList.data into array
    ROS_INFO_STREAM("Check array size");
    int dataSize = occupancyList.data.size();
    if(dataSize % rowLength != 0){
        ROS_INFO_STREAM("WARNING: Data can't be evenly distributed into array wtih given row length");
    }
    
    ROS_INFO_STREAM("Make new array");
    std::vector<std::vector<int>> occupancyArray;//([rowLength][dataSize/rowLength];
    // int array[rowLength][dataSize/rowLength];
    ROS_INFO_STREAM("Fill array");
    for(int i = 0; i < dataSize; i++){
        //Make sure that first dimension of array already exists
        while(occupancyArray.size() < i%rowLength){
            occupancyArray.push_back({});
        }
        occupancyArray[i%rowLength].push_back(occupancyList.data[i]);
        //occupancyArray[i%rowLength][i/rowLength] = occupancyList.data[i];
    }
    
    //int *occupancyArray[][occupancyList.data.size()/rowLength] = createOccupancyArray(occupancyList.data, rowLength);

    //cluster the frontiers
    ROS_INFO_STREAM("Cluster the frontier");
    std::vector<std::vector<std::array<int, 2>>> frontiers = findFrontiers(occupancyArray);

    //find centroids of frontiers
    ROS_INFO_STREAM("Get centroids of frontiers");
    std::vector<std::array<int, 2>> centroids = getCentroids(frontiers);

    //choose frontier
    ROS_INFO_STREAM("Choose frontiers");
    std::array<int, 2> closest = centroids[0];
    float smallestDistance = distance(centroids[0]);
    for(std::array<int, 2> c: centroids){
        float d = distance(c);
        if(d < smallestDistance){
            smallestDistance = d;
            closest = c;
        }
    }

    geometry_msgs::Point ideal_goal; //iDeAl gooooooooooooooooooooooooooooooooooooal
    ideal_goal.x = closest[0];
    ideal_goal.y = closest[1];
    ideal_goal.z = 0; //might need to change???
    //publish centroid of chosen frontier
    ROS_INFO_STREAM("Publishing");
    pointcloud_publisher.publish(ideal_goal);
    
}



int main(int argc, char *argv[])
{
    ROS_INFO_STREAM("FrontierExplorerNode");
    

    ros::init(argc, argv, "frontier_explorer_node");

    ros::NodeHandle n;
    pointcloud_publisher = n.advertise<geometry_msgs::Point>("ideal_goal", 1000); 

    occupancy_grid_listener = n.subscribe("projected_map", 1000, chooseFrontier);

    ROS_INFO_STREAM("Starting Spin");
    ros::spin();
}