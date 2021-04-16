#!/usr/bin/env python
import numpy as np
import rospy
import roslib
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import OccupancyGrid

occupancy_grid_listener = 0
c_space_publisher = 0
padding = 1

def makeCSpace(occupancyGrid):
    #add padding

    gridList = occupancyGrid.data
    cspaceGrid = [g for g in gridList]
    rowLength = occupancyGrid.info.width #this might need to be height

    for i, point in enumerate(gridList):
        # Leave points as-is unless an obstacle needs padding
        if point > 50:
            cspaceGrid[i] = 100 # this line is not actually necessary
            pad(cspaceGrid, i, rowLength)

    cspaceOccupancyGrid = OccupancyGrid()
    cspaceOccupancyGrid.data = cspaceGrid
    cspaceOccupancyGrid.info = occupancyGrid.info

    rospy.loginfo("Publishing")
    point_publisher.publish(cspaceOccupancyGrid)

def pad(cspaceGrid, index, rowLength):
    x = int(index / rowLength)
    y = index % rowLength
    # index = x*rowLength + y
    
    # will use padding

    for a in range (-padding, padding):
        for b in range (-padding, padding):
            # convert a, b to list index
            indexNew = (x+a)*rowLength + (y+b)
            if 0 < x + a < rowLength and 0 < y + b < len(cspaceGrid)/rowLength:
                cspaceGrid[indexNew] = 100


if __name__ == '__main__':
    rospy.loginfo("PythonFrontierExplorerNode")
    
    c_space_publisher = rospy.Publisher('c_space_map', OccupancyGrid)
    rospy.init_node('c_space_node')
    rate = rospy.Rate(10)

    occupancy_grid_listener = rospy.Subscriber("projected_map", OccupancyGrid, makeCSpace)

    rospy.loginfo("Starting Spin")
    rospy.spin()