#!/usr/bin/env python
import numpy as np
import rospy
import roslib
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import OccupancyGrid

occupancy_grid_listener = 0
c_space_publisher = 0

def makeCSpace(occupancyGrid)
    #add padding. That's all

    rospy.loginfo("Publishing")
    point_publisher.publish(cSpaceMap)

if __name__ == '__main__':
    rospy.loginfo("PythonFrontierExplorerNode")
    
    c_space_publisher = rospy.Publisher('c_space_map', OccupancyGrid)
    rospy.init_node('frontier_explorer_node')
    rate = rospy.Rate(10)

    occupancy_grid_listener = rospy.Subscriber("projected_map", OccupancyGrid, makeCSpace)

    rospy.loginfo("Starting Spin")
    rospy.spin()