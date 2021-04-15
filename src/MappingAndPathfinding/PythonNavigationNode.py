#!/usr/bin/env python
import numpy as np
import rospy
import roslib
from geometry_msgs.msg import Pose, Point, PoseStamped
from nav_msgs.msg import OccupancyGrid

c_space_listener = 0
point_listener = 0 
goal_publisher = 0
ideal_goal = 0
haveGoal = False

def findClosestWalkable(grid, x, y):
    giveUpThereIsntAWalkable = max(len(grid), len(grid[0]))
    counter = 0
    while counter < giveUpThereIsntAWalkable:
        for a in [-1, 0, 1]:
            for b in [-1, 0, 1]:
                if isWalkable(grid, x + counter*a, y + counter*b):
                    return [x+a, y+b]
        counter+=1
    rospy.loginfo("Your map has frontiers but no walkable spots")
    return None

def isWalkable(grid, x, y):
    if (0 <= x < len(grid)) and (0 <= y < len(grid[0])):
        return grid[x][y] > 50
    return False

def navigate(occupancyGrid):
    if(haveGoal):
        x = ideal_goal.x
        y = ideal_goal.y

        grid = occupancyGrid.data

        newGoal = findClosestWalkable(grid, x, y)

        newGoalPoint = PoseStamped()

        newGoalPoint.pose.position.x = x
        newGoalPoint.pose.position.y = y

        rospy.loginfo("Publishing")
        point_publisher.publish(newGoalPoint)

    else:
        rospy.loginfo("No goal point published yet")

def getIdealGoal(goal):
    rospy.loginfo("Storing ideal goal")
    haveGoal = True
    ideal_goal = goal

if __name__ == '__main__':
    rospy.loginfo("PythonFrontierExplorerNode")
    
    goal_publisher = rospy.Publisher('goalthing', PoseStamped)
    rospy.init_node('frontier_explorer_node')
    rate = rospy.Rate(10)

    c_space_listener = rospy.Subscriber("c_space_map", OccupancyGrid, navigate)
    point_listener = rospy.Subscriber("ideal_goal", Point, navigate)

    rospy.loginfo("Starting Spin")
    rospy.spin()