#!/usr/bin/python
"""Drive the robot around in some pattern using move_base.
move_base relies on amcl, which integrates both odometry and laser localization.
This together determines the robot's navigation accuracy."""

import roslib; roslib.load_manifest("rose_system_validation")

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

import numpy as np
import sys
from math import *

#Middle of the big room, looking at the fridge
middle_of_room_1 = PoseStamped(header=Header(frame_id="/map"), 
                            pose=Pose(position=Point(-1.5, 4.0, 0.000), 
                                      orientation=Quaternion(0.000, 0.000, 0.454, 0.891)))

#Near middle of the big room, facing the 'hall' to the door.
middle_of_room_2 = PoseStamped(header=Header(frame_id="/map"), 
                            pose=Pose(position=Point(-1.985, 4.107, 0.000), 
                                      orientation=Quaternion(0.000, 0.000, 0.955, -0.295)))

forward_half_meter = PoseStamped(header=Header(frame_id="/base_link"), 
                                  pose=Pose(position=Point(x=0.5)))

backward_half_meter = PoseStamped(header=Header(frame_id="/base_link"), 
                                  pose=Pose(position=Point(x=-0.5)))


class NavigationTester(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/move_base_smc', MoveBaseAction)

    def move(self, pose):
        self.client.send_goal(MoveBaseGoal())
        rospy.loginfo("Goal sent, waiting for completion")
        return self.client.wait_for_result(rospy.Duration.from_sec(50.0))

if __name__ == '__main__':
    rospy.init_node('nav_test')

    tester = NavigationTester()
    rospy.loginfo("Tester created")

    positions = [middle_of_room_1, middle_of_room_2, forward_half_meter, backward_half_meter]

    success = True
    for pose in positions:
        if success:
            success = tester.move(pose)
            if not success:
                rospy.logerr("Could not move to pose {0}".format(pose))
                break