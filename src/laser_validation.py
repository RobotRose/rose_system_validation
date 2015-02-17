#!/usr/bin/python

import roslib; roslib.load_manifest("rose_system_validation")

import rospy
import std_msgs
import geometry_msgs
from sensor_msgs.msg import LaserScan

import numpy as np
import sys
from math import *

data = []

angle = 0 #Middle of Rose's 490 laser beams
try:
    angle = float(sys.argv[1])
except IndexError:
    pass
except ValueError:
    print "Usage: laser_validation <angle>"
    sys.exit(-1)

#TODO: Log all sensor points and calc min/max mean, stddev min/max and index of those mins/maxs

def calc_index_for_angle(stepsize_in_radians, angle_in_degrees_from_center, center_index=245):
    angle_in_radians = radians(angle_in_degrees_from_center)
    steps = angle_in_radians / stepsize_in_radians
    return int(center_index + steps)

def analyze_scan(msg):
    global data
    global laser_listener
    if rospy.Time.now() < end_time and not rospy.is_shutdown():
        #distance = msg.ranges[beam_index]
        data += [msg.ranges] #Add another row of measurements of all beams. Each column contain thus the time-varying measurement of each beam
        rospy.loginfo(msg.ranges[calc_index_for_angle(msg.angle_increment, angle, center_index=len(msg.ranges)/2)])
    else:
        laser_listener.unregister()
        array = np.array(data)

        means = [np.mean(beam) for beam in array.T] #.T transposes, thus we iterate over columns effectively
        stddevs = [np.std(beam) for beam in array.T] #.T transposes, thus we iterate over columns effectively
        # means = [x for x in means if 0 < x < 10000]
        # stddevs = [x for x in stddevs if 0 < x < 10000]

        beam_index = calc_index_for_angle(msg.angle_increment, angle, center_index=len(msg.ranges)/2)
        
        selected_beam = array[:,beam_index]
        mean, stddev = means[beam_index], stddevs[beam_index]
        print "Mean \t\tstdDev over {2} samples\n{0} \t{1} at index {3}".format(mean, stddev, len(selected_beam), beam_index)

        print "min mean: {0}, max mean: {1}".format(min(means), max(means))
        rospy.signal_shutdown("Test complete")

if __name__ == "__main__":
    rospy.init_node("laser_validation")

    start_time = rospy.Time.now()
    end_time = start_time + rospy.Duration(5)

    laser_listener = rospy.Subscriber("/scan", LaserScan, analyze_scan)

    try:
        rospy.spin()
    except TypeError, te:
        print "uhoh"
