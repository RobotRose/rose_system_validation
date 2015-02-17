#!/usr/bin/python
"""Move the arm to various fs and compare the commanded position to the position of a Augmented Reality marker.
The AR-marker cannot be placed exactly at the gripper point that is suppcommanded_posed to move to the commanded position
Therefore, some transform between the marker and the gripper point must also be defined, e.g. right_arm_wrist_observed.

The transform between right_arm_wrist and right_arm_wrist_observed is the error we're interested in and should be zero, ideally. 

This script will log the commanded and observed positions of the arm so an analysis can be made for the differences between the two.
Plotting could be done with 
- http://docs.enthought.com/mayavi/mayavi/auto/mlab_helper_functions.html#mayavi.mlab.quiver3d 
- http://matplotlib.org/mpl_toolkits/mplot3d/tutorial.html#scatter-plots

The AR-marker detection has a little bit of jitter in the position and some more in the orientation or the marker. 
If we define a correction-transform from the marker to the wrist of gripper, this will amplify error in orientation. 

If we instead define a transform from the gripper or wrist frame to the expected position of the marker, there is no amplification.
So, we will define 3 frames:
/*_arm_wrist --> /*_arm_marker_expected
/*_arm_marker_observed

/*_arm_marker_observed --> /*_arm_marker_expected is the error, as they should ideally be the same
"""

import roslib; roslib.load_manifest("rose_system_validation")

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import *
from arm_controller.msg import manipulateAction, manipulateGoal
import actionlib

import numpy as np
import sys
from math import *
import tf
from tf import TransformListener
import csv
import datetime

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

RIGHT, LEFT = 0, 1
point, rot = 0,1
X,Y,Z = 0,1,2
config_left = {"arm_index":LEFT, "controlled_wrist":"/left_arm_wrist", "expected_arm_marker":"/left_arm_grippermarker_expected", "observed_arm_marker":"/left_arm_grippermarker_observed", "arm_frame":"/left_arm"}
config_right = {"arm_index":RIGHT, "controlled_wrist":"/right_arm_wrist", "expected_arm_marker":"/right_arm_grippermarker_expected", "observed_arm_marker":"/right_arm_grippermarker_observed", "arm_frame":"/right_arm"}

def floatrange(x, y, jump):
  while x < y:
    yield x
    x += jump

def generate_poses(corner1, corner2, steps, frame_id):
    """@param corner1 Minimum (X,Y,Z) point of the cube
    @param corner2 Maximum (X,Y,Z) point of the cube
    @param steps the space between points in each direction"""
    for x in floatrange(corner1[0], corner2[0], steps[0]):
        for y in floatrange(corner1[1], corner2[1], steps[1]):
            for z in floatrange(corner1[2], corner2[2], steps[2]):
                pose = PoseStamped(header=Header(frame_id=frame_id), 
                            pose=Pose(position=Point(x, y, z)))
                yield pose

def format_pair(k, v):
    """Print both in a fitting length"""
    k_str = str(k)
    v_str = "{0:.3f}".format(v) if isinstance(v, float) else str(v)

    length = len(k_str)
    
    k_str2 = k_str.rjust(length)
    v_str2 = v_str.rjust(length)
    return k_str2, v_str2

def typed_input(prompt, as_type, retries=5):
    exception = None
    value = None
    for attempt in range(retries):
        _input = raw_input(prompt)
        try:
            value = as_type(_input)
            break
        except ValueError, e:
            print "Please enter a {0}. {1} retries left".format(as_type.__name__, retries-attempt-1)
            if retries-attempt-1 == 0:
                raise e
    return value

class ArmTester(object):
    def __init__(self, csvfile, arm_index="/left_arm", controlled_wrist="/arm_left_wrist", expected_arm_marker="/arm_left_marker_expected", observed_arm_marker="/TEST_PATTERN", arm_frame="/left_arm"):
        """@param csvfile file to write measurement output data to
        @param arm_index action client URI
        @param controlled_wrist the frame that moves with the arm, e.g. the position of which is commanded through the arm action client
        @param expected_arm_marker a frame that is relative to the AR-marker. Ideally, both *_wrist_frames are at the same position and orientation
        @param observed_arm_marker the frame attached to the AR-marker, observed by ar_kinect"""
        self.client = actionlib.SimpleActionClient("/arms", manipulateAction)

        rospy.loginfo("Waiting for ActionServer('/arms')...")
        self.client.wait_for_server()
        rospy.loginfo("Waited for server")

        self.tf = TransformListener()

        self.arm_index = arm_index

        self.controlled_wrist = controlled_wrist
        self.expected_arm_marker = expected_arm_marker
        self.observed_arm_marker = observed_arm_marker

        self.poses = list(generate_poses((-0.4, -0.4, -0.4), (0.4, 0.4, 0.4), (0.1, 0.1, 0.05), arm_frame))
        rospy.logwarn("Measuring {0} poses".format(len(self.poses)))
        self.end_effector_pose = PoseStamped(header=Header(frame_id=controlled_wrist), pose=Pose(position=Point(0, 0, 0.50))) #point from where end effector is measured
        self.csvfile = csvfile
        self.datastore = None

    def move(self, pose, timeout=10.0):
        goal = manipulateGoal()
        goal.arm = self.arm_index
        goal.required_action = 1
        goal.required_end_effector_mode = 2
        goal.required_pose = pose.pose
        self.client.send_goal(goal)
        rospy.loginfo("Goal sent, waiting for completion")
        return self.client.wait_for_result(rospy.Duration.from_sec(timeout))

    def measure_positions(self):
        try:
            self.tf.waitForTransform("/base_link", self.controlled_wrist, rospy.Time(), rospy.Duration(4.0))
            controlled_tf = self.tf.lookupTransform("/base_link", self.controlled_wrist, rospy.Time(0))

            self.tf.waitForTransform("/base_link", self.expected_arm_marker, rospy.Time(), rospy.Duration(4.0))
            expected_marker_tf = self.tf.lookupTransform("/base_link", self.expected_arm_marker, rospy.Time(0))

            self.tf.waitForTransform("/base_link", self.observed_arm_marker, rospy.Time(), rospy.Duration(4.0))
            observed_marker_tf = self.tf.lookupTransform("/base_link", self.observed_arm_marker, rospy.Time(0))

            error_tf = self.tf.lookupTransform(self.observed_arm_marker, self.expected_arm_marker, rospy.Time(0))

            rospy.loginfo("Commanded wrist pose is {0}, Expected marker pose is {1}, observed marker pose is {2}".format(controlled_tf, expected_marker_tf, observed_marker_tf))

            return error_tf[point]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception), e:
            rospy.logerr("Could not find a required transform: {0}".format(e))
            return [None, None, None]

    def measure_deviation(self, commanded_pose):
        result = self.move(commanded_pose, timeout=10.0)

        if result: 
            rospy.loginfo("Arm commanded to {0.pose.position}".format(commanded_pose))
        else:
            rospy.logerr("Could not reach {0.pose}".format(commanded_pose))
            #break
        
        measured_unloaded = self.measure_positions()
        if not measured_unloaded[Z]:
            rospy.logerr("Could not measure marker position automatically, please proceed manually")
            measured_unloaded[Z] = typed_input("Distance from marker to ground, unloaded: [m] ", float, 5)

        weight = typed_input("Now attach a weight: [kg] ", float, 5)
        
        measured_loaded = self.measure_positions()
        if not measured_loaded[Z]:
            rospy.logerr("Could not measure marker position automatically, please proceed manually")
            measured_loaded[Z] = typed_input("Distance from marker to ground, loaded: [m] ", float, 5)

        raw_input("Please remove the load from the gripper before moving to the next tracepoint. Press Enter when done: ")

        measured_unloaded = [v if v != None else "?" for v in measured_unloaded]
        measured_loaded = [v if v != None else "?" for v in measured_loaded]

        dZ_unloaded = measured_unloaded[Z] - commanded_pose.pose.position.z
        dZ_loaded = measured_loaded[Z] - commanded_pose.pose.position.z
        dZ_due_to_load = measured_loaded[Z] - measured_unloaded[Z]

        measurements = {"X commanded":commanded_pose.pose.position.x, 
                        "Y commanded":commanded_pose.pose.position.y, 
                        "Z commanded":commanded_pose.pose.position.z, 
                        "X measured unloaded":measured_unloaded[X], 
                        "Y measured unloaded":measured_unloaded[Y], 
                        "Z measured unloaded":measured_unloaded[Z], 
                        "X measured loaded":measured_loaded[X], 
                        "Y measured loaded":measured_loaded[Y], 
                        "Z measured loaded":measured_loaded[Z], 
                        "weight":weight, 
                        "measurmt. timestamp":datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                        "dZ_unloaded":dZ_unloaded, 
                        "dZ_loaded":dZ_loaded, 
                        "dZ_due_to_load":dZ_due_to_load}

        return measurements

    def measure_weight_bending(self):
        self.datastore = csv.DictWriter(self.csvfile, [ "measurmt. timestamp",
                                                        "X commanded",
                                                        "Y commanded",
                                                        "Z commanded",
                                                        "X measured unloaded",
                                                        "Y measured unloaded",
                                                        "Z measured unloaded",
                                                        "X measured loaded",
                                                        "Y measured loaded",
                                                        "Z measured loaded",
                                                        "weight",
                                                        "dZ_unloaded",
                                                        "dZ_loaded",
                                                        "dZ_due_to_load"])
        self.datastore.writeheader()

        for commanded_pose in self.poses:
            measurements = self.measure_deviation(commanded_pose)

            measurements = dict([format_pair(a,b) for a,b in measurements.iteritems()])

            rospy.loginfo(measurements)
            self.datastore.writerow(measurements)
            self.csvfile.flush() #Be sure to write the data to file

#draw a vector
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)

class DeviationPlotter(object):
    """Plot the deviations """
    def __init__(self, datafilepath):
        self.dictreader = csv.DictReader(open(datafilepath))
        self.column_headers = self.dictreader.fieldnames
        #import ipdb; ipdb.set_trace()
        # break 201
        self.array = np.zeros((len(self.column_headers)))

    def parse_data(self):
        row_index = 0
        while True:
            try:
                row = self.dictreader.next()
                np_row = np.zeros((len(row)))
                for k,v in row.iteritems():
                    column_index = self.column_headers.index(k)
                    fl = 0
                    try:
                        fl = float(v)
                    except: 
                        pass
                    np_row[column_index] = fl #Replace with NotANumber if the value isn't a number
                self.array = np.vstack((self.array, np_row))
            except StopIteration:
                break

    def plot(self):
        X_commanded = self.array[:, self.column_headers.index("X commanded")]
        Y_commanded = self.array[:, self.column_headers.index("Y commanded")]
        Z_commanded = self.array[:, self.column_headers.index("Z commanded")]
        X_measured_unloaded = self.array[:, self.column_headers.index("X measured unloaded")]
        Y_measured_unloaded = self.array[:, self.column_headers.index("Y measured unloaded")]
        Z_measured_unloaded = self.array[:, self.column_headers.index("Z measured unloaded")]
        X_measured_loaded = self.array[:, self.column_headers.index("X measured loaded")]
        Y_measured_loaded = self.array[:, self.column_headers.index("Y measured loaded")]
        Z_measured_loaded = self.array[:, self.column_headers.index("Z measured loaded")]

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(X_commanded, Y_commanded, Z_commanded, 
            c='r', marker='o')

        ax.scatter(X_measured_unloaded, Y_measured_unloaded, Z_measured_unloaded, 
            c='g', marker='o')

        ax.scatter(X_measured_loaded, Y_measured_loaded, Z_measured_loaded, 
            c='b', marker='o')

        for index, v in enumerate(X_commanded):
            arrow = [X_commanded[index],X_measured_loaded[index]], [Y_commanded[index],Y_measured_loaded[index]], [Z_commanded[index],Z_measured_loaded[index]]
            #print arrow

            ax.add_artist(Arrow3D([X_commanded[index],X_measured_loaded[index]],
                                  [Y_commanded[index],Y_measured_loaded[index]],
                                  [Z_commanded[index],Z_measured_loaded[index]], 
                                  mutation_scale=20, lw=1, arrowstyle="->", color="r"))        

            ax.add_artist(Arrow3D([X_measured_loaded[index],X_measured_unloaded[index]],
                                  [Y_measured_loaded[index],Y_measured_unloaded[index]],
                                  [Z_measured_loaded[index],Z_measured_unloaded[index]],
                                  mutation_scale=20, lw=1, arrowstyle="->", color="k"))

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        plt.show()

if __name__ == '__main__':
    rospy.init_node('arm_test')

    import sys
    config_name = "right"
    try:
        config_name = sys.argv[1]
    except IndexError:
        pass

    datafilename = "arm_test_data.csv"

    if config_name in ["left", "right"]:
        tester = ArmTester(file(datafilename, 'a+'), **config_right)
        rospy.loginfo("Tester created")

        # import ipdb; ipdb.set_trace()
        # break 100 # , check result
        tester.measure_weight_bending()
    elif config_name == "plot":
        plotter = DeviationPlotter(datafilename)
        plotter.parse_data()
        plotter.plot()
    else:
        print "Pass 'left' 'right' or 'plot' as an argument"
