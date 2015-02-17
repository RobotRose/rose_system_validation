#!/usr/bin/python
"""Rose's TF tree is not fully correct and will maybe never be. This script helps you to find the largest error and tries to fix them
"""

import roslib; roslib.load_manifest("rose_system_validation")

import rospy
import std_msgs
import geometry_msgs

import numpy as np
import sys
from math import *

import tf
from tf import TransformListener
import csv
import datetime

point, rot = 0,1
X,Y,Z = 0,1,2

interesting_pairs = [
("base_link"                        , "neck_mount"),
("base_link"                        , "neck_pan"),
("base_link"                        , "neck_tilt_mount"),
("base_link"                        , "neck_tilt"),
("base_link"                        , "camera_rgb_frame"),
("base_link"                        , "right_arm_grippermarker_observed"),
# ("base_link"                        , "right_arm_grippermarker_expected"),
("base_link"                        , "arms"),
# ("base_link"                        , "ar_marker_1"),
# ("base_link"                        , "right_arm"),
# ("base_link"                        , "lift_link"),
("arms"                             , "right_arm"),
# ("camera_rgb_frame"                 , "ar_marker_1"),
("right_arm"                        , "neck_mount"),
# ("right_arm_grippermarker_expected" , "right_arm_grippermarker_observed"),
("lift_link"                        , "arms"),
# ("lift_link"                        , "right_arm"),
("lift_link"                        , "neck_mount"),
("neck_mount"                       , "neck_pan"),
("neck_pan"                         , "neck_tilt_mount"),
("neck_tilt_mount"                  , "neck_tilt"),
("neck_tilt"                        , "camera_rgb_frame"),
# ("base_link"                        , "right_arm_grippermarker_observed"),
("camera_rgb_frame"                 , "right_arm_grippermarker_observed"),
("neck_mount"                       , "neck_tilt_mount"),
("arms"                             , "camera_rgb_frame"),
("lift_link"                        , "camera_rgb_frame")]

def typed_input(prompt, as_type, retries=5, fallback=None):
    exception = None
    value = None
    for attempt in range(retries):
        _input = raw_input(prompt)
        try:
            value = as_type(_input)
            break
        except ValueError, e:
            if retries > 1: print "Please enter a {0}. {1} retries left".format(as_type.__name__, retries-attempt-1)
            if retries-attempt-1 <= 0:
                # raise e
                value = fallback
    return value

def format_pair(k, v):
    """Print both in a fitting length"""
    k_str = str(k)
    v_str = "{0:.3f}".format(v) if isinstance(v, float) else str(v)

    length = len(k_str)
    
    k_str2 = k_str.rjust(length)
    v_str2 = v_str.rjust(length)

    if str(None) in v_str2:
        v_str2 = ''

    return k_str2, v_str2

class TfNode(object):
    def __init__(self, name, child_nodes=None):
        self.name = name
        self.child_nodes = set(child_nodes if child_nodes else [])

    def __repr__(self, level=0):
        ret = "\t"*level+repr(self.name)+"\n"
        for child in self.child_nodes:
            ret += child.__repr__(level+1)
        return ret

    def add_descendant(self, parent, new):
        """Returns whether the child was added. If the parent was not yet in the tree, returns False, otherwise True"""
        # import ipdb; ipdb.set_trace()
        if parent == self.name:
            if new not in [child.name for child in self.child_nodes]:
                self.child_nodes.add(TfNode(new, []))
                return True
        else:#if parent in [child.name for child in self.child_nodes]:
            for child in self.child_nodes:
                if child.add_descendant(parent, new):
                    return True
        return False
    
    def add_family(self, parent_child_pairs):
        #TODO: There is parobably a niice way to 
#         import collections
#         queue = collections.deque(parent_child_pairs)
        
#         parents = set([parent for parent, child in parent_child_pairs.iteritems()])
#         childs = set([child for parent, child in parent_child_pairs.iteritems()])
#         roots = parents - childs
        
#         while queue:
#             pair = queue.pop()
        for i in range(100):
            for pair in parent_child_pairs:
                self.add_descendant(*pair)

    def has_descendant(self, name):
        return name in [n.name for n in self.child_nodes] or any(child.has_descendant(name) for child in self.child_nodes)

    def get_path(self, _from, to):
        if _from == self.name and self.has_descendant(to):
            if to in [n.name for n in self.child_nodes]:
                return [_from, to]
            
            for child in self.child_nodes:
                if child.has_descendant(to):
                    return [self.name] + child.get_path(child.name, to)
        elif self.has_descendant(_from):
            #print "I'm not 'from', but a descendant  is"
            for child in self.child_nodes:
                if child.has_descendant(_from) or child.name == _from:
                    #print "Child {0} should be able to get a path {1} --> {2}".format(child, _from, to)
                    return child.get_path(_from, to)


class TfChecker(object):
    def __init__(self, csvfile):
        self.tf = TransformListener()

        self.csvfile = csvfile
        self.datastore = csv.DictWriter(self.csvfile, [ "measrmnt. timestamp",
                                                        "parent TF",
                                                        "child  TF",
                                                        "X from TF",
                                                        "Y from TF",
                                                        "Z from TF",
                                                        "D from TF",
                                                        "X    Real",
                                                        "Y    Real",
                                                        "Z    Real",
                                                        "Dist Real",
                                                        "dX R - TF",
                                                        "dY R - TF",
                                                        "dZ R - TF",
                                                        "dD   R-TF",
                                                        "Max Error"])
        self.datastore.writeheader() 

        self.tree = self.build_tree(self._get_pairs())

    def _get_pairs(self):
        rospy.loginfo("Waiting for TFs to apppear...")
        self.tf.waitForTransform("base_link", "camera_rgb_frame", rospy.Time(), rospy.Duration(10.0))
        rospy.loginfo("TF tree/forrest received")
        str_repr = self.tf.allFramesAsString()
        # print str_repr
        # import ipdb; ipdb.set_trace()
        lines = str_repr.split("\n")
        lines = [line.split(" exists with parent ") for line in lines]

        child_to_parent_map = dict([(pairline[0].replace("Frame ", ''), pairline[1].replace(".", '')) for pairline in lines if pairline[0]])
        return child_to_parent_map

    def build_tree(self, child_to_parent_map, root="base_link"):
        # forest = []
        # import ipdb; ipdb.set_trace()
        # parents = set([parent for child, parent in child_to_parent_map.iteritems()])
        # childs = set([child for child, parent in child_to_parent_map.iteritems()])
        # roots = parents - childs
        # for root in roots:
        #     descendants = [parent for child, parent in child_to_parent_map.iteritems()]
        tree = TfNode(root)
        tree.add_family([(parent, child) for child, parent in child_to_parent_map.iteritems()])
        return tree


    def compare_tf(self, framepair):
        parent, child = framepair

        try:
            self.tf.waitForTransform(parent, child, rospy.Time(), rospy.Duration(4.0))
            translation, rotation = self.tf.lookupTransform(parent, child, rospy.Time(0))
            translation = list(translation)

            translation[X] = abs(translation[X])
            translation[Y] = abs(translation[Y])
            translation[Z] = abs(translation[Z])

            X_real = typed_input("The X distance between {0}  and {1}: ".format(parent, child), float, retries=1, fallback=None)
            Y_real = typed_input("The Y distance between {0}  and {1}: ".format(parent, child), float, retries=1, fallback=None)
            Z_real = typed_input("The Z distance between {0}  and {1}: ".format(parent, child), float, retries=1, fallback=None)

            dfromTF = sqrt(translation[X]**2 + translation[Y]**2 + translation[Z]**2)
            distReal = sqrt((X_real if X_real else 0)**2 + (Y_real if Y_real else 0)**2 + (Z_real if Z_real else 0)**2)
            dX = X_real - translation[X] if X_real else None
            dY = Y_real - translation[Y] if Y_real else None
            dZ = Z_real - translation[Z] if Z_real else None

            measurement = {"measrmnt. timestamp":datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                           "parent TF":parent,
                           "child  TF":child,
                           "X from TF":translation[X],
                           "Y from TF":translation[Y],
                           "Z from TF":translation[Z],
                           "D from TF":dfromTF,
                           "X    Real":X_real,
                           "Y    Real":Y_real,
                           "Z    Real":Z_real,
                           "Dist Real":distReal,
                           "dX R - TF":dX,
                           "dY R - TF":dY,
                           "dZ R - TF":dZ,
                           "dD   R-TF":distReal - dfromTF,
                           "Max Error":max([val for val in [dX, dY, dZ] if val])}
            return measurement

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception), e:
            rospy.logerr("Could not find a required transform: {0}".format(e))

    def check_tfs(self, exclude=None):
        exclude = [] if exclude == None else exclude
        pairs_to_check = [pair for pair in interesting_pairs if pair not in exclude]

        for index, pair in enumerate(pairs_to_check):
            rospy.loginfo("Checking transform {0}/{1}: {2} to {3}".format(index, len(pairs_to_check), pair[0], pair[1]))
            measurements = self.compare_tf(pair)

            if measurements:
                measurements = dict([format_pair(a,b) for a,b in measurements.iteritems()])

                rospy.loginfo(measurements)

                self.datastore.writerow(measurements)
                self.csvfile.flush() #Be sure to write the data to file

    def verify_tf_tree(self, _from="base_link", to="camera_link", investigate_if_error_larger_than=0.005):
        #import ipdb; ipdb.set_trace()
        chain = self.tree.get_path(_from, to)
        if not chain:
            rospy.logerr("There was no chain from {0} to {1}".format(_from, to))
            return
        print "The chain {0} needs checking".format(chain, investigate_if_error_larger_than)
        measurements = self.compare_tf((chain[0], chain[-1]))

        formatted_measurements = dict([format_pair(a,b) for a,b in measurements.iteritems()])
        self.datastore.writerow(formatted_measurements)
        self.csvfile.flush()

        chain_error = abs(measurements["Max Error"])

        if chain_error > investigate_if_error_larger_than: #Largest than 5mm
            rospy.logerr("The error between {0} and {1} is {2:.4f}m > {3:.4f}m".format(_from, to, chain_error, investigate_if_error_larger_than))
            split_index = int(len(chain)/2)
            lower = chain[0:split_index]
            upper = chain[split_index:]
            
            self.verify_tf_tree(_from=lower[0], to=lower[-1])
            self.verify_tf_tree(_from=upper[0], to=upper[-1])
        else:
            rospy.loginfo("The error over the whole chain {0} is {1:.4f}m <= {2:.4f}m".format(chain, chain_error, investigate_if_error_larger_than))


if __name__ == '__main__':
    rospy.init_node('tf_check')

    datafilename = "tf_check_data.csv"


    csvfile = file(datafilename, 'a+')
    existing_data = csv.DictReader(csvfile)

    already_measured_framepairs = None
    #import ipdb; ipdb.set_trace()
    
    try:
        if "parent TF" in existing_data.fieldnames and "child  TF" in existing_data.fieldnames:
            already_measured_framepairs = [(row["parent TF"].strip(), row["child  TF"].strip()) for row in existing_data]
            rospy.logwarn("Skipping pairs {0} because they were measured earlier in the current data file".format(already_measured_framepairs))
    except TypeError, te:
        pass

    checker = TfChecker(csvfile)
    print checker.tree

    #checker.check_tfs(exclude=already_measured_framepairs)
    checker.verify_tf_tree(to="neck_tilt", investigate_if_error_larger_than=0.005)
    checker.verify_tf_tree(to="right_arm", investigate_if_error_larger_than=0.005)
    checker.verify_tf_tree(to="right_arm_grippermarker_observed", investigate_if_error_larger_than=0.005)

    