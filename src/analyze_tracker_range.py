#!/usr/bin/python
"""This scrips checks and analyses the range where marker trackers, like ar_kinect or ar_track_alvar, can track markers.
Most interesting is the combination of distance and 'skew'. 
A tracker, I assume, works best when the Z-axis (its looking direction, perpendicular to the sensor plane) of the camera is parallel to the Z-axis of the marker,
which is perpendicular to its face. 
The skew is the angle between these Z-axes.
"""

import roslib; roslib.load_manifest("rose_system_validation")

import rospy
import std_msgs
import geometry_msgs.msg as gm
from std_msgs.msg import Header, Float64
from dynamixel_msgs.msg import JointState
from sensor_msgs.msg import LaserScan

import numpy as np
import sys
import actionlib
from math import *

import tf
from tf.transformations import euler_from_quaternion
import datetime
import csv
import matplotlib.pyplot as plt
import pylab

TF_WAIT_TIME = 0.5 #4

x,y,z = 0,1,2
qx, qy, qz, qw = 0,1,2,3

timestamp_format = "%Y-%m-%d %H:%M:%S"

#ar_marker_9 = gm.PoseStamped(header=Header(frame_id="wall"), position=gm.Point(0,0,1.924), orientation=gm.Quaternion(0,0,0,1))
# ar_marker_10 = gm.PoseStamped(header=Header(frame_id="ar_marker_9"), position=gm.Point(0,0,Z), orientation=gm.Quaternion(0,0,0,1))
markers     = ["ar_marker_9", "ar_marker_10", "ar_marker_11", "ar_marker_12", "ar_marker_13", "ar_marker_14", "ar_marker_15", "ar_marker_16", "ar_marker_17"]
top_row     = ["ar_marker_9", "ar_marker_10", "ar_marker_11"]
middle_row  = ["ar_marker_12", "ar_marker_13", "ar_marker_14"]
bottom_row  = ["ar_marker_15", "ar_marker_16", "ar_marker_17"]
marker_array = [top_row, middle_row, bottom_row]

# def neighbors(arr, coord):
#     above = (coord[0]+1,  coord[1])
#     right = (coord[0],    coord[1]+1)
#     below = (coord[0]-1,  coord[1])
#     left = (coord[0],     coord[1]-1)
#     neighs = [above, right, below, left]
#     for neighbor in neighs:
#         if 0 <= neighbor[0] < len(arr)-1 and 0 <= neighbor[1] < len(arr[0])-1:
#             yield arr[neighbor[0]][neighbor[1]]

# def gen_pairs(array):
#     for y, row in enumerate(array):
#         for x, marker in enumerate(row):
#             for neighbor in neighbors(array, (y,x)):
#                 yield tuple(sorted([marker, neighbor]))

# pairs = gen_pairs(marker_array)
# pairs_set = sorted(set(list(pairs)))
# spacings = {pair:0.0 for pair in pairs_set}
# print spacings
# 
# Z_heights = {marker:0.0 for marker in markers}

spacings = { ('ar_marker_15', 'ar_marker_16')   : 0.261,
             ('ar_marker_10', 'ar_marker_11')   : 0.266,
             ('ar_marker_10', 'ar_marker_9')    : 0.260,
             ('ar_marker_16', 'ar_marker_17')   : 0.267,
             ('ar_marker_13', 'ar_marker_16')   : 0.136,
             ('ar_marker_12', 'ar_marker_13')   : 0.263,
             ('ar_marker_14', 'ar_marker_17')   : 0.135,
             ('ar_marker_10', 'ar_marker_13')   : 0.156,
             ('ar_marker_11', 'ar_marker_14')   : 0.163,
             ('ar_marker_12', 'ar_marker_9')    : 0.154,
             ('ar_marker_13', 'ar_marker_14')   : 0.268,
             ('ar_marker_12', 'ar_marker_15')   : 0.138}

Z_heights = {   
                'ar_marker_3': 0.950,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_4': 0.945,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_5': 0.938,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_6': 0.950,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_7': 0.945,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_8': 0.938,    #[m] from ground plane, e.g. the height from /base_link

                'ar_marker_9':  1.924,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_10': 1.926,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_11': 1.924,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_12': 1.770,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_13': 1.770,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_14': 1.762,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_15': 1.632,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_16': 1.633,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_17': 1.626    #[m] from ground plane, e.g. the height from /base_link
            }

def frange(x, y, jump):
  while x < y:
    yield x
    x += jump

def calculate_angle_to_z_axis(trans, rot):
    Schuine = distance(trans)
    Aanliggende = trans[z]
    hoek_met_Z_as = acos(Aanliggende/Schuine) #sosCAStoa
    return hoek_met_Z_as

def distance(trans):
    return sqrt(trans[x]**2 + trans[y]**2 + trans[z]**2)

def calc_index_for_angle(stepsize_in_radians, angle_in_degrees_from_center, center_index=245):
    angle_in_radians = radians(angle_in_degrees_from_center)
    steps = angle_in_radians / stepsize_in_radians
    return int(center_index + steps)

class TrackerRangeAnalyzer(object):
    def __init__(self, marker_frame, camera_frame):
        self.tf = tf.TransformListener()

        self.marker_frame = marker_frame
        self.camera_frame = camera_frame

        self.rate = rospy.Rate(20)

        self.transforms = []
        self.measurements = []

        self.active = False

        datapath = "{0}_TO_{1}.csv".format(marker_frame, camera_frame).replace("/", "")
        self.csvfile = open(datapath, "a+")

        self.datastore = csv.DictWriter(self.csvfile, [ "timestamp",
                                                        "X  camera",
                                                        "Y  camera",
                                                        "Z  camera",
                                                        "qX camera",
                                                        "qY camera",
                                                        "qZ camera",
                                                        "qW camera",
                                                        "distance ",
                                                        "angle   Z",
                                                        "Xmarker_in_base",
                                                        "Ymarker_in_base",
                                                        "Zmarker_in_base"])
        self.datastore.writeheader()

    def listen(self):
        while self.active and not rospy.is_shutdown():
            try:
                (trans,rot) = self.tf.lookupTransform(self.marker_frame, self.camera_frame, rospy.Time(0))
                self.measurements += [self.analyze_measurement(trans, rot)]

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Marker lost")

            self.rate.sleep()

    def activate(self, switch_on=True):
        self.active = switch_on
        if self.active:
            self.listen()

    def analyze_measurement(self, trans, rot):
        self.transforms += [(trans,rot)]

        dist = distance(trans)
        angle = calculate_angle_to_z_axis(trans,rot)
        rospy.loginfo("Distance: {0:.3f}, angle marker's Z axis: {1:.3f}".format(dist, degrees(angle)))

        (marker_in_base,rot) = self.tf.lookupTransform("/base_link", self.marker_frame, rospy.Time(0))

        measurement = {"timestamp":datetime.datetime.now().strftime(timestamp_format),
                        "X  camera":trans[x],
                        "Y  camera":trans[y],
                        "Z  camera":trans[z],
                        "qX camera":rot[qx],
                        "qY camera":rot[qy],
                        "qZ camera":rot[qz],
                        "qW camera":rot[qw],
                        "distance ":dist,
                        "angle   Z":degrees(angle)%90,
                        "Xmarker_in_base":marker_in_base[x],
                        "Ymarker_in_base":marker_in_base[y],
                        "Zmarker_in_base":marker_in_base[z]}
        self.datastore.writerow(measurement)
        self.csvfile.flush() #Be sure to write the data to file
        return measurement

    def analyze_range(self):
        camera_points = [transform[0] for transform in self.transforms]
        
        distances = map(distance, camera_points)
        angles = map(lambda tup: calculate_angle_to_z_axis(*tup), self.transforms)
        
        max_distance = max(distances)

        index_of_max_distance = distances.index(max_distance)
        point_with_max_distance = camera_points[index_of_max_distance]
        rospy.loginfo("Max distance: {0:.3f}m, measured at point ({1:.3f}, {2:.3f}, {3:.3f})".format(max_distance, *point_with_max_distance))
        return distances, [degrees(angle)%90 for angle in angles]

class TrackerAccuracyAnalyzer(object):
    def __init__(self, expected_spacings, expected_z_heights, origin_marker):
        """Initialize a accuracy analyzer. It takes some measurements (from real life, that reflect the actual state of the world).
        @param expected_spacings The distances between neighboring marker pairs.
        @param expected_z_heights The distance from each marker to the ground. 
        @param origin_marker Which marker is to be considered the 'origin' of the marker pattern, e.g. which one is in the top-left-corder?
        """
        self.expected_spacings = expected_spacings
        self.expected_z_heights = expected_z_heights
        self.origin_marker = origin_marker
        self.marker_names = sorted(expected_z_heights.keys())

        self.tilt_publisher     =  rospy.Publisher("/neck_tilt_controller/command", Float64)
        self.tilt_subscriber    = rospy.Subscriber("/neck_tilt_controller/state",   JointState, self.store_tilt)

        self.pan_publisher     =  rospy.Publisher("/neck_pan_controller/command", Float64)
        self.pan_subscriber    = rospy.Subscriber("/neck_pan_controller/state",   JointState, self.store_pan)

        self.laser_listener     = rospy.Subscriber("/scan", LaserScan, self.analyze_laser_scan)

        self.pan = 0.0
        self.tilt = 0.0

        self.laser_to_wall = None
        self.baselink_to_wall = None

        self.tf = tf.TransformListener()

        self.rate = rospy.Rate(20)

        self.transforms = []
        self.measurements = []

        self.active = False

        datapath = "marker_grid.csv"
        self.csvfile = open(datapath, "a+")

        spacing_expectation_names = ["{0}_TO_{1}_expected".format(expected_spacing[0], expected_spacing[1]) for expected_spacing in expected_spacings]
        spacing_measurement_names = ["{0}_TO_{1}_observed".format(expected_spacing[0], expected_spacing[1]) for expected_spacing in expected_spacings]
        spacing_error_names = ["{0}_TO_{1}_error".format(expected_spacing[0], expected_spacing[1]) for expected_spacing in expected_spacings]

        height_expectation_names = ["Height_{0}_expected".format(name) for name in self.marker_names]
        height_measurement_names = ["Height_{0}_observed".format(name) for name in self.marker_names]
        height_error_names = ["Height_{0}_error".format(name) for name in self.marker_names]

        X_expectation_names = ["X_{0}_expected".format(name) for name in self.marker_names]
        X_measurement_names = ["X_{0}_observed".format(name) for name in self.marker_names]
        X_error_names = ["X_{0}_error".format(name) for name in self.marker_names]

        spacing_fieldnames = [item for sublist in zip(spacing_expectation_names, spacing_measurement_names, spacing_error_names) for item in sublist]
        height_fieldnames = [item for sublist in zip(height_expectation_names, height_measurement_names, height_error_names) for item in sublist]
        X_fieldnames = [item for sublist in zip(X_expectation_names, X_measurement_names, X_error_names) for item in sublist]

        self.datastore = csv.DictWriter(self.csvfile, ["timestamp", "head_tilt", "head_pan"]+sorted(spacing_fieldnames+height_fieldnames+X_fieldnames))
        self.datastore.writeheader()

    def store_tilt(self, jointstate_msg):
        self.tilt = jointstate_msg.current_pos
        #rospy.loginfo("tilt={0}".format(self.tilt))

    def store_pan(self, jointstate_msg):
        self.pan = jointstate_msg.current_pos
        #rospy.loginfo("pan={0}".format(self.pan))

    def analyze_laser_scan(self, msg):
        beam_index = calc_index_for_angle(msg.angle_increment, 0.0, center_index=len(msg.ranges)/2)
        self.laser_to_wall = msg.ranges[beam_index]
        self.tf.waitForTransform("/base_link", "/laser", rospy.Time(0), rospy.Duration.from_sec(4))
        (trans,rot) = self.tf.lookupTransform("/base_link", "/laser", rospy.Time(0))
        self.baselink_to_wall = trans[x]+msg.ranges[beam_index]

        # rospy.loginfo("Beam index: {0}, laser_to_wall: {1}, baselink_to_wall: {2}".format(beam_index, self.laser_to_wall, self.baselink_to_wall))

    def check_marker_height(self, name):
        rospy.logdebug("Checking height of {0}...".format(name))

        observed_z_height = None
        error = None

        try:
            self.tf.waitForTransform("/base_link", "/"+name, rospy.Time(0), rospy.Duration.from_sec(4))
            (trans,rot) = self.tf.lookupTransform("/base_link", "/"+name, rospy.Time(0))
            observed_z_height = trans[z]
            error = observed_z_height-self.expected_z_heights[name]

            rospy.logdebug("Height of {0} is {1}".format(name, observed_z_height))
        except tf.Exception, e:
            rospy.logerr("Could not get height of {0}: {1}".format(name, e))

        return {"Height_{0}_expected".format(name):self.expected_z_heights[name], 
                "Height_{0}_observed".format(name):observed_z_height, 
                "Height_{0}_error".format(name):error}

    def check_marker_X(self, name):
        rospy.logdebug("Checking X distance of {0}...".format(name))

        observed_X = None
        error = None

        try:
            self.tf.waitForTransform("/base_link", "/"+name, rospy.Time(0), rospy.Duration.from_sec(4))
            (trans,rot) = self.tf.lookupTransform("/base_link", "/"+name, rospy.Time(0))
            observed_X = trans[x]
            error = observed_X-self.baselink_to_wall

            rospy.logdebug("Height of {0} is {1}".format(name, observed_X))
        except tf.Exception, e:
            rospy.logerr("Could not get X of {0}: {1}".format(name, e))

        return {"X_{0}_expected".format(name):self.baselink_to_wall, 
                "X_{0}_observed".format(name):observed_X, 
                "X_{0}_error".format(name):error}

    def check_spacing_pair(self, pair):
        rospy.logdebug("Checking distance between pair {0}...".format(pair))
        a = pair[0]
        b = pair[1]

        dist = None
        error = None

        try:
            self.tf.waitForTransform("/"+a, "/"+b, rospy.Time(0), rospy.Duration.from_sec(TF_WAIT_TIME))
            (trans,rot) = self.tf.lookupTransform("/"+a, "/"+b, rospy.Time(0))
            dist = distance(trans)
            error = dist-self.expected_spacings[pair]

            rospy.logdebug("Distance between pair {0} is {1}".format(pair, dist))
        except tf.Exception, e:
            rospy.logerr("Could not get position of {0} wrt. to {1}: {2}".format(b, a, e))

        return {"{0}_TO_{1}_expected".format(a, b):self.expected_spacings[pair], 
                "{0}_TO_{1}_observed".format(a, b):dist, 
                "{0}_TO_{1}_error".format(a, b):error}

    def check_accuracy(self):
        measurement = {}
        for name in self.marker_names:
            measurement.update(self.check_marker_height(name)) #Add the height measurement to the current measurement-dict
            measurement.update(self.check_marker_X(name)) #Add the distance to the wall to the current measurement-dict

        for pair in self.expected_spacings.keys():
            measurement.update(self.check_spacing_pair(pair))

        measurement["timestamp"] = datetime.datetime.now().strftime(timestamp_format)
        measurement["head_tilt"] = self.tilt
        measurement["head_pan"] = self.pan

        #rospy.loginfo("Measurements: {0}".format(measurement))
        for k,v in measurement.iteritems():
            rospy.logdebug("{0}: {1}".format(k,v))

        self.measurements += [measurement]

        invalid_keys = [key for key in measurement.keys() if key not in self.datastore.fieldnames]
        if any(invalid_keys):
            rospy.logerr("These fieldnames are invalid: {0}".format(invalid_keys))
            #import ipdb; ipdb.set_trace()
        
        large_errors = {k:v for k,v in measurement.iteritems() if "error" in k and v >= 0.03}
        if any(large_errors):
            rospy.logerr("Encountered an error measurement larger than 3cm.: {0}".format(large_errors))
            #import ipdb; ipdb.set_trace()

        self.datastore.writerow(measurement)
        self.csvfile.flush() #Be sure to write the data to file
        return measurement

    def check_accuracy_and_jitter(self, frequency=10):
        rate = rospy.Rate(frequency)
        while not rospy.is_shutdown():
            try:
                self.check_accuracy()
                rate.sleep()
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
                rospy.logwarn("Problem find TF transforms: {0}".format(e))
            except KeyboardInterrupt:
                break

    def check_accuracy_at_tilt(self, tilts, pans, reset_to=(0.0, 0,0)):
        tilts = list(tilts)
        pans = list(pans)
        for tilt in tilts:
            for pan in pans:
                rospy.logwarn("Head moving to (pan, tilt) = ({0}, {1})".format(pan, tilt))
                self.tilt_publisher.publish(tilt)
                self.pan_publisher.publish(pan)
                rospy.sleep(1.0)
                self.tilt_publisher.publish(tilt)
                self.pan_publisher.publish(pan)
                rospy.sleep(2.0)
                rospy.loginfo("Head moved to (pan, tilt) = ({0}, {1})".format(self.pan, self.tilt))
                
                self.check_accuracy()
                
                rospy.sleep(1.0)

        self.pan_publisher.publish(reset_to[0])
        self.tilt_publisher.publish(reset_to[1])

    def check_neck_repeatability(self, reset_to=(0.0, 0,0)):
        tilt = 1.0
        pan = 0.8

        for index in range(10):
            rospy.logwarn("Head moving to (pan, tilt) = ({0}, {1})".format(pan, tilt))
            self.tilt_publisher.publish(tilt)
            self.pan_publisher.publish(pan)
            rospy.sleep(1.0)
            self.tilt_publisher.publish(tilt)
            self.pan_publisher.publish(pan)
            rospy.sleep(2.0)
            rospy.loginfo("Head moved to (pan, tilt) = ({0}, {1})".format(self.pan, self.tilt))
            
            rospy.sleep(1.0)

            self.pan_publisher.publish(reset_to[0])
            self.tilt_publisher.publish(reset_to[1])
            self.check_accuracy()
            
            rospy.sleep(1.0)


def parse_csv(csvpath):
    dr = csv.DictReader(open(csvpath))
    for row in dr:
        try:
            yield { "timestamp":datetime.datetime.strptime(row["timestamp"], timestamp_format),
                    "X  camera":float(row["X  camera"]),
                    "Y  camera":float(row["Y  camera"]),
                    "Z  camera":float(row["Z  camera"]),
                    "qX camera":float(row["qX camera"]),
                    "qY camera":float(row["qY camera"]),
                    "qZ camera":float(row["qZ camera"]),
                    "qW camera":float(row["qW camera"]),
                    "distance ":float(row["distance "]),
                    "angle   Z":float(row["angle   Z"]),
                    "Xmarker_in_base":float(row.get("Xmarker_in_base", 0)),
                    "Ymarker_in_base":float(row.get("Ymarker_in_base", 0)),
                    "Zmarker_in_base":float(row.get("Zmarker_in_base", 0))}
        except ValueError, e:
            header = {field:field for field in dr.fieldnames}
            if row != header:
                rospy.logerr("{0} could not be parsed: {1}".format(row, e))

def parse_csv_generic(csvpath):
    dr = csv.DictReader(open(csvpath))
    
    # import ipdb; ipdb.set_trace()
    
    for index, row in enumerate(dr):
        processed = {}
        if all([k==v for k,v in row.iteritems()]):
            continue #Skip this row, its a header as all the keys and values match
        for key, value in row.iteritems():
            processed[key] = value #Will be overwritten if can be parsed. Strings etc will be left as they are
            if value:
                try:
                    processed[key] = float(value)
                    continue
                except ValueError:
                    pass
                except TypeError:
                    pass

                try:
                    processed[key] = datetime.datetime.strptime(value, timestamp_format)
                    continue
                except TypeError, te:
                    rospy.logerr("'{0}':'{1}'' at row {3} could not be parsed: {2}".format(key, value, te, index))
                except ValueError, ve:
                    processed[key] = 0.0 #TODO: Could be due to '2.54667546e-6' stuff.
                    rospy.logerr("'{0}':'{1}'' at row {3} could not be parsed: {2}".format(key, value, ve, index))
            else:
                rospy.logdebug("'{0}':'{1}'' at row {2} is not a value".format(key, value, index))
                processed[key] = 0.0
        yield processed 

def plot_range(distances, angles, marker_frame, camera_frame):
    plt.title('{0} to {1} (n={2})'.format(camera_frame, marker_frame, len(distances)))
    plt.scatter(distances, angles)
    plt.xlabel("Distance")
    plt.ylabel("Angle with marker Z-axis")

    measurement_name = "{1}_TO_{0} (n={2})".format(marker_frame, camera_frame, len(distances)).replace("/", "")
    plt.savefig('{1}_{0}.png'.format(datetime.datetime.now().strftime("%Y_%m_%d__%H_%M_%S"), measurement_name))
    
    plt.show()

def plot_precision(measurements, marker_frame, camera_frame):
    distances       = [meas["distance "]        for meas in measurements]
    angles          = [meas["angle   Z"]        for meas in measurements]
    Xmarker_in_base = [meas["Xmarker_in_base"]  for meas in measurements]
    Ymarker_in_base = [meas["Ymarker_in_base"]  for meas in measurements]
    Zmarker_in_base = [meas["Zmarker_in_base"]  for meas in measurements]

    plt.title('How much does marker Z (in base) vary with camera angle (n={2})'.format(camera_frame, marker_frame, len(measurements)))
    plt.scatter(angles, Zmarker_in_base)
    plt.xlabel("Camera angle with marker Z-axis")
    plt.ylabel("Z marker in wrt base")

    measurement_name = "static_{1}_TO_{0}_with_varying_camera_angle(n={2})".format(marker_frame, "/base_link", len(distances)).replace("/", "")
    plt.savefig('{1}_{0}.png'.format(datetime.datetime.now().strftime("%Y_%m_%d__%H_%M_%S"), measurement_name))
    
    plt.show()

def plot_accuracy(measurements, marker_name):
    """Plot measurements made by TrackerAccuracyAnalyzer. It plots how the errors vary with the pan/tilt angle"""
    height_errors           = [meas["Height_{0}_error".format(marker_name)]     for meas in measurements]
    max_height_error        = max(height_errors)
    height_errors_for_plot  = [abs(meas)*10000 for meas in height_errors]
    
    X_errors                = [meas["X_{0}_error".format(marker_name)] for meas in measurements]
    max_X_error             = max(X_errors)
    X_errors_for_plot       = [abs(value*10000) for value in X_errors]
    
    tilts                   = [meas["head_tilt"]                    for meas in measurements]
    pans                    = [meas["head_pan"]                     for meas in measurements]

    # for tilt, error in zip(tilts, height_errors):
    #     rospy.loginfo("(tilt,error) = ({0}, {1})".format(tilt, error))
    # rospy.loginfo("There are {0} tilts, {1} pans and {2} height errors".format(len(tilts), len(pans), len(height_errors)))
    # 
    total_errors = [sqrt(a**2 + b**2) for a,b in zip(height_errors, X_errors)]
    max_total_error = max(total_errors)
    total_errors_for_plot = [value*10000 for value in total_errors]

    
    plt.subplot(211)
    plt.title('Z of marker {0}. Max error={2:.4f}m, N={1}'.format(marker_name, len(measurements), max_height_error))
    plt.scatter(pans, tilts, s=height_errors_for_plot, alpha=0.5)
    #plt.xlabel("Camera pan [rad]")
    plt.ylabel("Camera tilt [rad]")

    plt.subplot(212)
    plt.title('X of marker {0}. Max error={2:.4f}m, N={1}'.format(marker_name, len(measurements), max_X_error))
    plt.scatter(pans, tilts, s=X_errors_for_plot, alpha=0.5)
    plt.xlabel("Camera pan [rad]")
    plt.ylabel("Camera tilt [rad]")

    plt.savefig('errors_of_{0}_over_tilt_{1}.png'.format(marker_name, datetime.datetime.now().strftime("%Y_%m_%d__%H_%M_%S")))
    
    plt.show()

if __name__ =="__main__":
    rospy.init_node("tracker_range_analyzer")

    commands = ["measure", "plot_range", "plot_precision"]
    command = rospy.get_param('~command', "measure")

    # command = "measure"
    # try:
    #     command = sys.argv[1]
    #     if command == "measure":
    #         try:
    #             marker_frame = sys.argv[2]    
    #         except IndexError:
    #             rospy.logerr("Optionally, pass a frame name" 
    # except IndexError:
    #     print ""

    if command == "measure":
        marker_frame = rospy.get_param('~marker_frame', "/ar_marker_1")
        frames = (marker_frame, "/camera_rgb_frame")

        rospy.loginfo("Tracking transform between {0}, {1}. Set _marker_frame:=<frameid> to change".format(*frames))
        tra = TrackerRangeAnalyzer(*frames)
        tra.activate()

        distances, angles = tra.analyze_range()
        plot_range(distances, angles, *frames)
        plot_precision(tra.measurements, *frames)
    elif command == "plot_range":
        plotdata = rospy.get_param('~plotdata')
        
        measurements = list(parse_csv(plotdata))
        distances = [meas["distance "] for meas in measurements]
        angles = [meas["angle   Z"] for meas in measurements]

        frames = plotdata.replace(".csv", "").split("_TO_")
        plot(distances, angles, *frames)
    elif command == "plot_precision":
        plotdata = rospy.get_param('~plotdata')
        
        measurements = list(parse_csv(plotdata))

        frames = plotdata.replace(".csv", "").split("_TO_")
        if not frames or len(frames) != 2:
            frames = ("/ar_marker_1", "/camera_rgb_frame")
        plot_precision(measurements, *frames)
    elif command == "measure_accuracy":
        tracker = TrackerAccuracyAnalyzer(spacings, Z_heights, "ar_marker_9")
        print tracker.check_accuracy()
    elif command == "measure_accuracy_jitter":
        tracker = TrackerAccuracyAnalyzer(spacings, Z_heights, "ar_marker_9")
        tracker.check_accuracy_and_jitter(frequency=1)    
    elif command == "measure_accuracy_tilts":
        # import ipdb; ipdb.set_trace()
        tracker = TrackerAccuracyAnalyzer(spacings, Z_heights, "ar_marker_9")
        tracker.check_accuracy_at_tilt(tilts=frange(-0.25, 0.25, 0.2), pans=frange(-0.3, 0.3, 0.2))
    elif command == "plot_accuracy_tilts":
        plotdata = rospy.get_param('~plotdata')

        # import ipdb; ipdb.set_trace()
        measurements = list(parse_csv_generic(plotdata))
        plot_accuracy(measurements, "ar_marker_11")    
    elif command == "measure_neck_repeatability":
        # import ipdb; ipdb.set_trace()
        tracker = TrackerAccuracyAnalyzer(spacings, Z_heights, "ar_marker_9")
        tracker.check_neck_repeatability()
