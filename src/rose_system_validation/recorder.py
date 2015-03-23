import roslib

import rospy
import std_msgs
import geometry_msgs
from geometry_msgs.msg import Twist, Vector3
import actionlib

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os, sys
from os.path import expanduser
from math import *
import itertools
import pprint
from termcolor import colored, cprint
from datetime import datetime
import time
from docopt import docopt

import tf
import math

def ros_time_to_datetime(ros_time):
    return datetime.fromtimestamp(ros_time.to_sec())

class Recorder(object):
    
    """Baseclass for data recorders."""
    def __init__(self, description, headers=None, autostart=False):
        """Initialize a new Recorder. By default, it does not start recording directly, only after calling start()
        @param description what data is this recorder recording.
        @param headers What columns to create in the datastore?
        @param autostart Start recording directly or not?"""
        self.dataframe = pd.DataFrame(columns=headers)
        self.dataframe.index.name = "timestamp"

        self.description = description

        self.recording = autostart

    def add_row(self, index=None, row=None):
        """Add a row to the datastore at a given index
        @param index may be a timestamp
        @param row a list of data. Must correspond with the headers passed at construction
        """
        if not index:
            index = datetime.now()
        if not row:
            {col:np.nan for col in self.dataframe.columns}

        self.dataframe.loc[index] = row

    def save(self, filepath, append=True):
        self.dataframe.to_csv(path_or_buf=filepath, mode="a" if append else "w")


class InternallyTriggeredRecorder(Recorder):
    def __init__(self, description, headers=None, autostart=False):
        Recorder.__init__(self, description, headers)
        self.autostart = autostart

    def start(self):
        self.recording = True

    def stop(self):
        self.recording = False


class ExternallyTriggeredRecorder(Recorder):
    def __init__(self, description, headers=None):
        Recorder.__init__(self, description, headers)

    def trigger(self, time):
        raise NotImplementedError("Implement this method in subclasses")


class OdomRecorder(InternallyTriggeredRecorder):
    
    """Records odometry messages for later analysis.
    An nav_msgs/Odometry message look like this:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    string child_frame_id
    geometry_msgs/PoseWithCovariance pose
      geometry_msgs/Pose pose
        geometry_msgs/Point position
          float64 x
          float64 y
          float64 z
        geometry_msgs/Quaternion orientation
          float64 x
          float64 y
          float64 z
          float64 w
      float64[36] covariance
    geometry_msgs/TwistWithCovariance twist
      geometry_msgs/Twist twist
        geometry_msgs/Vector3 linear
          float64 x
          float64 y
          float64 z
        geometry_msgs/Vector3 angular
          float64 x
          float64 y
          float64 z
      float64[36] covariance

      Only the pose.position and pose.orientation are recorded with their timestamp
    """
    def __init__(self):
        headers = [ "pose.pos.x", "pose.pos.y","pose.pos.z", 
                    "pose.ori.x", "pose.ori.y", "pose.ori.z", "pose.ori.w", 
                    "twist.lin.x", "twist.lin.y","twist.lin.z", 
                    "twist.ang.x", "twist.ang.y", "twist.ang.z"]
        InternallyTriggeredRecorder.__init__(self, description="odometry", headers=headers) #We want to save 14 datafields of each Odom-message
        
        self.odom_subscriber = rospy.Subscriber(
            "/odom", Odometry, self.process_odometry_msg)

        self.integratedTwist = geometry_msgs.msg.Twist()
        self.startPose = geometry_msgs.msg.Pose()

        self.latestPose = None

        self.integratedAngleZ = 0
        self.previousOdomStamp = rospy.Time.now()
        

        self._listeners = [] #listeners is a list of functions that are called when an odom-message is received

    def add_listener(self, listener):
        """This recorder also serves as a trigger for other functions/recorders. You can add functions to be triggered here."""
        self._listeners += [listener]

    def process_odometry_msg(self, odom):
        """Record odometry message and trigger other listeners for this event"""
        if self.recording:
            row = [ odom.pose.pose.position.x,
                    odom.pose.pose.position.y,
                    odom.pose.pose.position.z,
                    odom.pose.pose.orientation.x,
                    odom.pose.pose.orientation.y,
                    odom.pose.pose.orientation.z,
                    odom.pose.pose.orientation.w,
        
                    odom.twist.twist.linear.x,
                    odom.twist.twist.linear.y,
                    odom.twist.twist.linear.z,
                    odom.twist.twist.angular.x,
                    odom.twist.twist.angular.y,
                    odom.twist.twist.angular.z]
            self.add_row(ros_time_to_datetime(odom.header.stamp), row)

        for listener in self._listeners:
            listener(odom.header.stamp)


class TfRecorder(ExternallyTriggeredRecorder):

    """Records TF messages for later analysis"""
    def __init__(self, listener, target_frame, source_frame, timeout=rospy.Duration(1.0), print_tf_error=True):
        headers = [ "tf.pos.x", "tf.pos.y","tf.pos.z", 
                    "tf.ori.x", "tf.ori.y", "tf.ori.z", "tf.ori.w"]
        ExternallyTriggeredRecorder.__init__(self, headers=headers, description="TF-{0}-{1}".format(target_frame, source_frame).replace("/",'')) # 8 columns: time, position.{x,y,z}, orientation.{x,y,z,w}
        self.tf = listener
        self.timeout = timeout

        self.target_frame, self.source_frame = target_frame, source_frame
        self.print_tf_error = print_tf_error

    def trigger(self, time):
        try:
            self.tf.waitForTransform(self.target_frame, self.source_frame, rospy.Time(0), self.timeout)
            time = self.tf.getLatestCommonTime(self.target_frame, self.source_frame)
            position, quaternion = self.tf.lookupTransform(self.target_frame, self.source_frame, time)  # -> position, quaternion

            row = [ position[0], position[1], position[2],
                    quaternion[0], quaternion[1], quaternion[2], quaternion[3]]

            self.add_row(ros_time_to_datetime(time), row)

            return row
        except tf.Exception, e:
            if self.print_tf_error:
                rospy.logerr(e)


class LoopTrigger(object):
    def __init__(self, recorder, interval):
        self.recorder = recorder
        self.interval = interval
        self.trigger = None

    def start(self):
        self.timer = rospy.Timer(rospy.Duration(self.interval), self.recorder.trigger, oneshot=False)

    def stop(self):
        self.timer.shutdown()

    def save(self, *args, **kwargs):
        self.recorder.save(*args, **kwargs)

    description = property(lambda self: self.recorder.description)

class Combined(object):
    def __init__(self, parts, csvpath):
        self.parts = parts
        self.csvpath = csvpath
        self.headers = []
        for part in self.parts:
            self.headers += part.headers
        self.data = pd.DataFrame(columns=self.headers)
        self.previous_measurement = {col:np.nan for col in self.data.columns}
        self.timer = None

    def start(self):
        self.timer = rospy.Timer(rospy.Duration(0.5), self.trigger, oneshot=False)

    def stop(self):
        self.timer.shutdown()
        self.data.to_csv(path_or_buf=self.csvpath, mode="a")

    def trigger(self, *args, **kwargs):
        measurement = {}
        for part in self.parts:
            fields = part.trigger()
            measurement.update(fields)
        try:
            timestamp = datetime.now()
            self.data.loc[timestamp] = measurement
        except ValueError, ve:
            rospy.loginfo("Error ({0}) on data {1}".format(ve, measurement))

        self.previous_measurement = measurement 
