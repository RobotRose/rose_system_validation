#!/usr/bin/env python

"""Get wireless LAN parameters"""
import rospy

from sh import ErrorReturnCode, rostopic # sh creates Python fucntions around command line utilities.
import datetime
import time
import pandas as pd
import numpy as np
import sys
import re
import Queue

import rose_system_validation.recorder as rec

def isnan(value):
    """Check whether value == numpy.nan"""
    try:
        return np.isnan(value)
    except TypeError:  # If its not a numpy usable value, it's certainly non an np.nan
        return False

def format_differences(current, previous, selection):
    changes = {k:v for k,v in current.iteritems() if previous[k] != v}
    return "\t".join(["{0}={1}".format(k, changes[k]) for k in sorted(changes.keys()) if k in selection and not isnan(changes[k])])

def dump_output(*args, **kwargs):
    print "dump_output received error"
    pass


class RosTopicHz(rec.InternallyTriggeredRecorder):
    def __init__(self, topic, static_window=10):
        self.topic = topic
        self.headers = ["rate", "min", "max", "stddev", "window"]
        rec.InternallyTriggeredRecorder.__init__(self, headers=self.headers, description="hz_of_{0}".format(topic).replace('/','-'))
        self.logger = None
        self.static_window = static_window

        # format "average rate: 22.217 min: 0.000s max: 0.196s std dev: 0.04703s window: 618"
        self.pattern = r"average rate: (?P<rate>[0-9]+\.[0-9]+) min: (?P<min>[0-9]+\.[0-9]+)s max: (?P<max>[0-9]+\.[0-9]+)s std dev: (?P<stddev>[0-9]+\.[0-9]+)s window: (?P<window>[0-9]+)"
        self.current_measurement = {}
        self.commands = Queue.Queue()

    def start(self):
        super(RosTopicHz, self).start()
        self.logger = rostopic.hz(self.topic, w=self.static_window, _out=self.process_output, _err=self.process_output, _bg=True, _in=self.commands)

    def stop(self):
        super(RosTopicHz, self).stop()
        self.logger = None
        self.commands.put('^C') #Send a crtl-C when stopping
        # raise NotImplementedError("sh commands are hard to kill")

    def process_output(self, line, stdin, process):
        """Output is:
        subscribed to [/odom]
    average rate: 20.129
    min: 0.003s max: 0.143s std dev: 0.04838s window: 11
    average rate: 19.556
    min: 0.000s max: 0.146s std dev: 0.04833s window: 30
    average rate: 19.535
    min: 0.000s max: 0.159s std dev: 0.04852s window: 50"""
        line = line.strip()
        
        if line.startswith("average"):
            self.current_measurement = line
        if line.startswith("min"): 
            self.current_measurement += " " + line

            measurement = {col:np.nan for col in self.dataframe.columns}
            match = re.match(self.pattern, self.current_measurement)
            if match:
                measurement['rate'] = match.group('rate')
                measurement['min'] = match.group('min')
                measurement['max'] = match.group('max')
                measurement['stddev'] = match.group('stddev')
                measurement['window'] = match.group('window')
                self.add_row(datetime.datetime.now(), measurement)


class RostopicPubsSubs(rec.ExternallyTriggeredRecorder):
    def __init__(self, topics):
        self.topics = topics
        self.headers = [topic.replace("/", "-") for topic in topics]
        rec.ExternallyTriggeredRecorder.__init__(self, headers=self.headers, description="rostopic_subscribers")

    def trigger(self, time):
        """Log how many subscribers there are for the given topics"""
        
        measurement = {t:np.nan for t in self.dataframe.columns}

        raw = rostopic.list(v=True)

        lines = raw.split("\n")
        interesting_lines = [line for line in lines if any(topic in line for topic in self.topics)]
        interesting_subscribers = [line for line in interesting_lines if "subscriber" in line]
        for line in interesting_subscribers:
            #Each line is u' * /manual_cmd_vel [geometry_msgs/Twist] 1 subscriber'
            parts = line.strip().split(" ")
            topic = parts[1].replace("/", "-")
            _type = parts[2]
            subscribercount = parts[3]

            measurement[topic] = subscribercount
        
        self.add_row(datetime.datetime.now(), measurement)
        

if __name__ == "__main__":
    rospy.init_node("rostopic_monitor")

    topics = [arg for arg in sys.argv[1:] if not arg.startswith("__")]
    topicmons = [RosTopicHz(host) for host in topics]
    pubsub = rec.LoopTrigger(RostopicPubsSubs(topics), 1.0)

    recorders = topicmons + [pubsub]
    for recorder in recorders:
        recorder.start()

    try:
        rospy.spin()
    except OSError:
        pass

    for recorder in recorders:
        print "Saving {0}".format(recorder.description+".csv")
        recorder.save(recorder.description+".csv", append=True)
        recorder.stop()
