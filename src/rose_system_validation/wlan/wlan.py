#!/usr/bin/env python

"""Get wireless LAN parameters"""
import rospy

from sh import ErrorReturnCode, iwconfig, ping, traceroute, ifconfig, sudo  # sh creates Python fucntions around command line utilities.
import datetime
import time
import pandas as pd
import numpy as np
import sys
import re
import tf

import rose_system_validation.recorder as rec

# iwconfig()
# print ping('8.8.8.8', c=1)
# print traceroute('8.8.8.8')

PRINT_CHANGES_OF = ["Access Point", "Link Quality", "Bit Rate", "Signal level", "8.8.8.8_time", "10.8.0.1_time"]
KEEP_CHANGES_OF = ["Access Point"]

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

class IwConfig(object):
    def __init__(self, interface=None):
        self.interface = interface
        if not self.interface:
            self.interface = IwConfig.find_interface()
        rospy.loginfo("Logging wireless interface {0}".format(self.interface))

        self.headers = ['Power Management', 
                        'Fragment thr', 
                        'Access Point', 
                        'Link Quality', 
                        'Encryption key', 
                        'Signal level', 
                        'Tx excessive retries', 
                        'RTS thr', 
                        'Rx invalid frag', 
                        'Frequency', 
                        'Tx-Power', 
                        'ESSID', 
                        'long limit', 
                        'Missed beacon', 
                        'Bit Rate', 
                        'Rx invalid crypt', 
                        'Rx invalid nwid', 
                        'Invalid misc', 
                        'Mode']
        self.data = pd.DataFrame(columns=self.headers)
        self.previous_measurement = {col:np.nan for col in self.data.columns}

    @staticmethod
    def find_interface():
        """Raw output of iwconfig is:
        $ iwconfig 
        eth6      no wireless extensions.

        eth7      IEEE 802.11abg  ESSID:"ROSE_WIFI"  
                  Mode:Managed  Frequency:5.22 GHz  Access Point: 10:C3:7B:DE:58:5C   
                  Retry  long limit:7   RTS thr:off   Fragment thr:off
                  Power Management:off
                  
        lo        no wireless extensions.

        tap0      no wireless extensions.

        So, we look for a line iwth ESSID: in it, then take the first word in that line
        """
        all_output = iwconfig()
        for line in all_output.split('\n'):
            if "ESSID:" in line:
                parts = line.split(" ")
                interface_name = parts[0]
                return interface_name

    def trigger(self):
        """Raw format is something like
        
        eth7      IEEE 802.11abg  ESSID:"ROSE_WIFI"  
                  Mode:Managed  Frequency:5.22 GHz  Access Point: 10:C3:7B:DE:58:5C   
                  Retry  long limit:7   RTS thr:off   Fragment thr:off
                  Power Management:off
        
        Or, when run with sudo:
        
        eth7      IEEE 802.11abg  ESSID:"ROSE_WIFI"  
                  Mode:Managed  Frequency:5.22 GHz  Access Point: 10:C3:7B:DE:58:5C   
                  Bit Rate=270 Mb/s   Tx-Power=200 dBm   
                  Retry  long limit:7   RTS thr:off   Fragment thr:off
                  Encryption key:off
                  Power Management:off
                  Link Quality=70/70  Signal level=-39 dBm  
                  Rx invalid nwid:0  Rx invalid crypt:0  Rx invalid frag:0
                  Tx excessive retries:0  Invalid misc:0   Missed beacon:0
        
        
        The fields are explained at http://linuxcommand.org/man_pages/iwconfig8.html"""
        
        raw = iwconfig(self.interface)

        parts = raw.split("  ")

        def parse_raw(parts):
            parts = raw.split("  ")  # Note: 2 spaces
            for part in parts:
                part = part.replace("=", ":").strip()
                split = part.split(':', 1)
                if len(split) >= 2:
                    yield split[0], split[1].replace('"', '')
        
        measurement = {col:np.nan for col in self.data.columns}
        measurement.update(dict(parse_raw(parts)))
        measurement_selected = {k:v for k, v in measurement.iteritems() if k in self.data.columns}
        return measurement_selected

    def measure(self):
        while True:
            try:
                self.trigger()
                time.sleep(0.5)
            except KeyboardInterrupt:
                break
        self.data.to_csv(path_or_buf="wlan.csv", mode="a")


class Ping(object):
    def __init__(self, host):
        self.host = host
        headers = ['time', 'ttl']
        self.headers = [self.host+"_"+col for col in headers]
        self.data = pd.DataFrame(columns=self.headers)
        self.previous_measurement = {col:np.nan for col in self.data.columns}

        self.pattern = r"(?P<bytes>\w+) bytes from (?P<host>([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+)): icmp_req=(?P<icmp_req>\w+) ttl=(?P<ttl>\w+) time=(?P<time>\w+)"

    def trigger(self):
        measurement = {col:np.nan for col in self.data.columns}
        
        try:
            output = ping(self.host, c=1)
            if "Destination Host Unreachable" in output:
                # measurement['bytes'] = np.nan
                # measurement['host'] = np.nan
                # measurement['icmp_req'] = np.nan
                measurement[self.host+"_"+'ttl'] = np.nan
                measurement[self.host+"_"+'time'] = np.nan
                return measurement

            raw = output.split("\n")[1]

            match = re.match(self.pattern, raw)
            if match:
                # measurement['bytes'] = match.group('bytes')
                # measurement['host'] = match.group('host')
                # measurement['icmp_req'] = match.group('icmp_req')
                measurement[self.host+"_"+'ttl'] = match.group('ttl')
                measurement[self.host+"_"+'time'] = match.group('time')
        except ErrorReturnCode, erc:
            pass #rospy.logerr(erc)

        return measurement

    def measure(self):
        while True:
            try:
                self.trigger()
                time.sleep(0.5)
            except KeyboardInterrupt:
                break
        self.data.to_csv(path_or_buf="ping.csv", mode="a")


class ExternallyTriggeredTfRecorder(rec.TfRecorder):

    """Records TF messages for later analysis"""
    def __init__(self, listener, target_frame, source_frame, timeout=rospy.Duration(1.0), print_tf_error=False):
        self.headers = [ "tf.pos.x", "tf.pos.y","tf.pos.z", 
                    "tf.ori.x", "tf.ori.y", "tf.ori.z", "tf.ori.w"]
        rec.TfRecorder.__init__(self, listener, target_frame, source_frame, timeout, print_tf_error)

    def trigger(self, *args, **kwargs):
        self.recording = True
        data =  super(ExternallyTriggeredTfRecorder, self).trigger(rospy.Time.now())
        measurements = {k:np.nan for k in self.headers}
        if not data:
            return measurements  # All measurements are np.nan be default, so if there is no data, just return this
        try:
            measurements = dict(pair for pair in zip(self.dataframe.columns, data))
        except Exception, e:
            rospy.logerr(e)
            pass #TODO: Fix exception here
        return measurements


if __name__ == "__main__":
    rospy.init_node("wireless_monitor")

    print iwconfig()

    tflistener = tf.TransformListener()
    hosts = ["8.8.8.8", "10.8.0.1", "10.8.0.6"]
    pings = [Ping(host) for host in hosts]

    combined = rec.Combined([IwConfig(), ExternallyTriggeredTfRecorder(tflistener, "/map", "/base_link")] + pings, "wlan.csv")
    combined.start()

    rospy.spin()

    combined.stop()
