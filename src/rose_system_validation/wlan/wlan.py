#!/usr/bin/env python

"""Get wireless LAN parameters"""
import rospy

from sh import iwconfig, ping, traceroute  # sh creates Python fucntions around command line utilities.
import datetime
import time
import pandas as pd
import numpy as np
import sys
import re

print iwconfig("eth7")
# print ping('8.8.8.8', c=1)
# print traceroute('8.8.8.8')

PRINT_CHANGES_OF = ["Access Point", "Link Quality", "Bit Rate", "Signal level"]
KEEP_CHANGES_OF = ["Access Point"]

class IwConfig(object):
    def __init__(self, interface='wlan0'):
        self.interface = interface
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

    def format_differences(self, current, previous):
        changes = {k:v for k,v in current.iteritems() if previous[k] != v}
        return "\t".join(["{0}={1}".format(k, changes[k]) for k in sorted(changes.keys()) if k in PRINT_CHANGES_OF])

    def measure_once(self):
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

        return measurement

    def measure(self):
        while True:
            try:
                self.measure_once()
                time.sleep(0.5)
            except KeyboardInterrupt:
                break
        self.data.to_csv(path_or_buf="wlan.csv", mode="a")


class Ping(object):
    def __init__(self, host):
        self.host = host
        self.headers = ['time', 'ttl']
        self.data = pd.DataFrame(columns=self.headers)
        self.previous_measurement = {col:np.nan for col in self.data.columns}

        self.pattern = r"(?P<bytes>\w+) bytes from (?P<host>([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+)): icmp_req=(?P<icmp_req>\w+) ttl=(?P<ttl>\w+) time=(?P<time>\w+)"

    def measure_once(self):
        output = ping(self.host, c=1)
        raw = output.split("\n")[1]

        measurement = {col:np.nan for col in self.data.columns}

        match = re.match(self.pattern, raw)
        if match:
            # measurement['bytes'] = match.group('bytes')
            # measurement['host'] = match.group('host')
            # measurement['icmp_req'] = match.group('icmp_req')
            measurement['ttl'] = match.group('ttl')
            measurement['time'] = match.group('time')

        return measurement

    def measure(self):
        while True:
            try:
                self.measure_once()
                time.sleep(0.5)
            except KeyboardInterrupt:
                break
        self.data.to_csv(path_or_buf="ping.csv", mode="a")

def isnan(value):
    """Check whether value == numpy.nan"""
    try:
        return np.isnan(value)
    except TypeError:  # If its not a numpy usable value, it's certainly non an np.nan
        return False

class Combined(object):
    def __init__(self, parts):
        self.parts = parts
        self.headers = []
        for part in self.parts:
            self.headers += part.headers
        self.data = pd.DataFrame(columns=self.headers)
        self.previous_measurement = {col:np.nan for col in self.data.columns}
        self.timer = None

    def start(self):
        self.timer = rospy.Timer(rospy.Duration(0.1), self.measure_once, oneshot=False)

    def stop(self):
        self.timer.shutdown()
        self.data.to_csv(path_or_buf="wlan.csv", mode="a")

    def format_differences(self, current, previous):
        changes = {k:v for k,v in current.iteritems() if previous[k] != v}
        return "\t".join(["{0}={1}".format(k, changes[k]) for k in sorted(changes.keys()) if k in PRINT_CHANGES_OF and not isnan(changes[k])])

    def measure_once(self, *args, **kwargs):
        measurement = {}
        for part in self.parts:
            fields = part.measure_once()
            measurement.update(fields)
        try:
            timestamp = datetime.datetime.now()
            self.data.loc[timestamp] = measurement
        except ValueError, ve:
            rospy.loginfo("Error ({0}) on data {1}".format(ve, measurement))

        changes = self.format_differences(measurement, self.previous_measurement)
        if changes:
            rospy.loginfo("{0}: {1}".format(timestamp, changes))

            # # If the change is not that important, we keep overwriting the output in console.
            # if not any([(keep in changes) for keep in KEEP_CHANGES_OF]):
            #     sys.stdout.write("\033[F") # Cursor up one line
            #     sys.stdout.write("\033[K") # Clear to the end of line

        self.previous_measurement = measurement

if __name__ == "__main__":
    rospy.init_node("wireless_monitor")

    combined = Combined([IwConfig("eth7"), Ping("8.8.8.8")])
    combined.start()

    rospy.spin()

    combined.stop()