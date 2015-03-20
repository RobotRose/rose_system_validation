#!/usr/bin/env python

"""Get wireless LAN parameters"""
import rospy

from sh import ErrorReturnCode, ifstat  # sh creates Python functions around command line utilities.
import datetime
from dateutil import parser
import time
import pandas as pd
import numpy as np
import sys
import re
import Queue

import rose_system_validation.recorder as rec

class Ifstat(rec.Recorder):
    """Records TF messages for later analysis"""
    def __init__(self, interface):
        self.interface = interface
        self.headers = ["KB/s in",  "KB/s out"]
        rec.Recorder.__init__(self, "Throughput_of_{0}".format(interface), self.headers)

        self.logger = None
        self.current_measurement = {}
        self.commands = Queue.Queue()

    def start(self):
        super(Ifstat, self).start()
        self.logger = ifstat(i=self.interface, t=True, _out=self.process_output, _err=self.process_output, _bg=True, _in=self.commands)

    def stop(self):
        super(Ifstat, self).stop()
        self.logger = None
        self.commands.put('^C') #Send a crtl-C when stopping

    def process_output(self, line, stdin, process):
        """Output is:
          Time           eth7       
HH:MM:SS   KB/s in  KB/s out
14:43:01     96.63      7.80
14:43:02     97.99      6.36
...
14:43:24     96.50      5.23
14:43:25     96.18     25.88
  Time           eth7       
HH:MM:SS   KB/s in  KB/s out
14:43:25     96.18     25.88
...
"""
        if not "Time" in line and not "HH:MM:SS" in line:
            line = line.strip()
            parts = [part.strip() for part in line.split(" ") if part]
            timestr, _in, out = parts
            timestamp = parser.parse(timestr)
            measurement = {"KB/s in":_in,  "KB/s out":out}
            print timestamp, measurement
            self.add_row(timestamp, measurement)

if __name__ == "__main__":
    rospy.init_node("ifstat_monitor")

    from wlan import IwConfig
    interface = IwConfig.find_interface()
    logger = Ifstat(interface)
    logger.start()

    try:
        rospy.spin()
    except OSError:
        pass

    logger.save(logger.description+".csv", append=True)
    logger.stop()