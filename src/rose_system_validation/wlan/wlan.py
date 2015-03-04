#!/usr/bin/env python

"""Get wireless LAN parameters"""

from sh import iwconfig, ping, traceroute  # sh creates Python fucntions around command line utilities.
import datetime
import time
import pandas as pd
import numpy as np

print iwconfig("eth7")
# print ping('8.8.8.8', c=1)
# print traceroute('8.8.8.8')

PRINT_CHANGES_OF = ["Access Point", "Link Quality"]

class IwConfig(object):
    def __init__(self, interface='wlan0'):
        self.interface = interface
        self.data = pd.DataFrame(columns=[  'Power Management', 
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
                                            'Mode'])
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
        timestamp = datetime.datetime.now()

        # print "{0} : {1}".format(timestamp, measurement)
        changes = self.format_differences(measurement, self.previous_measurement)
        if changes:
            print("{0}: {1}".format(timestamp, changes))

        self.previous_measurement = measurement

        try:
            self.data.loc[timestamp] = measurement
        except ValueError, ve:
            print "Error ({0}) on data {1}".format(ve, raw)

    def measure(self):
        while True:
            try:
                self.measure_once()
                time.sleep(0.5)
            except KeyboardInterrupt:
                break
        self.data.to_csv(path_or_buf="wlan.csv", mode="a")


if __name__ == "__main__":
    iw = IwConfig("eth7")
    iw.measure()