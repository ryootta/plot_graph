#!/usr/bin/env python

import csv
import os
import errno
import datetime

class CsvWriter():
    def __init__(self, csv_name, header):
        # csv directory path  
        dir_path =  os.path.join(os.path.dirname(__file__), "..", "data")
        # Make today file
        dt_now = datetime.datetime.now()
        new_dir_path = os.path.join(dir_path, dt_now.strftime('%Y-%m-%d'))
        try:
            os.mkdir(new_dir_path)
        except OSError as e:
            if e.errno == errno.EEXIST:
                # print('Directory not created.')
                pass
            else:
                raise
        # Add time information
        csv_name = csv_name + dt_now.strftime("-%H-%M") + ".csv"
        self._file_path = os.path.join(new_dir_path, csv_name)
        self._header = header 
        self.header_writer()

    def header_writer(self):
        #with open(self._file_path, 'w', newline='') as f:
        with open(self._file_path, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(self._header)

    def over_writer(self, data):
        #with open(self._file_path, 'a', newline='') as f:
        with open(self._file_path, 'a') as f:
            writer = csv.writer(f)
            writer.writerow(data)

def test():
    csv_name = "test"
    header = ["time", "x", "y", "vx", "vy"]
    w = CsvWriter(csv_name, header)
#    for i in range(10):
#        data = [i, i*2, i*3, i*4, i*5] 
#        w.over_writer(data)
    for i in range(10):
        data = [i, i*2, i*3, i*4, i*5, 10*i, 11*i, 12*i, 13*i] 
        w.over_writer(data)