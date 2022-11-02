#!/usr/bin/env python

import csv
import os
import rospy
from nav_msgs.msg import Odometry
from obstacle_grid_map.msg import Obstacles

class Witer:
    def __init__(self):
        self.last_time = rospy.Time.now()
        rospy.Subscriber("tracked_obstacles", Obstacles, self.csv_callback)
        self.file_path =  os.path.dirname(__file__) + "/../../data/"
        self.name = "obstacle_tracked_data.csv"

    def header_writer(self):
        header = ['time','x','y','vx','vy','radius']
        with open(self.file_path + self.name, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(header)

    def csv_callback(self,data):

        # nano is 10^-9
        t = data.header.stamp.secs + data.header.stamp.nsecs*(10**(-9))
        if len(data.circles) == 0:
            x = None
            y = None
            vx = None
            vy = None
            r = None
        elif len(data.circles) == 1:
            x = data.circles[0].center.x
            y = data.circles[0].center.y
            vx = data.circles[0].velocity.x
            vy = data.circles[0].velocity.y
            r = data.circles[0].true_radius
        else :
            print("too data")
            return

        with open(self.file_path + self.name, 'a') as f:
            f.write("%s,%s,%s,%s,%s,%s\n" % (str(t),str(x),str(y),str(vx),str(vy),str(r)))

        if len(data.circles) == 1:
            rospy.loginfo("(t,x,y)=(%1.5f,%1.5f,%1.5f,%1.5f)" % (t,x,y,r))

if __name__ == '__main__':
    rospy.init_node('tracked_csv_witer')
    w = Witer()
    w.header_writer()
    rospy.spin()
