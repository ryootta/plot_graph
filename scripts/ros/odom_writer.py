#!/usr/bin/env python

import csv
import os
import rospy
from nav_msgs.msg import Odometry

class Witer:
    def __init__(self):
        self.last_time = rospy.Time.now()
        rospy.Subscriber("cmd_vel", Odometry, self.csv_callback)
        self.file_path =  os.path.dirname(__file__) + "/../../data/"
        self.name = "obstacle_odom_data.csv"

    def header_writer(self):
        header = ['time','true_x','true_y','true_vx','true_vy']
        with open(self.file_path + self.name, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(header)
        
    def csv_callback(self,data):
        t = data.header.stamp
        pos_p = data.pose.pose.position
        vel_p = data.twist.twist.linear
        current_time = t.nsecs*0.000000001 + t.secs
        time = current_time - self.last_time.to_sec()
        
        x = pos_p.x
        y = pos_p.y
        vx = round(vel_p.x,3)
        vy = round(vel_p.y,3)

    
        with open('/home/ownet10/csv/odom_data.csv', 'a') as f:
            f.write("%s,%s,%s,%s,%s\n" % (str(time),str(x),str(y),str(vx),str(vy)))

        rospy.loginfo("(time,x,y)=(%1.5f,%1.5f,%1.5f)" % (time,x,y))

if __name__ == '__main__':
    rospy.init_node('true_csv_witer')
    w = Witer()
    w.header_writer()
    rospy.spin()
