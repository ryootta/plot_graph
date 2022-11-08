#!/usr/bin/env python

import csv_writer as cw 
import rospy
from nav_msgs.msg import Odometry

class Witer(cw.CsvWriter):
    def __init__(self):
        self.last_time = rospy.Time.now()
        rospy.Subscriber("/human/odom", Odometry, self.odom_callback)
        csv_name = "obstacle_odom"
        header = ['time','true_x','true_y','true_vx','true_vy']
        self.OdomWriter = cw.CsvWriter(csv_name, header)

    def odom_callback(self,data):
        t = data.header.stamp
        current_time = t.nsecs*0.000000001 + t.secs
        time = current_time - self.last_time.to_sec()
        
        csv_data = [time, \
                    data.pose.pose.position.x, data.pose.pose.position.y, \
                    data.twist.twist.linear.x, data.twist.twist.linear.y]

        print(csv_data) 
        self.OdomWriter.over_writer(csv_data)


    
if __name__ == '__main__':
    rospy.init_node('odo_witer')
    w = Witer()
    rospy.spin()
