#!/usr/bin/env python

import csv
import os
import rospy
from gazebo_msgs.msg import ModelStates

class Witer:
    def __init__(self):
        self.last_time = rospy.Time.now()
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.csv_callback)
        self.file_path =  os.path.dirname(__file__) + "/../../data/"
        self.name = "obstacle_true_data.csv"

    def header_writer(self):
        header = ['true_x','true_y','true_vx','true_vy']
        with open(self.file_path + self.name, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(header)
        
    def csv_callback(self,data):

        for i in range(len(data.name)):
            if data.name[i] == "human":
                index = i
        x = data.pose[index].position.x
        y = data.pose[index].position.y
        vx = data.twist[index].linear.x
        vy = data.twist[index].linear.y

        with open(self.file_path + self.name, 'a') as f:
            f.write("%s,%s,%s,%s\n" % (str(x),str(y),str(vx),str(vy)))
    
        rospy.loginfo("(x,y)=(%1.5f,%1.5f)" % (x,y))

if __name__ == '__main__':
    rospy.init_node('true_csv_witer')
    w = Witer()
    w.header_writer()
    rospy.spin()
