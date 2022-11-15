#!/usr/bin/env python

import csv_writer as cw 
import rospy
# data type
from obstacle_grid_map.msg import Obstacles
from gazebo_msgs.msg import ModelStates

class Writer(cw.CsvWriter):
    def __init__(self):
        rospy.Subscriber("/raw_obstacles", Obstacles, self.raw_obstacles_callback)
        rospy.Subscriber("/tracked_obstacles", Obstacles, self.tracked_obstacles_callback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.human_callback)
        csv_name = "raw_obstacles"
        header = ["times", "x", "y", "vx", "vy", "radius"]
        self.RawObstaclesWriter = cw.CsvWriter(csv_name, header)
        csv_name = "tracked_obstacles"
        self.TrackedObstaclesWriter = cw.CsvWriter(csv_name, header)
        csv_name = "true_obstacle"
        header = ["x", "y", "vx", "vy"]
        self.TrueObstacleWriter = cw.CsvWriter(csv_name, header)

    def raw_obstacles_callback(self, msg):
        # nano is 10^-9
        t = msg.header.stamp.secs + msg.header.stamp.nsecs*(10**(-9))
        data = [t]
        # -1 mean None.
        if len(msg.circles) == 0:
            data.extend([-1, -1, -1, -1, -1])
        else:
            for i in range(len(msg.circles)):
               data.extend([msg.circles[i].center.x, msg.circles[i].center.y,\
                            msg.circles[i].velocity.x, msg.circles[i].velocity.y,\
                            msg.circles[i].true_radius]) 
        self.RawObstaclesWriter.over_writer(data)

    def tracked_obstacles_callback(self, msg):
        # nano is 10^-9
        t = msg.header.stamp.secs + msg.header.stamp.nsecs*(10**(-9))
        data = [t]
        # -1 mean None.
        if len(msg.circles) == 0:
            data.extend([-1, -1, -1, -1, -1])
        else:
            for i in range(len(msg.circles)):
               data.extend([msg.circles[i].center.x, msg.circles[i].center.y,\
                            msg.circles[i].velocity.x, msg.circles[i].velocity.y,\
                            msg.circles[i].true_radius]) 
        self.TrackedObstaclesWriter.over_writer(data)

    def human_callback(self, msg):
        for i in range(len(msg.name)):
            if msg.name[i] == "human":
                human = i
        data = [msg.pose[human].position.x, msg.pose[human].position.y,\
                msg.twist[human].linear.x, msg.twist[human].linear.y]
        self.TrueObstacleWriter.over_writer(data)

if __name__ == '__main__':
    rospy.init_node('human_data_writer')
    w = Writer()
    rospy.spin()
