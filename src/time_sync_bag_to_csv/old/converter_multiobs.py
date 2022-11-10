#!/usr/bin/env python

"""
    Author: Mingi Jeong

    CLASS converter for multi obstacle data analysis
    
    purpose: 
    - Class definition of 'Converter' 
    - properties and functions to convert .bag to .csv
"""

import csv

# import of relevant ROS libraries
import rospy
import rospkg
import message_filters
import math

# messages
from sensor_msgs.msg import Range
from ysi_exo.msg import Sonde
from visualization_msgs.msg import Marker

from obstacle_avoidance_ros_pkg.msg import ais_info, boundary_info, running_time

timed_compass_msg = Range()

OBS_TOTAL = 30
DEFAULT_AIS_TOPIC = "ais_info"
DEFAULT_BOUNDARY_TOPIC = "boundary"
DEFAULT_RUNNING_TIME_TOPIC = '/running_time'

class ConverterMultiobs:
    def __init__(self):
        """constructure"""

        self.bag_file_name = str(rospy.get_param('~bag_file'))
        self.output_file_dir_name = self.bag_file_name.replace(".bag", ".csv")

        # self._output_dir = None
        self.first_row = True
        self.header = None
        
        # self.total_robot = int(rospy.get_param('/obstacle_total'))
        self.total_robot = int(self.extract_robot_number()) # cast: str -> int

        # combined callback
        s = "def combined_callback(self"
        for i in range(self.total_robot+1):
            s += ", robot_" + str(i) + "_msg"
            s += ", robot_" + str(i) + "_boundary_msg"
        s += "): \n"
        s += "\t data_array = " 
        h = "[robot_0_msg.header.stamp.to_sec()"
        for i in range(self.total_robot+1):
            h += ", [robot_{}_msg.pose.pose.position.x,robot_{}_msg.pose.pose.position.y,robot_{}_msg.pose.pose.position.z]".format(i,i,i)
            h += ", [robot_{}_msg.pose.pose.orientation.x,robot_{}_msg.pose.pose.orientation.y,robot_{}_msg.pose.pose.orientation.z,robot_{}_msg.pose.pose.orientation.w]".format(i,i,i,i)
            h += ", [robot_{}_msg.twist.twist.linear.x,robot_{}_msg.twist.twist.linear.y,robot_{}_msg.twist.twist.linear.z]".format(i,i,i) 
            h += ", [robot_{}_msg.twist.twist.angular.x,robot_{}_msg.twist.twist.angular.y,robot_{}_msg.twist.twist.angular.z], math.degrees(robot_{}_msg.heading)".format(i,i,i,i) 
            h += ", [robot_{}_boundary_msg.pose.position.x,robot_{}_boundary_msg.pose.position.y,robot_{}_boundary_msg.pose.position.z]".format(i,i,i) 
            h += ", [robot_{}_boundary_msg.pose.orientation.x,robot_{}_boundary_msg.pose.orientation.y,robot_{}_boundary_msg.pose.orientation.z,robot_{}_boundary_msg.pose.orientation.w]".format(i,i,i,i) 
        s += h
        s += "] \n"
        s += "\t self.make_csv(data_array)"

        exec(s)
        exec("setattr(ConverterMulti, 'combined_callback', combined_callback)")

        # clean header to be added
        self.header_clean(h)


    def initializer(self):
        self.update_save_path()


    def extract_robot_number(self):
        """
        find robot number from the bag file name
        """
        count = 0
        range_interest = []

        for i, s in enumerate(self.bag_file_name):
            if s == "_" and count < 2:
                count += 1
                range_interest.append(i)

        robot_number = self.bag_file_name[range_interest[0] + 1: range_interest[1]]
        return robot_number


    def header_clean(self, header_string):
        """
        clean up header string based on strings input for exec
        """
        tmp = header_string.replace('math.degrees','')
        tmp = tmp.replace('(','')
        tmp = tmp.replace(')','')

        # tmp = tmp.replace('[','')
        # tmp = tmp.replace(']','')

        # separately save header with only strings
        # if '[' replace --> tmp[:]
        self.header = tmp[1:].split(", ") # separator by ", space" for each field


    def update_save_path(self):
        # saving path
        rospack = rospkg.RosPack()
        self._output_dir = rospack.get_path('time_sync_bag_to_csv') + "/rss_data/" + str(self.output_file_name)


    def _compass_callback(self, msg):
        """
        compass does not have timestamp, so we map timestamp with current time
        """
        timed_compass_msg.header.stamp = rospy.Time.now()
        timed_compass_msg.range = msg.data # compass heading NED frame


    def build_subscriber(self):
        sub_list = []

        # sub_list.append(message_filters.Subscriber(DEFAULT_RUNNING_TIME_TOPIC, running_time, callback = self.ais_info_callback, queue_size=1, buff_size=2**24))
        for id in range(0, self.total_robot +1): # global param
            sub_list.append(message_filters.Subscriber('/robot_{}'.format(id) + '/' + DEFAULT_AIS_TOPIC, ais_info))
            sub_list.append(message_filters.Subscriber('/robot_{}'.format(id) + '/' + DEFAULT_BOUNDARY_TOPIC, Marker))

        return sub_list


    def time_sync_callback(self):
        """
        reference http://wiki.ros.org/message_filters

        If some messages are of a type that doesn't contain the header field, 
        ApproximateTimeSynchronizer refuses by default adding such messages. 
        However, its Python version can be constructed with allow_headerless=True, which uses current ROS time in place of any missing header.stamp field:
        """
        sub_list = self.build_subscriber()
        ts = message_filters.ApproximateTimeSynchronizer(sub_list, 100, 1.5, allow_headerless=True)
        ts.registerCallback(self.combined_callback)

        while not rospy.is_shutdown():
            rospy.spin()

    
    def make_csv(self, data_array):
        """
        write csv file with data array
        """

        with open(self.output_file_dir_name, 'a') as csv_file:
            writer = csv.writer(csv_file)
            
            # header rows
            if self.first_row:
                # writer.writerow([i for i in self.header])
                writer.writerow(self.header)
                self.first_row = False

            # main data rows
            writer.writerow(data_array)
            rospy.loginfo("data saving!")

    