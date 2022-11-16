#!/usr/bin/env python

"""
    harness.py

    - Class definition of 'harness' 
    - roslaunch python script for automated running

    NOTE: currently single bag file converting
"""

# essential packages
from time import sleep
import csv
import copy

# import of relevant ROS libraries
import rospy
import roslaunch  
import rospkg
from rosgraph_msgs.msg import Clock

TEST_INITIALIZATION_WAIT_SECS = 3.0
DETECT_WAIT_SECS = 20.0
DEFAULT_BAG_STATE_TOPIC = 'clock'

class ConvertHarness:

    def __init__(self):
        rospy.sleep(1.0)

        self.evaluation_subscriber = rospy.Subscriber(DEFAULT_BAG_STATE_TOPIC, Clock, self.clock_callback, queue_size=10)

        self.logging_time = rospy.Time.now()


    def clock_initialize(self):
        self.logging_time = rospy.Time.now()


    def clock_callback(self, msg):
        self.logging_time = rospy.Time.now()



    def run_convert(self, convert_launch_file, launch_arguments):
            rospy.loginfo("--------------------------------------------------------")
            rospy.loginfo("node START")

            self.clock_initialize()

            """ api configuration """
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)


            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(convert_launch_file)[0], 
                        launch_arguments)]

            parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

            parent.start()
            rospy.on_shutdown(parent.shutdown) # killer

            # bag_file_name extract
            bag_file_name = None
            for arg in launch_arguments:
                if 'bag_file:=' in arg:
                    bag_file_name = arg[len('bag_file:='):]

            rospy.logwarn("bag_file: {}".format(bag_file_name))

            """ running conversion """
            while True:                
                maintain_duration = (rospy.Time.now() - self.logging_time).to_sec()
                if maintain_duration > DETECT_WAIT_SECS:
                    break

            parent.shutdown()

            """ finish the test """
            rospy.loginfo("Terminating ..............................")

            sleep(10.0)