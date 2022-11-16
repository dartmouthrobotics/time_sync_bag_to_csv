#!/usr/bin/env python

"""
    Author: Mingi Jeong

    CLASS converter
    
    purpose: 
    - Class definition of 'Converter' 
    - properties and functions to convert .bag to .csv
"""

import csv
import simplejson

# import of relevant ROS libraries
import rospy
import rospkg
import message_filters

# messages
from sensor_msgs.msg import NavSatFix, Range, Temperature
from mavros_msgs.msg import VFR_HUD
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
from ysi_exo.msg import Sonde


timed_compass_msg = Range()

class Converter:
    def __init__(self):
        """constructor"""
        self._sonde_sub = None
        self._gps_sub = None
        self._gps_vel_sub = None    
        self._vfr_hud_sub = None
        self._sonar_sub = None
        self._imu_sub = None
        self._compass_sub = None
        self._sub_flag_dict = dict()
        self._bag_file_name = None
        self._output_dir = None
        self.param_saved = False
        self.initializer()

    def initializer(self):
        sonde_use = rospy.get_param('~sonde')
        gps_use = rospy.get_param('~gps')
        vfr_hud_use = rospy.get_param('~vfr_hud')
        gps_vel_use = rospy.get_param('~gps_vel')
        imu_use = rospy.get_param('~imu')
        sonar_use = rospy.get_param('~sonar')
        compass_use =rospy.get_param('~compass')

        # sonde
        try:
            self._sonde_sub = message_filters.Subscriber(rospy.get_param('~sonde_topic'), Sonde)
            self._sub_flag_dict['sonde'] = sonde_use
        except:
            rospy.logerr("sonde subscriber initialization failed")
        
        # gps
        try:
            self._gps_sub = message_filters.Subscriber(rospy.get_param('~gps_topic'), NavSatFix)
            self._sub_flag_dict['gps'] = gps_use
        except:
            rospy.logerr("gps subscriber initialization failed")

        # gps vel
        try:
            self._gps_vel_sub = message_filters.Subscriber(rospy.get_param('~gps_vel_topic'), TwistStamped)
            self._sub_flag_dict['gps_vel'] = gps_vel_use
        except:
            rospy.logerr("gps vel subscriber initialization failed")

        # imu
        try:
            self._imu_sub = message_filters.Subscriber(rospy.get_param('~imu_topic'), Odometry)
            self._sub_flag_dict['imu'] = imu_use
        except:
            rospy.logerr("IMU subscriber initialization failed")

        # vfr_hud
        try:
            self._vfr_hud_sub = message_filters.Subscriber(rospy.get_param('~vfr_hud_topic'), VFR_HUD)
            self._sub_flag_dict['vfr_hud'] = vfr_hud_use
        except:
            rospy.logerr("vfr_hud subscriber initialization failed")

        # sonar
        try:
            self._sonar_sub = message_filters.Subscriber(rospy.get_param('~sonar_topic'), Range)
            self._sub_flag_dict['sonar'] = sonar_use
        except:
            rospy.logerr("sonar subscriber initialization failed")

        # compass
        try:
            # independent subscriber for time mapping
            self._compass_sub = rospy.Subscriber(rospy.get_param('~compass_topic'), Float64, self._compass_callback)
            self._sub_flag_dict['compass'] = compass_use
        except:
            rospy.logerr("compass subscriber initialization failed")

        # rospy.loginfo("gps_vel_topic {} and type {}".format(gps_vel_topic, type(gps_vel_topic)))

        self.update_save_path()
        rospy.loginfo("to convert topics are as follows: \n {} ".format(self._sub_flag_dict))


    def update_save_path(self):
        """
        saving path (param.yaml --> save file name)

        if rosbag_run True: directly that file name as output
        else: user-intended output file name
        """

        rosbag_run = rospy.get_param('~bag_file_run')

        if rosbag_run:
            self._bag_file_name = str(rospy.get_param('~bag_file'))
            self._output_dir = self._bag_file_name.replace(".bag", ".csv")
            self._output_param_dir = self._output_dir.replace(".csv", "_parameter.txt")

        else:
            rospack = rospkg.RosPack()
            self._output_dir = rospack.get_path('time_sync_bag_to_csv') + "/data/" + rospy.get_param('~output_file_name')
            self._output_param_dir = self._output_dir.replace(".csv", "_parameter.txt")
        
        rospy.logwarn("Converted data will be saved into ==========> {}".format(self._output_dir))


    def _compass_callback(self, msg):
        """
        compass does not have timestamp, so we map timestamp with current time
        """
        timed_compass_msg.header.stamp = rospy.Time.now()
        timed_compass_msg.range = msg.data # compass heading NED frame


    def build_subscriber(self):
        sub_list = []

        if bool(self._sub_flag_dict['sonde']):
            sub_list.append(self._sonde_sub)

        # if bool(self._sub_flag_dict['compass']):
        #     sub_list.append(self._compass_sub)

        if bool(self._sub_flag_dict['sonar']):
            sub_list.append(self._sonar_sub)

        if bool(self._sub_flag_dict['gps']):
            sub_list.append(self._gps_sub)

        if bool(self._sub_flag_dict['vfr_hud']):
            sub_list.append(self._vfr_hud_sub)

        if bool(self._sub_flag_dict['gps_vel']):
            sub_list.append(self._gps_vel_sub)

        if bool(self._sub_flag_dict['imu']):
            sub_list.append(self._imu_sub)


        return sub_list


    def time_sync_callback(self):
        """
        reference http://wiki.ros.org/message_filters

        If some messages are of a type that doesn't contain the header field, 
        ApproximateTimeSynchronizer refuses by default adding such messages. 
        However, its Python version can be constructed with allow_headerless=True, which uses current ROS time in place of any missing header.stamp field:
        """
        sub_list = self.build_subscriber()
        ts = message_filters.ApproximateTimeSynchronizer(sub_list, 100, 0.5, allow_headerless=True)
        if bool(self._sub_flag_dict['sonar']):
            ts.registerCallback(self.combined_callback)
        else:
            ts.registerCallback(self.combined_callback_no_sonar)


    # TODO 2022.11.10 change order more dynamically // message and subscriber without (no sonar separate function)

    def combined_callback(self, sonde_msg, sonar_msg, gps_msg, vfr_hud_msg, gps_velocity_msg, imu_msg):
        # reference: https://github.com/dartmouthrobotics/gds_tools.git
        # The callback processing the pairs of numbers that arrived at approximately the same time
        data_array = [
                gps_msg.header.stamp.to_sec(), gps_msg.latitude, gps_msg.longitude,
                timed_compass_msg.header.stamp.to_sec(), timed_compass_msg.range,
                vfr_hud_msg.header.stamp.to_sec(), vfr_hud_msg.groundspeed,
                sonar_msg.header.stamp.to_sec(), sonar_msg.range,
                gps_velocity_msg.header.stamp.to_sec(),
                gps_velocity_msg.twist.linear.x, gps_velocity_msg.twist.linear.y,
                imu_msg.header.stamp.to_sec(),
                imu_msg.pose.pose.position.x, imu_msg.pose.pose.position.y, imu_msg.pose.pose.position.z,
                imu_msg.pose.pose.orientation.x, imu_msg.pose.pose.orientation.y, imu_msg.pose.pose.orientation.z, imu_msg.pose.pose.orientation.w,
                imu_msg.twist.twist.linear.x, imu_msg.twist.twist.linear.y, imu_msg.twist.twist.linear.z,
                imu_msg.twist.twist.angular.x, imu_msg.twist.twist.angular.y, imu_msg.twist.twist.angular.z,
                sonde_msg.header.stamp.to_sec()
                ]
        data_array.extend(sonde_msg.data) # attach all sonde
        self.make_csv(data_array)
        self.save_param(sonde_msg)

    def combined_callback_no_sonar(self, sonde_msg, gps_msg, vfr_hud_msg, gps_velocity_msg, imu_msg):
        # reference: https://github.com/dartmouthrobotics/gds_tools.git

        # The callback processing the pairs of numbers that arrived at approximately the same time
        data_array = [
                gps_msg.header.stamp.to_sec(), gps_msg.latitude, gps_msg.longitude,
                timed_compass_msg.header.stamp.to_sec(), timed_compass_msg.range,
                vfr_hud_msg.header.stamp.to_sec(), vfr_hud_msg.groundspeed,
                gps_velocity_msg.header.stamp.to_sec(),
                gps_velocity_msg.twist.linear.x, gps_velocity_msg.twist.linear.y,
                imu_msg.header.stamp.to_sec(),
                imu_msg.pose.pose.position.x, imu_msg.pose.pose.position.y, imu_msg.pose.pose.position.z,
                imu_msg.pose.pose.orientation.x, imu_msg.pose.pose.orientation.y, imu_msg.pose.pose.orientation.z, imu_msg.pose.pose.orientation.w,
                imu_msg.twist.twist.linear.x, imu_msg.twist.twist.linear.y, imu_msg.twist.twist.linear.z,
                imu_msg.twist.twist.angular.x, imu_msg.twist.twist.angular.y, imu_msg.twist.twist.angular.z,
                sonde_msg.header.stamp.to_sec()
                ]
        data_array.extend(sonde_msg.data) # attach all sonde
        self.make_csv(data_array) # main data saving 
        self.save_param(sonde_msg) # parameter saving

    
    def make_csv(self, data_array):
        """
        function to save csv
        """
        with open(self._output_dir, 'a') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(data_array)
            rospy.loginfo("data saving!")


    def save_param(self, msg):
        """
        function to save sonde parameters
        """
        # reference: https://answers.ros.org/question/341940/custom-ros-message-with-unit8-in-python/
        # https://stackoverflow.com/questions/899103/writing-a-list-to-a-file-with-python-with-newlines
        if not self.param_saved:
            sonde_param = [ord(c) for c in msg.parameters]
            with open(self._output_param_dir, 'wb') as f:
                simplejson.dump(sonde_param, f)
                self.param_saved = True # only one time saving
    