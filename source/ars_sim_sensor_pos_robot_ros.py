#!/usr/bin/env python

import numpy as np
from numpy import *

import os




# ROS

import rospy

import rospkg

import std_msgs.msg
from std_msgs.msg import Bool
from std_msgs.msg import Header


import geometry_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped



import tf_conversions

import tf2_ros



#
import ars_lib_helpers





class ArsSimSensorPosRobotRos:

  #######

  # Covariance on measurement of position
  cov_meas_pos = None

  # Covariance on measurement of orientation
  cov_meas_att = None


  # Robot pose subscriber
  robot_pose_sub = None

  # Meas robot pose pub
  meas_robot_pose_pub = None


  # Robot Pose
  flag_robot_pose_set = False
  robot_frame_id = None
  robot_pose_timestamp = None
  robot_posi = None
  robot_atti_quat_simp = None


  # Measurement sensor loop
  # freq
  meas_sens_loop_freq = None
  # Timer
  meas_sens_loop_timer = None


  


  #########

  def __init__(self):

    # Covariance on measurement of position
    self.cov_meas_pos = {'x': 0.1, 'y': 0.1, 'z': 0.1}

    # Covariance on measurement of orientation
    self.cov_meas_att = {'z': 0.1}

    #
    self.flag_robot_pose_set = False
    self.robot_frame_id = ''
    self.robot_pose_timestamp = rospy.Time()
    self.robot_posi = np.zeros((3,), dtype=float)
    self.robot_atti_quat_simp = ars_lib_helpers.Quaternion.zerosQuatSimp()

    # Measurement sensor loop
    # freq
    self.meas_sens_loop_freq = 1.0
    # Timer
    self.meas_sens_loop_timer = None



    # end
    return


  def init(self, node_name='ars_sim_sensor_pos_robot_ros_node'):
    #

    # Init ROS
    rospy.init_node(node_name, anonymous=True)

    
    # Package path
    pkg_path = rospkg.RosPack().get_path('ars_sim_sensor_pos_robot')
    

    #### READING PARAMETERS ###
    
    # TODO

    ###


    
    # End
    return


  def open(self):


    # Subscribers

    # 
    self.robot_pose_sub = rospy.Subscriber('robot_pose', PoseStamped, self.robotPoseCallback)


    # Publishers

    # 
    self.meas_robot_pose_pub = rospy.Publisher('meas_robot_pose', PoseStamped, queue_size=1)


    # Timers
    #
    self.meas_sens_loop_timer = rospy.Timer(rospy.Duration(1.0/self.meas_sens_loop_freq), self.measSensorLoopTimerCallback)



    # End
    return


  def run(self):

    rospy.spin()

    return


  def robotPoseCallback(self, robot_pose_msg):

    #
    self.flag_robot_pose_set = True

    self.robot_frame_id = robot_pose_msg.header.frame_id

    self.robot_pose_timestamp = robot_pose_msg.header.stamp

    # Position
    self.robot_posi[0] = robot_pose_msg.pose.position.x
    self.robot_posi[1] = robot_pose_msg.pose.position.y
    self.robot_posi[2] = robot_pose_msg.pose.position.z

    # Attitude quat simp
    robot_atti_quat = ars_lib_helpers.Quaternion.zerosQuat()
    robot_atti_quat[0] = robot_pose_msg.pose.orientation.w
    robot_atti_quat[1] = robot_pose_msg.pose.orientation.x
    robot_atti_quat[2] = robot_pose_msg.pose.orientation.y
    robot_atti_quat[3] = robot_pose_msg.pose.orientation.z

    self.robot_atti_quat_simp = ars_lib_helpers.Quaternion.getSimplifiedQuatRobotAtti(robot_atti_quat)

    
    #
    return



  def measSensorLoopTimerCallback(self, timer_msg):

    # Get time
    time_stamp_current = rospy.Time.now()

    #
    if(self.flag_robot_pose_set == False):
      return

    #
    meas_robot_pose_msg = PoseStamped()

    #
    meas_robot_pose_msg.header.frame_id = self.robot_frame_id
    meas_robot_pose_msg.header.stamp = self.robot_pose_timestamp

    # Position
    # TODO
    meas_robot_pose_msg.pose.position.x = self.robot_posi[0]
    meas_robot_pose_msg.pose.position.y = self.robot_posi[1]
    meas_robot_pose_msg.pose.position.z = self.robot_posi[2]


    # Attitude
    # TODO
    meas_robot_pose_msg.pose.orientation.w = 1.0
    meas_robot_pose_msg.pose.orientation.x = 0.0
    meas_robot_pose_msg.pose.orientation.y = 0.0
    meas_robot_pose_msg.pose.orientation.z = 0.0


    #
    self.meas_robot_pose_pub.publish(meas_robot_pose_msg)

    #
    return
