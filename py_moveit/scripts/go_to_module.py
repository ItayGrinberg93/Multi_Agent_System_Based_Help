import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np

def get_quaternion_from_euler(roll, pitch, yaw):
 
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

  return [qx, qy, qz, qw]


  

def go_to(x_pos,y_pos,z_pos,roll, pitch, yaw):
  
  pose_goal = geometry_msgs.msg.Pose()
  deg_to_rad = 0.0174533

  roll_rad = roll*deg_to_rad
  pitch_rad = pitch*deg_to_rad
  yaw_rad = yaw*deg_to_rad

  quaterions = get_quaternion_from_euler(pitch_rad,yaw_rad,roll_rad)
  print(quaterions)

  
    
  pose_goal.orientation.x = quaterions[0]
  pose_goal.orientation.y = quaterions[1]
  pose_goal.orientation.z = quaterions[2]
  pose_goal.orientation.w = quaterions[3]
  pose_goal.position.x = x_pos
  pose_goal.position.y = y_pos
  pose_goal.position.z = z_pos
  

  return pose_goal