#!/usr/bin/env python

##

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np
from go_to_module import go_to
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.srv import GetMap
import actionlib
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped
from get_blocks import get_block_pos
from geometry_msgs.msg import PoseStamped


def all_close(goal, actual, tolerance):

  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "arm" #TODO
    group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = group.get_planning_frame()

    eef_link = group.get_end_effector_link()

  
    group_names = robot.get_group_names()

    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_joint_state(self):
   
    group = self.group
    pose = group.get_current_pose(end_effector_link = "virtual_ee_link" )
    #print(pose)
   
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0
    joint_goal[6] = 0


    group.go(joint_goal, wait=True)

    group.stop()

    # close_gripper(self)
    # rospy.sleep(2)
    # open_gripper(self)

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.03)

  def go_to_pose_goal(self,pick):
   
    group = self.group
    
    x_pos= pick[0]#-0.182
    y_pos = pick[1] #0.0
    z_pos = 0.174#0.174
    
    roll_deg = 0
    pitch_deg =-180
    yaw_deg = 0

    pose_goal = go_to(x_pos, y_pos, z_pos, roll_deg, pitch_deg, yaw_deg)

    pose = group.get_current_pose(end_effector_link = "plate_link" )
    
    group.set_goal_position_tolerance(0.02)
    group.set_goal_orientation_tolerance(0.02)

    group.set_pose_target(pose_goal)
    

    plan = group.go(wait=True)
    
    
    group.stop()

    current_pose = self.group.get_current_pose().pose
    return plan


  def plan_cartesian_path(self,dist, scale=1,):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.z += scale * dist  # First move up (z)
    
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

   
    return plan, fraction


  def display_trajectory(self, plan):
 
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);


  def execute_plan(self, plan,open):
    
    group = self.group

    group.execute(plan, wait=True)
    rospy.sleep(1)
    if open:
      pass
      #open_gripper(self)
    if not open:
      pass
      #close_gripper(self)
    return 


  def pick_n_place(self,pick,place):
    picked = self.go_to_pose_goal(pick)
    if picked:
      rospy.sleep(3)
      cartesian_plan, fraction = self.plan_cartesian_path(dist=-0.08)
      self.display_trajectory(cartesian_plan)
      self.execute_plan(cartesian_plan, open=False)
      print('ready to place')
      rospy.sleep(3)
      self.go_to_pose_goal(place)
      cartesian_plan, fraction = self.plan_cartesian_path(dist=0.08)
      self.display_trajectory(cartesian_plan)
      self.execute_plan(cartesian_plan, open=True)
      return True
    else: return False
   


      

    def move_to_point(self, goal_pose):
        try:
            result = self.movebase_client(goal_pose)
        except rospy.ROSInterruptException:
            reverse()



def main():

    tutorial = MoveGroupPythonIntefaceTutorial()
 
    tutorial.go_to_joint_state()
  
    moved = tutorial.pick_n_place([0,0.18],[0,-0.18])
    
    # except rospy.ROSInterruptException:
    #     return
    # except KeyboardInterrupt:
    #     return

if __name__ == '__main__':
  main()
