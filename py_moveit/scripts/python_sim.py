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
from attach import attach 
from detach import detach


def reverse():
    print('in reverse')
    pub = rospy.Publisher('tb3_0/cmd_vel', Twist, queue_size=10)
    twist = Twist()
    i = 0
    r = rospy.Rate(10)
    while i < 30:
        twist.linear.x = -0.2
        twist.angular.z = 0
        pub.publish(twist)
        r.sleep()
        i += 1
    i = 0
    r = rospy.Rate(10)
    while i < 30:
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)
        r.sleep()
        i += 1
    return


def close_gripper(self, goal = 0.02):
        
    group = moveit_commander.MoveGroupCommander('gripper')
    joint_goal = group.get_current_joint_values()
       
    joint_goal[0] = goal
    joint_goal[1] = -goal 
    group.go(joint_goal, wait=True)
    group.stop()

def open_gripper(self, goal = 0):
        
    group = moveit_commander.MoveGroupCommander('gripper')
    joint_goal = group.get_current_joint_values()
       
    joint_goal[0] = goal
    joint_goal[1] = goal 
    group.go(joint_goal, wait=True)
    group.stop()


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

        group_name = "arm"
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
        pose = group.get_current_pose(end_effector_link="plate_link")
        # print(pose)
        
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

        close_gripper(self)
        rospy.sleep(2)
        open_gripper(self)

        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.03)

    def go_to_pose_goal(self, pick):

        group = self.group

        x_pos = pick[0]  # -0.182
        y_pos = pick[1]  # 0.0
        z_pos = 0.174  # 0.174

        roll_deg = 0
        pitch_deg = -180
        yaw_deg = 0

        pose_goal = go_to(x_pos, y_pos, z_pos, roll_deg, pitch_deg, yaw_deg)

        pose = group.get_current_pose(end_effector_link="plate_link")

        group.set_goal_position_tolerance(0.02)
        group.set_goal_orientation_tolerance(0.02)

        group.set_pose_target(pose_goal)

        plan = group.go(wait=True)

        group.stop()

        current_pose = self.group.get_current_pose().pose
        return plan

    def plan_cartesian_path(self, dist, scale=1, ):

        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group
        waypoints = []
        wpose = group.get_current_pose().pose
        wpose.position.z += scale * dist  # First move up (z)
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold
        return plan, fraction

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan, open):

        group = self.group

        group.execute(plan, wait=True)
        rospy.sleep(1)
        if open:
            open_gripper(self)
        if not open:
            close_gripper(self)
        return

    def pick_n_place(self, pick, place):

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
        else:
            return False


class MapService(object):

    def __init__(self):
        """
        Class constructor
        """
        rospy.wait_for_service('static_map')
        static_map = rospy.ServiceProxy('static_map', GetMap)
        self.map_data = static_map().map
        self.map_org = np.array([self.map_data.info.origin.position.x, self.map_data.info.origin.position.y])
        shape = self.map_data.info.height, self.map_data.info.width
        self.map_arr = np.array(self.map_data.data, dtype='float32').reshape(shape)
        self.resolution = self.map_data.info.resolution

    def show_map(self, point=None):
        plt.imshow(self.map_arr)
        if point is not None:
            plt.scatter([point[0]], [point[1]])
        plt.show()

    def position_to_map(self, pos):
        return (pos - self.map_org) // self.resolution

    def map_to_position(self, indices):
        return indices * self.resolution + self.map_org


class AskOfHelp(object):

    def __init__(self, box_pose, cyton_pose, drop_pose):
        self.goal_pose = box_pose
        self.cyton_pose = cyton_pose
        self.client = actionlib.SimpleActionClient('tb3_0/move_base', MoveBaseAction)
        self.goal = MoveBaseGoal()
        self.x = -1.5
        self.y = 0
        self.boxcounter = 0
        self.drop_pose = drop_pose

    def movebase_client(self, goal_pose):
        self.client.wait_for_server()
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        # Move to position 0.5 on the x axis of the "map" coordinate frame
        goal.target_pose.pose.position.x = goal_pose[0]
        # Move to position 0.5 on the y axis of the "map" coordinate frame
        goal.target_pose.pose.position.y = goal_pose[1]
        # No rotation of the mobile base frame w.r.t. map frame
        #goal.target_pose.pose.orientation.z = goal_pose[3]
        goal.target_pose.pose.orientation.w = np.arctan2(goal_pose[1],goal_pose[0])
        # Sends the goal to the action server.
        self.client.send_goal(goal)
        time.sleep(0.1)
        rospy.loginfo("New goal command received!")
        # Waits for the server to finish performing the action.
        wait = self.client.wait_for_result()
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            # Result of executing the action
            return self.client.get_result()
        time.sleep(0.1)

    def move_to_point(self, goal_pose):
        try:
            result = self.movebase_client(goal_pose)
        except rospy.ROSInterruptException:
            reverse()


def main():
    order = ['box_1', 'box_2', 'box_3']
    tutorial = MoveGroupPythonIntefaceTutorial()
    box_list = get_block_pos()
    tutorial.go_to_joint_state()

    for box in order:
        box_to_get = box_list[box]
        try:
            x_pos = box_to_get[0]
            y_pos = box_to_get[1]
            z_pos = box_to_get[2]
            roll_deg = 0
            pitch_deg = 90
            yaw_deg = 0
            pose = [x_pos, y_pos, z_pos, roll_deg, pitch_deg, yaw_deg]
            pick = [pose[0], pose[1]]
            place = [0.182, 0]
            moved = tutorial.pick_n_place(pick, place)
            r = rospy.Rate(1)
            r.sleep()

            if moved:
                print('placed box:', box)
            else:
                print('Help!!!!!!')
                cyton_pose = [0, 0, 0]
                drop_pose = [0, 0.17]
                pose_tb3 = box_to_get
                roll = 0
                pitch = 0
                yaw = pose_tb3[2]

                pose_tb3 = [x_pos, y_pos]
                a = AskOfHelp(pose_tb3, cyton_pose, drop_pose)
                a.move_to_point(pose_tb3)
                temp = attach(box, "link", "tb3_0", "base_footprint")
                a.move_to_point(drop_pose)
                temp = detach(box, "link", "tb3_0", "base_footprint")
                reverse()
                a.move_to_point([0, 1.5])
                pick = box_list[box]
                print(pick)
                moved = tutorial.pick_n_place(pick, place)

        except rospy.ROSInterruptException:
            return
        except KeyboardInterrupt:
            return


if __name__ == '__main__':
    main()
