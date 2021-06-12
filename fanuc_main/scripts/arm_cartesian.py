#! /usr/bin/env python
from __future__ import division
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import actionlib
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np

class aRmMoveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_set_pose_aRm', anonymous=True)

        self._planning_group = "arm_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        # rospy.loginfo('\033[94m' + "Planning frame: {}".format(self._planning_frame) + '\033[0m')
        # rospy.loginfo('\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        # rospy.loginfo('\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        # rospy.loginfo('\033[94m' + " >>> aRmMoveit init done." + '\033[0m')

    def go_to_pose(self, waypoints):

        plan, fraction = self._group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

        self._group.execute(plan, wait=True)


    def go_to_defined_pose(self, Plan_group, arg_pose_name):
        '''prefined pose combined with plan_group to minimise error '''
        self._planning_group = Plan_group
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._group.set_named_target(arg_pose_name)
        rospy.sleep(1)
        # plan_success, plan, planning_time, error_code = self._group.plan() 
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        rospy.sleep(1)
        self._exectute_trajectory_client.wait_for_result()

        
    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class aRmMoveit Deleted." + '\033[0m')



# def main():
#     aRm = aRmMoveit()
#     # print('SOME')

#     waypoints = []
#     x1, y1 = 72, 19
#     x2, y2 = 72, -19
#     x3, y3 = 86, -19
#     x4, y4 = 86, 19
#     resolution = 1
#     aRm_pose = geometry_msgs.msg.Pose()
    
#     for i in range(y2,y1,resolution):
#         # print(i/100)
#         for j in range(x2,x3,resolution):
#             aRm_pose.position.x = (j/100)
#             aRm_pose.position.y = (i/100)
#             aRm_pose.position.z = 0.628
#             aRm_pose.orientation.x = 0.0
#             aRm_pose.orientation.y = 0.0
#             aRm_pose.orientation.z = 0.707
#             aRm_pose.orientation.w = 0.707

#             waypoints.append(copy.deepcopy(aRm_pose))
#             # print(i,j)
#     aRm.go_to_pose(waypoints)
#     print(len(waypoints))


def main():
    aRm = aRmMoveit()
    # aRm_pose = geometry_msgs.msg.Pose()

    waypoints = []

    x1, y1 = 1.82, 0.37
    x2, y2 = 2.26, 0.37
    x3, y3 = 2.26, -0.37
    x4, y4 = 1.82, -0.37
    resolution = 1

    first_quad = geometry_msgs.msg.Pose()
    first_quad.position.x = 1.828
    first_quad.position.y = 0.37
    first_quad.position.z = 1.05
    first_quad.orientation.x = 0.002
    first_quad.orientation.y = 0.710
    first_quad.orientation.z = -0.01
    first_quad.orientation.w = 0.703
    waypoints.append(copy.deepcopy(first_quad)) # Init pose

    # first_quad.position.x -= 0.196
    # waypoints.append(copy.deepcopy(first_quad)) # step1: back

    # first_quad.position.z -= 0.1
    # waypoints.append(copy.deepcopy(first_quad)) #step2: down

    # first_quad.position.y += 0.183
    # waypoints.append(copy.deepcopy(first_quad)) # step3: (x,y): 0,0

    # first_quad.position.x += 0.121
    # waypoints.append(copy.deepcopy(first_quad)) # step4: (x,y): 0,1

    # first_quad.position.y -= 0.385
    # waypoints.append(copy.deepcopy(first_quad)) # step5: (x,y): 1,1

    # first_quad.position.x -= 0.121
    # waypoints.append(copy.deepcopy(first_quad)) # step6: (x,y): 1,0

    # aRm.go_to_pose(waypoints)

    second_quad = geometry_msgs.msg.Pose()
    second_quad.position.x = x2
    second_quad.position.y = y2
    second_quad.position.z = 1.05
    second_quad.orientation.x = 0.002
    second_quad.orientation.y = 0.710
    second_quad.orientation.z = -0.01
    second_quad.orientation.w = 0.703
    waypoints.append(copy.deepcopy(second_quad))
    
    l = int((y2-y3)*100)
    b = int((x1-x2)*100)
    stride = 3
    for i in range(1,l+1,stride):
        middle_waypoints_one = geometry_msgs.msg.Pose()
        middle_waypoints_one.position.x = x1
        middle_waypoints_one.position.y = y1 - i/100
        middle_waypoints_one.position.z = 1.05
        middle_waypoints_one.orientation.x = 0.002
        middle_waypoints_one.orientation.y = 0.710
        middle_waypoints_one.orientation.z = -0.01
        middle_waypoints_one.orientation.w = 0.703
        waypoints.append(copy.deepcopy(middle_waypoints_one))

        middle_waypoints_two = geometry_msgs.msg.Pose()
        middle_waypoints_two.position.x = x2
        middle_waypoints_two.position.y = y1 - i/100
        middle_waypoints_two.position.z = 1.05
        middle_waypoints_two.orientation.x = 0.002
        middle_waypoints_two.orientation.y = 0.710
        middle_waypoints_two.orientation.z = -0.01
        middle_waypoints_two.orientation.w = 0.703
        waypoints.append(copy.deepcopy(middle_waypoints_two))
    
    target_poses = []

    while not rospy.is_shutdown():
        # target_poses.append(copy.deepcopy(waypoints[3]))
        # target_poses.append(copy.deepcopy(waypoints[4]))
        # target_poses.append(copy.deepcopy(waypoints[5]))
        # target_poses.append(copy.deepcopy(waypoints[6]))
        # aRm.go_to_pose(target_poses)
        # target_poses = []

        print(waypoints[0:4])
        print(len(waypoints))
        aRm.go_to_pose(waypoints) 

    del aRm   

if __name__ == '__main__':
    # while not rospy.is_shutdown():
    main()
