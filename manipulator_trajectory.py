#! /usr/bin/env python

#Code taken from Robot Ignite Academy. If perfroms multiple movements in the joint task space. The arm
# will move from the home position to a position above a 40x40mm cube, the pick the cube up, return home, 
# then put the cube bacl where it got it from.

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

names1 = 'position1'
values1 = [-0.038,0.147,0.474,-0.012]
names2 = 'position2'
values2 = [-0.031,0.588,-0.228,-0.026]

names3 = 'position3'
values3 = [-0.031,0.590,-0.227,-0.476]

names4 = 'position4'
values4 = [-0.043,-0.197,0.628,-0.708]


###### Functions ########
def open_gripper():
	print ("Opening Gripper...")
	gripper_group_variable_values[0] = 00.009
	gripper_group.set_joint_value_target(gripper_group_variable_values)
	plan2 = gripper_group.go()
	gripper_group.stop()
	gripper_group.clear_pose_targets()
	rospy.sleep(1)

def close_gripper():
	print ("Closing Gripper...")
	gripper_group_variable_values[0] = -00.0006
	gripper_group.set_joint_value_target(gripper_group_variable_values)
	plan2 = gripper_group.go()
	gripper_group.stop()
	gripper_group.clear_pose_targets()
	rospy.sleep(1)

def move_home():
	arm_group.set_named_target("home")
	print ("Executing Move: Home")
	plan_success, plan1, planning_time, error_code = arm_group.plan()
	arm_group.execute(plan1, wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
	rospy.sleep(1)

# def move_zero():
# 	arm_group.set_named_target("zero")
# 	print ("Executing Move: Zero")
# 	plan_success, plan1, planning_time, error_code = arm_group.plan()
# 	arm_group.execute(plan1, wait=True)
# 	arm_group.stop()
# 	arm_group.clear_pose_targets()
# 	variable = arm_group.get_current_pose()
# 	print (variable.pose)
# 	rospy.sleep(1)

def move_position1():
	arm_group.set_named_target("position1")
	print ("Executing Move: Position1")
	plan_success, plan1, planning_time, error_code = arm_group.plan()
	arm_group.execute(plan1, wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
	rospy.sleep(1)

def move_position2():
	arm_group.set_named_target("position2")
	print ("Executing Move: Position2")
	plan_success, plan2, planning_time, error_code = arm_group.plan()
	arm_group.execute(plan2, wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
	rospy.sleep(1)

def move_position3():
	arm_group.set_named_target("position3")
	print ("Executing Move: Position3")
	plan_success, plan2, planning_time, error_code = arm_group.plan()
	arm_group.execute(plan2, wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
	rospy.sleep(1)

def move_position4():
	arm_group.set_named_target("position4")
	print ("Executing Move: Position4")
	plan_success, plan2, planning_time, error_code = arm_group.plan()
	arm_group.execute(plan2, wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
	rospy.sleep(1)

###### Setup ########
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_execute_trajectory', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_group = moveit_commander.MoveGroupCommander("arm")
gripper_group = moveit_commander.MoveGroupCommander("gripper")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

#Had probelms with planner failing, Using this planner now. I believe default is OMPL
arm_group.set_planner_id("RRTConnectkConfigDefault")
#Increased available planning time from 5 to 10 seconds
arm_group.set_planning_time(20)

arm_group.remember_joint_values(names1, values1)
arm_group.remember_joint_values(names2, values2)
arm_group.remember_joint_values(names3, values3)
arm_group.remember_joint_values(names4, values4)

gripper_group_variable_values = gripper_group.get_current_joint_values()

###### Main ########
# move_home()
# open_gripper()
# move_position3()
# move_position1()
# close_gripper()
# move_home()
# move_position2()
# open_gripper()
# move_home()

# go home, open gripper, pos1, pos2, pos3, close gripper, pos2, pos4, open gripper, home
# move_zero()
# move_home()
# open_gripper()
move_position1()
# move_position2()
# move_position3()
# # close_gripper()
# # move_position2()
# move_position4()
# open_gripper()
move_home()
# close_gripper()
moveit_commander.roscpp_shutdown()