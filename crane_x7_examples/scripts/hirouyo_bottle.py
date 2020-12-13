#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler

def main():
	
	rospy.init_node("crane_x7_pick_and_place_controller")
	robot = moveit_commander.RobotCommander()
	arm = moveit_commander.MoveGroupCommander("arm")
	def move_max_velocity(value = 0.5):
		arm.set_max_velocity_scaling_factor(value)
	gripper = moveit_commander.MoveGroupCommander("gripper")

	while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
		rospy.sleep(1.0)
	rospy.sleep(1.0)

	print("Group names:")
	print(robot.get_group_names())

	print("Current state:")
	print(robot.get_current_state())


	# アーム初期ポーズ表示
	arm_initial_pose = arm.get_current_pose().pose
	print("Arm initial pose:")
	print(arm_initial_pose)

	# ハンド開閉
	def move_gripper(pou):
		gripper.set_joint_value_target([pou, pou])
		gripper.go()

	#アーム移動
	def move_arm(pos_x, pos_y, pos_z):
		target_pose = geometry_msgs.msg.Pose()
		target_pose.position.x = pos_x
		target_pose.position.y = pos_y
		target_pose.position.z = pos_z
		q = quantation_from_euler(-3.14/2.0, 0.0, -3.14/2.0)
		target_pose.orientation.x = q[0]
		target_pose.orientation.y = q[1]
		target_pose.orientation.z = q[2]
		target_pose.orientation.w = q[3]
		arm.set_pose_target(target_pose)
		arm.go()


	# ハンド開く
	move_gripper(1.3)

	# 掴みに行く(1回目)
	x = 0.24
	y = 0.20
	move_arm(x, y, 0.25)

	# 掴む
	move_gripper(0.25)
	move_max_velocity()

	# 持ち上げる
	move_arm(0.15, 0.2, 0.4)

	'''
	仮置き２回目以降
	move_arm(0.2, 0.0, 1.0)
	while True:
		if sen_x < 320:
			sen_x = ?とりあえず
	move_arm(sen_x, sen_y, 1.0)
	'''
	

if __name__ == '__main__':

	try:
		if not rospy.is_shutdown():
			main()
	except rospy.ROSInterruptException:
		pass
