#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Int32

def main():
    rospy.init_node("test")
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

    print("current state:")
    print(robot.get_current_state())

    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)

    def move_gripper(pou):
        gripper.set_joint_value_target([pou, pou])
	gripper.go()

    def move_arm(pos_x, pos_y, pos_z):
        target_pose = geometry_msgs.msg.Pose()
	target_pose.position.x = pos_x
	target_pose.position.y = pos_y
	target_pose.position.z = pos_z
	q = quaternion_from_euler(-3.14/2.0, 0.0, -3.14/2.0)
	target_pose.orientation.x = q[0]
	target_pose.orientation.y = q[1]
	target_pose.orientation.z = q[2]
	target_pose.orientation.w = q[3]
	arm.set_pose_target(target_pose)
	arm.go()

    def bottle_splash():
#        data = [[[1,20,1]],[[3,-60,0.4],[5,-40,0.4]],[[3,1,1],[5,40,1]],[[3,-60,0.4],[5,-40,0.4]],[[3,1,1],[5,40,1]],[[3,1,1][5,40,1]],[[3,-60,0.4],[5,-40,0.4]],[[3,1,1],[5,40,1]]]
        data = [[["a",1,20,1]],[["a",3,-60,0.4],["a",5,-40,0.4]],[["a",3,1,1],["a",5,40,1]],[["a",3,-60,0.4],["a",5,-40,0.4]],[["a",3,1,1],["a",5,40,1]],[["a",3,1,1],["a",5,40,1]],[["a",3,-60,0.4],["a",5,-40,0.4]],[["a",3,1,1],["a",5,40,1]]]
	arm_joint_values = arm.get_current_joint_values()
	for flame in range(len(data)):
	    for joint_data in range(len(data[flame])):
		part = data[flame][joint_data][0]
		joint = int(data[flame][joint_data][1])
		angle = float(data[flame][joint_data][2])/180.0*math.pi
		speed = float(data[flame][joint_data][3])
		print(joint, angle, speed)
		arm_joint_values[joint] = angle
	    print("flame")
	    print(arm_joint_values)
	    arm.set_joint_value_target(arm_joint_values)
	    arm.set_max_velocity_scaling_factor(speed)
	    arm.go()
	print("done")

    arm.set_named_target("home")
    arm.go()

    move_gripper(1.57)
    y = -0.20
    move_arm(0.22,y,0.3)
    move_arm(0.22,y,0.15)
    move_arm(0.25,y,0.15)
    move_gripper(0.25)
    arm.set_named_target("vertical")
    arm.go()

    bottle_splash()

    arm.set_named_target("vertical")
    arm.go()

if __name__ == '__main__':
    
    try:
        if not rospy.is_shutdown():
	    main()
    except rospy.ROSInterruptException:
	pass
