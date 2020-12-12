#! /usr/bin/env python
# -*- coding: utf-8 -*-

import csv
import math
import time
import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler

def main():
    rospy.init_node("swing_object")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.3)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)

    print("Group names:")
    print(robot.get_group_names())

    print("Current state:")
    print(robot.get_current_state())

    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)

    # 何かを掴んでいた時のためにハンドを開く
    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()
    gripper.set_joint_value_target([0.7, 0.7])
    gripper.go()


    # 掴む準備をする
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.2
    target_pose.position.y = 0.0
    target_pose.position.z = 0.3
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    # ハンドを開く
    gripper.set_joint_value_target([0.7, 0.7])
    gripper.go()

    # 掴みに行く
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.2
    target_pose.position.y = 0.0
    target_pose.position.z = 0.1
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    # ハンドを閉じる
    gripper.set_joint_value_target([0.4, 0.4]) #掴むobjectによって変更する
    gripper.go()

    # 持ち上げる
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.2
    target_pose.position.y = 0.0
    target_pose.position.z = 0.3
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()							# 実行

    arm.set_named_target("vertical")
    arm.go()

    # with open('swing_object.csv') as f:   #csvファイルを読み込む
    #     reader = csv.reader(f)#, quoting=csv.QUOTE_NONNUMERIC)
    #     data = [row for row in reader]

    # data = [[["a",1,20,0.3]],[["a",2,20,0.3]],[["a",3,-60,0.9],["a",5,0,0.9]],[["a",3,1,0.3],["a",5,30,0.3]],[["a",3,-60,0.9],["a",5,-30,0.9]],[["a",3,1,0.3],["a",5,30,0.3]]]
    data = [[["a",1,20,1]],[["a",3,-60,0.4],["a",5,-40,0.4]],[["a",3,1,1],["a",5,40,1]],[["a",3,-60,0.4],["a",5,-40,0.4]],[["a",3,1,1],["a",5,40,1]],[["a",3,1,1],["a",5,40,1]],[["a",3,-60,0.4],["a",5,-40,0.4]],[["a",3,1,1],["a",5,40,1]]]

    arm_joint_values = arm.get_current_joint_values()
    for flame in range(len(data)):
        for joint_data in range(len(data[flame])):
            part=data[flame][joint_data][0]
            joint=int(data[flame][joint_data][1])
            angle = float(data[flame][joint_data][2])/180.0*math.pi
            speed =float(data[flame][joint_data][3])

            print(part, joint, angle, speed)
            if part == "a":
                # arm_joint_values = arm.get_current_joint_values()
                arm_joint_values[joint] = angle
            # elif part == "g":
            #     gripper_joint_values = gripper.get_current_joint_values()
            #     gripper_joint_values[joint] = angle
            #     gripper.set_joint_value_target(gripper_joint_values)
        print("flame")
        print(arm_joint_values)
        arm.set_joint_value_target(arm_joint_values)
        arm.set_max_velocity_scaling_factor(speed)
        arm.go()
        # gripper.go()
    print("done")

if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
