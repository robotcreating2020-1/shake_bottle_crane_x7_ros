#! /usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import math
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Int32

flag = 0

def callback(data):
    pub = rospy.Publisher("find_red", Int32, queue_size=1)
    pub.publish(flag) 

    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def main():
    global flag
    rospy.init_node("crane_x7_show_bottle")

    #sub = rospy.Subscriber('area_size', Int32, callback)

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



    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)

    # ハンドを開く/ 閉じる
    def move_gripper(pou):
        gripper.set_joint_value_target([pou, pou])
        gripper.go()

    def pre_show_bottle(pos_y):
        #見せに行く準備
        move_arm(0.15, pos_y, 0.3)
        move_arm(0.17, pos_y, 0.25)

    def show_bottle(pos_y):
        #ボトルを見に行く
        move_max_velocity()
        move_arm(0.30, pos_y, 0.1)
        sub = rospy.Subscriber('bottle_size', Int32, callback)
        #pub.publish(n)
        rospy.sleep(5)


    # アームを移動する
    def move_arm(pos_x, pos_y, pos_z):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x
        target_pose.position.y = pos_y
        target_pose.position.z = pos_z
        q = quaternion_from_euler(-3.14/2.0, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行


    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()
 
    #ハンドを開く
    #move_gripper(1.3)

    #1つめ
    move_arm(0.15, 0.10, 0.3)
    pre_show_bottle(0.20)
    flag = 1
    show_bottle(0.20)

    arm.set_named_target("home")
    arm.go()

    #2つめ
    pre_show_bottle(0.0)
    flag = 2
    show_bottle(0.0)

    arm.set_named_target("home")
    arm.go()

    #3つめ
    move_arm(0.15, -0.10, 0.3)
    pre_show_bottle(-0.20)
    flag = 3
    show_bottle(-0.20)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()

    #verticalに戻る
    arm.set_named_target("vertical")
    arm.go()

if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
