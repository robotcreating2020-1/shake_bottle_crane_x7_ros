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



    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)

    # ハンドを開く/ 閉じる
    def move_gripper(pou):
        gripper.set_joint_value_target([pou, pou])
        gripper.go()


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
    move_gripper(1.3)
    
    
    #掴む準備
    move_arm(0.25, 0.15, 0.3)
    move_arm(0.23, 0.20, 0.25)

    #掴みに行く
    arm.set_max_velocity_scaling_factor(0.1)
    move_arm(0.25, 0.20, 0.1)

    #ハンドを閉じる
    move_gripper(0.25)
    move_max_velocity()
    
    #持ち上げる
    move_arm(0.15, 0.2, 0.2)


    #SRDFに定義されている"home"の姿勢にする                                                         
    arm.set_named_target("home")
    arm.go()
      
    #ボトルを振る動作  
    target_joint_values = arm.get_current_joint_values()
    for i in range(4):
      joint_angle1 = math.radians(-30)
      joint_angle2 = math.radians(-55)
      for j in range(0, 7, 6):
        target_joint_values[0] = joint_angle1
        target_joint_values[6] = joint_angle2
        arm.set_joint_value_target(target_joint_values)
        arm.go()
        move_max_velocity()
        print str(j) + "-> joint_value_target (degrees):",
        print math.degrees( arm.get_joint_value_target()[j] ),
        print ", current_joint_values (degrees):",
        print math.degrees( arm.get_current_joint_values()[j] )
        print("shake!!shake!!")
      joint_angle1 = math.radians(30)
      joint_angle2 = math.radians(55)
      for j in range(0, 7, 6):
        target_joint_values[0] = joint_angle1
        target_joint_values[6] = joint_angle2
        arm.set_joint_value_target(target_joint_values)
        arm.go()
        move_max_velocity()
        print str(j) + "-> joint_value_target (degrees):",
        print math.degrees( arm.get_joint_value_target()[j] ),
        print ", current_joint_values (degrees):",
        print math.degrees( arm.get_current_joint_values()[j] )
        print("shake!!shake!!")
      i+=1
      rospy.sleep(0.1)
    #ここまで 

    #homeに戻る
    arm.set_named_target("home")
    arm.go()

    #下ろす準備
    move_arm(0.15, 0.2, 0.2)

    #下ろす
    move_arm(0.455, 0.0, 0.1)

    #ハンドを開く
    move_gripper(1.57)

    #少しだけハンドを持ち上げる
    move_arm(0.455, 0.0, 0.15)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()

    #verticalに戻る（降ったことがバレないよう初期姿勢に戻る）
    arm.set_named_target("vertical")
    arm.go()

    print("done(振ってやったぜ)")


if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
