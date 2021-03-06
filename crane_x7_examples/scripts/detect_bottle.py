#! /usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import math
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Int32

#グローバル変数を初期化
finish = True
flag = 0
y = 0

#callback関数
def callback(data):
    global finish
    if finish == True:
      global y
      pub = rospy.Publisher("find_red", Int32, queue_size=1)
      pub.publish(flag) 
      if flag == 1: #yにボトルのy座標を格納
        y = 0.30
      if flag == 2:
        y = 0.0
      if flag == 3:
        y = -0.30

    finish = False

    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def main():
    global flag
    #global finish
    rospy.init_node("crane_x7_show_and_drop_bottle")

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
        move_arm(0.13, pos_y, 0.25)
        move_arm(0.15, pos_y, 0.20)

    def show_bottle(pos_y):
        #ボトルを見に行く
        move_max_velocity()
        move_arm(0.15, pos_y, 0.20)
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

    def move_arm2(pos_x,pos_y,pos_z):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x
        target_pose.position.y = pos_y
        target_pose.position.z = pos_z
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0) #(x軸、y軸、z軸)にそれぞれ回転する
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose) #目標ポーズの設定
        arm.go() #実行
    
    #gripperの角度をつける関数(ボトルに角度をつける部分)
    def radian_arm(pos_x,pos_y,pos_z):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x
        target_pose.position.y = pos_y
        target_pose.position.z = pos_z
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0) #(x軸、y軸、z軸)にそれぞれ回転する
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose) #目標ポーズの設定
        arm.go() #実行

    #ボトル(2回目以降)を落とす関数を定義
    def Drop_bottle2(x, y, z1, z2):
        move_arm2(x, y, 0.15)
        move_arm2(x, y, z1)
        move_gripper(0.23)
        move_arm2(x, y, z2)
        target_joint_values = arm.get_current_joint_values()
        arm.set_joint_value_target(target_joint_values)
        arm.go()

    #ボトルを振る動作
    def bottle_splash(pos_y):
        #move_gripper(1.57)
        #move_arm(0.17,pos_y,0.3)
        #move_arm(0.20,pos_y,0.15)
        #move_arm(0.23,pos_y,0.13)
        #move_arm(0.25,pos_y,0.13)
        #move_gripper(0.25)
        arm.set_named_target("vertical")
        arm.go()
        data = [[["s",3,-60,0.4],["s",5,-40,0.4]],[["s",3,1,1],["s",5,40,1]],[["s",3,-60,0.4],["s",5,-40,0.4]],[["s",3,1,1],["s",5,40,1]],[["s",3,1,1],["s",5,40,1]],[["s",3,-60,0.4],["s",5,-40,0.4]],[["s",3,1,1],["s",5,40,1]]]
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
        arm.set_named_target("vertical")
        arm.go()

    arm.go()

    #ハンドを開く
    move_gripper(1.3)


    #1つめのボトルを見る
    flag = 1
    move_arm(0.15, 0.20, 0.3)
    pre_show_bottle(0.30)
    show_bottle(0.30)

    arm.set_named_target("home")
    arm.go()

    #2つめのボトルを見る
    flag = 2
    pre_show_bottle(0.0)
    show_bottle(0.0)

    arm.set_named_target("home")
    arm.go()

    #3つめのボトルを見る
    flag = 3
    move_arm(0.15, -0.20, 0.3)
    pre_show_bottle(-0.30)
    show_bottle(-0.30)

    #homeに戻る
    arm.set_named_target("home")
    arm.go()

    #verticalに戻る
    arm.set_named_target("vertical")
    arm.go()

    #yに格納できてるかの確認用
    print('{0}にボトルがあるね！'.format(y))
    rospy.sleep(2)

    arm.set_named_target("home")
    arm.go()

    move_gripper(1.3)

    #ボトルを掴んで落とすのはここから
    move_arm(0.34, y-0.1, 0.2)
    move_arm(0.25, y, 0.15)
    move_arm(0.25, y, 0.13)



    #ボトルを掴む
    move_arm(0.34, y, 0.1)
    move_gripper(0.25)

    arm.set_named_target("home")
    arm.go()
    #佐藤担当　ここまで

    #實川担当　ボトルを振る
    bottle_splash(y)
    #實川担当 ここまで  
    
    #金子担当 ボトルを落とす
    radian_arm(0.2, 0, 0.20)

    move_gripper(1.57)
    for i in range(4):
      Drop_bottle2(0.2, 0, 0.11, 0.30)
      radian_arm(0.2, 0, 0.25)
      move_gripper(1.57)
      i += 1
    #ボトルを振るのはここまで
    
    #ボトルを元の場所に戻す(實川)
    move_arm2(0.20, 0, 0.10)
    move_gripper(0.23)
    move_arm2(0.20, 0, 0.20)
    arm.set_named_target("home")
    arm.go()
    move_arm(0.35, y, 0.11)
    move_gripper(1.57)


    arm.set_named_target("home")
    arm.go()
    

    arm.set_named_target("vertical")
    arm.go()


if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass

