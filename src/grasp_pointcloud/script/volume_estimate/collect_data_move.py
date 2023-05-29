#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
import moveit_commander
from trans_func import matrix_from_quaternion, rot_to_ori, tran_to_point, euler_to_matrix, tran_to_matrix, real_width_to_num, num_to_real_length, matrix_to_quaternion
from std_msgs.msg import String
import copy


TRAN = [-0.0065, -0.07, 0.0739784679795] #手眼标定的平移
INIT_HEIGHT = 0.25   # 初始化时相机坐标距离平面的距离
TOLERANCE = 0.0005
SCALING_FACTOR = 0.03

# 初始化ros节点
rospy.init_node("collect_data_move")
rospy.loginfo("机器人移动程序开始")

# 等待参数服务器设置平面矩阵
while not rospy.has_param("plane_matrix"):
    rospy.sleep(1)

# 获取平面矩阵
plane_matrix = rospy.get_param("plane_matrix")
plane_matrix = np.array(plane_matrix)

# 平面矩阵转换
trans_matrix = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, -INIT_HEIGHT],
    [0, 0, 0, 1]
])
plane_matrix_transform = np.dot(plane_matrix, trans_matrix)

# 初始化moveit参数
arm = moveit_commander.MoveGroupCommander("manipulator")
arm.set_goal_position_tolerance(TOLERANCE)
arm.set_goal_orientation_tolerance(TOLERANCE)
arm.allow_replanning(True)
arm.set_pose_reference_frame("base_link")
arm.set_max_velocity_scaling_factor(SCALING_FACTOR)
arm.set_max_acceleration_scaling_factor(SCALING_FACTOR)
arm.set_planning_time(5)

# 坐标监听
listener = tf.TransformListener()
rate = rospy.Rate(30)
while not rospy.is_shutdown():
    try:
        # 计算末端坐标到基坐标的变换矩阵
        (trans,rot) = listener.lookupTransform('/base_link', '/tool0', rospy.Time(0))
        ori_end_to_base = rot_to_ori(rot)
        point_end_to_base = tran_to_point(trans)
        matrix_end_to_base = matrix_from_quaternion(ori_end_to_base, point_end_to_base)
        # 计算目标位置在基坐标的变换矩阵
        matrix_dst_to_base = np.dot(matrix_end_to_base, plane_matrix_transform)
        # 机械臂开始调整
        q = matrix_to_quaternion(matrix_dst_to_base)
        pose = arm.get_current_pose("tool0")
        pose.pose.position.x = matrix_dst_to_base[0][3]
        pose.pose.position.y = matrix_dst_to_base[1][3]
        pose.pose.position.z = matrix_dst_to_base[2][3]
        pose.pose.orientation.w = q.w
        pose.pose.orientation.x = q.x
        pose.pose.orientation.y = q.y
        pose.pose.orientation.z = q.z
        arm.set_pose_target(pose, "tool0")
        arm.go(wait = True)
        print("调整完毕")
        break
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    rate.sleep()

ans = raw_input("调整完成，退出请按Ctrl+Z,继续按任意键：").lower()

# 初始化一些参数
ANGLE_STEP = np.pi/12
pose = arm.get_current_pose("tool0")
height_step = [i*0.1 for i in range(3)]
angle_step = [j*ANGLE_STEP for j in range(7)]
pub = rospy.Publisher("collect/collectable", String, queue_size=1)

num = 0
while True:
    for i in height_step:
        # 到达指定高度的初始位置
        new_pose = copy.deepcopy(pose)
        new_pose.pose.position.z += i
        arm.set_pose_target(new_pose, "tool0")
        arm.go(wait = True)
        matrix_end_to_base_new = matrix_from_quaternion(new_pose.pose.orientation, new_pose.pose.position)
        init_joint = arm.get_current_joint_values()
        for j in angle_step:
            # 到达指定角度
            # 点在末端坐标系的位置
            pos_to_end = np.array([TRAN[1] * np.sin(j), TRAN[1] * (1-np.cos(j)), 0, 1])
            pos_to_base = np.dot(matrix_end_to_base_new, pos_to_end)
            # # 让机械臂末端运动到位置点
            # new_new_pose = copy.deepcopy(new_pose)
            # new_new_pose.pose.position.x = pos_to_base[0]
            # new_new_pose.pose.position.y = pos_to_base[1]
            # arm.set_pose_target(new_new_pose, "tool0")
            # arm.go(wait = True)
            # 让机械臂末端运动到位置点
            new_new_pose = arm.get_current_pose("tool0")
            new_new_pose.pose.position.x = pos_to_base[0]
            new_new_pose.pose.position.y = pos_to_base[1]
            arm.set_pose_target(new_new_pose, "tool0")
            arm.go(wait=True)
            # 让末端转动到相机对准物体
            joint = arm.get_current_joint_values()
            joint[5] = init_joint[5]+j
            arm.set_joint_value_target(joint)
            arm.go(wait = True)
            print(str(num)+"-"+str(i+INIT_HEIGHT)+"-"+str(j))
            pub.publish(str(num)+"-"+str(i+INIT_HEIGHT)+"-"+str(j))
            rospy.sleep(2)
        # 返回指定高度的初始位置
        arm.set_pose_target(new_pose, "tool0")
        arm.go(wait = True)
    # 返回初始位置
    arm.set_pose_target(pose, "tool0")
    arm.go(wait = True)
    num += 1
    ans = raw_input("退出请按Ctrl+Z,继续按任意键：").lower()