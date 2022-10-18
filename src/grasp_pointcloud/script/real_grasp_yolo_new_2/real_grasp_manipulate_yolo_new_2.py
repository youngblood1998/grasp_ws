#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import moveit_commander
import moveit_msgs.msg
import tf
from geometry_msgs.msg import (
    Quaternion,
    Point
)
from grasp_pointcloud.msg import GraspParams
from trans_func import matrix_from_quaternion, rot_to_ori, tran_to_point, euler_to_matrix, tran_to_matrix, real_width_to_num, num_to_real_length, matrix_to_quaternion

END_TO_END = 0.161    # 机器人末端到夹爪末端
TRAN = [-0.007, -0.0664528072638, 0.0739784679795] #手眼标定的平移
ROT = [-0.0322859285682, -0.00222200140914, -0.0294295826053, 0.999042832512]   #手眼标定的旋转
END_JOINT = [-1.1180594603167933, -1.8702004591571253, -1.35732347169985, 4.850101470947266, -4.704089466725485, 0.1362917274236679]  # 抓取之后放置的位置
Z_DISTANCE = 0.050     # 抓取位置前一个位置的距离
ADD_WIDTH = 16
SUB_WIDTH = 10
TOLERANCE = 0.001
SCALING_FACTOR = 0.05
GRIPPER_HEIGHT = 4  # 夹爪厚


class Grasp_manipulate:
    def __init__(self):
        #设置ur5的moveit参数
        # self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander("manipulator")
        self.arm.set_goal_position_tolerance(TOLERANCE)
        self.arm.set_goal_orientation_tolerance(TOLERANCE)
        self.arm.allow_replanning(True)
        self.arm.set_pose_reference_frame("base_link")
        self.arm.set_max_velocity_scaling_factor(SCALING_FACTOR)
        self.arm.set_max_acceleration_scaling_factor(SCALING_FACTOR)
        self.arm.set_planning_time(5)
        #机械臂回到初始位置并打开夹爪
        self.init_joint = self.arm.get_current_joint_values()
        rospy.set_param("/robotiq_command",'o')
        rospy.sleep(1)
        #订阅物体位置的话题
        self.grasp_params_sub = rospy.Subscriber("real_detect/grasp_params", GraspParams, self.callback, queue_size=1, buff_size=52428800)
        # self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
        #                                        moveit_msgs.msg.DisplayTrajectory,
        #                                        queue_size=20)

    def callback(self, grasp_params):
        print(grasp_params)
        rospy.set_param("/grasp_step", 1)
        #坐标监听
        listener = tf.TransformListener()
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            try:
                # 计算相机坐标到末端坐标的变换矩阵
                ori_cam_to_end = rot_to_ori(ROT)
                point_cam_to_end = tran_to_point(TRAN)
                matrix_cam_to_end = matrix_from_quaternion(ori_cam_to_end, point_cam_to_end)
                #计算末端坐标到基坐标的变换矩阵
                (trans,rot) = listener.lookupTransform('/base_link', '/tool0', rospy.Time(0))
                ori_end_to_base = rot_to_ori(rot)
                # print(ori_end_to_base.x, ori_end_to_base.y, ori_end_to_base.z, ori_end_to_base.w)
                point_end_to_base = tran_to_point(trans)
                matrix_end_to_base = matrix_from_quaternion(ori_end_to_base, point_end_to_base)
                # 物体相对于相机位置
                point_obj_to_cam = [grasp_params.x, grasp_params.y, grasp_params.z, 1]
                # 物体相对于基坐标位置
                point_obj_to_base = np.dot(np.dot(matrix_end_to_base, matrix_cam_to_end), point_obj_to_cam)
                # 物体相对于基坐标的变换矩阵
                matrix_obj_to_base = copy.deepcopy(matrix_end_to_base)
                matrix_obj_to_base[0][3],matrix_obj_to_base[1][3],matrix_obj_to_base[2][3] = point_obj_to_base[0], point_obj_to_base[1], point_obj_to_base[2]
                # 先进行rotate角度变换
                angle_z = -grasp_params.rotate_angle if grasp_params.rotate_angle < np.pi/2 else np.pi-grasp_params.rotate_angle
                rotate_z = euler_to_matrix([0, 0, angle_z])
                matrix_obj_to_base = np.dot(matrix_obj_to_base, rotate_z)
                # 再进行tilt角度变换
                angle_y = -grasp_params.tilt_angle if grasp_params.rotate_angle < np.pi/2 else grasp_params.tilt_angle
                rotate_y = euler_to_matrix([0, angle_y, 0])
                matrix_obj_to_base = np.dot(matrix_obj_to_base, rotate_y)
                # 平移一个夹爪长度和当前位置的抓取输入数值
                grasp_num = real_width_to_num(grasp_params.grasp_width_second)
                add_length = num_to_real_length(grasp_num)/1000.0
                tran_z_2 = [[1,0,0,0],[0,1,0,0],[0,0,1,-END_TO_END-add_length],[0,0,0,1]]
                # print(-END_TO_END-add_length)
                matrix_obj_to_base_2 = np.dot(matrix_obj_to_base, tran_z_2)
                # 平移一个夹爪长度加一个安全距离
                tran_z_1 = [[1,0,0,0],[0,1,0,0],[0,0,1,-END_TO_END-add_length-Z_DISTANCE],[0,0,0,1]]
                matrix_obj_to_base_1 = np.dot(matrix_obj_to_base, tran_z_1)
                # 先转动最后一个关节
                joint = self.arm.get_current_joint_values()
                joint[5] += angle_z
                self.arm.set_joint_value_target(joint)
                rospy.set_param("/robotiq_command",str(grasp_num_1))
                self.arm.go(wait = True)
                # 开始运动1
                q_1 = matrix_to_quaternion(matrix_obj_to_base)
                # print(q_1.x, q_1.y, q_1.z, q_1.w)
                pose_1 = self.arm.get_current_pose("tool0")
                # print(pose_1.pose.position)
                # print(pose_1.pose.orientation.x, pose_1.pose.orientation.y, pose_1.pose.orientation.z, pose_1.pose.orientation.w)
                pose_1.pose.position.x = matrix_obj_to_base_1[0][3]
                pose_1.pose.position.y = matrix_obj_to_base_1[1][3]
                pose_1.pose.position.z = matrix_obj_to_base_1[2][3]
                pose_1.pose.orientation.w = q_1.w
                pose_1.pose.orientation.x = q_1.x
                pose_1.pose.orientation.y = q_1.y
                pose_1.pose.orientation.z = q_1.z
                self.arm.set_pose_target(pose_1, "tool0")
                self.arm.go(wait = True)
                # 开始运动2
                pose_2 = self.arm.get_current_pose("tool0")
                pose_2.pose.position.x = matrix_obj_to_base_2[0][3]
                pose_2.pose.position.y = matrix_obj_to_base_2[1][3]
                pose_2.pose.position.z = matrix_obj_to_base_2[2][3]
                pose_2.pose.orientation.w = q_1.w
                pose_2.pose.orientation.x = q_1.x
                pose_2.pose.orientation.y = q_1.y
                pose_2.pose.orientation.z = q_1.z
                self.arm.set_pose_target(pose_2, "tool0")
                self.arm.go(wait = True)
                rospy.set_param("/robotiq_command",'c')
                rospy.sleep(1)
                # 开始运动3
                self.arm.set_pose_target(pose_1, "tool0")
                self.arm.go(wait = True)
                # 先转动最后一个关节
                joint = self.arm.get_current_joint_values()
                joint[5] -= angle_z
                self.arm.set_joint_value_target(joint)
                self.arm.go(wait = True)
                # 到达指定位置放下
                self.arm.set_joint_value_target(END_JOINT)
                self.arm.go(wait = True)
                rospy.set_param("/robotiq_command",'o')
                rospy.sleep(1)
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()
        #抓取结束,夹爪打开,机械臂回初始点
        self.arm.set_joint_value_target(self.init_joint)
        rospy.set_param("/robotiq_command",'o')
        self.arm.go(wait = True)
        #将grasp_param设置为0,开始检测，停止抓取
        rospy.set_param("/grasp_step", 0)
        # rospy.sleep(2)
        ans_2 = raw_input("退出请按Ctrl+C,继续按任意键：").lower()


if __name__ == "__main__":
    rospy.init_node("grasp_manipulate")
    rospy.loginfo("抓取开始")
    Grasp_manipulate()
    rospy.spin()