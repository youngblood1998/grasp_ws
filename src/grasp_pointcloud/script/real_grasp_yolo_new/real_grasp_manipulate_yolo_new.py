#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import moveit_commander
import tf
from geometry_msgs.msg import (
    Quaternion,
    Point
)
from grasp_pointcloud.msg import GraspParams
from trans_func import matrix_from_quaternion, rot_to_ori, tran_to_point, euler_to_matrix, tran_to_matrix, real_width_to_num, num_to_real_length, matrix_to_quaternion

END_TO_END = 0.236    # 机器人末端到夹爪末端
TRAN = [0.0, -0.0524528072638, 0.0739784679795] #手眼标定的平移
ROT = [-0.0322859285682, -0.00222200140914, -0.0294295826053, 0.999042832512]   #手眼标定的旋转
END_JOINT = [0.9221269488334656, -0.6543424765216272, 0.8431277275085449, -1.7985013167010706, -1.584637467061178, -0.10116988817323858]  # 抓取之后放置的位置
Z_DISTANCE = 0.050     # 抓取位置前一个位置的距离
ADD_WIDTH = 20
SUB_WIDTH = 10
TOLERANCE = 0.007
SCALING_FACTOR = 0.02

class Grasp_manipulate:
    def __init__(self):
        #设置ur5的moveit参数
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

    def callback(self, grasp_params):
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
                (trans,rot) = listener.lookupTransform('/base_link', '/wrist_3_link', rospy.Time(0))
                ori_end_to_base = rot_to_ori(rot)
                point_end_to_base = tran_to_point(trans)
                matrix_end_to_base = matrix_from_quaternion(ori_end_to_base, point_end_to_base)
                # 物体相对相机位置
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
                add_length = num_to_real_length(grasp_num)/100.0
                tran_z_2 = [[1,0,0,0],[0,1,0,0],[0,0,1,-END_TO_END-add_length],[0,0,0,1]]
                matrix_obj_to_base_2 = np.dot(matrix_obj_to_base, tran_z_2)
                grasp_num_2 = real_width_to_num(grasp_params.grasp_width_second-SUB_WIDTH)
                # 平移一个夹爪长度加一个安全距离
                tran_z_1 = [[1,0,0,0],[0,1,0,0],[0,0,1,-END_TO_END-add_length-Z_DISTANCE],[0,0,0,1]]
                matrix_obj_to_base_1 = np.dot(matrix_obj_to_base, tran_z_1)
                grasp_num_1 = real_width_to_num(grasp_params.grasp_width_second+ADD_WIDTH)
                q_1 = matrix_to_quaternion(matrix_obj_to_base)
                # 开始运动1
                pose_1 = self.arm.get_current_pose()
                pose_1.pose.position.x = matrix_obj_to_base_1[0][3]
                pose_1.pose.position.y = matrix_obj_to_base_1[1][3]
                pose_1.pose.position.z = matrix_obj_to_base_1[2][3]
                pose_1.pose.orientation.w = q_1.w
                pose_1.pose.orientation.x = q_1.x
                pose_1.pose.orientation.y = q_1.y
                pose_1.pose.orientation.z = q_1.z
                self.arm.set_pose_target(pose_1, "wrist_3_link")
                rospy.set_param("/robotiq_command",str(grasp_num_1))
                self.arm.go(wait = True)
                # 开始运动2
                pose_2 = self.arm.get_current_pose()
                pose_2.pose.position.x = matrix_obj_to_base_2[0][3]
                pose_2.pose.position.y = matrix_obj_to_base_2[1][3]
                pose_2.pose.position.z = matrix_obj_to_base_2[2][3]
                pose_1.pose.orientation.w = q_1.w
                pose_1.pose.orientation.x = q_1.x
                pose_1.pose.orientation.y = q_1.y
                pose_1.pose.orientation.z = q_1.z
                self.arm.set_pose_target(pose_2, "wrist_3_link")
                self.arm.go(wait = True)
                rospy.set_param("/robotiq_command",str(grasp_num_2))
                rospy.sleep(1)
                # # 到达指定位置放下
                self.arm.set_joint_value_target(END_JOINT)
                self.arm.go(wait = True)
                rospy.set_param("/robotiq_command",'o')
                rospy.sleep(2)
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

    # def callback(self, grasp_params):
    #     #将grasp_step设置为1,停止检测，开始抓取
    #     rospy.set_param("/grasp_step", 1)
    #     # 计算相机坐标到末端坐标的变换矩阵
    #     ori_cam_to_end = rot_to_ori(ROT)
    #     point_cam_to_end = tran_to_point(TRAN)
    #     matrix_cam_to_end = matrix_from_quaternion(ori_cam_to_end, point_cam_to_end)
    #     # 计算物体到相机的变换矩阵
    #     point_ = tran_to_point((0,0,0))
    #     matrix_obj_to_cam = matrix_from_quaternion(ori_cam_to_end, point_)
    #     matrix_obj_to_cam = np.linalg.inv(matrix_obj_to_cam)
    #     matrix_obj_to_cam[0][3],matrix_obj_to_cam[1][3],matrix_obj_to_cam[2][3] = grasp_params.x,grasp_params.y,grasp_params.z
    #     # 先进行rotate角度变换
    #     angle_z = -grasp_params.rotate_angle if grasp_params.rotate_angle < np.pi/2 else np.pi-grasp_params.rotate_angle
    #     rotate_z = euler_to_matrix([0, 0, angle_z])
    #     matrix_obj_to_cam = np.dot(matrix_obj_to_cam, rotate_z)
    #     # 再进行tilt角度变换
    #     angle_y = -grasp_params.tilt_angle if grasp_params.rotate_angle < np.pi/2 else grasp_params.tilt_angle
    #     rotate_y = euler_to_matrix([0, angle_y, 0])
    #     matrix_obj_to_cam = np.dot(matrix_obj_to_cam, rotate_y)
    #     # 平移一个夹爪长度和当前位置的抓取输入数值
    #     grasp_num = real_width_to_num(grasp_params.grasp_width_second)
    #     add_length = num_to_real_length(grasp_num)
    #     tran_z_2 = [[1,0,0,0],[0,1,0,0],[0,0,1,-END_TO_END-add_length],[0,0,0,1]]
    #     matrix_obj_to_cam_2 = np.dot(matrix_obj_to_cam, tran_z_2)
    #     grasp_num_2 = real_width_to_num(grasp_params.grasp_width_second-SUB_WIDTH)
    #     # 平移一个夹爪长度加一个安全距离
    #     tran_z_1 = [[1,0,0,0],[0,1,0,0],[0,0,1,-END_TO_END-add_length-Z_DISTANCE],[0,0,0,1]]
    #     matrix_obj_to_cam_1 = np.dot(matrix_obj_to_cam, tran_z_1)
    #     grasp_num_1 = real_width_to_num(grasp_params.grasp_width_second+ADD_WIDTH)
    #     #坐标监听
    #     listener = tf.TransformListener()
    #     rate = rospy.Rate(30)
    #     while not rospy.is_shutdown():
    #         try:
    #             #计算末端坐标到基坐标的变换矩阵
    #             (trans,rot) = listener.lookupTransform('/base_link', '/wrist_3_link', rospy.Time(0))
    #             ori_end_to_base = rot_to_ori(rot)
    #             point_end_to_base = tran_to_point(trans)
    #             matrix_end_to_base = matrix_from_quaternion(ori_end_to_base, point_end_to_base)
    #             # 计算两个位置到基坐标的变换矩阵以及抓取输入数值
    #             matrix_obj_to_base_1 = np.dot(matrix_end_to_base, np.dot(matrix_cam_to_end, matrix_obj_to_cam_1))
    #             q_1 = matrix_to_quaternion(matrix_obj_to_base_1)
    #             matrix_obj_to_base_2 = np.dot(matrix_end_to_base, np.dot(matrix_cam_to_end, matrix_obj_to_cam_2))
    #             # 开始运动1
    #             pose_1 = self.arm.get_current_pose()
    #             pose_1.pose.position.x = matrix_obj_to_base_1[0][3]
    #             pose_1.pose.position.y = matrix_obj_to_base_1[1][3]
    #             pose_1.pose.position.z = matrix_obj_to_base_1[2][3]
    #             pose_1.pose.orientation.w = q_1.w
    #             pose_1.pose.orientation.x = q_1.x
    #             pose_1.pose.orientation.y = q_1.y
    #             pose_1.pose.orientation.z = q_1.z
    #             self.arm.set_pose_target(pose_1, "wrist_3_link")
    #             rospy.set_param("/robotiq_command",str(grasp_num_1))
    #             self.arm.go(wait = True)
    #             # 开始运动2
    #             pose_2 = self.arm.get_current_pose()
    #             pose_2.pose.position.x = matrix_obj_to_base_2[0][3]
    #             pose_2.pose.position.y = matrix_obj_to_base_2[1][3]
    #             pose_2.pose.position.z = matrix_obj_to_base_2[2][3]
    #             self.arm.set_pose_target(pose_2, "wrist_3_link")
    #             self.arm.go(wait = True)
    #             rospy.set_param("/robotiq_command",str(grasp_num_2))
    #             rospy.sleep(1)
    #             # 到达指定位置放下
    #             self.arm.set_joint_value_target(END_JOINT)
    #             self.arm.go(wait = True)
    #             rospy.set_param("/robotiq_command",'o')
    #             rospy.sleep(2)
    #             break
    #         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #             continue
    #         rate.sleep()
    #     #抓取结束,夹爪打开,机械臂回初始点
    #     self.arm.set_joint_value_target(self.init_joint)
    #     rospy.set_param("/robotiq_command",'o')
    #     self.arm.go(wait = True)
    #     #将grasp_param设置为0,开始检测，停止抓取
    #     rospy.set_param("/grasp_step", 0)

if __name__ == "__main__":
    rospy.init_node("grasp_manipulate")
    rospy.loginfo("抓取开始")
    Grasp_manipulate()
    rospy.spin()