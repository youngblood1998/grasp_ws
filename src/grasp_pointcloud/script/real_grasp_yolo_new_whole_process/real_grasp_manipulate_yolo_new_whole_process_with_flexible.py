#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import copy
import numpy as np
import configparser
import json
import moveit_commander
import moveit_msgs.msg
import tf
from geometry_msgs.msg import (
    Quaternion,
    Point
)
from grasp_pointcloud.msg import GraspParams
from grasp_pointcloud.msg import VolumeParams
from trans_func import matrix_from_quaternion, rot_to_ori, tran_to_point, euler_to_matrix, tran_to_matrix, real_width_to_num, num_to_real_length, matrix_to_quaternion

END_TO_END = 0.150+0.072    # 机器人末端到夹爪末端
TRAN = [-0.0065, -0.0874528072638, 0.0739784679795] #手眼标定的平移
ROT = [-0.0322859285682, -0.00222200140914, -0.0294295826053, 0.999042832512]   #手眼标定的旋转
Z_DISTANCE = 0.050     # 抓取位置前一个位置的距离
ADD_WIDTH = 12
SUB_WIDTH = 10
TOLERANCE = 0.0005
SCALING_FACTOR = 0.1
GRIPPER_HEIGHT = 4  # 夹爪厚
MAX_TILT = 15   # 最大偏转角
MIN_WIDTH = 40
MAX_WIDTH = 64
RESULT_PATH = "/home/jay/grasp_ws/src/grasp_pointcloud/script/real_placement_pose_estimation/config/pose_result.ini"
LIFT_DISTANCE = 0.1
PUT_DISTANCE = 0.01

class Grasp_manipulate:
    def __init__(self):
        
        # 设置ur5的moveit参数
        self.arm = moveit_commander.MoveGroupCommander("manipulator")
        self.arm.set_goal_position_tolerance(TOLERANCE)
        self.arm.set_goal_orientation_tolerance(TOLERANCE)
        self.arm.allow_replanning(True)
        self.arm.set_pose_reference_frame("base_link")
        self.arm.set_max_velocity_scaling_factor(SCALING_FACTOR)
        self.arm.set_max_acceleration_scaling_factor(SCALING_FACTOR)
        self.arm.set_planning_time(5)
        
        # 机械臂回到初始位置并打开夹爪
        rospy.set_param("/robotiq_command",'o')
        rospy.sleep(1)
        
        # 获取各个位置的关节角信息
        raw_input("1、请将机械臂调整到合适位姿，并按任意键将当前的位姿作为抓取检测的位姿，退出请按Ctrl+C：")
        self.init_joint = self.arm.get_current_joint_values()
        raw_input("2、请将机械臂调整到合适位姿，并按任意键将当前的位姿作为草莓重量检测的位姿，退出请按Ctrl+C：")
        self.place_joint = self.arm.get_current_joint_values()
        
        # 保证正确获得盒子的槽位才退出
        while True:
            self.box_num = int(raw_input("3、请输入放置盒子的数量，退出请按Ctrl+C："))
            
            # 重量间隔
            self.box_cap = []
            if self.box_num > 1:
                for i in range(self.box_num - 1):
                    self.box_cap.append(int(raw_input("3、请输入第{}个和第{}个盒子指间的重量间隔(由小到大输入)，退出请按Ctrl+C：".format(i+1, i+2))))
            elif self.box_num < 1:
                print("请输入正确的盒子数量")
            
            # 每个盒子的所在的位姿
            self.put_pose_array = []
            if self.box_num > 1:
                for i in range(self.box_num):
                    raw_input("4、请将机械臂调整到合适位姿，并检测盒子的槽位，然后按任意键将当前的位姿作为第{}个盒子的位姿，退出请按Ctrl+C：".format(i+1))
                    self.put_pose_array.append(self.arm.get_current_pose("tool0"))
            elif self.box_num == 1:
                raw_input("4、请将机械臂调整到合适位姿，并检测盒子的槽位，然后按任意键将当前的位姿作为盒子的位姿，退出请按Ctrl+C：")
                self.put_pose_array.append(self.arm.get_current_pose("tool0"))
            
            # 读取盒子的槽位数组
            self.grooves_array = []
            config = configparser.ConfigParser()
            config.read(RESULT_PATH)
            flag = False
            for i in range(self.box_num):
                if config.has_option('Result', 'result_'+str(i+1)):
                    list_string = config.get('Result', 'result_'+str(i+1))
                    config.remove_option('Result', 'result_'+str(i+1))
                    self.grooves_array.append(json.loads(list_string))
                else:
                    flag = True
                    print("槽位数量错误，请重新检测盒子")
                    self.box_cap = []
                    self.put_pose_array = []
                    self.grooves_array = []
                    break
            
            # 槽位检测是否出错，出错重检
            if flag:
                continue
            with open(RESULT_PATH, 'w') as configfile:
                config.write(configfile)
            break
        
        # 订阅物体位置的话题
        self.grasp_params_sub = rospy.Subscriber("real_detect/grasp_params", GraspParams, self.callback, queue_size=1, buff_size=52428800)


    def callback(self, grasp_params):
        
        # 同步控制：停止抓取检测
        rospy.set_param("/grasp_step", 1)
        
        # 计算：相机坐标到末端坐标的变换矩阵
        ori_cam_to_end = rot_to_ori(ROT)
        point_cam_to_end = tran_to_point(TRAN)
        matrix_cam_to_end = matrix_from_quaternion(ori_cam_to_end, point_cam_to_end)
        
        # 1、抓取检测
        # 计算：末端坐标到基坐标的变换矩阵
        pose_grasp_detect = self.arm.get_current_pose("tool0")
        matrix_end_to_base = matrix_from_quaternion(pose_grasp_detect.pose.orientation, pose_grasp_detect.pose.position)
        
        # 计算：物体相对于相机位置
        point_obj_to_cam = [grasp_params.x, grasp_params.y, grasp_params.z, 1]
        current_joint
        # 计算：物体相对于基坐标位置
        point_obj_to_base = np.dot(np.dot(matrix_end_to_base, matrix_cam_to_end), point_obj_to_cam)
        
        # 计算：物体相对于基坐标的变换矩阵
        matrix_obj_to_base = copy.deepcopy(matrix_end_to_base)
        matrix_obj_to_base[0][3],matrix_obj_to_base[1][3],matrix_obj_to_base[2][3] = point_obj_to_base[0], point_obj_to_base[1], point_obj_to_base[2]
        
        # 计算：物体相对于基坐标的变换矩阵乘rotate角度变换
        angle_z = -grasp_params.rotate_angle if grasp_params.rotate_angle < np.pi/2 else np.pi-grasp_params.rotate_angle
        rotate_z = euler_to_matrix([0, 0, angle_z])
        matrix_obj_to_base = np.dot(matrix_obj_to_base, rotate_z)
        
        # 计算：物体相对于基坐标的变换矩阵乘tilt角度变换
        angle_y = -grasp_params.tilt_angle if grasp_params.rotate_angle < np.pi/2 else grasp_params.tilt_angle
        angle_y = np.pi*MAX_TILT/180 if angle_y > np.pi*MAX_TILT/180 else -np.pi*MAX_TILT/180 if angle_y < -np.pi*MAX_TILT/180 else angle_y
        rotate_y = euler_to_matrix([0, angle_y, 0])
        matrix_obj_to_base = np.dot(matrix_obj_to_base, rotate_y)
        
        # 计算：平移一个夹爪长度的变换矩阵和当前位置的抓取输入宽度
        grasp_num = real_width_to_num(grasp_params.grasp_width_second)
        add_length = num_to_real_length(grasp_num)/1000.0
        tran_z_2 = [[1,0,0,0],[0,1,0,0],[0,0,1,-END_TO_END-add_length],[0,0,0,1]]
        matrix_obj_to_base_2 = np.dot(matrix_obj_to_base, tran_z_2)
        grasp_num_2 = real_width_to_num(grasp_params.grasp_width_second-SUB_WIDTH)
        
        # 计算：平移一个夹爪长度加一个安全距离的变换矩阵和当前位置的抓取输入宽度
        tran_z_1 = [[1,0,0,0],[0,1,0,0],[0,0,1,-END_TO_END-add_length-Z_DISTANCE],[0,0,0,1]]
        matrix_obj_to_base_1 = np.dot(matrix_obj_to_base, tran_z_1)
        grasp_num_1 = real_width_to_num(min(max(min(grasp_params.grasp_width_second+ADD_WIDTH, grasp_params.grasp_width_first+GRIPPER_HEIGHT), MIN_WIDTH), MAX_WIDTH))
        
        # 运动：机器人先转动最后一个关节
        joint = self.arm.get_current_joint_values()
        joint[5] += angle_z
        self.arm.set_joint_value_target(joint)
        rospy.set_param("/robotiq_command",str(grasp_num_1))
        self.arm.go(wait = True)
        
        # 运动：机器人到达抓取位姿的上方
        q_1 = matrix_to_quaternion(matrix_obj_to_base)
        pose_1 = self.arm.get_current_pose("tool0")
        pose_1.pose.position.x = matrix_obj_to_base_1[0][3]
        pose_1.pose.position.y = matrix_obj_to_base_1[1][3]
        pose_1.pose.position.z = matrix_obj_to_base_1[2][3]
        pose_1.pose.orientation.w = q_1.w
        pose_1.pose.orientation.x = q_1.x
        pose_1.pose.orientation.y = q_1.y
        pose_1.pose.orientation.z = q_1.z
        self.arm.set_pose_target(pose_1, "tool0")
        self.arm.go(wait = True)
        
        # 运动：机器人到达抓取位姿
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
        rospy.set_param("/robotiq_command",str(grasp_num_2))
        rospy.sleep(1)
        
        # 运动：机器人到达抓取位姿的上方
        self.arm.set_pose_target(pose_1, "tool0")
        self.arm.go(wait = True)
        
        # 运动：机器人先转动最后一个关节
        joint = self.arm.get_current_joint_values()
        joint[5] -= angle_z
        self.arm.set_joint_value_target(joint)
        self.arm.go(wait = True)
        
        # 2、重量检测
        # 运动：机器人到达放置草莓的位置并打开夹爪
        self.arm.set_joint_value_target(self.place_joint)
        self.arm.go(wait = True)
        rospy.set_param("/robotiq_command", 'o')
        rospy.sleep(1)
        
        # 运动：机器人到达检测重量的位置
        pose_place = self.arm.get_current_pose("tool0")
        matrix_place_to_base = matrix_from_quaternion(pose_place.pose.orientation, pose_place.pose.position)
        point_detect_to_place = [0, -TRAN[1], -LIFT_DISTANCE, 1]
        point_detect_to_base = np.dot(matrix_place_to_base, point_detect_to_place)
        pose_detect = copy.deepcopy(pose_place)
        pose_detect.pose.position.x = point_detect_to_base[0]
        pose_detect.pose.position.y = point_detect_to_base[1]
        pose_detect.pose.position.z = point_detect_to_base[2]
        self.arm.set_pose_target(pose_detect)
        self.arm.go(wait = True)
        
        # 等待重量检测
        rospy.set_param("/grasp_step", 2)
        volume_params = rospy.wait_for_message("real_detect/volume_params", VolumeParams)
        rospy.set_param("/grasp_step", 1)
        
        # 运动：再抓取
        grasp_num_3 = real_width_to_num(volume_params.width*1000)
        add_length = num_to_real_length(grasp_num_3)/1000.0
        pose_regrasp = copy.deepcopy(pose_detect)
        matrix_end_to_base = matrix_from_quaternion(pose_regrasp.pose.orientation, pose_regrasp.pose.position)
        point_obj_to_cam = [volume_params.x, volume_params.y, volume_params.z-END_TO_END-add_length, 1]
        point_obj_to_base = np.dot(np.dot(matrix_end_to_base, matrix_cam_to_end), point_obj_to_cam)
        pose_regrasp.pose.position.x = point_obj_to_base[0]
        pose_regrasp.pose.position.y = point_obj_to_base[1]
        pose_regrasp.pose.position.z = point_obj_to_base[2]
        self.arm.set_pose_target(pose_regrasp)
        self.arm.go(wait = True)
        current_joint = self.arm.get_current_joint_values()
        current_joint[5] += volume_params.rotate_angle * np.pi / 180
        self.arm.set_joint_value_target(current_joint)
        self.arm.go(wait = True)
        rospy.set_param("/robotiq_command", str(grasp_num_3))
        rospy.sleep(1)

        # 重新到达检测位置
        self.arm.set_pose_target(pose_detect)
        self.arm.go(wait = True)

        # 3、放置
        flag_full = False
        if self.box_num > 1:
            # 有多个盒子
            weight_rank = 0
            while weight_rank < len(self.box_cap) and volume_params.volume > self.box_cap[weight_rank]:
                weight_rank += 1
            
            pose_put = self.put_pose_array[weight_rank]
            matrix_groove_to_cam = self.grooves_array[weight_rank][0]
            del self.grooves_array[weight_rank][0]

            if len(self.grooves_array[weight_rank]) == 0:
                flag_full = True

        else:
            # 只有一个盒子的情况
            pose_put = self.put_pose_array[0]
            matrix_groove_to_cam = self.grooves_array[0][0]
            del self.grooves_array[0][0]

            if len(self.grooves_array[0]) == 0:
                flag_full = True

        # 运动：先到达放置检测的位置
        self.arm.set_pose_target(pose_put)
        self.arm.go(wait = True)
        
        # 计算：槽位相对于基坐标的变换矩阵，需要区分草莓正反
        matrix_end_to_base = matrix_from_quaternion(pose_put.pose.orientation, pose_put.pose.position)
        tran_z_3 = [[1,0,0,0],[0,1,0,0],[0,0,1,-END_TO_END-add_length-PUT_DISTANCE],[0,0,0,1]]
        matrix_groove_to_base = np.dot(matrix_end_to_base, np.dot(matrix_cam_to_end, np.dot(matrix_groove_to_cam, tran_z_3)))
        
        # 如果颠倒，则绕z轴旋转
        if volume_params.reverse:
            rotate_z = euler_to_matrix([0, 0, np.pi])
            matrix_groove_to_base = np.dot(matrix_groove_to_base, rotate_z)
        
        # 运动：到达槽位
        q_groove = matrix_to_quaternion(matrix_groove_to_base)
        pose_put.pose.position.x = matrix_groove_to_base[0][3]
        pose_put.pose.position.y = matrix_groove_to_base[1][3]
        pose_put.pose.position.z = matrix_groove_to_base[2][3]
        pose_put.pose.orientation.w = q_groove.w
        pose_put.pose.orientation.x = q_groove.x
        pose_put.pose.orientation.y = q_groove.y
        pose_put.pose.orientation.z = q_groove.z
        self.arm.set_pose_target(pose_put, "tool0")
        self.arm.go(wait = True)
        rospy.set_param("/robotiq_command", 'o')
        rospy.sleep(1)

        # 运动：先到达放置检测的位置
        self.arm.set_pose_target(pose_put)
        self.arm.go(wait = True)

        # 判断是否放满
        if flag_full:
            raw_input("盒子已放满，按Ctrl+C退出")

        # 抓取结束,夹爪打开,机械臂回初始点
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