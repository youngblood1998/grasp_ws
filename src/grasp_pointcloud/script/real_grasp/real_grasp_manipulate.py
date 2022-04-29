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
from std_msgs.msg import Float32MultiArray

from real_grasp_manipulate_function import matrix_from_quaternion


# INIT_JOINT = [4.57848596572876, -1.8588832060443323, -0.49966174760927373, -2.9048364798175257, -5.310735468064443, 0.7387813925743103] #初始关节角度
GRASP_HEIGTH = 0.19     #抓取时末端应在z轴上的高度
GRASP_ADD_HEIGTH = 0.30 #抓取准备时末端应在z轴上的高度
DATA_NUM = 6            #一组抓取数据的数据个数
TRAN = [0.0, -0.0524528072638, 0.0739784679795] #手眼标定的平移
ROT = [-0.0322859285682, -0.00222200140914, -0.0294295826053, 0.999042832512]   #手眼标定的旋转

class Grasp_manipulate:
    def __init__(self):
        #设置ur5的moveit参数
        self.arm = moveit_commander.MoveGroupCommander("manipulator")
        self.arm.set_goal_position_tolerance(0.005)
        self.arm.set_goal_orientation_tolerance(0.005)
        self.arm.allow_replanning(True)
        self.arm.set_pose_reference_frame("base_link")
        self.arm.set_max_velocity_scaling_factor(0.06)
        self.arm.set_max_acceleration_scaling_factor(0.06)
        self.arm.set_planning_time(5)
        #机械臂回到初始位置并打开夹爪
        self.init_joint = self.arm.get_current_joint_values()
        # self.arm.set_joint_value_target(INIT_JOINT)
        # self.arm.go(wait = True)
        rospy.set_param("/robotiq_command",'o')
        rospy.sleep(1)
        #订阅物体位置的话题
        self.array_sub = rospy.Subscriber("/point/grasp_point", Float32MultiArray, self.callback, queue_size=1, buff_size=52428800)

    def callback(self, array):
        array = list(array.data)
        print(array)
        #将grasp_step设置为1,停止检测，开始抓取
        rospy.set_param("/grasp_step", 1)
        #将抓取数据分离
        grasp_data = []
        for i in range(int(len(array)/DATA_NUM)):
            grasp_data.append(array[i*DATA_NUM:(i+1)*DATA_NUM])
        #坐标监听
        listener = tf.TransformListener()
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            try:
                #计算末端坐标到基坐标的变换矩阵
                (trans,rot) = listener.lookupTransform('/base_link', '/wrist_3_link', rospy.Time(0))
                ori1 = Quaternion()
                ori1.x = rot[0]
                ori1.y = rot[1]
                ori1.z = rot[2]
                ori1.w = rot[3]
                point1 = Point()
                point1.x = trans[0]
                point1.y = trans[1]
                point1.z = trans[2]
                rotate1 = matrix_from_quaternion(ori1, point1)
                #计算相机坐标到末端坐标的变换矩阵
                ori2 = Quaternion()
                ori2.x = ROT[0]
                ori2.y = ROT[1]
                ori2.z = ROT[2]
                ori2.w = ROT[3]
                point2 = Point()
                point2.x = TRAN[0]
                point2.y = TRAN[1]
                point2.z = TRAN[2]
                rotate2 = matrix_from_quaternion(ori2,point2)
                #相机到基坐标的变换矩阵
                matrix = np.dot(rotate1,rotate2)
                #取第一个可抓取物体的位置
                point = grasp_data[0][0:3]
                point_a = point[:]
                point_b = point[:]
                point_a[2] = point_a[2]-GRASP_ADD_HEIGTH
                point_b[2] = point_b[2]-GRASP_HEIGTH
                point_a.append(1)
                point_b.append(1)
                point_a = np.array(point_a).reshape(4,1)
                point_b = np.array(point_b).reshape(4,1)
                #计算抓取的位置
                point_a = np.dot(matrix, point_a)
                point_b = np.dot(matrix, point_b)
                #先将末端转到抓取角度并闭合到预抓取宽度
                joint = self.arm.get_current_joint_values()
                if grasp_data[0][3] > np.pi/2:
                    joint[5] += np.pi - grasp_data[0][3]
                else:
                    joint[5] -= grasp_data[0][3]
                add_width = int(255-grasp_data[0][5]*3)
                rospy.set_param("/robotiq_command",add_width)
                self.arm.set_joint_value_target(joint)
                self.arm.go(wait = True)
                #夹爪向抓取位置运动并闭合夹爪
                # pose_list = []
                pose_a = self.arm.get_current_pose()
                pose_a.pose.position.x = point_a[0][0]
                pose_a.pose.position.y = point_a[1][0]
                pose_a.pose.position.z = point_a[2][0]
                pose_b = self.arm.get_current_pose()
                pose_b.pose.position.x = point_b[0][0]
                pose_b.pose.position.y = point_b[1][0]
                pose_b.pose.position.z = point_b[2][0]
                # pose_list.append(pose_a)
                # pose_list.append(pose_b)
                # self.arm.set_pose_targets(pose_list)
                self.arm.set_pose_target(pose_a)
                self.arm.go(wait = True)
                self.arm.set_pose_target(pose_b)
                self.arm.go(wait = True)
                rospy.set_param("/robotiq_command",'c')
                rospy.sleep(2)
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()
        #抓取结束,夹爪打开,机械臂回初始点
        # self.arm.set_joint_value_target(INIT_JOINT)
        self.arm.set_joint_value_target(self.init_joint)
        self.arm.go(wait = True)
        rospy.set_param("/robotiq_command",'o')
        rospy.sleep(2)
        #将grasp_param设置为0,开始检测，停止抓取
        rospy.set_param("/grasp_step", 0)

if __name__ == "__main__":
    rospy.init_node("grasp_manipulate")
    rospy.loginfo("抓取开始")
    Grasp_manipulate()
    rospy.spin()