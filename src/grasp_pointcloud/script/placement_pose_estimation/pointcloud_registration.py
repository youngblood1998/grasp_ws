import open3d as o3d
import numpy as np
from copy import deepcopy
from math import radians
import configparser
from trans_func import euler_to_matrix, tran_to_matrix

# 读取局部点云和全局点云
target = o3d.io.read_point_cloud("./pcd/box4.pcd")
source = o3d.io.read_point_cloud("./pcd/target_full_box_transform.pcd")

voxel_size=0.005

# 进行点云降采样
source_down = source.voxel_down_sample(voxel_size=voxel_size)
target_down = target.voxel_down_sample(voxel_size=voxel_size)

# 用来显示坐标系的点云
source_down_axis = deepcopy(source_down)

# 将target_down复制并绕z轴旋转180度
source_down_reverse = deepcopy(source_down)
# 定义旋转角度（单位：度）
angle_deg = 180
# 将角度转换为弧度
angle_rad = np.deg2rad(angle_deg)
# 定义旋转矩阵
rotation_matrix = np.array([[np.cos(angle_rad), -np.sin(angle_rad), 0],
                            [np.sin(angle_rad), np.cos(angle_rad), 0],
                            [0, 0, 1]])
# 对点云进行旋转
source_down_reverse.rotate(rotation_matrix, center=True)

# 使用 RANSAC 进行平面分割
plane_model, inliers = target_down.segment_plane(distance_threshold=0.01,
                                            ransac_n=3,
                                            num_iterations=100)
# 将平面内的点提取出来
target_down = target_down.select_down_sample(inliers, invert=True)

# 离群点滤波
target_down, ind = target_down.remove_statistical_outlier(nb_neighbors=5, std_ratio=1.0)

# 设置DBSCAN参数
eps = 0.01
min_points = 10

# 进行DBSCAN聚类
labels = np.array(target_down.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))

# 统计每个簇的点数
unique, counts = np.unique(labels, return_counts=True)
cluster_sizes = dict(zip(unique, counts))

# 找到最大的簇
largest_cluster_label = max(cluster_sizes, key=cluster_sizes.get)

# 提取最大的簇
target_down = target_down.select_down_sample(np.where(labels == largest_cluster_label)[0])

# 直通滤波
box_height = 0.04
x_min, x_max = -0.2, 0.2   # x轴过滤范围
y_min, y_max = -0.2, 0.2   # y轴过滤范围
z_min, z_max = target_down.get_max_bound()[2]-box_height, target_down.get_max_bound()[2]   # z轴过滤范围

bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(x_min, y_min, z_min), max_bound=(x_max, y_max, z_max))
target_down = target_down.crop(bbox)

# 计算包围盒
source_bbox = source_down.get_axis_aligned_bounding_box()
target_bbox = target_down.get_axis_aligned_bounding_box()

# 将包围盒中心对齐
source_center = source_bbox.get_center()
target_center = target_bbox.get_center()
translate = target_center - source_center
source_down.translate(translate)
source_down_reverse.translate(translate)
print(translate)

# 设置ICP参数
threshold = 0.01
trans_init = np.asarray([[1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
reg_p2p_1 = o3d.registration.registration_icp(
    source_down, target_down, threshold, trans_init,
    o3d.registration.TransformationEstimationPointToPoint())
reg_p2p_2 = o3d.registration.registration_icp(
    source_down_reverse, target_down, threshold, trans_init,
    o3d.registration.TransformationEstimationPointToPoint())

# if reg_p2p_1.inlier_rmse < reg_p2p_2.inlier_rmse:
#     print(reg_p2p_1.transformation)
#     source_down.transform(reg_p2p_1.transformation)
#     # 可视化
#     o3d.visualization.draw_geometries([source_down, target_down])
# else:
#     print(reg_p2p_2.transformation)
#     source_down_reverse.transform(reg_p2p_2.transformation)
#     # 可视化
#     o3d.visualization.draw_geometries([source_down_reverse, target_down])
transformation = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])
if reg_p2p_1.inlier_rmse < reg_p2p_2.inlier_rmse:
    transformation = np.dot(reg_p2p_1.transformation, tran_to_matrix(translate))
else:
    transformation = np.dot(reg_p2p_2.transformation, np.dot(tran_to_matrix(translate), euler_to_matrix([0, 0, radians(180)])))

source_down_axis.transform(transformation)

# 读取配置文件
config = configparser.ConfigParser()
config.read("./config/pose.ini", encoding="utf-8")
row_num = config.getint("Parameters", "row_num")
col_num = config.getint("Parameters", "col_num")
row_step = config.getfloat("Parameters", "row_step")
col_step = config.getfloat("Parameters", "col_step")
x_init = config.getfloat("Parameters", "x_init")
y_init = config.getfloat("Parameters", "y_init")
angle = config.getint("Parameters", "angle")
frame_array = []
for i in range(row_num):
    y = y_init + i * col_step
    for j in range(col_num):
        x = x_init + j * row_step
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.02, origin=[0, 0, 0])
        frame.transform(np.dot(transformation, np.dot(tran_to_matrix([x, y, 0]), euler_to_matrix([0, 0, radians(angle)]))))
        frame_array.append(frame)

# 可视化
o3d.visualization.draw_geometries([target] + frame_array)