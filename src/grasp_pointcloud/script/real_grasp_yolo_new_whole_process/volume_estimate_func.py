#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import open3d as o3d
import copy
import time

STEP = 0.002
X_VAL = 0.01
THRESHOLD = 0.001
X_MIN, X_MAX = -0.1, 0.1
Y_MIN, Y_MAX = -0.1, 0.1
Z_MIN, Z_MAX = 0.18, 1.0
VOXEL_SIZE = 0.001

def compute_triangle_mesh_volume(mesh):
    """
    计算 TriangleMesh 的体积。

    Args:
        mesh: open3d.geometry.TriangleMesh 对象。

    Returns:
        float: 三角网格的体积。
    """

    vertices = mesh.vertices
    triangles = mesh.triangles
    # print(len(vertices))
    # print(len(triangles))

    # 遍历每个三角形，计算其面积并累加到总面积中
    total_volume = 0.0
    for triangle in triangles:
        p1 = vertices[triangle[0]]
        p2 = vertices[triangle[1]]
        p3 = vertices[triangle[2]]
        volume = abs(np.dot(np.cross(p2 - p1, p3 - p1), p1))
        total_volume += volume

    return total_volume / 6.0


def compute_strawberry_volume(pcd, show=False):

    # 进行直通滤波
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(X_MIN, Y_MIN, Z_MIN), max_bound=(X_MAX, Y_MAX, Z_MAX))
    pcd_filtered = pcd.crop(bbox)

    # 平面分割
    plane_model, inliers = pcd_filtered.segment_plane(distance_threshold=0.003, ransac_n=3, num_iterations=100)
    inlier_cloud = pcd_filtered.select_down_sample(inliers)     # 平面
    outlier_cloud = pcd_filtered.select_down_sample(inliers, invert=True)       # 平面之外的点云
    n = np.asarray(plane_model)[:3]     # 平面的法向量
    # print('平面法向量的值:' + str(plane_model))

    # 离群点滤波
    filtered_pcd, ind = outlier_cloud.remove_statistical_outlier(nb_neighbors=10, std_ratio=1.0)

    # 复制预处理后的点云
    filtered_pcd_copy = copy.deepcopy(filtered_pcd)
    # 计算点云中心
    center = filtered_pcd_copy.get_center()

    # 计算平面法向量与基坐标系z轴的变换矩阵
    # 找到平面法向量在三维坐标系中的投影方向
    z = np.array([0, 0, 1])
    proj_dir = np.cross(n, z)
    proj_dir = proj_dir / np.linalg.norm(proj_dir)
    # 找到第二个与平面法向量和其投影方向都垂直的向量
    perp_dir = np.cross(n, proj_dir)
    perp_dir = perp_dir / np.linalg.norm(perp_dir)
    # 构造旋转矩阵
    rot_mat = np.linalg.inv(np.array([proj_dir, perp_dir, n]))
    # print('变换后z轴的单位向量:' + str(rot_mat[:, 2]))
    # 创建变换矩阵
    t_matrix = np.identity(4)
    t_matrix[:3, :3] = rot_mat
    t_matrix[0:3, 3] = center

    # 计算点云主方向
    # 计算PCA
    filtered_pcd_copy.estimate_normals()
    points = np.asarray(filtered_pcd_copy.points)
    points -= center
    cov = np.dot(points.T, points) / points.shape[0]
    # Singular value decomposition.
    eigen_values, eigen_vectors = np.linalg.eig(cov)
    idx = eigen_values.argsort()[::-1]
    eigen_values = eigen_values[idx]
    eigen_vectors = eigen_vectors[:, idx]
    # 获取主方向向量
    main_direction = eigen_vectors[:, 0]
    # print("主方向向量:" + str(main_direction))
    # 计算主方向在平面的投影
    projection_vector = main_direction - np.dot(main_direction, n) / np.dot(n, n) * n
    # print("主方向向量在平面的投影:" + str(projection_vector))

    # 计算坐标系y轴的单位向量
    y_axis = rot_mat[:, 1]                      # 提取该坐标系的 Y 轴方向
    y_unit_vector = y_axis / np.linalg.norm(y_axis)   # 转化为单位向量
    # print("坐标系的y轴单位向量:" + str(y_unit_vector))

    # 计算点云主方向在平面上的投影与变换后的坐标系的y轴之间的变换矩阵
    # 计算两个向量的点积
    dot_product = np.dot(y_unit_vector, projection_vector)
    # 计算两个向量的模长
    norm_vec1 = np.linalg.norm(y_unit_vector)
    norm_vec2 = np.linalg.norm(projection_vector)
    # 计算两个向量的夹角
    angle = np.arccos(dot_product / (norm_vec1 * norm_vec2))
    # print("向量夹角:" + str(angle))
    # 定义一个旋转矩阵
    R = np.array([[np.cos(angle), -np.sin(angle), 0, 0],
                  [np.sin(angle), np.cos(angle), 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])  # 绕 z 轴旋转矩阵
    # 变换矩阵
    T_matrix = np.dot(t_matrix, R)

    # 判断变换是否正确
    # 判断变换后的夹角是否在0附近
    y_unit_vector = T_matrix[0:3, 1]
    # 计算两个向量的点积
    dot_product = np.dot(y_unit_vector, projection_vector)
    # 计算两个向量的模长
    norm_vec1 = np.linalg.norm(y_unit_vector)
    norm_vec2 = np.linalg.norm(projection_vector)
    # 计算两个向量的夹角
    angle_transformed = np.arccos(max(min(dot_product/(norm_vec1 * norm_vec2), 1), -1))
    # print("y轴与主方向夹角:" + str(angle_transformed))
    # 如果变换错误，则矫正
    if not angle_transformed < 0.04:
        angle = -angle
        # print("矫正后的向量夹角:" + str(angle))
        # 定义一个旋转矩阵
        R = np.array([[np.cos(angle), -np.sin(angle), 0, 0],
                      [np.sin(angle), np.cos(angle), 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])  # 绕 z 轴旋转矩阵
        # 变换矩阵
        T_matrix = np.dot(t_matrix, R)

    filtered_pcd_transform = copy.deepcopy(filtered_pcd)
    filtered_pcd_transform.transform(np.linalg.inv(T_matrix))
    # 进行体素下采样
    filtered_pcd_transform = filtered_pcd_transform.voxel_down_sample(voxel_size=VOXEL_SIZE)

    aabb = o3d.geometry.AxisAlignedBoundingBox.create_from_points(filtered_pcd_transform.points)
    # # 获取OBB对象
    obb = aabb.get_oriented_bounding_box()
    # # 对OBB进行旋转和平移操作
    obb.rotate(T_matrix[:3, :3])
    obb.translate(T_matrix[:3, 3])

    # 调整草莓抓取坐标
    coord_axes_center = T_matrix[:3, 3]
    # 找到草莓最高点和最低点（平面）
    top = np.min(np.array(filtered_pcd_transform.points)[:, 2])
    bottom = np.abs(plane_model[0]*coord_axes_center[0]+plane_model[1]*coord_axes_center[1]
                     +plane_model[2]*coord_axes_center[2]+plane_model[3]) / np.sqrt(plane_model[0]**2
                                                                                    +plane_model[1]**2
                                                                                    +plane_model[2]**2)
    # 找到草莓y轴最前点和最后点
    front = np.min(np.array(filtered_pcd_transform.points)[:, 1])
    end = np.max(np.array(filtered_pcd_transform.points)[:, 1])
    left = np.min(np.array(filtered_pcd_transform.points)[:, 0])
    right = np.max(np.array(filtered_pcd_transform.points)[:, 0])
    points = np.array(filtered_pcd_transform.points)
    points_x = points[:, 0]
    points_y = points[:, 1]
    # 沿y轴方向截取点云计算x轴方向的最大方位的位置，即找到草莓直径最大处的位置
    best_y_pos = 0
    scope = 0
    arithmetic_arr = np.arange(front, end, STEP)
    for i in arithmetic_arr:
        idx = (points_y > i) & (points_y < i+STEP)
        points_x_cut = points_x[idx]
        if len(points_x_cut) == 0:
            continue
        if np.max(points_x_cut) - np.min(points_x_cut) > scope:
            scope = np.max(points_x_cut) - np.min(points_x_cut)
            best_y_pos = i + STEP/2
    # 调整草莓坐标系的z轴原点到草莓最高点和平面的中间，y轴原点到最大直径处
    trans = np.array([[1, 0, 0, 0],
                      [0, 1, 0, best_y_pos],
                      [0, 0, 1, (top+bottom)/2],
                      [0, 0, 0, 1]])
    T_matrix_grasp = np.dot(T_matrix, trans)
    # # 判断y轴方向是否指向草莓头，不是则绕自身z轴旋转180度
    if abs(front - best_y_pos) < abs(end - best_y_pos):
        rotate_z = np.array([
            [-1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        T_matrix = np.dot(T_matrix, rotate_z)
        T_matrix_grasp = np.dot(T_matrix_grasp, rotate_z)
        filtered_pcd_transform = copy.deepcopy(filtered_pcd)
        filtered_pcd_transform.transform(np.linalg.inv(T_matrix))

    # points_transform = np.array(filtered_pcd_transform.points)
    # colors_transform = np.array(filtered_pcd_transform.colors)
    # green_indexes = colors_transform[:, 0] <= colors_transform[:, 1]
    # points_green = points_transform[green_indexes]
    # point_green_mean = np.mean(points_green, axis=0)
    # print(point_green_mean[1])
    # if point_green_mean[1] < 0:
    #     rotate_z = np.array([
    #         [-1, 0, 0, 0],
    #         [0, -1, 0, 0],
    #         [0, 0, 1, 0],
    #         [0, 0, 0, 1]
    #     ])
    #     T_matrix = np.dot(T_matrix, rotate_z)
    #     T_matrix_grasp = np.dot(T_matrix_grasp, rotate_z)
    #     filtered_pcd_transform = copy.deepcopy(filtered_pcd)
    #     filtered_pcd_transform.transform(np.linalg.inv(T_matrix))

    # 二分法查找点云对称平面
    points = np.array(filtered_pcd_transform.points)
    point_1 = points[np.argmin(points[:, 1])]
    z_top = top
    z_bottom = bottom
    reflect_points = []
    # 二分查找
    while True:
        z_middle = (z_top+z_bottom)/2
        # print(z_middle)
        # 定义三个点，并由三点计算平面法向量
        point_2 = np.array([-X_VAL, end, z_middle])
        point_3 = np.array([X_VAL, end, z_middle])
        vector_1 = point_1 - point_2
        vector_2 = point_1 - point_3
        vector_normal = np.cross(vector_1, vector_2)
        vector_normal_normalized = vector_normal/np.linalg.norm(vector_normal)
        # 将点相对于平面镜像
        for point in points:
            distance = np.dot(vector_normal_normalized, point-point_1)
            reflect_points.append(point - 2*distance*vector_normal_normalized)
        # 计算最低点与平面的距离是否在阈值内
        deviation = np.max(np.array(reflect_points)[:, 2]) - bottom
        if np.abs(deviation) < THRESHOLD or np.abs(z_top - z_bottom) < THRESHOLD:
            break
        else:
            # 更新
            reflect_points.clear()
            if deviation > 0:
                z_bottom = z_middle
            else:
                z_top = z_middle
    reflect_points = np.array(reflect_points)

    # 合并镜像点云和原点云
    points = np.concatenate((points, reflect_points), axis=0)
    filtered_pcd_transform.points = o3d.utility.Vector3dVector(points)
    # filtered_pcd_transform.colors = o3d.utility.Vector3dVector([color for i in range(len(filtered_pcd_transform.points))])

    filtered_pcd_transform_2 = copy.deepcopy(filtered_pcd_transform)
    filtered_pcd_transform_2.transform(T_matrix)

    # 离群点滤波
    filtered_pcd_transform, ind = filtered_pcd_transform.remove_statistical_outlier(nb_neighbors=5, std_ratio=1)

    hull, _ = filtered_pcd_transform.compute_convex_hull()
    hull.paint_uniform_color([1, 1, 1])

    # 提取所有非共面的三角形边缘
    edge_set = set()
    triangles = np.asarray(hull.triangles)
    for i in range(triangles.shape[0]):
        edges = [(triangles[i, j], triangles[i, (j + 1) % 3]) for j in range(3)]
        for e in edges:
            if (e[1], e[0]) in edge_set:
                edge_set.remove((e[1], e[0]))
            else:
                edge_set.add(e)
    # 将边缘保存为线段集合的形式
    lines = []
    for e in edge_set:
        lines.append([e[0], e[1]])
    # 创建Open3D线段集合对象
    lineset = o3d.geometry.LineSet()
    lineset.points = hull.vertices
    lineset.lines = o3d.utility.Vector2iVector(lines)
    # 将线段染成红色
    lineset.paint_uniform_color([0, 1, 0])

    lineset.transform(T_matrix)
    # print(type(hull))

    volume = compute_triangle_mesh_volume(hull)
    # print("TriangleMesh 体积：", volume)

    # 创建坐标轴
    coord_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.03)
    # 对坐标轴进行变换
    coord_axes.transform(T_matrix_grasp)

    # 抓取位置，是否颠倒，抓取角度
    position_grasp = T_matrix_grasp[:3, 3]
    flag_reverse = False
    if T_matrix_grasp[1, 1] < 0:
        flag_reverse = True
    angle_grasp = np.arctan(-T_matrix_grasp[0, 1] / T_matrix_grasp[1, 1]) * 180 / np.pi
    print("是否颠倒：" + str(flag_reverse))
    print("抓取位置：" + str(position_grasp))
    print("抓取角度：" + str(angle_grasp))
    print("重量:" + str(volume))
    print("宽度:" + str(right-left))

    # 可视化结果
    if show:
        coord_axes_base = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.03)
        # o3d.visualization.draw_geometries([coord_axes_base, coord_axes, pcd, obb, filtered_pcd_transform_2, lineset], "result")
        o3d.visualization.draw_geometries([coord_axes_base, coord_axes, pcd, obb, filtered_pcd_transform_2, lineset], "result")

    return position_grasp, angle_grasp, flag_reverse, volume, right-left