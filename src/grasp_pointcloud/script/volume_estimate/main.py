import numpy
import numpy as np
import open3d as o3d
import copy

for i in range(1, 11):
    # 读取点云
    pcd = o3d.io.read_point_cloud("../../pcd/single{}.pcd".format(str(i)))  # single4.pcd尾朝上，别用

    # 直通滤波
    # 定义过滤范围
    x_min, x_max = -0.1, 0.1   # x轴过滤范围
    y_min, y_max = -0.1, 0.1   # y轴过滤范围
    z_min, z_max = 0, 1.0   # z轴过滤范围
    # 进行直通滤波
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(x_min, y_min, z_min), max_bound=(x_max, y_max, z_max))
    pcd_filtered = pcd.crop(bbox)

    # 平面分割
    plane_model, inliers = pcd_filtered.segment_plane(distance_threshold=0.003, ransac_n=3, num_iterations=100)
    inlier_cloud = pcd_filtered.select_down_sample(inliers)     # 平面
    outlier_cloud = pcd_filtered.select_down_sample(inliers, invert=True)       # 平面之外的点云
    n = np.asarray(plane_model)[:3]     # 平面的法向量
    print('平面法向量的值:' + str(n))

    # 离群点滤波
    cl, ind = outlier_cloud.remove_statistical_outlier(nb_neighbors=10, std_ratio=1.0)

    # 再次过滤，RGB值过滤
    # 将点云转换为NumPy数组，以便进行RGB值的过滤
    points = np.asarray(cl.points)
    colors = np.asarray(cl.colors)
    # 设置颜色筛选条件
    red_channel_values = colors[:, 0]
    green_channel_values = colors[:, 1]
    color_filter = ((red_channel_values > 0.2) & (red_channel_values < 0.4)) | (green_channel_values > 0) & (green_channel_values < 0.4)
    # 使用颜色筛选条件选择点云
    filtered_points = points[color_filter]
    filtered_colors = colors[color_filter]
    # 创建新的点云对象并保存
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
    color = [1, 0, 1]
    filtered_pcd.colors = o3d.utility.Vector3dVector([color for i in range(len(filtered_pcd.points))])

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
    # rot_mat = np.array([proj_dir, perp_dir, n])
    rot_mat = np.linalg.inv(np.array([proj_dir, perp_dir, n]))
    print('变换后z轴的单位向量:' + str(rot_mat[:, 2]))
    # 创建变换矩阵
    t_matrix = np.identity(4)
    t_matrix[:3, :3] = rot_mat
    t_matrix[0:3, 3] = center
    # print(t_matrix)

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
    print("主方向向量:" + str(main_direction))
    # 计算主方向在平面的投影
    projection_vector = main_direction - np.dot(main_direction, n) / np.dot(n, n) * n
    print("主方向向量在平面的投影:" + str(projection_vector))

    # 计算坐标系y轴的单位向量
    y_axis = rot_mat[:, 1]                      # 提取该坐标系的 Y 轴方向
    y_unit_vector = y_axis / np.linalg.norm(y_axis)   # 转化为单位向量
    print("坐标系的y轴单位向量:" + str(y_unit_vector))

    # 计算点云主方向在平面上的投影与变换后的坐标系的y轴之间的变换矩阵
    # 计算两个向量的点积
    dot_product = np.dot(y_unit_vector, projection_vector)
    # 计算两个向量的模长
    norm_vec1 = np.linalg.norm(y_unit_vector)
    norm_vec2 = np.linalg.norm(projection_vector)
    # 计算两个向量的夹角
    angle = np.arccos(dot_product / (norm_vec1 * norm_vec2))
    print("向量夹角:" + str(angle))
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
    print("y轴与主方向夹角:" + str(angle_transformed))
    # 如果变换错误，则矫正
    if not angle_transformed < 0.04:
        angle = -angle
        print("矫正后的向量夹角:" + str(angle))
        # 定义一个旋转矩阵
        R = np.array([[np.cos(angle), -np.sin(angle), 0, 0],
                      [np.sin(angle), np.cos(angle), 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])  # 绕 z 轴旋转矩阵
        # 变换矩阵
        T_matrix = np.dot(t_matrix, R)

    # 创建坐标轴
    coord_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.03)
    # 对坐标轴进行变换
    coord_axes.transform(T_matrix)

    # 计算主方向和包围盒
    bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(filtered_pcd.points)
    obb = filtered_pcd.get_oriented_bounding_box()
    # print(numpy.asarray(bbox.get_box_points()))
    # print(numpy.asarray(obb.get_box_points()))

    # 可视化结果
    coord_axes_base = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    o3d.visualization.draw_geometries([coord_axes_base, coord_axes, pcd, obb], "result")
    print('--'*20)