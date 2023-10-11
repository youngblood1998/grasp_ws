import open3d as o3d
from placement_pose_estimation_func import pose_estimation


target = o3d.io.read_point_cloud("./pcd/box5.pcd")
pose_estimation(source_path="./pcd/target_full_box_transform.pcd", target=target, config_path="./config/pose.ini")