# import open3d as o3d
# from placement_pose_estimation_func import pose_estimation
#
#
# target = o3d.io.read_point_cloud("./pcd/box5.pcd")
# frame_value_array = pose_estimation(source_path="./pcd/target_full_box_transform.pcd", target=target, config_path="./config/pose_config.ini")
# print(frame_value_array)

#--------------------------------------------------------------------------------------------------------
# import json
#
# def convert_3d_list_to_string(list_3d):
#     # 使用JSON将三维列表转换为字符串
#     list_string = json.dumps(list_3d)
#     return list_string
#
# def convert_string_to_3d_list(list_string):
#     # 使用JSON将字符串转换为三维列表
#     list_3d = json.loads(list_string)
#     return list_3d
#
# # 示例三维列表
# three_dimensional_list = [[[1, 2], [3, 4]], [[5, 6], [7, 8]]]
#
# # 将三维列表转换为字符串
# converted_string = convert_3d_list_to_string(three_dimensional_list)
# print("Converted String:", converted_string)
# print(type(converted_string))
#
# # 将字符串转换回三维列表
# converted_list = convert_string_to_3d_list(converted_string)
# print("Converted List:", converted_list)
# print(type(converted_list))

#--------------------------------------------------------------------------------------------------------
# import open3d as o3d
#
#
# pcd = o3d.io.read_point_cloud("./pcd/target_full_box_transform.pcd")
# print(pcd)

#--------------------------------------------------------------------------------------------------------
# import configparser
# import json

# # 将三维列表转换为字符串并写入pose_result.ini文件
# frame_value_array = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]  # 你的三维列表数据
# list_string = json.dumps(frame_value_array)

# config = configparser.ConfigParser()
# mydict = {
#     'result1': list_string,
#     'result2': list_string
# }
# config['Result'] = mydict

# with open('/home/jay/grasp_ws/src/grasp_pointcloud/script/real_placement_pose_estimation/config/pose_result.ini', 'w') as configfile:
#     config.write(configfile)

# # 从pose_result.ini中读取字符串并将其转换为三维列表
# config = configparser.ConfigParser()
# config.read('/home/jay/grasp_ws/src/grasp_pointcloud/script/real_placement_pose_estimation/config/pose_result.ini')

# if 'Result' in config:
#     list_string_new = config.get('Result', 'result1')
#     list_3d = json.loads(list_string_new)
#     print(type(list_3d))
# else:
#     print("No 'Result' section found in pose_result.ini")
#------------------------------------------------------------------------------------------------
import configparser

def clear_config(filename):
    # 创建 configparser 对象
    config = configparser.ConfigParser()

    # 读取配置文件内容
    config.read(filename)

    # 移除所有的节和配置项
    for option in config.options('Result'):
        config.remove_option('Result', option)

    # 将修改后的配置写回到文件中
    with open(filename, 'w') as configfile:
        config.write(configfile)

# 调用示例
clear_config('./config/pose_result.ini')