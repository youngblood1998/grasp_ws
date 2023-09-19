import open3d as o3d
from volume_estimate_func import compute_strawberry_volume
import csv
import os
import time

NUM = 20
INIT_HEIGHT = 0.25
ANGLE_STEP_INT = 15
num_step = [i+1 for i in range(NUM)]
height_step = [INIT_HEIGHT+j*0.1 for j in range(3)]
angle_step_int = [k*ANGLE_STEP_INT for k in range(7)]
path = "./pcd/"
csv_path = "./data.csv"

def write_csv(pcd_path, position_grasp, angle_grasp, flag_reverse, volume, length, width, height):
    if not os.path.isfile(csv_path):
        with open(csv_path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["no.", "pcd_name", "x", "y", "z", "angle", "reverse", "volume", "length", "width", "height"])

    # 打开csv文件获取相关信息
    with open(csv_path, 'r', newline='') as file:
        reader = csv.reader(file)
        reader_list = list(reader)
        csv_length = len(reader_list)
        last_row = reader_list[-1]

    with open(csv_path, "a", newline='') as file:
        writer = csv.writer(file)
        if csv_length == 1:
            writer.writerow([1, pcd_path, position_grasp[0], position_grasp[1], position_grasp[2], angle_grasp,
                             flag_reverse, volume, length, width, height])
        else:
            writer.writerow([int(last_row[0])+1, pcd_path, position_grasp[0], position_grasp[1], position_grasp[2],
                             angle_grasp, flag_reverse, volume, length, width, height])

def save_data():
    for i in num_step:
        for j in height_step:
            for k in angle_step_int:
                pcd_name = str(i) + "-" + str(j) + "-" + str(k) + ".pcd"
                print(pcd_name)
                # 读取点云
                pcd = o3d.io.read_point_cloud(path + pcd_name)
                # t = time.time()
                position_grasp, angle_grasp, flag_reverse, volume, length, width, height = compute_strawberry_volume(pcd)
                # print(time.time()-t)
                # write_csv(pcd_name, position_grasp, angle_grasp, flag_reverse, volume, length, width, height)


if __name__ == "__main__":
    save_data()