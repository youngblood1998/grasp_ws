from detect_bias_save import color_segment_new, cal_moments
from volume_estimate_func import compute_strawberry_main_direction_and_histogram
import open3d as o3d
import csv
import os

NUM = 20
INIT_HEIGHT = 0.25
ANGLE_STEP_INT = 15
num_step = [i+1 for i in range(NUM)]
height_step = [INIT_HEIGHT+j*0.1 for j in range(3)]
angle_step_int = [k*ANGLE_STEP_INT for k in range(7)]
csv_path = "./data_cmp_raw.csv"

def write_csv(path, result):
    if not os.path.isfile(csv_path):
        with open(csv_path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["no.", "name", "length", "width", "area_ratio", "h1", "h2", "h3", "h4", "h5", "h6",
                             "h7", "h8", "h9", "h10", "h11", "h12", "x", "y", "z", "main_x", "main_y", "main_z"])

    # 打开csv文件获取相关信息
    with open(csv_path, 'r', newline='') as file:
        reader = csv.reader(file)
        reader_list = list(reader)
        csv_length = len(reader_list)
        last_row = reader_list[-1]

    with open(csv_path, "a", newline='') as file:
        writer = csv.writer(file)
        if csv_length == 1:
            writer.writerow([1, path]+result)
        else:
            writer.writerow([int(last_row[0])+1, path]+result)
    print(path + "已保存")

def save_data():
    for i in num_step:
        for j in height_step:
            for k in angle_step_int:
                name = str(i) + "-" + str(j) + "-" + str(k)
                # name = "1-0.25-0"
                # 读取点云
                pcd = o3d.io.read_point_cloud("./pcd/" + name + ".pcd")
                mask = color_segment_new("./rgb_img/" + name + ".png", [0, 106, 0], [179, 255, 255])
                result1 = cal_moments(mask)
                result2 = compute_strawberry_main_direction_and_histogram(pcd)
                result1.extend(result2[0].tolist())
                result1.extend(result2[1].tolist())
                result1.extend(result2[2].tolist())
                write_csv(name, result1)
    #
    # name = "20-0.45-90"
    # # 读取点云
    # pcd = o3d.io.read_point_cloud("./pcd/" + name + ".pcd")
    # mask = color_segment_new("./rgb_img/" + name + ".png", [0, 106, 0], [179, 255, 255])
    # result1 = cal_moments(mask)
    # result2 = compute_strawberry_main_direction_and_histogram(pcd)

if __name__ == "__main__":
    save_data()