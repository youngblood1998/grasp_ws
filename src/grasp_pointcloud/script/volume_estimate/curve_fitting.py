import numpy as np
import matplotlib.pyplot as plt
import csv
import os

WEIGHT = [19.01, 11.73, 15.07, 16.30, 12.42, 16.41, 12.00, 13.71, 15.39, 11.60,
          13.06, 15.21, 14.72, 14.80, 13.02, 13.15, 12.19, 13.65, 14.55, 13.26]
FIT_INDEXES = [1, 2, 4, 12, 14, 15, 16, 17, 18, 19]     # 拟合的索引
ANALYZE_INDEXES = list(set(range(1, 21)) - set(FIT_INDEXES))    # 分析误差的索引
NUM = 3*7
RESULT_PATH = './data/data_result.csv'

def curve_fit(x, y, x_label, y_label, save_path):
    # 使用最小二乘法拟合直线 y = kx
    k = np.dot(x, y) / np.dot(x, x)
    # 绘制原始数据点和拟合直线
    # Set the font size for all text in the plot
    plt.rcParams.update({'font.size': 14})
    color_idx = np.arange(len(x)) // NUM
    plt.scatter(x, y, c=color_idx)
    x_line = np.array([0, int(np.max(x)) + 1])
    plt.plot(x_line, k * x_line)
    # 添加文字标注直线方程
    plt.text(0.7 * np.mean(x_line), 0.6 * k * np.mean(x_line), "y = {}x".format(round(k, 5)))
    # 设置坐标轴标签
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    # 设置坐标轴范围
    plt.xlim(0, int(np.max(x)) + 1)
    plt.ylim(0, int(np.max(y)) + 1)
    plt.savefig(save_path, dpi=300)
    # 显示图形
    plt.show()
    plt.close()
    return k


def accuracy_statistics(y_array, y_correct, x_label, y_label, lines_label, save_path):
    # 统计0.05到0.30的占比
    accuracy_array = [i * 0.05 for i in range(7)]
    # 对每一种方法求准确率
    for i, y in zip(list(range(len(y_array))), y_array):
        accuracy = np.abs(y - y_correct) / y_correct
        percent_accuracy = []
        for thresh in accuracy_array:
            percent_accuracy.append(np.count_nonzero(accuracy < thresh)/len(accuracy))
        accuracy_array = np.array(accuracy_array)
        percent_accuracy = np.array(percent_accuracy)
        # 绘制折线图
        plt.plot(accuracy_array, percent_accuracy, label=lines_label[i])
    # 设置坐标轴标签
    plt.legend()
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    # 设置坐标轴范围
    plt.xlim(0, max(accuracy_array))
    plt.ylim(0, 1)
    plt.grid(True)
    plt.savefig(save_path, dpi=300)
    plt.show()

def write_csv(name, y_test, y_volume, y_xyz, y_xxy):
    if not os.path.isfile(RESULT_PATH):
        with open(RESULT_PATH, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["no.", "pcd_name", "y_test", "y_volume", "y_xyz", 'y_xxy'])

    for i in range(len(name)):
        # 打开csv文件获取相关信息
        with open(RESULT_PATH, 'r', newline='') as file:
            reader = csv.reader(file)
            reader_list = list(reader)
            csv_length = len(reader_list)
            last_row = reader_list[-1]

        with open(RESULT_PATH, "a", newline='') as file:
            writer = csv.writer(file)
            if csv_length == 1:
                writer.writerow([1, name[i], y_test[i], y_volume[i], y_xyz[i], y_xxy[i]])
            else:
                writer.writerow([int(last_row[0])+1, name[i], y_test[i], y_volume[i], y_xyz[i], y_xxy[i]])

if __name__ == "__main__":
    # 获取数据
    csv_path = "./data/data.csv"
    # 打开csv文件获取相关信息
    with open(csv_path, 'r', newline='') as file:
        reader = csv.reader(file)
        reader_list = list(reader)

    # 将拟合数据分离
    x_volume_fit = []
    x_xyz_volume_fit = []
    x_xxy_volume_fit = []
    y_fit = []
    name = []
    for index in FIT_INDEXES:
        for i in range(NUM):
            row = (index-1)*NUM+i+1
            name.append(reader_list[row][1])
            x_volume_fit.append(float(reader_list[row][7]))
            x_xyz_volume_fit.append(float(reader_list[row][8]) * float(reader_list[row][9]) * float(reader_list[row][10]))
            x_xxy_volume_fit.append(float(reader_list[row][8]) * float(reader_list[row][9]) * float(reader_list[row][9]))
            y_fit.append(float(reader_list[row][12]))

    # 进行数据拟合
    y_fit = np.array(y_fit)
    # 体积拟合
    x_volume_fit = np.array(x_volume_fit)
    x_volume_fit = x_volume_fit * 10 ** 6
    k1 = curve_fit(x_volume_fit, y_fit, 'Volume/cm$^{3}$', 'Weight/g', "./data/curve_fitting_volume_new.png")
    # obb xyz体积拟合
    x_xyz_volume_fit = np.array(x_xyz_volume_fit)
    x_xyz_volume_fit = np.abs(x_xyz_volume_fit * 10 ** 6)
    k2 = curve_fit(x_xyz_volume_fit, y_fit, 'Volume/cm$^{3}$', 'Weight/g', "./data/curve_fitting_xyz_new.png")
    # obb xxy体积拟合
    x_xxy_volume_fit = np.array(x_xxy_volume_fit)
    x_xxy_volume_fit = np.abs(x_xxy_volume_fit * 10 ** 6)
    k3 = curve_fit(x_xxy_volume_fit, y_fit, 'Volume/cm$^{3}$', 'Weight/g', "./data/curve_fitting_xxy_new.png")

    # 将分析的数据分离
    x_volume_analyze = []
    x_xyz_volume_analyze = []
    x_xxy_volume_analyze = []
    y_analyze = []
    for index in ANALYZE_INDEXES:
        for i in range(NUM):
            row = (index-1)*NUM+i+1
            x_volume_analyze.append(float(reader_list[row][7]))
            x_xyz_volume_analyze.append(float(reader_list[row][8]) * float(reader_list[row][9]) * float(reader_list[row][10]))
            x_xxy_volume_analyze.append(float(reader_list[row][8]) * float(reader_list[row][9]) * float(reader_list[row][9]))
            y_analyze.append(float(reader_list[row][12]))

    # 数据转换成numpy格式
    x_volume_analyze = np.array(x_volume_analyze)
    x_volume_analyze = np.abs(x_volume_analyze) * 10 ** 6
    x_xyz_volume_analyze = np.array(x_xyz_volume_analyze)
    x_xyz_volume_analyze = np.abs(x_xyz_volume_analyze) * 10 ** 6
    x_xxy_volume_analyze = np.array(x_xxy_volume_analyze)
    x_xxy_volume_analyze = np.abs(x_xxy_volume_analyze) * 10 ** 6
    # 使用拟合的函数估计重量
    y_analyze = np.array(y_analyze)
    y_volume_analyze = k1 * x_volume_analyze
    y_xyz_volume_analyze = k2 * x_xyz_volume_analyze
    y_xxy_volume_analyze = k3 * x_xxy_volume_analyze
    # write_csv(name, y_analyze, y_volume_analyze, y_xyz_volume_analyze, y_xxy_volume_analyze)
    # 画准确率图
    accuracy_statistics([y_volume_analyze, y_xyz_volume_analyze, y_xxy_volume_analyze], y_analyze, 'Percentage ranges',
                        'Cumulative percentage', ["hull", "xyz", "xxy"], "./data/accuracy_statistics.png")