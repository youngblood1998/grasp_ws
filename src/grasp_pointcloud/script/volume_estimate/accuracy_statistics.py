import numpy as np
import matplotlib.pyplot as plt
import csv

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
        print(percent_accuracy)
        # 绘制折线图
        plt.plot(accuracy_array, percent_accuracy*100, label=lines_label[i])
    # 设置坐标轴标签
    plt.legend()
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    # 设置坐标轴范围
    plt.xlim(0, max(accuracy_array))
    plt.ylim(0, 100)
    # plt.grid(True)
    plt.savefig(save_path, dpi=300)
    plt.show()

if __name__ == '__main__':
    # 获取数据
    data_path = "./data/data_result.csv"
    # 打开csv文件获取相关信息
    with open(data_path, 'r', newline='') as file:
        reader = csv.reader(file)
        data_list = np.array(list(reader))
    y_test = data_list[1:, 2].astype(np.float)
    y_volume = data_list[1:, 3].astype(np.float)
    y_xyz = data_list[1:, 4].astype(np.float)
    y_xxy = data_list[1:, 5].astype(np.float)

    # 获取数据
    data_cmp_path = "./data/data_cmp_result.csv"
    # 打开csv文件获取相关信息
    with open(data_cmp_path, 'r', newline='') as file:
        reader = csv.reader(file)
        data_cmp_list = np.array(list(reader))
    y_no_pca = data_cmp_list[1:, 3].astype(np.float)
    y_with_pca = data_cmp_list[1:, 4].astype(np.float)

    # 画准确率图
    accuracy_statistics([y_volume, y_xyz, y_xxy, y_no_pca, y_with_pca], y_test, 'PCW tolerance level',
                        'Correct weight estimation rate / %', ["Ours", "XYZ", "XXY", "Random forest", "Random forest (PCA)"], "./data/accuracy_statistics_all.png")