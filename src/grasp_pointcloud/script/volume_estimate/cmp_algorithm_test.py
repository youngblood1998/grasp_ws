import pandas as pd
import joblib
import os
import csv

FIT_INDEXES = [1, 2, 4, 12, 14, 15, 16, 17, 18, 19]     # 拟合的索引
ANALYZE_INDEXES = list(set(range(1, 21)) - set(FIT_INDEXES))    # 分析误差的索引
NUM = 3*7
DATA_PATH = './data/data_cmp.csv'
SAVE_PATH = './model/'
RESULT_PATH = './data/data_cmp_result.csv'

def write_csv(name, y_test, y_no_pca, y_with_pca):
    if not os.path.isfile(RESULT_PATH):
        with open(RESULT_PATH, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["no.", "pcd_name", "y_test", "y_no_pca", "y_with_pca"])

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
                writer.writerow([1, name[i], y_test[i], y_no_pca[i], y_with_pca[i]])
            else:
                writer.writerow([int(last_row[0])+1, name[i], y_test[i], y_no_pca[i], y_with_pca[i]])

if __name__ == '__main__':
    data = pd.read_csv(DATA_PATH)
    X_no_pca = data.drop(['no.', 'name', 'main_x', 'main_y', 'main_z', 'weight'], axis=1)
    X_with_pca = data.drop(['no.', 'name', 'weight'], axis=1)
    Y = data['weight']
    name = data['name']

    x_no_pca_test_to_concat = []
    x_with_pca_test_to_concat = []
    y_test_to_concat = []
    name_test_to_concat = []
    for index in ANALYZE_INDEXES:
        x_no_pca_test_to_concat.append(X_no_pca.iloc[(index-1)*NUM: index*NUM])
        x_with_pca_test_to_concat.append(X_with_pca.iloc[(index-1)*NUM: index*NUM])
        y_test_to_concat.append(Y.iloc[(index-1)*NUM: index*NUM])
        name_test_to_concat.append(name.iloc[(index-1)*NUM: index*NUM])
    x_no_pca_test = pd.concat(x_no_pca_test_to_concat, axis=0)
    x_with_pca_test = pd.concat(x_with_pca_test_to_concat, axis=0)
    y_test = list(pd.concat(y_test_to_concat, axis=0))
    name_test = list(pd.concat(name_test_to_concat, axis=0))

    rf_regressor_no_pca = joblib.load(SAVE_PATH + 'model_no_pca.pkl')
    rf_regressor_with_pca = joblib.load(SAVE_PATH + 'model_with_pca.pkl')

    y_no_pca_pred = rf_regressor_no_pca.predict(x_no_pca_test)
    y_with_pca_pred = rf_regressor_with_pca.predict(x_with_pca_test)

    write_csv(name_test, y_test, y_no_pca_pred, y_with_pca_pred)