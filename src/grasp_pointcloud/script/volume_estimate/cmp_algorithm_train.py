import pandas as pd
from sklearn.ensemble import RandomForestRegressor
import joblib

FIT_INDEXES = [1, 2, 4, 12, 14, 15, 16, 17, 18, 19]     # 拟合的索引
ANALYZE_INDEXES = list(set(range(1, 21)) - set(FIT_INDEXES))    # 分析误差的索引
NUM = 3*7
DATA_PATH = './data/data_cmp.csv'
SAVE_PATH = './model/'

def random_forest_train(x_train, y_train, save_path, n_estimators=50, random_state=42):
    # 实例化模型
    rf_regressor = RandomForestRegressor(n_estimators=n_estimators, random_state=random_state)
    # 训练模型
    rf_regressor.fit(x_train, y_train)
    # 使用joblib库保存模型
    joblib.dump(rf_regressor, save_path)

if __name__ == '__main__':
    data = pd.read_csv(DATA_PATH)
    X_no_pca = data.drop(['no.', 'name', 'main_x', 'main_y', 'main_z', 'weight'], axis=1)
    X_with_pca = data.drop(['no.', 'name', 'weight'], axis=1)
    Y = data['weight']

    x_no_pca_train_to_concat = []
    x_with_pca_train_to_concat = []
    y_train_to_concat = []
    for index in FIT_INDEXES:
        x_no_pca_train_to_concat.append(X_no_pca.iloc[(index-1)*NUM: index*NUM])
        x_with_pca_train_to_concat.append(X_with_pca.iloc[(index-1)*NUM: index*NUM])
        y_train_to_concat.append(Y.iloc[(index-1)*NUM: index*NUM])

    x_no_pca_train = pd.concat(x_no_pca_train_to_concat, axis=0)
    x_with_pca_train = pd.concat(x_with_pca_train_to_concat, axis=0)
    y_train = pd.concat(y_train_to_concat, axis=0)
    random_forest_train(x_no_pca_train, y_train, SAVE_PATH+'model_no_pca.pkl')
    random_forest_train(x_with_pca_train, y_train, SAVE_PATH + 'model_with_pca.pkl')