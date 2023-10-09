import numpy as np
from math import sin, cos


#欧拉角->旋转矩阵
def euler_to_matrix(theta):
    R_x = np.array([[1, 0,              0,              0],
                    [0, cos(theta[0]),  -sin(theta[0]), 0],
                    [0, sin(theta[0]),  cos(theta[0]),  0],
                    [0, 0,              0,              1]
                    ])
    R_y = np.array([[cos(theta[1]), 0,  sin(theta[1]),  0],
                    [0,             1,  0,              0],
                    [-sin(theta[1]),0,  cos(theta[1]),  0],
                    [0,             0,  0,              1]
                    ])
    R_z = np.array([[cos(theta[2]), -sin(theta[2]), 0,  0],
                    [sin(theta[2]), cos(theta[2]),  0,  0],
                    [0,             0,              1,  0],
                    [0,             0,              0,  1]
                    ])
    R = np.dot(R_z, np.dot( R_y, R_x ))
    return R

#平移矩阵
def tran_to_matrix(p):
    matrix = np.array(
        [
            [1,0,0,p[0]],
            [0,1,0,p[1]],
            [0,0,1,p[2]],
            [0,0,0,1]
        ]
    )
    return matrix