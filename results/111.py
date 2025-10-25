# 打开文件
import numpy as np


def trans_matrix(filename1, filename2, n, m):
    with open(filename1, 'r') as file:
        lines = file.readlines()
    string_list = lines
    split_list = [item.split() for item in string_list]
    data = []
    for i in range(n):
        if i % 2 == 0:
            data.append(split_list[i])

    array = np.array(data)
    float_data = array.astype(float)
    matrix = float_data.reshape(m, 8)
    with open(filename2, 'r') as file:
        lines1 = file.readlines()
    split_list1 = [item.split() for item in lines1]
    array1 = np.array(split_list1)
    float_data1 = array1.astype(float)
    matrix1 = float_data1.reshape(m, 8)
    for i in range(m):
        matrix[i, 0] = matrix1[i, 0]
    odom11 = []
    for i in range(m):
        for j in range(8):
            odom11.append(matrix[i, j])
            odom11.append(' ')
        odom11.append('\n')

    with open('odom11111.txt', 'w') as f:
        for arr in odom11:
            f.write("%s" % arr)


trans_matrix('odom.txt', 'lio_sam_mapping_odometry.txt', 652, 326)
