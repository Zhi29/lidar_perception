import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import os

max_min_length = "/home/lizhi/sample_test_result/data_evaluation/max_min_length.txt"
max_min_width = "/home/lizhi/sample_test_result/data_evaluation/max_min_width.txt"
max_min_height = "/home/lizhi/sample_test_result/data_evaluation/max_min_height.txt"
max_min_yaw = "/home/lizhi/sample_test_result/data_evaluation/max_min_yaw.txt"
#print(filepath1)

length_err = np.loadtxt(max_min_length)
width_err = np.loadtxt(max_min_width)
height_err = np.loadtxt(max_min_height)
yaw_err = np.loadtxt(max_min_yaw)

fig