#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import errno
import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def import_data(date, time):
    # ファイルpathの設定
    dir_path =  os.path.join(os.path.dirname(__file__), "..", "data")
    new_dir_path = os.path.join(dir_path, date)
    # ファイルの名前の用意 
    raw_file_name = "raw_obstacles" + time + ".csv"
    filter_file_name = "tracked_obstacles" +  time + ".csv"
    true_file_name = "true_obstacle" + time + ".csv"
    # ファイルの読み取り
    df_raw = pd.read_csv(os.path.join(new_dir_path, raw_file_name))
    df_filter = pd.read_csv(os.path.join(new_dir_path, filter_file_name))
    df_true = pd.read_csv(os.path.join(new_dir_path, true_file_name))
    print(df_raw)
    # pandasのdfからnumpyのndarray型に変換
    raw_array = df_raw.to_numpy()
    filter_array = df_filter.to_numpy()
    true_array = df_true.to_numpy()
    return raw_array, filter_array, true_array 

def print_graph(array, date, time):
    dir_path =  os.path.join(os.path.dirname(__file__), "..", "data", date, "figure")
    try:
        os.mkdir(dir_path)
    except OSError as e:
        if e.errno == errno.EEXIST:
            # print('Directory not created.')
            pass
        else:
            raise
    fig, axs = plt.subplots(nrows=1, ncols=2, figsize=(12, 5))
    fig.suptitle('Raw Data', fontsize=16)
    index = [i for i in range(len(array[0]))]
    axs[0].scatter(index, array[0][:, 1], s=1, label="x")
    axs[0].set_ylabel("x")
    axs[0].set_xlabel("Time")
    axs[1].scatter(index, array[0][:, 2], s=1, label="y")
    axs[1].set_ylabel("y")
    axs[1].set_xlabel("Time")
    #axs[0].set_ylim(ylimit[0], ylimit[1])
    plt.savefig(os.path.join("figure", "raw_data" + time + ".jpg"))

    fig, axs = plt.subplots(nrows=2, ncols=2, figsize=(12, 6))
    fig.suptitle('Tracked Data', fontsize=16)
    axs[0][0].scatter(array[1][:, 0], array[1][:, 1], s=1, label="x")
    axs[0][0].set_ylabel("x")
    #axs[0][0].set_ylim(ylimit[0], ylimit[1])
    axs[0][1].scatter(array[1][:, 0], array[1][:, 2], s=1, label="y")
    axs[0][1].set_ylabel("y")
    axs[1][0].scatter(array[1][:, 0], array[1][:, 3], s=1, label="vx")
    axs[1][0].set_ylabel("vx")
    axs[1][1].scatter(array[1][:, 0], array[1][:, 4], s=1, label="vy")
    axs[1][1].set_ylabel("vy")

    #plt.show()
    plt.savefig(os.path.join("figure", "tracked_data" + time + ".jpg"))


date = "2022-11-08"
time = "-12-17"
array = import_data(date, time)
print(array[0])
print_graph(array, date, time)