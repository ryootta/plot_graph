#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import errno
import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def import_data(date, time):
    # dateファイルまでのパス設定
    dir_path =  os.path.join(os.path.dirname(__file__), "..", "data")
    new_dir_path = os.path.join(dir_path, date)
    # 各ファイルのパスの用意 
    raw_file_path = os.path.join(new_dir_path, "raw_obstacles" + time + ".csv")
    filter_file_path = os.path.join(new_dir_path, "tracked_obstacles" +  time + ".csv")
    true_file_path = os.path.join(new_dir_path, "true_obstacle" + time + ".csv")
    # ファイルの読み取り
    #try:
    #    df_raw = pd.read_csv(raw_file_path)
    #    df_filter = pd.read_csv(filter_file_path)
    #    df_true = pd.read_csv(true_file_path)
    #except Exception as err:
    #    print("obstacles number is more than 2.")
    #    df_raw = pd.read_csv(raw_file_path, na_values=["no_value", 0, "missing"])
    #    df_filter = pd.read_csv(filter_file_path, na_values=["no_value", 0, "missing"])
    #    df_true = pd.read_csv(true_file_path, na_values=["no_value", 0, "missing"])
    
    df_raw = pd.read_csv(raw_file_path, na_values=["no_value", 0, "missing"])
    df_filter = pd.read_csv(filter_file_path, na_values=["no_value", 0, "missing"])
    df_true = pd.read_csv(true_file_path, na_values=["no_value", 0, "missing"])

    print(df_raw)
    # pandasのdfからnumpyのndarray型に変換
    raw_array = df_raw.to_numpy()
    filter_array = df_filter.to_numpy()
    true_array = df_true.to_numpy()
    return raw_array, filter_array, true_array 

def print_graph(array, date, time, use_radius=False):
    # figureフォルダの作成
    dir_path =  os.path.join(os.path.dirname(__file__), "..", "data", date, "figure")
    try:
        os.mkdir(dir_path)
    except OSError as e:
        if e.errno == errno.EEXIST:
            # print('Directory not created.')
            pass
        else:
            raise

    # raw_data.jpgの作成
    fig, axs = plt.subplots(nrows=1, ncols=2, figsize=(12, 5))
    fig.suptitle('Raw Data', fontsize=16)
    raw_time = [i/40 for i in range(len(array[0]))]
    true_time = [i/1000 for i in range(len(array[2]))]
    axs[0].scatter(true_time, array[2][:, 0], s=1, label="tx")
    axs[0].scatter(raw_time,  array[0][:, 1], s=1, label="x1")
    axs[0].scatter(raw_time,  array[0][:, 1+5], s=1, label="x2")
    #axs[1].scatter(true_time, array[2][:, 1], s=1, label="ty")
    axs[1].scatter(raw_time,  array[0][:, 2], s=1, label="y1")
    axs[1].scatter(raw_time,  array[0][:, 2+5], s=1, label="y2")
    axs[0].set_ylabel("x")
    axs[0].set_xlabel("Time [s]")
    axs[1].set_ylabel("y")
    axs[1].set_xlabel("Time [s]")
    #axs[0].set_ylim(ylimit[0], ylimit[1])
    axs[0].set_xlim(0, 20)
    axs[1].set_xlim(0, 20)
    axs[0].set_ylim(0, 2.2)
    axs[1].set_ylim(-2.2, 2.2)

    plt.savefig(os.path.join(dir_path, "raw_data" + time + ".jpg"))

    # tracked_ata.jpgの作成
    if not use_radius:
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
    else:
        fig, axs = plt.subplots(nrows=3, ncols=2, figsize=(12, 6))
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
        axs[2][0].scatter(array[1][:, 0], array[1][:, 5], s=1, label="radius")
        axs[2][0].set_ylabel("radius")
        axs[2][0].set_ylim(0, 0.5)

    plt.savefig(os.path.join(dir_path, "tracked_data" + time + ".jpg"))

    fig, axs = plt.subplots(nrows=3, ncols=1, figsize=(12, 7))
    fig.suptitle('Tracked Data', fontsize=16)
    true_time = [i/1000 for i in range(len(array[2]))]
    axs[0].scatter(array[1][:, 0], array[1][:, 4], s=1, label="vy")
    #axs[0].scatter(true_time, array[2][:, 3], s=1, label="tvy")
    axs[0].set_ylabel("obstacle 1 [m/s]")
    axs[1].scatter(array[1][:, 0], array[1][:, 9], s=1, label="vy")
    axs[1].set_ylabel("obstacle 2 [m/s]")
    axs[2].scatter(array[1][:, 0], array[1][:, 14], s=1, label="vy")
    axs[2].set_ylabel("obstacle 3 [m/s]")
    plt.savefig(os.path.join(dir_path, "tracked_vy_data" + time + ".jpg"))

    fig, axs = plt.subplots(nrows=3, ncols=1, figsize=(12, 7))
    fig.suptitle('Tracked Data', fontsize=16)
    true_time = [i/1000 for i in range(len(array[2]))]
    axs[0].scatter(array[1][:, 0], array[1][:, 5], s=1, label="r")
    #axs[0].scatter(true_time, array[2][:, 3], s=1, label="tvy")
    axs[0].set_ylabel("obstacle 1 [m]")
    axs[1].scatter(array[1][:, 0], array[1][:, 10], s=1, label="r")
    axs[1].set_ylabel("obstacle 2 [m]")
    axs[2].scatter(array[1][:, 0], array[1][:, 15], s=1, label="r")
    axs[2].set_ylabel("obstacle 3 [m]")
    plt.savefig(os.path.join(dir_path, "tracked_r_data" + time + ".jpg"))


#date = "2022-11-08"
#time = "-12-17"

date = "2022-11-10"
time = "-13-25"
time = "-13-36"
time = "-13-39"

date = "2022-11-11"
time = "-08-01"

date = "2022-11-14"
time = "-05-24"
time = "-05-32"
time = "-05-37"
time = "-05-42"
time = "-06-12"

date = "2022-11-17"
time = "-07-29"
time = "-07-43"
time = "-07-51"
time = "-07-58"
time = "-09-20"

date = "2022-11-21"
time = "-07-50"
time = "-08-18"
time = "-08-37"
time = "-08-42"
time = "-08-51"
time = "-14-25"

#date = "2022-11-14"
#time = "-05-37"

array = import_data(date, time)
print_graph(array, date, time, True)