#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, sys
import errno
import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def import_data(date, time):
    # dateファイルまでのパス設定
    dir_path =  os.path.join(os.path.dirname(__file__), "..", "data")
    new_dir_path = os.path.join(dir_path, date)
    # ファイルの名前の確認
    if not os.path.isfile(os.path.join(new_dir_path, "raw_obstacles" +  time + ".csv")):
        print("FileNotFoundError: No such file or directory : " + os.path.join(new_dir_path, "raw_obstacles" +  time + ".csv"))
        sys.exit()
    # 各ファイルのパスの用意 
    raw_file_path = os.path.join(new_dir_path, "raw_obstacles" + time + ".csv")
    true_file_path = os.path.join(new_dir_path, "true_obstacle" + time + ".csv")
    # ファイルの読み取り
    df_raw = pd.read_csv(raw_file_path, na_values=["no_value", 0, "missing"])
    df_true = pd.read_csv(true_file_path, na_values=["no_value", 0, "missing"])
    # pandasのdfからnumpyのndarray型に変換
    raw_array = df_raw.to_numpy()
    true_array = df_true.to_numpy()

    # mix fileterの有無で分ける。
    if os.path.isfile(os.path.join(new_dir_path, "kalman_filtered_obstacles" +  time + ".csv")):
        kalman_filter_file_path = os.path.join(new_dir_path, "kalman_filtered_obstacles" +  time + ".csv")
        mix_filter_file_path = os.path.join(new_dir_path, "tracked_obstacles" +  time + ".csv")
        df_kalman_filter = pd.read_csv(kalman_filter_file_path, na_values=["no_value", 0, "missing"])
        try:
            df_mix_filter = pd.read_csv(mix_filter_file_path, na_values=["no_value", 0, "missing"])
            mix_filter_array = df_mix_filter.to_numpy()
        except:
            print("Error : fix tracked_obstacle file.")
            sys.exit()
        kalman_filter_array = df_kalman_filter.to_numpy()
        if len(kalman_filter_array) == 0:
            return raw_array, mix_filter_array, true_array 
        return raw_array, kalman_filter_array, mix_filter_array, true_array 
    else:
        tracked_file_path = os.path.join(new_dir_path, "tracked_obstacles" +  time + ".csv")
        try:
            df_tracked_filter = pd.read_csv(tracked_file_path, na_values=["no_value", 0, "missing"])
            tracked_array = df_tracked_filter.to_numpy()
        except:
            print("Error : fix tracked_obstacle file.")
            sys.exit()
        return raw_array, tracked_array, true_array 

class PrintGraph:
    def __init__(self, array, date, time):
        # figureフォルダの作成
        self.dir_path =  os.path.join(os.path.dirname(__file__), "..", "data", date, "figure")
        try:
            os.mkdir(self.dir_path)
        except OSError as e:
            if e.errno == errno.EEXIST:
                # print('Directory not created.')
                pass
            else:
                raise
        self.time = time
        # 時間リストの用意
        self.raw_time = [i/40 for i in range(len(array[0]))]

        # mix fileterの有無を調べる。
        if len(array) == 4 and len(array[1] != 0):
            self.is_mix_data = True 
        else:
            self.is_mix_data = False 

        # mix fileterの有無で分ける。
        if self.is_mix_data:
            self.raw_array = array[0]
            self.kf_array = array[1]
            self.mix_array = array[2]
            self.true_array = array[3]
            self.true_time = [i/1000 for i in range(len(array[3]))]
        else:
            self.raw_array = array[0]
            self.tracked_array = array[1]
            self.true_array = array[2]
            self.true_time = [i/1000 for i in range(len(array[2]))]

    def print_raw_data(self):
        # raw_data.jpgの作成
        fig, axs = plt.subplots(nrows=1, ncols=2, figsize=(12, 5))
        fig.suptitle('Raw Data', fontsize=16)
        axs[0].scatter(self.true_time, self.true_array[:, 0],  s=1, label="tx")
        axs[0].scatter(self.raw_time,  self.raw_array[:, 1],   s=1, label="obs1_x")
        axs[0].scatter(self.raw_time,  self.raw_array[:, 1+5], s=1, label="obs2_x")
        axs[1].scatter(self.true_time, self.true_array[:, 1],  s=1, label="ty")
        axs[1].scatter(self.raw_time,  self.raw_array[:, 2],   s=1, label="obs1_y")
        axs[1].scatter(self.raw_time,  self.raw_array[:, 2+5], s=1, label="obs2_y")
        axs[0].set_ylabel("x")
        axs[0].set_xlabel("Time [s]")
        axs[1].set_ylabel("y")
        axs[1].set_xlabel("Time [s]")
        #axs[0].set_ylim(ylimit[0], ylimit[1])
        axs[0].set_xlim(0, 20)
        axs[1].set_xlim(0, 20)
        axs[0].set_ylim(0, 2.2)
        axs[1].set_ylim(-2.2, 2.2)
        axs[0].legend()
        axs[1].legend()
        plt.savefig(os.path.join(self.dir_path, "raw_data" + self.time + ".jpg"))
        print("save fie : " + os.path.join("raw_data" + self.time + ".jpg"))

    def print_tracked_data(self, use_radius=False):
        if self.is_mix_data:
            print("I haven't had the code yet.")
            return
        if use_radius:
            fig, axs = plt.subplots(nrows=2, ncols=2, figsize=(12, 6))
            fig.suptitle('Tracked Data', fontsize=16)
            axs[0][0].scatter(self.tracked_array[:, 0], self.tracked_array[:, 1], s=1, label="x")
            axs[0][1].scatter(self.tracked_array[:, 0], self.tracked_array[:, 2], s=1, label="y")
            axs[1][0].scatter(self.tracked_array[:, 0], self.tracked_array[:, 3], s=1, label="vx")
            axs[1][1].scatter(self.tracked_array[:, 0], self.tracked_array[:, 4], s=1, label="vy")
            axs[0][0].set_ylabel("x")
            axs[0][1].set_ylabel("y")
            axs[1][0].set_ylabel("vx")
            axs[1][1].set_ylabel("vy")
            #axs[0][0].set_ylim(ylimit[0], ylimit[1])
        else:
            fig, axs = plt.subplots(nrows=3, ncols=2, figsize=(12, 6))
            fig.suptitle('Tracked Data', fontsize=16)
            axs[0][0].scatter(self.tracked_array[:, 0], self.tracked_array[:, 1], s=1, label="x")
            axs[0][1].scatter(self.tracked_array[:, 0], self.tracked_array[:, 2], s=1, label="y")
            axs[1][0].scatter(self.tracked_array[:, 0], self.tracked_array[:, 3], s=1, label="vx")
            axs[1][1].scatter(self.tracked_array[:, 0], self.tracked_array[:, 4], s=1, label="vy")
            axs[2][0].scatter(self.tracked_array[:, 0], self.tracked_array[:, 5], s=1, label="radius")
            axs[0][0].set_ylabel("x")
            axs[0][1].set_ylabel("y")
            axs[1][0].set_ylabel("vx")
            axs[1][1].set_ylabel("vy")
            axs[2][0].set_ylabel("radius")
            #axs[0][0].set_ylim(ylimit[0], ylimit[1])
            axs[2][0].set_ylim(0, 0.5)

        # tracked_ata.jpgの作成
        plt.savefig(os.path.join(self.dir_path, "tracked_data" + self.time + ".jpg"))
        print("save fie : " + os.path.join("tracked_data" + self.time + ".jpg"))

    def print_radius(self):
        if self.is_mix_data:
            print("I haven't had the code yet.")
            return
        fig, axs = plt.subplots(nrows=3, ncols=1, figsize=(12, 7))
        fig.suptitle('Tracked Data', fontsize=16)
        axs[0].scatter(self.tracked_array[:, 0], self.tracked_array[:, 5], s=1, label="r")
        axs[1].scatter(self.tracked_array[:, 0], self.tracked_array[:, 10], s=1, label="r")
        axs[2].scatter(self.tracked_array[:, 0], self.tracked_array[:, 15], s=1, label="r")
        #axs[0].scatter(true_time, array[2][:, 3], s=1, label="tvy")
        axs[0].set_ylabel("obstacle 1 [m]")
        axs[1].set_ylabel("obstacle 2 [m]")
        axs[2].set_ylabel("obstacle 3 [m]")
        plt.savefig(os.path.join(self.dir_path, "tracked_r_data" + self.time + ".jpg"))
        print("save fie : " + os.path.join("tracked_r_data" + self.time + ".jpg"))

    def print_y_velocity_data(self):
        fig, axs = plt.subplots(nrows=3, ncols=1, figsize=(12, 7))
        fig.suptitle('Tracked Data', fontsize=16)
        if self.is_mix_data:
            axs[0].scatter(self.kf_array[:, 0], self.kf_array[:, 4], s=1, label="kalman_vy")
            axs[0].scatter(self.kf_array[:, 0], self.mix_array[:, 4], s=1, label="mix_vy")
            axs[1].scatter(self.kf_array[:, 0], self.kf_array[:, 9], s=1, label="kalman_vy")
            axs[1].scatter(self.kf_array[:, 0], self.mix_array[:, 9], s=1, label="mix_vy")
            axs[2].scatter(self.kf_array[:, 0], self.kf_array[:, 14], s=1, label="kalma_vy")
            axs[2].scatter(self.kf_array[:, 0], self.mix_array[:, 14], s=1, label="mix_vy")
        else:
            axs[0].scatter(self.tracked_array[:, 0], self.tracked_array[:, 4], s=1, label="vy")
            axs[1].scatter(self.tracked_array[:, 0], self.tracked_array[:, 9], s=1, label="vy")
            axs[2].scatter(self.tracked_array[:, 0], self.tracked_array[:, 14], s=1, label="vy")
        #axs[0].scatter(true_time, array[3][:, 3], s=1, label="tvy")
        axs[0].set_ylabel("obstacle 1 [m/s]")
        axs[1].set_ylabel("obstacle 2 [m/s]")
        axs[2].set_ylabel("obstacle 3 [m/s]")
        axs[0].legend()
        plt.savefig(os.path.join(self.dir_path, "tracked_vy_data" + self.time + ".jpg"))

        n = 7
        fig, axs = plt.subplots(nrows=n, ncols=1, figsize=(12, 14))
        fig.suptitle('Tracked Data', fontsize=16)
        for i in range(n):
            axs[i].scatter(self.tracked_array[:, 0], self.tracked_array[:, (i+1)*5-1], s=1)
            axs[i].set_ylabel("obstacle"+ str(i+1) + "[m/s]")
        plt.savefig(os.path.join(self.dir_path, "tracked_vy_data" + self.time + ".jpg"))
        print("save fie : " + os.path.join("tracked_vy_data" + self.time + ".jpg"))
        #fig, axs = plt.subplots(nrows=4, ncols=1, figsize=(12, 7))
        #fig.suptitle('Tracked Data', fontsize=16)
        #true_time = [i/1000 for i in range(len(array[2]))]
        #axs[0].scatter(array[1][:, 0], array[1][:, 4], s=1, label="vy")
        #axs[0].set_ylabel("obstacle 1 [m/s]")
        #axs[1].scatter(array[1][:, 0], array[1][:, 9], s=1, label="vy")
        #axs[1].set_ylabel("obstacle 2 [m/s]")
        #axs[2].scatter(array[1][:, 0], array[1][:, 14], s=1, label="vy")
        #axs[2].set_ylabel("obstacle 3 [m/s]")
        #axs[3].scatter(array[1][:, 0], array[1][:, 19], s=1, label="vy")
        #axs[3].set_ylabel("obstacle 4 [m/s]")

date = "2022-12-19"
time = "-11-05"
time = "-12-32"
time = "-13-51"
time = "-13-56"

date = "2022-12-20"
time = "-11-26"

array = import_data(date, time)
print_graph = PrintGraph(array, date, time)
#print_graph.print_raw_data()
#print_graph.print_tracked_data(True)
print_graph.print_y_velocity_data()
#print_graph.print_radius()

