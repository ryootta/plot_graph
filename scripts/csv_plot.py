#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, sys
import errno
import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
import matplotlib.transforms as transforms
import matplotlib.ticker as ticker
from collections import deque

class Config:
    def __init__(self):
        self.is_robot = False
        self.is_mix_data = False
        self.is_cov_data = False
        self.is_target_data = False

def import_data(date, time, config):
    # dateファイルまでのパス設定
    dir_path =  os.path.join(os.path.dirname(__file__), "..", "data")
    new_dir_path = os.path.join(dir_path, date)

    # ファイルの名前の確認
    if not os.path.isfile(os.path.join(new_dir_path, "raw_obstacles" +  time + ".csv")):
        print("FileNotFoundError: No such file or directory : " + os.path.join(new_dir_path, "raw_obstacles" +  time + ".csv"))
        sys.exit()

    # 各ファイルのパスの用意 
    raw_file_path = os.path.join(new_dir_path, "raw_obstacles" + time + ".csv")
    true_obs_file_path = os.path.join(new_dir_path, "true_obstacle" + time + ".csv")
    true_robot_file_path = os.path.join(new_dir_path, "true_robot" + time + ".csv")
    target_obs_file_path = os.path.join(new_dir_path, "target_obstacle" + time + ".csv")

    # ファイルの読み取り
    df_raw = pd.read_csv(raw_file_path, na_values=["no_value", 0, "missing"])
    df_true_obs = pd.read_csv(true_obs_file_path, na_values=["no_value", 0, "missing"])
    if os.path.isfile(true_robot_file_path):
        df_true_robot = pd.read_csv(true_robot_file_path, na_values=["no_value", 0, "missing"])
        config.is_robot = True
    else:
        print("FileNotFoundError: No such file or directory : " + true_obs_file_path)
    if os.path.isfile(target_obs_file_path):
        df_target_obs = pd.read_csv(target_obs_file_path, na_values=["no_value", 0, "missing"])
        config.is_target_data = True
    else:
        print("FileNotFoundError: No such file or directory : " + target_obs_file_path)

    # pandasのdfからnumpyのndarray型に変換
    raw_array = df_raw.to_numpy()
    true_obs_array = df_true_obs.to_numpy()
    true_robot_array = df_true_robot.to_numpy() if config.is_robot else None
    target_array = df_target_obs.to_numpy() if config.is_target_data else None

    # mix fileterの有無で分ける。(kfのみrobotを追加実装)
    if os.path.isfile(os.path.join(new_dir_path, "kalman_filtered_obstacles" +  time + ".csv")):
        # 各ファイルのパスの用意 
        kalman_filter_file_path = os.path.join(new_dir_path, "kalman_filtered_obstacles" +  time + ".csv")
        mix_filter_file_path = os.path.join(new_dir_path, "tracked_obstacles" +  time + ".csv")
        cov_file_path = os.path.join(new_dir_path, "cov_obstacles" +  time + ".csv")
        # ファイルの読み取り
        df_kalman_filter = pd.read_csv(kalman_filter_file_path, na_values=["no_value", 0, "missing"])
        try:
            df_mix_filter = pd.read_csv(mix_filter_file_path, na_values=["no_value", 0, "missing"])
            mix_filter_array = df_mix_filter.to_numpy()
        except:
            print("Error : fix tracked_obstacle file.")
            sys.exit()
        try:
            df_cov_filter = pd.read_csv(cov_file_path, na_values=["no_value", 0, "missing"])
            cov_array = df_cov_filter.to_numpy()
            config.is_cov_data = True
        except:
            print("FileNotFoundError: No such file or directory : " + cov_file_path)
            
        kalman_filter_array = df_kalman_filter.to_numpy()
        # Kalman filterのデータの有無でmix_filterを使用しているか仕分ける
        if len(kalman_filter_array) == 0:
            # Kalman filterのみの障害物データ
            config.is_mix_data = False
            if config.is_cov_data and not config.is_target_data:
                return raw_array, mix_filter_array, cov_array, true_obs_array, true_robot_array
            elif not config.is_cov_data and not config.is_target_data:
                return raw_array, mix_filter_array, true_obs_array, true_robot_array
            elif not config.is_cov_data and config.is_target_data:
                return raw_array, mix_filter_array, true_obs_array, true_robot_array, target_array
            else:
                print("no code")
        else:
            # Kalman filterのみの障害物データ
            config.is_mix_data = True 
        return raw_array, kalman_filter_array, mix_filter_array, true_obs_array 
    # 旧データの取り扱い用
    else:
        tracked_file_path = os.path.join(new_dir_path, "tracked_obstacles" +  time + ".csv")
        try:
            df_tracked_filter = pd.read_csv(tracked_file_path, na_values=["no_value", 0, "missing"])
            tracked_array = df_tracked_filter.to_numpy()
        except:
            print("Error : fix tracked_obstacle file.")
            sys.exit()
        config.is_mix_data = False
        config.is_cov_data = False
        return raw_array, tracked_array, cov_array, true_obs_array, true_robot_array

class PrintGraph:
    def __init__(self, array, date, time, config):
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



        # mix fileterの有無で分ける。
        if config.is_mix_data and not config.is_cov_data and not config.is_target_data:
            self.raw_array = array[0]
            self.kf_array = array[1]
            self.mix_array = array[2]
            self.true_array = array[3]
            self.true_time = [i/1000 + array[2][0,0] for i in range(len(array[3]))]
        elif not config.is_mix_data and not config.is_cov_data and not config.is_target_data:
            self.raw_array = array[0]
            self.tracked_array = array[1]
            self.true_array = array[2]
            self.robot_array = array[3]
            self.raw_array[:, 0] = [i - array[0][0, 0] for i in array[0][:,0]]
            self.tracked_array[:, 0] = [i - array[1][0, 0] for i in array[1][:,0]]
            self.true_time = [i/1000 for i in range(len(array[2]))]
        elif not config.is_mix_data and config.is_cov_data and not config.is_target_data:
            self.raw_array = array[0]
            self.tracked_array = array[1]
            self.cov_array = array[2]
            self.true_array = array[3]
            self.robot_array = array[4]
            self.raw_array[:, 0] = [i - array[0][0, 0] for i in array[0][:,0]]
            self.tracked_array[:, 0] = [i - array[1][0, 0] for i in array[1][:,0]]
            self.cov_array[:, 0] = [i - array[2][0, 0] for i in array[2][:,0]]
            self.true_time = [i/1000 for i in range(len(array[3]))]
        elif not config.is_mix_data and not config.is_cov_data and config.is_target_data:
            self.raw_array = array[0]
            self.tracked_array = array[1]
            self.true_array = array[2]
            self.robot_array = array[3]
            self.target_array = array[4]
            self.raw_array[:, 0] = [i - array[0][0, 0] for i in array[0][:,0]]
            self.tracked_array[:, 0] = [i - array[1][0, 0] for i in array[1][:,0]]
            self.true_time = [i/1000 for i in range(len(array[2]))]
            self.target_time = [i/10 for i in range(len(array[4]))]
        else:
            print("Error: 想定していないデータの組み合わせです")

        # pltの設定
        del matplotlib.font_manager.weight_dict['roman']
        matplotlib.font_manager._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'stix'


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

    def plot_tracked_data(self, use_radius=False):        

        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'stix'
        #plt.rcParams["font.size"] = 15
        plt.rcParams["font.size"] = 18

        def create_predict_obstacle(tracked_array, predict_time):
            x_array = tracked_array[:, 1]
            y_array = tracked_array[:, 2]
            vx_array = tracked_array[:, 3]
            vy_array = tracked_array[:, 4]
            time_array = tracked_array[:, 0]
            predict_x_array = [x+v*predict_time for x, v in zip(x_array, vx_array)]
            predict_y_array = [y+v*predict_time for y, v in zip(y_array, vy_array)]
            time_array = [t+predict_time for t in time_array]
            return time_array, predict_x_array, predict_y_array
        if config.is_mix_data:
            fig, axs = plt.subplots(nrows=2, ncols=2, figsize=(12, 6))
            fig.suptitle('Tracked Data', fontsize=16)
            predict_array_1 = create_predict_obstacle(self.mix_array, 1.0)
            predict_array_2 = create_predict_obstacle(self.mix_array, 1.0)
            axs[0][0].scatter(self.mix_array[:, 0], self.mix_array[:, 1], s=1, label="x")
            axs[0][0].scatter(self.true_time, self.true_array[:, 0],  s=1, label="tx")
            axs[0][0].scatter(predict_array_1[0], predict_array_1[1],  s=1, label="pt")
            axs[0][0].scatter(predict_array_2[0], predict_array_2[1],  s=1, label="pt")
            axs[0][1].scatter(self.mix_array[:, 0], self.mix_array[:, 2], s=1, label="y")
            axs[0][1].scatter(self.true_time, self.true_array[:, 1],  s=1, label="ty")
            axs[0][1].scatter(predict_array_1[0], predict_array_1[2],  s=1, label="pt")
            axs[0][1].scatter(predict_array_2[0], predict_array_2[2],  s=1, label="pt")
            axs[1][0].scatter(self.mix_array[:, 0], self.mix_array[:, 3], s=1, label="vx")
            axs[1][0].scatter(self.true_time, self.true_array[:, 2],  s=1, label="tvx")
            axs[1][1].scatter(self.mix_array[:, 0], self.mix_array[:, 4], s=1, label="vy")
            axs[1][1].scatter(self.true_time, self.true_array[:, 3],  s=1, label="tvx")
            axs[0][0].set_ylabel("x")
            axs[0][1].set_ylabel("y")
            axs[1][0].set_ylabel("vx")
            axs[1][1].set_ylabel("vy")
        else:
            fig, axs = plt.subplots(nrows=2, ncols=2, figsize=(12, 10))
            fig.suptitle('Tracked Data'+str(self.time), fontsize=10)
            predict_array_1 = create_predict_obstacle(self.tracked_array, 1.0)
            predict_array_2 = create_predict_obstacle(self.tracked_array, 2.0)
            axs[0][0].scatter(self.true_time, self.true_array[:, 0],  s=1, label="tx", color="tab:orange")
            axs[0][0].scatter(self.tracked_array[:, 0], self.tracked_array[:, 1], s=1, label="x", color="tab:blue")
            axs[0][0].scatter(predict_array_2[0], predict_array_2[1],  s=1, label="pt", color="tab:green")
            axs[0][0].scatter(predict_array_1[0], predict_array_1[1],  s=1, label="pt", color="tab:red")

            axs[0][1].scatter(self.tracked_array[:, 0], self.tracked_array[:, 2], s=1, label="y", color="tab:orange")
            axs[0][1].scatter(self.true_time, self.true_array[:, 1],  s=1, label="ty", color="tab:blue")
            #axs[0][1].scatter(predict_array_1[0], predict_array_1[2],  s=1, label="pt")
            axs[0][1].scatter(predict_array_2[0], predict_array_2[2],  s=1, label="pt", color="tab:green")

            axs[1][0].scatter(self.tracked_array[:, 0], self.tracked_array[:, 3], s=1, label="vx", color="tab:blue")
            axs[1][0].scatter(self.true_time, self.true_array[:, 2],  s=1, label="tvx", color="tab:orange")
            axs[1][1].scatter(self.tracked_array[:, 0], self.tracked_array[:, 4], s=1, label="vy", color="tab:blue")
            axs[1][1].scatter(self.true_time, self.true_array[:, 3],  s=1, label="tvx", color="tab:orange")
            axs[0][0].set_ylabel("x")
            axs[0][0].set_xlabel("time")
            axs[0][1].set_ylabel("y")
            axs[1][0].set_ylabel("vx")
            axs[1][1].set_ylabel("vy")
            axs[0][0].xaxis.set_major_locator(ticker.MultipleLocator(5))
            axs[1][0].xaxis.set_major_locator(ticker.MultipleLocator(5))
            axs[0][1].xaxis.set_major_locator(ticker.MultipleLocator(5))
            axs[1][1].xaxis.set_major_locator(ticker.MultipleLocator(5))
            axs[1][1].set_xlim(0, self.tracked_array[-1, 0])
            axs[1][0].set_xlim(0, self.tracked_array[-1, 0])
            axs[0][1].set_xlim(0, self.tracked_array[-1, 0])
            axs[0][0].set_xlim(0, self.tracked_array[-1, 0])
            #axs[0][0].set_ylim(ylimit[0], ylimit[1])
        #    axs[2][0].scatter(self.tracked_array[:, 0], self.tracked_array[:, 5], s=1, label="radius")

        # tracked_ata.jpgの作成
        plt.savefig(os.path.join(self.dir_path, "tracked_data" + self.time + ".jpg"))
        print("save fie : " + os.path.join("tracked_data" + self.time + ".jpg"))

    def plot_radius(self):
        if config.is_mix_data:
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

    def plot_y_velocity_data(self):
        fig, axs = plt.subplots(nrows=3, ncols=1, figsize=(12, 7))
        fig.suptitle('Tracked Data', fontsize=16)
        if config.is_mix_data:
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

        n = int((len(self.tracked_array[0, :])-1)/5)
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

    def plot_robot_human_data(self):
        plt.rcParams["font.size"] = 30
        #plt.rcParams['xtick.labelsize'] = 20
        #plt.rcParams['ytick.labelsize'] = 20
        cm = plt.cm.get_cmap('RdYlBu')
        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(10, 10))
        fig.suptitle('Human Robot Data2', fontsize=16)
        #z_o = range(len(self.true_array[:, 0]))
        #z_r = range(len(self.robot_array[:, 0]))
        #ax.scatter(self.true_array[:, 0], self.true_array[:, 1], c=z_o, s=10, cmap=cm, label="obstacle")
        #ax.scatter(self.robot_array[:, 0], self.robot_array[:, 1], c=z_r, s=10, label="robot")
        #ax.set_xlim(0, 4)
        #ax.set_ylim(-2, 2)

        ax.scatter(self.true_array[:, 1], self.true_array[:, 0], s=10, label="obstacle")
        ax.scatter(self.robot_array[:, 1], self.robot_array[:, 0], s=10, label="robot")
        ax.set_xlim(2., -2.)
        ax.set_ylim(0., 4.)
        ax.set_xlim(2., -2.)
        ax.set_ylim(0., 4.)
        ax.set_yticklabels(["0.0", "", "1.0", "", "2.0", "", "3.0", "", "4.0"])
        ax.set_ylabel(r"Position $X$ [m]")
        ax.set_xlabel(r"Position $Y$ [m]")

        plt.savefig(os.path.join(self.dir_path, "xy_data" + self.time + ".jpg"))
        print("save fie : " + os.path.join("xy_data" + self.time + ".jpg"))

    def plot_xy_data(self):
        def create_predict_obstacle(tracked_array, predict_time):
            x_array = tracked_array[:, 1]
            y_array = tracked_array[:, 2]
            vx_array = tracked_array[:, 3]
            vy_array = tracked_array[:, 4]
            predict_x_array = [x+v*predict_time for x, v in zip(x_array, vx_array)]
            predict_y_array = [y+v*predict_time for y, v in zip(y_array, vy_array)]
            return predict_x_array, predict_y_array

        plt.rcParams["font.size"] = 34
        cm = plt.cm.get_cmap('Blues')
        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(10, 10))
        fig.suptitle('Human Robot Data2 '+str(self.time), fontsize=16)
        
        if config.is_mix_data:
            ax.scatter(self.true_array[:, 1], self.true_array[:, 0], s=1, label="true_data")
            ax.scatter(self.mix_array[:, 2], self.mix_array[:, 1], s=1, label="tracked_data")
            predict_array = create_predict_obstacle(self.mix_array, 1.0)
            ax.scatter(predict_array[1], predict_array[0], s=1, label="predict_data")
            predict_array = create_predict_obstacle(self.mix_array, 2.0)
            ax.scatter(predict_array[1], predict_array[0], s=1, label="predict_data")
        else:
            ax.scatter(self.true_array[:, 1], self.true_array[:, 0], s=1, label="true_data" , color="tab:orange")
            ax.scatter(self.tracked_array[:, 2], self.tracked_array[:, 1], s=4, label="tracked_data", color="green")
            predict_array = create_predict_obstacle(self.tracked_array, 1.0)
            ax.scatter(predict_array[1], predict_array[0], s=4, label="predict_data", color=cm(1.0))
            predict_array = create_predict_obstacle(self.tracked_array, 2.0)
            ax.scatter(predict_array[1], predict_array[0], s=4, label="predict_data", color=cm(0.5))
        #ax.scatter(self.raw_array[:, 2],  self.raw_array[:, 1],   s=1, label="raw_data")

        ax.set_xlim(2, -2)
        ax.set_ylim(0, 4)
        ax.set_xticklabels(["2.0", "1.0", "0.0", "-1.0", "-2.0"])
        ax.set_yticklabels(["0.0", "", "1.0", "", "2.0", "", "3.0", "", "4.0"])
        ax.get_xaxis().set_tick_params(pad=20)
        #ax.set_ylabel(r"Position $X$ [m]")
        #ax.set_xlabel(r"Position $Y$ [m]")

        plt.savefig(os.path.join(self.dir_path, "xy_data" + self.time + ".jpg"))
        print("save fie : " + os.path.join("xy_data" + self.time + ".jpg"))
    
    def plot_tracked_data2(self):        
        #plt.rcParams["font.size"] = 15
        plt.rcParams["font.size"] = 22
        def create_predict_obstacle(tracked_array, predict_time):
            x_array = tracked_array[:, 1]
            y_array = tracked_array[:, 2]
            vx_array = tracked_array[:, 3]
            vy_array = tracked_array[:, 4]
            time_array = tracked_array[:, 0]
            predict_x_array = [x+v*predict_time for x, v in zip(x_array, vx_array)]
            predict_y_array = [y+v*predict_time for y, v in zip(y_array, vy_array)]
            time_array = [t+predict_time for t in time_array]
            return time_array, predict_x_array, predict_y_array
        if config.is_mix_data:
            print("no code")
        else:
            predict_array_1 = create_predict_obstacle(self.tracked_array, 1.0)
            predict_array_2 = create_predict_obstacle(self.tracked_array, 2.0)

            fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(12, 4))
            ax.scatter(self.true_time, self.true_array[:, 0],  s=1, label="tx", color="tab:orange")
            #ax.scatter(self.raw_array[:, 0], self.raw_array[:, 1], s=4, label="x", color="tab:green")
            ax.scatter(predict_array_2[0], predict_array_2[1],  s=4, label="pt", color="tab:red")
            ax.scatter(predict_array_1[0], predict_array_1[1],  s=4, label="pt", color="tab:green")
            #ax.scatter(self.tracked_array[:, 0], self.tracked_array[:, 1], s=4, label="x", color="tab:blue")
            ax.set_xlim(0, self.tracked_array[-1, 0])
            ax.xaxis.set_major_locator(ticker.MultipleLocator(5))
            ax.grid(linestyle='dotted', color="black")

            plt.savefig(os.path.join(self.dir_path, "tracked_data_1" + self.time + ".jpg"))
            print("save fie : " + os.path.join("tracked_data_1" + self.time + ".jpg"))

            fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(12, 4))
            ax.scatter(self.tracked_array[:, 0], self.tracked_array[:, 3], s=4, label="vx", color="tab:blue")
            ax.scatter(self.true_time, self.true_array[:, 2],  s=2, label="tvx", color="tab:orange")
            ax.set_xlim(0, self.tracked_array[-1, 0])
            ax.grid(linestyle='dotted', color="black")
            ax.set_ylabel("vx")

            ax.xaxis.set_major_locator(ticker.MultipleLocator(5))
            plt.savefig(os.path.join(self.dir_path, "tracked_data_2" + self.time + ".jpg"))
            print("save fie : " + os.path.join("tracked_data_2" + self.time + ".jpg"))

    def plot_tracked_data3(self):        
        def create_predict_obstacle(tracked_array, predict_time):
            x_array = tracked_array[:, 1]
            y_array = tracked_array[:, 2]
            vx_array = tracked_array[:, 3]
            vy_array = tracked_array[:, 4]
            time_array = tracked_array[:, 0]
            predict_x_array = [x+v*predict_time for x, v in zip(x_array, vx_array)]
            predict_y_array = [y+v*predict_time for y, v in zip(y_array, vy_array)]
            time_array = [t+predict_time for t in time_array]
            return time_array, predict_x_array, predict_y_array
        plt.rcParams["font.size"] = 22
        if config.is_mix_data:
            print("no code")
        else:
            fig, axs = plt.subplots(nrows=1, ncols=2, figsize=(10, 4))
            fig.subplots_adjust(left=0.05, wspace=0.3, right=0.95)
            #fig.subplots_adjust(left=0.1, wspace=0.3, right=0.95)
            fig.suptitle('Tracked Data'+str(self.time), fontsize=10)
            predict_array_1 = create_predict_obstacle(self.tracked_array, 1.0)
            predict_array_2 = create_predict_obstacle(self.tracked_array, 2.0)
            axs[0].scatter(self.true_time, self.true_array[:, 0],  s=1, label="tx", color="tab:orange")
            #axs[0].scatter(predict_array_2[0], predict_array_2[1],  s=4, label="pt", color="tab:red")
            #axs[0].scatter(predict_array_1[0], predict_array_1[1],  s=4, label="pt", color="tab:green")
            axs[0].scatter(self.tracked_array[:, 0], self.tracked_array[:, 1], s=4, label="x", color="tab:blue")

            #axs[0].scatter(self.tracked_array[:, 0], self.tracked_array[:, 3], s=4, label="vx", color="tab:blue")
            #axs[0].scatter(self.true_time, self.true_array[:, 2],  s=2, label="tvx", color="tab:orange")

            axs[1].scatter(self.true_time, self.true_array[:, 1],  s=1, label="tx", color="tab:orange")
            #axs[1].scatter(predict_array_2[0], predict_array_2[2],  s=4, label="pt", color="tab:red")
            #axs[1].scatter(predict_array_1[0], predict_array_1[2],  s=4, label="pt", color="tab:green")
            axs[1].scatter(self.tracked_array[:, 0], self.tracked_array[:, 2], s=4, label="x", color="tab:blue")

            #axs[1].scatter(self.tracked_array[:, 0], self.tracked_array[:, 4], s=4, label="vx", color="tab:blue")
            #axs[1].scatter(self.true_time, self.true_array[:, 3],  s=2, label="tvx", color="tab:orange")

            axs[0].xaxis.set_major_locator(ticker.MultipleLocator(5))
            axs[1].xaxis.set_major_locator(ticker.MultipleLocator(5))
            axs[0].yaxis.set_major_locator(ticker.MultipleLocator(1))
            axs[1].yaxis.set_major_locator(ticker.MultipleLocator(1))
            #axs[0].set_ylim(0.5, 3.5)
            #axs[1].set_ylim(0.5, 3.5)
            axs[0].set_xlim(0, self.tracked_array[-1, 0])
            axs[1].set_xlim(0, self.tracked_array[-1, 0])
            axs[0].set_ylabel("x")
            axs[1].set_ylabel("y")

            axs[0].grid(linestyle='dotted', color="black")
            axs[1].grid(linestyle='dotted', color="black")

            plt.savefig(os.path.join(self.dir_path, "tracked_data_1" + self.time + ".jpg"))
            print("save fie : " + os.path.join("tracked_data_1" + self.time + ".jpg"))

            fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(12, 4))
            ax.scatter(self.tracked_array[:, 0], self.tracked_array[:, 3], s=4, label="vx", color="tab:blue")
            ax.scatter(self.true_time, self.true_array[:, 2],  s=2, label="tvx", color="tab:orange")
            ax.set_xlim(0, self.tracked_array[-1, 0])
            ax.grid(linestyle='dotted', color="black")
            ax.set_ylabel("vx")

            ax.xaxis.set_major_locator(ticker.MultipleLocator(5))
            plt.savefig(os.path.join(self.dir_path, "tracked_data_2" + self.time + ".jpg"))
            print("save fie : " + os.path.join("tracked_data_2" + self.time + ".jpg"))

    def plot_covariance_data(self):        
        if not config.is_cov_data:
            print("error: don't have covarince data")
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'stix'
        #plt.rcParams["font.size"] = 15
        plt.rcParams["font.size"] = 22
        if config.is_mix_data:
            print("no code")
        else:
            fig, axs = plt.subplots(nrows=1, ncols=2, figsize=(10, 4))
            fig.subplots_adjust(left=0.1, wspace=0.3, right=0.95)
            fig.suptitle('Covariance Data'+str(self.time), fontsize=10)
            axs[0].scatter(self.cov_array[100:298, 0], self.cov_array[100:298, 6], s=4, label="x", color="tab:blue")
            axs[1].scatter(self.cov_array[100:, 0], self.cov_array[100:, 8], s=4, label="y", color="tab:blue")

            #axs[0].xaxis.set_major_locator(ticker.MultipleLocator(5))
            #axs[1].xaxis.set_major_locator(ticker.MultipleLocator(5))
            #axs[0].yaxis.set_major_locator(ticker.MultipleLocator(1))
            #axs[1].yaxis.set_major_locator(ticker.MultipleLocator(1))
            #axs[0].set_ylim(0.5, 3.5)
            #axs[1].set_ylim(0.5, 3.5)
            axs[0].set_xlim(0, self.tracked_array[-1, 0])
            axs[1].set_xlim(0, self.tracked_array[-1, 0])
            axs[0].set_ylabel("cxx")
            axs[1].set_ylabel("cyy")

            axs[0].grid(linestyle='dotted', color="black")
            axs[1].grid(linestyle='dotted', color="black")

            plt.savefig(os.path.join(self.dir_path, "xy_cov_data" + self.time + ".jpg"))
            print("save fie : " + os.path.join("xy_cov_data" + self.time + ".jpg"))

            fig, axs = plt.subplots(nrows=1, ncols=2, figsize=(10, 4))
            fig.subplots_adjust(left=0.1, wspace=0.3, right=0.95)
            fig.suptitle('Tracked Data'+str(self.time), fontsize=10)
            axs[0].scatter(self.cov_array[100:, 0], self.cov_array[100:, 7], s=4, label="cvx", color="tab:blue")
            axs[1].scatter(self.cov_array[:, 0], self.cov_array[:, 9], s=4, label="cvy", color="tab:blue")
            axs[0].set_ylabel("cvxx")
            axs[1].set_ylabel("cvyy")

            plt.savefig(os.path.join(self.dir_path, "vxy_cov_data" + self.time + ".jpg"))
            print("save fie : " + os.path.join("vxy_cov_data" + self.time + ".jpg"))

    def plot_vy_filter_data(self):        
        def sum_data_computer(tracked_array, n):
            vy_array = tracked_array[:, 4]
            time_array = tracked_array[:, 0]
            time_array = time_array[n:len(time_array)]
            vy_que = deque()
            vy_list = []
            for vy in vy_array:
                if len(vy_que) < n:
                    vy_que.append(vy)
                elif len(vy_que) == n:
                    vy_que.append(vy)
                    vy_que.popleft()
                    vy_hat = sum(vy_que)/n
                    vy_list.append(vy_hat)
                else:
                    print("ERRRRRRRRRRRRRRRRRRRRRRRRR")
            return time_array, vy_list 
        def print_zero_vy(sum_data):
            for i, vy in enumerate(sum_data[1]):
                if vy < -0.01:
                    print(sum_data[0][i])
                    break
        plt.rcParams["font.size"] = 22
        if config.is_mix_data:
            print("no code")
        else:
            sum_data5 = sum_data_computer(self.tracked_array, 5) 
            sum_data10 = sum_data_computer(self.tracked_array, 10) 
            sum_data20 = sum_data_computer(self.tracked_array, 20) 
            sum_data40 = sum_data_computer(self.tracked_array, 40) 
            fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(12, 4))
            ax.scatter(self.tracked_array[:, 0], self.tracked_array[:, 4], s=4, label="vx", color="tab:blue")
            ax.scatter(self.true_time, self.true_array[:, 3],  s=2, label="tvy", color="tab:orange")
            ax.scatter(sum_data5[0],  sum_data5[1],  s=2, label="tvy", color="tab:red")
            ax.scatter(sum_data10[0], sum_data10[1],  s=2, label="tvy", color="tab:green")
            ax.scatter(sum_data20[0], sum_data20[1],  s=2, label="tvy", color="tab:orange")
            ax.scatter(sum_data40[0], sum_data40[1],  s=2, label="tvy", color="tab:orange")
            ax.set_xlim(0, self.tracked_array[-1, 0])
            ax.grid(linestyle='dotted', color="black")
            ax.set_ylabel("vy")

            ax.xaxis.set_major_locator(ticker.MultipleLocator(5))
            plt.savefig(os.path.join(self.dir_path, "vy_filter_data" + self.time + ".jpg"))
            print("save fie : " + os.path.join("vy_filter_data" + self.time + ".jpg"))

            print("")
            print_zero_vy(sum_data5)
            print_zero_vy(sum_data10)
            print_zero_vy(sum_data20)
            print_zero_vy(sum_data40)

    def plot_vy_make_data(self):        
        def vy_data_computer(tracked_array, n):
            y_array = tracked_array[:, 2]
            time_array = tracked_array[:, 0]
            time_array = time_array[n:len(time_array)]
            y_old = y_array[0]
            vy_list = []
            for y in y_array:
                delta_y = y - y_old
                y_old = y
                vy = delta_y / 0.1
                vy_list.append(vy)

            vy_que = deque()
            vy_array = vy_list
            vy_list = []
            for vy in vy_array:
                if len(vy_que) < n:
                    vy_que.append(vy)
                elif len(vy_que) == n:
                    vy_que.append(vy)
                    vy_que.popleft()
                    vy_hat = sum(vy_que)/n
                    vy_list.append(vy_hat)
            
            return time_array, vy_list 

        def sum_data_computer(tracked_array, n):
            vy_array = tracked_array[:, 4]
            time_array = tracked_array[:, 0]
            time_array = time_array[n:len(time_array)]
            vy_que = deque()
            vy_list = []
            for vy in vy_array:
                if len(vy_que) < n:
                    vy_que.append(vy)
                elif len(vy_que) == n:
                    vy_que.append(vy)
                    vy_que.popleft()
                    vy_hat = sum(vy_que)/n
                    vy_list.append(vy_hat)
                else:
                    print("ERRRRRRRRRRRRRRRRRRRRRRRRR")
            return time_array, vy_list 
        plt.rcParams["font.size"] = 22
        if config.is_mix_data:
            print("no code")
        else:
            sum_data10 = sum_data_computer(self.tracked_array, 20)
            sum_data10_y = vy_data_computer(self.tracked_array, 20)
            #sum_data10_raw_y = vy_data_computer(self.raw_array, 1)
            sum_data10_raw_y10 = vy_data_computer(self.raw_array, 10)

            fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(12, 4))
            #ax.scatter(self.tracked_array[:, 0], self.tracked_array[:, 4], s=4, label="vx", color="tab:blue")
            ax.scatter(self.true_time, self.true_array[:, 3],  s=2, label="tvy", color="tab:orange")
            ax.scatter(sum_data10[0],  sum_data10[1],  s=2, label="tvy", color="tab:red")
            ax.scatter(sum_data10_y[0],  sum_data10_y[1],  s=2, label="tvy", color="tab:blue")
            ax.scatter(sum_data10_raw_y[0],  sum_data10_raw_y[1],  s=2, label="tvy", color="tab:green")
            ax.scatter(sum_data10_raw_y10[0],  sum_data10_raw_y10[1],  s=2, label="tvy", color="tab:red")
            ax.set_xlim(0, self.tracked_array[-1, 0])
            ax.grid(linestyle='dotted', color="black")
            ax.set_ylabel("vy")

            ax.xaxis.set_major_locator(ticker.MultipleLocator(5))
            plt.savefig(os.path.join(self.dir_path, "vy_filter_data" + self.time + ".jpg"))
            print("save fie : " + os.path.join("vy_filter_data" + self.time + ".jpg"))

    def plot_target_obstacle(self):
        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(12, 4))
        #ax.scatter(self.tracked_array[:, 0], self.tracked_array[:, 4], s=4, label="vx", color="tab:blue")
        ax.scatter(self.true_time, self.true_array[:, 3],  s=2, label="tvy", color="tab:orange")
        ax.scatter(self.target_time,  self.target_array[:, 3],  s=2, label="tvy", color="tab:green")
        ax.set_xlim(0, self.tracked_array[-1, 0])
        ax.grid(linestyle='dotted', color="black")
        ax.set_ylabel("vy")

        ax.xaxis.set_major_locator(ticker.MultipleLocator(5))
        plt.savefig(os.path.join(self.dir_path, "target_data" + self.time + ".jpg"))
        print("save fie : " + os.path.join("target_data" + self.time + ".jpg"))

date = "2022-12-19"
time = "-11-05"
time = "-12-32"
time = "-13-51"
time = "-13-56"

date = "2022-12-20"
time = "-11-26"

date = "2023-01-16"
time = "-07-02"
time = "-07-06"
#time = "-05-22"
#
#date = "2023-01-23"
#time = "-15-01"
date = "2023-01-24"
time = "-12-44"
time = "-14-00"
time = "-14-04"
time = "-14-18" #0.2[m/s]
time = "-14-20" #0.4[m/s]
#time = "-15-26"
#time = "-15-54"
#
#time = "-13-41" #0.4[m/s]
#time = "-13-11" #0.2[m/s]


date = "2023-01-28"
time = "-14-36"
time = "-15-46"
time = "-18-22"
time = "-18-27"

date = "2023-02-02"
time = "-13-09" #cu omega=0.1 v=0.3
time = "-13-16" #cu omega=0.1 v=0.3
time = "-13-30" #cu omega=0.3 v=0.3
time = "-13-16" #cu omega=0.1 v=0.3
time = "-13-09" #cu omega=0.1 v=0.3

#date = "2023-02-03"
#time = "-10-51" #cu omega=0.1 v=0.3
#time = "-14-13" #cu omega=0.1 v=0.3


date = "2023-02-05"
time = "-06-27"

config = Config()
array = import_data(date, time, config)
plot_graph = PrintGraph(array, date, time, config)
#plot_graph.plot_raw_data()
plot_graph.plot_tracked_data(True)
#plot_graph.plot_y_velocity_data()
#plot_graph.plot_radius()
plot_graph.plot_y_velocity_data()
#plot_graph.plot_robot_human_data()
plot_graph.plot_xy_data()
#plot_graph.plot_tracked_data2()
plot_graph.plot_tracked_data3()
#plot_graph.plot_covariance_data()
plot_graph.plot_vy_filter_data()
plot_graph.plot_vy_make_data()
plot_graph.plot_target_obstacle()

