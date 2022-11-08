#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def import_data(raw_file_name, filter_file_name, true_file_name, date):
    #rawファイルの読み込み
    dir_path =  os.path.join(os.path.dirname(__file__), "..", "..", "data")
    new_dir_path = os.path.join(dir_path, str(date))

    df_raw = pd.read_csv(os.path.join(new_dir_path, raw_file_name), skiprows=[0])
    df_filter = pd.read_csv(os.path.join(new_dir_path, filter_file_name))
    df_true = pd.read_csv(os.path.join(new_dir_path, true_file_name))
    raw_array = df_raw.to_numpy()
    filter_array = df_filter.to_numpy()
    true_array = df_true.to_numpy()
    print(df_raw)
    print(raw_array)
    return raw_array, filter_array, true_array 

def print_graph(array):
    fig, axs = plt.subplots(nrows=1, ncols=2, figsize=(12, 3))
    fig.suptitle('Raw Data', fontsize=16)
    index = [i for i in range(len(array[0]))]
    axs[0].scatter(index, array[0][:, 1], s=1, label="x")
    axs[0].set_ylabel("x")
    #axs[0].set_ylim(ylimit[0], ylimit[1])
    axs[1].scatter(index, array[0][:, 2], s=1, label="y")
    axs[1].set_ylabel("y")
    axs[1].set_xlabel("Time")

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
name = "test.csv"
date = datetime.date(2022, 11, 5)
array = import_data(name, name, name, date)
print_graph(array)