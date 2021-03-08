import sqlite3
from sqlite3 import Error
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import matplotlib.patches as mpatches

def create_connection(db_file):
    """ create a database connection to the SQLite database
        specified by the db_file
    :param db_file: database file
    :return: Connection object or None
    """
    conn = None
    try:
        conn = sqlite3.connect(db_file)
    except Error as e:
        print(e)

    return conn


def plot_status():
    '''
    UNKNOWN = 0,
    INVALID_START,
    INVALID_GOAL,
    UNRECOGNIZED_GOAL_TYPE,
    TIMEOUT,
    APPROXIMATE_SOLUTION,
    EXACT_SOLUTION,
    CRASH,
    ABORT,
    TYPE_COUNT
    '''

    RRTstar = []
    PRMstar = []
    LazyPRMstar = []
    RRTX = []
    FMTstar = []
    BITstar = []
    ABITstar = []
    CFOREST = []

    for i in range(0, 6, 1):
        database = "/home/ros2-foxy/ECMR2021/benchmark_results/DUBINS_" + str(
            i) + ".db"
        conn = create_connection(database)

        cur = conn.cursor()
        cur.execute(
            "SELECT plannerid, status FROM runs"
        )
        datas = cur.fetchall()

        for data in datas:
            data = list(data)
            if data[1] != None:
                if data[0] == 1:
                    RRTstar.append(data[1])
                if data[0] == 2:
                    PRMstar.append(data[1])
                if data[0] == 3:
                    LazyPRMstar.append(data[1])
                if data[0] == 4:
                    RRTX.append(data[1])
                if data[0] == 5:
                    FMTstar.append(data[1])
                if data[0] == 6:
                    BITstar.append(data[1])
                if data[0] == 7:
                    ABITstar.append(data[1])
                if data[0] == 8:
                    CFOREST.append(data[1])
                    
    unique, counts = np.unique(RRTstar, return_counts=True)
    print(dict(zip(unique, counts)))
    unique, counts = np.unique(PRMstar, return_counts=True)
    print(dict(zip(unique, counts)))
    unique, counts = np.unique(LazyPRMstar, return_counts=True)
    print(dict(zip(unique, counts)))    
    unique, counts = np.unique(RRTX, return_counts=True)
    print(dict(zip(unique, counts)))    
    unique, counts = np.unique(FMTstar, return_counts=True)
    print(dict(zip(unique, counts)))    
    unique, counts = np.unique(BITstar, return_counts=True)
    print(dict(zip(unique, counts)))    
    unique, counts = np.unique(ABITstar, return_counts=True)
    print(dict(zip(unique, counts)))     
    unique, counts = np.unique(CFOREST, return_counts=True)
    print(dict(zip(unique, counts)))     

def plot_normalized_path_lengths():

    GT = []
    RRTstar = []
    PRMstar = []
    LazyPRMstar = []
    RRTX = []
    FMTstar = []
    BITstar = []
    ABITstar = []
    CFOREST = []

    for i in range(0, 6, 1):
        database = "/home/ros2-foxy/ECMR2021/benchmark_results/DUBINS_" + str(
            i) + ".db"
        conn = create_connection(database)

        cur = conn.cursor()
        cur.execute(
            "SELECT plannerid, solution_length FROM runs"
        )
        datas = cur.fetchall()

        cur.execute("SELECT gt_path_length FROM experiments")
        gt_path_length = cur.fetchone()
        print(gt_path_length[0])
        gt_path_length = gt_path_length[0]

        normalization_value = 100
        normalization_coeff = normalization_value / gt_path_length
        gt_path_length = normalization_coeff * gt_path_length

        for data in datas:
            data = list(data)
            print(data[1])
            if data[1] != None:
                data[1] = normalization_coeff * data[1]
                if data[0] == 1:
                    RRTstar.append(data[1])
                if data[0] == 2:
                    PRMstar.append(data[1])
                if data[0] == 3:
                    LazyPRMstar.append(data[1])
                if data[0] == 4:
                    RRTX.append(data[1])
                if data[0] == 5:
                    FMTstar.append(data[1])
                if data[0] == 6:
                    BITstar.append(data[1])
                if data[0] == 7:
                    ABITstar.append(data[1])
                if data[0] == 8:
                    CFOREST.append(data[1])
                GT.append(gt_path_length)

    plot = [
        RRTstar, PRMstar, LazyPRMstar, RRTX, FMTstar, BITstar, ABITstar,
        CFOREST, GT
    ]

    fig1, ax1 = plt.subplots()
    ax1.set_title('Normalized Path Lengths')
    # x-axis labels
    ax1.set_xticklabels([
        'RRTstar', 'PRMstar', 'LazyPRMstar', ' RRTX', 'FMTstar', 'BITstar',
        'ABITstar', 'CFOREST', 'GT Length'
    ], fontsize=20)

    ax1.boxplot(plot, vert=1, patch_artist=True, showfliers=False)
    plt.grid()
    plt.show()

def plot_smoothness():

    GT = []
    RRTstar = []
    PRMstar = []
    LazyPRMstar = []
    RRTX = []
    FMTstar = []
    BITstar = []
    ABITstar = []
    CFOREST = []

    for i in range(0, 6, 1):
        database = "/home/ros2-foxy/ECMR2021/benchmark_results/DUBINS_" + str(
            i) + ".db"
        conn = create_connection(database)

        cur = conn.cursor()
        cur.execute(
            "SELECT plannerid, solution_smoothness FROM runs"
        )
        datas = cur.fetchall()

        cur.execute("SELECT gt_path_smoothness FROM experiments")
        gt_path_smoothness = cur.fetchone()
        gt_path_smoothness = gt_path_smoothness[0]

        normalization_value = 0.01
        normalization_coeff = normalization_value / gt_path_smoothness
        gt_path_smoothness = normalization_coeff * gt_path_smoothness
        print(gt_path_smoothness)

        for data in datas:
            data = list(data)
            if data[1] != None:
                data[1] = normalization_coeff * data[1]
                if data[0] == 1:
                    RRTstar.append(data[1])
                if data[0] == 2:
                    PRMstar.append(data[1])
                if data[0] == 3:
                    LazyPRMstar.append(data[1])
                if data[0] == 4:
                    RRTX.append(data[1])
                if data[0] == 5:
                    FMTstar.append(data[1])
                if data[0] == 6:
                    BITstar.append(data[1])
                if data[0] == 7:
                    ABITstar.append(data[1])
                if data[0] == 8:
                    CFOREST.append(data[1])
                GT.append(gt_path_smoothness)

    plot = [
        RRTstar, PRMstar, LazyPRMstar, RRTX, FMTstar, BITstar, ABITstar,
        CFOREST, GT
    ]

    fig1, ax1 = plt.subplots()
    ax1.set_title('Smoothness')
    # x-axis labels
    ax1.set_xticklabels([
        'RRTstar', 'PRMstar', 'LazyPRMstar', ' RRTX', 'FMTstar', 'BITstar',
        'ABITstar', 'CFOREST', 'GT Smoothness'
    ],fontsize=18)

    ax1.boxplot(plot, vert=1, patch_artist=True, showfliers=False)
    plt.grid()
    plt.show()


def normalize(value, min, max):
    return (value - min) / (max - min)


if __name__ == '__main__':
    #plot_normalized_path_lengths()
    #plot_status()
    plot_smoothness()