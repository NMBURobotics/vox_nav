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


def plot_status(space, epochs):
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
    # ALL SBO PLANNERS
    PRMstar = []
    LazyPRMstar = []
    RRTstar = []
    RRTsharp = []
    RRTXstatic = []
    InformedRRTstar = []
    BITstar = []
    ABITstar = []
    AITstar = []
    LBTRRT = []
    SST = []
    SPARS = []
    SPARStwo = []
    FMT = []
    CForest = []
    AnytimePathShortening = []

    for i in range(0, epochs, 1):
        database = "/home/ros2-foxy/ECMR2021/benchmark_results/" + space + "_" + str(
            i) + ".db"
        conn = create_connection(database)

        cur = conn.cursor()
        cur.execute("SELECT plannerid, status FROM runs")
        datas = cur.fetchall()

        for data in datas:
            data = list(data)
            if data[1] != None:
                if data[0] == 1:
                    PRMstar.append(data[1])
                if data[0] == 2:
                    LazyPRMstar.append(data[1])
                if data[0] == 3:
                    RRTstar.append(data[1])
                if data[0] == 4:
                    RRTsharp.append(data[1])
                if data[0] == 5:
                    RRTXstatic.append(data[1])
                if data[0] == 6:
                    InformedRRTstar.append(data[1])
                if data[0] == 7:
                    BITstar.append(data[1])
                if data[0] == 8:
                    ABITstar.append(data[1])
                if data[0] == 9:
                    AITstar.append(data[1])
                if data[0] == 10:
                    LBTRRT.append(data[1])
                if data[0] == 11:
                    SST.append(data[1])
                if data[0] == 12:
                    SPARS.append(data[1])
                if data[0] == 13:
                    SPARStwo.append(data[1])
                if data[0] == 14:
                    FMT.append(data[1])
                if data[0] == 15:
                    CForest.append(data[1])
                if data[0] == 16:
                    AnytimePathShortening.append(data[1])

    unique, counts = np.unique(PRMstar, return_counts=True)
    print("PRMstar", dict(zip(unique, counts)))
    unique, counts = np.unique(LazyPRMstar, return_counts=True)
    print("LazyPRMstar", dict(zip(unique, counts)))
    unique, counts = np.unique(RRTstar, return_counts=True)
    print("RRTstar", dict(zip(unique, counts)))
    unique, counts = np.unique(RRTsharp, return_counts=True)
    print("RRTsharp", dict(zip(unique, counts)))
    unique, counts = np.unique(RRTXstatic, return_counts=True)
    print("RRTXstatic", dict(zip(unique, counts)))
    unique, counts = np.unique(InformedRRTstar, return_counts=True)
    print("InformedRRTstar", dict(zip(unique, counts)))
    unique, counts = np.unique(BITstar, return_counts=True)
    print("BITstar", dict(zip(unique, counts)))
    unique, counts = np.unique(ABITstar, return_counts=True)
    print("ABITstar", dict(zip(unique, counts)))
    unique, counts = np.unique(AITstar, return_counts=True)
    print("AITstar", dict(zip(unique, counts)))
    unique, counts = np.unique(LBTRRT, return_counts=True)
    print("LBTRRT", dict(zip(unique, counts)))
    unique, counts = np.unique(SST, return_counts=True)
    print("SST", dict(zip(unique, counts)))
    unique, counts = np.unique(SPARS, return_counts=True)
    print("SPARS", dict(zip(unique, counts)))
    unique, counts = np.unique(SPARStwo, return_counts=True)
    print("SPARStwo", dict(zip(unique, counts)))
    unique, counts = np.unique(FMT, return_counts=True)
    print("FMT", dict(zip(unique, counts)))
    unique, counts = np.unique(CForest, return_counts=True)
    print("CForest", dict(zip(unique, counts)))
    unique, counts = np.unique(AnytimePathShortening, return_counts=True)
    print("AnytimePathShortening", dict(zip(unique, counts)))


def plot_normalized_path_lengths(space, epochs):

    # ALL SBO PLANNERS
    GT = []
    PRMstar = []
    LazyPRMstar = []
    RRTstar = []
    RRTsharp = []
    RRTXstatic = []
    InformedRRTstar = []
    BITstar = []
    ABITstar = []
    AITstar = []
    LBTRRT = []
    SST = []
    SPARS = []
    SPARStwo = []
    FMT = []
    CForest = []
    AnytimePathShortening = []

    for i in range(0, epochs, 1):
        database = "/home/ros2-foxy/ECMR2021/benchmark_results/" + space + "_" + str(
            i) + ".db"
        conn = create_connection(database)

        cur = conn.cursor()
        cur.execute("SELECT plannerid, solution_length FROM runs")
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
                    PRMstar.append(data[1])
                if data[0] == 2:
                    LazyPRMstar.append(data[1])
                if data[0] == 3:
                    RRTstar.append(data[1])
                if data[0] == 4:
                    RRTsharp.append(data[1])
                if data[0] == 5:
                    RRTXstatic.append(data[1])
                if data[0] == 6:
                    InformedRRTstar.append(data[1])
                if data[0] == 7:
                    BITstar.append(data[1])
                if data[0] == 8:
                    ABITstar.append(data[1])
                if data[0] == 9:
                    AITstar.append(data[1])
                if data[0] == 10:
                    LBTRRT.append(data[1])
                if data[0] == 11:
                    SST.append(data[1])
                if data[0] == 12:
                    SPARS.append(data[1])
                if data[0] == 13:
                    SPARStwo.append(data[1])
                if data[0] == 14:
                    FMT.append(data[1])
                if data[0] == 15:
                    CForest.append(data[1])
                if data[0] == 16:
                    AnytimePathShortening.append(data[1])
                GT.append(gt_path_length)

    plot = [
        GT, PRMstar, LazyPRMstar, RRTstar, RRTsharp, RRTXstatic,
        InformedRRTstar, BITstar, ABITstar, AITstar, LBTRRT, SST, SPARS,
        SPARStwo, FMT, CForest, AnytimePathShortening
    ]

    fig1, ax1 = plt.subplots()
    ax1.set_title('Normalized Path Lengths')
    # x-axis labels
    ax1.set_xticklabels([
        "GT", "PRMstar", "LazyPRMstar", "RRTstar", "RRTsharp", "RRTXstatic",
        "InformedRRTstar", "BITstar", "ABITstar", "AITstar", "LBTRRT", "SST",
        "SPARS", "SPARStwo", "FMT", "CForest", "AnytimePathShortening"
    ],
                        fontsize=11)

    ax1.boxplot(plot,
                vert=1,
                patch_artist=True,
                showfliers=False,
                showmeans=True)

    plt.rcParams.update({'font.size': 30})
    plt.xticks(rotation=90)
    plt.grid()
    plt.show()


def plot_smoothness(space, epochs):

    # ALL SBO PLANNERS
    GT = []
    PRMstar = []
    LazyPRMstar = []
    RRTstar = []
    RRTsharp = []
    RRTXstatic = []
    InformedRRTstar = []
    BITstar = []
    ABITstar = []
    AITstar = []
    LBTRRT = []
    SST = []
    SPARS = []
    SPARStwo = []
    FMT = []
    CForest = []
    AnytimePathShortening = []

    for i in range(0, epochs, 1):
        database = "/home/ros2-foxy/ECMR2021/benchmark_results/" + space + "_" + str(
            i) + ".db"
        conn = create_connection(database)

        cur = conn.cursor()
        cur.execute("SELECT plannerid, solution_smoothness FROM runs")
        datas = cur.fetchall()

        gt_path_smoothness = 1.0

        normalization_value = 0.0
        normalization_coeff = normalization_value / gt_path_smoothness
        gt_path_smoothness = normalization_coeff * gt_path_smoothness

        for data in datas:
            data = list(data)
            if data[1] != None:
                if data[0] == 1:
                    PRMstar.append(data[1])
                if data[0] == 2:
                    LazyPRMstar.append(data[1])
                if data[0] == 3:
                    RRTstar.append(data[1])
                if data[0] == 4:
                    RRTsharp.append(data[1])
                if data[0] == 5:
                    RRTXstatic.append(data[1])
                if data[0] == 6:
                    InformedRRTstar.append(data[1])
                if data[0] == 7:
                    BITstar.append(data[1])
                if data[0] == 8:
                    ABITstar.append(data[1])
                if data[0] == 9:
                    AITstar.append(data[1])
                if data[0] == 10:
                    LBTRRT.append(data[1])
                if data[0] == 11:
                    SST.append(data[1])
                if data[0] == 12:
                    SPARS.append(data[1])
                if data[0] == 13:
                    SPARStwo.append(data[1])
                if data[0] == 14:
                    FMT.append(data[1])
                if data[0] == 15:
                    CForest.append(data[1])
                if data[0] == 16:
                    AnytimePathShortening.append(data[1])
                GT.append(gt_path_smoothness)

    plot = [
        GT, PRMstar, LazyPRMstar, RRTstar, RRTsharp, RRTXstatic,
        InformedRRTstar, BITstar, ABITstar, AITstar, LBTRRT, SST, SPARS,
        SPARStwo, FMT, CForest, AnytimePathShortening
    ]

    fig1, ax1 = plt.subplots()
    ax1.set_title('SMoothness')
    # x-axis labels
    ax1.set_xticklabels([
        "GT", "PRMstar", "LazyPRMstar", "RRTstar", "RRTsharp", "RRTXstatic",
        "InformedRRTstar", "BITstar", "ABITstar", "AITstar", "LBTRRT", "SST",
        "SPARS", "SPARStwo", "FMT", "CForest", "AnytimePathShortening"
    ],
                        fontsize=11)

    ax1.boxplot(plot,
                vert=1,
                patch_artist=True,
                showfliers=False,
                showmeans=True)
    plt.xticks(rotation=90)
    plt.grid()
    plt.show()


def normalize(value, min, max):
    return (value - min) / (max - min)


if __name__ == '__main__':
    plot_normalized_path_lengths("SE3", 5)
    #plot_status("SE3", 5)
    #plot_smoothness("SE3", 5)
