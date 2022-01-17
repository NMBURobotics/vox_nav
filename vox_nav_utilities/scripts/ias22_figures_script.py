import sqlite3
from sqlite3 import Error
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from numpy.core.fromnumeric import mean
from numpy.ma.core import std
from scipy.stats import norm
from matplotlib import colors as mcolors
import math
import seaborn as sn
import pandas as pd


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


UNKNOWN = 0
INVALID_START = 1
INVALID_GOAL = 2
UNRECOGNIZED_GOAL_TYPE = 3
TIMEOUT = 4
APPROXIMATE_SOLUTION = 5
EXACT_SOLUTION = 6
CRASH = 7
ABORT = 8
TYPE_COUNT = 9


def plot_status():
    # ALL SBO PLANNERS
    PRMstar = []
    LazyPRMstar = []
    RRTstar = []
    RRTsharp = []
    RRTXstatic = []
    InfRRTstar = []
    BITstar = []
    ABITstar = []
    AITstar = []
    LBTRRT = []
    SST = []
    SPARS = []
    SPARStwo = []
    FMT = []
    CForest = []
    APS = []

    database = "/home/atas/test/benchmark.db"
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
                InfRRTstar.append(data[1])
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
                APS.append(data[1])
    unknown = []
    exact_sol = []
    apprx_sol = []
    timeout = []

    f = [
        PRMstar, LazyPRMstar, RRTstar, RRTsharp, RRTXstatic,
        InfRRTstar, BITstar, ABITstar, AITstar, LBTRRT, SST,
        SPARS, SPARStwo, FMT, CForest, APS
    ]

    x = [
        'PRMstar', 'LazyPRMstar', 'RRTstar', 'RRTsharp', 'RRTXstatic',
        'InfRRTstar', 'BITstar', 'ABITstar', 'AITstar', 'LBTRRT', 'SST',
        'SPARS', 'SPARStwo', 'FMT', 'CForest', 'APS'
    ]

    index = 0
    for t in x:
        unique, counts = np.unique(f[index], return_counts=True)
        D = dict(zip(unique, counts))
        print(t, D)
        unknown.append(D.get(UNKNOWN, 0))
        exact_sol.append(D.get(EXACT_SOLUTION, 0))
        apprx_sol.append(D.get(APPROXIMATE_SOLUTION, 0))
        timeout.append(D.get(TIMEOUT, 0))
        index += 1

    unknown = np.array(unknown)
    exact_sol = np.array(exact_sol)
    apprx_sol = np.array(apprx_sol)
    timeout = np.array(timeout)

    # plot bars in stack manner
    plt.bar(x, unknown, color='r')
    plt.bar(x, exact_sol, bottom=unknown, color='g')
    plt.bar(x, apprx_sol, bottom=unknown + exact_sol, color='y')
    plt.bar(x, timeout, bottom=unknown + exact_sol + apprx_sol, color='b')
    plt.xlabel("Planners")
    plt.ylabel("Total Number of Runs")
    plt.legend(["Unknown", "Exact Solution",
               "Approximate Solution", "Timeout"])
    plt.title(
        "Stacked bar plot representing status of solutions by evaluated planners")
    plt.show()


def plot_gaussian_length():
    # ALL SBO PLANNERS
    PRMstar = np.empty([], dtype=float)
    LazyPRMstar = np.empty([], dtype=float)
    RRTstar = np.empty([], dtype=float)
    RRTsharp = np.empty([], dtype=float)
    RRTXstatic = np.empty([], dtype=float)
    InfRRTstar = np.empty([], dtype=float)
    BITstar = np.empty([], dtype=float)
    ABITstar = np.empty([], dtype=float)
    AITstar = np.empty([], dtype=float)
    LBTRRT = np.empty([], dtype=float)
    SST = np.empty([], dtype=float)
    SPARS = np.empty([], dtype=float)
    SPARStwo = np.empty([], dtype=float)
    FMT = np.empty([], dtype=float)
    CForest = np.empty([], dtype=float)
    APS = np.empty([], dtype=float)

    database = "/home/atas/test/benchmark.db"
    conn = create_connection(database)

    cur = conn.cursor()
    cur.execute("SELECT plannerid, solution_length FROM runs")
    datas = cur.fetchall()

    for data in datas:
        data = list(data)
        if data[1] != None:
            if data[0] == 1:
                PRMstar = np.append(PRMstar, data[1])
            if data[0] == 2:
                LazyPRMstar = np.append(LazyPRMstar, data[1])
            if data[0] == 3:
                RRTstar = np.append(RRTstar, data[1])
            if data[0] == 4:
                RRTsharp = np.append(RRTsharp, data[1])
            if data[0] == 5:
                RRTXstatic = np.append(RRTXstatic, data[1])
            if data[0] == 6:
                InfRRTstar = np.append(InfRRTstar, data[1])
            if data[0] == 7:
                BITstar = np.append(BITstar, data[1])
            if data[0] == 8:
                ABITstar = np.append(ABITstar, data[1])
            if data[0] == 9:
                AITstar = np.append(AITstar, data[1])
            if data[0] == 10:
                LBTRRT = np.append(LBTRRT, data[1])
            if data[0] == 11:
                SST = np.append(SST, data[1])
            if data[0] == 12:
                SPARS = np.append(SPARS, data[1])
            if data[0] == 13:
                SPARStwo = np.append(SPARStwo, data[1])
            if data[0] == 14:
                FMT = np.append(FMT, data[1])
            if data[0] == 15:
                CForest = np.append(CForest, data[1])
            if data[0] == 16:
                APS = np.append(APS, data[1])

    x = [
        'PRMstar', 'LazyPRMstar', 'RRTstar', 'RRTsharp', 'RRTXstatic',
        'InfRRTstar', 'BITstar', 'ABITstar', 'AITstar', 'LBTRRT', 'SST',
        'SPARS', 'SPARStwo', 'CForest', 'APS'
    ]

    f = [
        PRMstar, LazyPRMstar, RRTstar, RRTsharp, RRTXstatic,
        InfRRTstar, BITstar, ABITstar, AITstar, LBTRRT, SST,
        SPARS, SPARStwo, CForest, APS
    ]

    means = []
    stds = []
    for t in f:
        means.append(np.mean(t))
        stds.append(np.std(t))

    f = np.array(f, dtype=object)
    x = np.array(x)
    means = np.array(means, dtype=np.float32)
    means_copy = means

    means, f = zip(*sorted(zip(means, f)))
    means_copy, x = zip(*sorted(zip(means_copy, x)))

    index = 0

    colors = dict(mcolors.BASE_COLORS, **mcolors.TABLEAU_COLORS,
                  **mcolors.CSS4_COLORS)

    for t in f:
        x_axis = np.arange(20, 120, 1)
        mean = round(np.mean(t), 3)
        std = round(np.std(t), 3)

        legend_label = "{}, μ={}, σ={}".format(x[index],
                                               mean, std)
        color_index = index
        if color_index == 7:  # This is white color so skip it as its not visible
            color_index += 1

        plt.plot(x_axis, norm.pdf(x_axis, mean, std),
                 label=legend_label, marker=index % 11, color=list(colors.values())[color_index])
        plt.legend(loc="best", prop={'size': 16})

        plt.title(
            "Normal Distribution of Path Lenghts, Sorted by Lowest Mean(μ) Values", fontsize=18)
        plt.xlabel("Path Length", fontsize=18)
        plt.ylabel("Probablity Density", fontsize=18)
        index += 1
    plt.show()


def plot_gaussian_smoothness():
    # ALL SBO PLANNERS
    PRMstar = np.empty([], dtype=float)
    LazyPRMstar = np.empty([], dtype=float)
    RRTstar = np.empty([], dtype=float)
    RRTsharp = np.empty([], dtype=float)
    RRTXstatic = np.empty([], dtype=float)
    InfRRTstar = np.empty([], dtype=float)
    BITstar = np.empty([], dtype=float)
    ABITstar = np.empty([], dtype=float)
    AITstar = np.empty([], dtype=float)
    LBTRRT = np.empty([], dtype=float)
    SST = np.empty([], dtype=float)
    SPARS = np.empty([], dtype=float)
    SPARStwo = np.empty([], dtype=float)
    FMT = np.empty([], dtype=float)
    CForest = np.empty([], dtype=float)
    APS = np.empty([], dtype=float)

    database = "/home/atas/test/benchmark.db"
    conn = create_connection(database)

    cur = conn.cursor()
    cur.execute("SELECT plannerid, solution_smoothness FROM runs")
    datas = cur.fetchall()

    for data in datas:
        data = list(data)
        if data[1] != None:
            if data[0] == 1:
                PRMstar = np.append(PRMstar, data[1])
            if data[0] == 2:
                LazyPRMstar = np.append(LazyPRMstar, data[1])
            if data[0] == 3:
                RRTstar = np.append(RRTstar, data[1])
            if data[0] == 4:
                RRTsharp = np.append(RRTsharp, data[1])
            if data[0] == 5:
                RRTXstatic = np.append(RRTXstatic, data[1])
            if data[0] == 6:
                InfRRTstar = np.append(InfRRTstar, data[1])
            if data[0] == 7:
                BITstar = np.append(BITstar, data[1])
            if data[0] == 8:
                ABITstar = np.append(ABITstar, data[1])
            if data[0] == 9:
                AITstar = np.append(AITstar, data[1])
            if data[0] == 10:
                LBTRRT = np.append(LBTRRT, data[1])
            if data[0] == 11:
                SST = np.append(SST, data[1])
            if data[0] == 12:
                SPARS = np.append(SPARS, data[1])
            if data[0] == 13:
                SPARStwo = np.append(SPARStwo, data[1])
            if data[0] == 14:
                FMT = np.append(FMT, data[1])
            if data[0] == 15:
                CForest = np.append(CForest, data[1])
            if data[0] == 16:
                APS = np.append(APS, data[1])

    x = [
        'PRMstar', 'LazyPRMstar', 'RRTstar', 'RRTsharp', 'RRTXstatic',
        'InfRRTstar', 'BITstar', 'ABITstar', 'AITstar', 'LBTRRT', 'SST',
        'SPARS', 'SPARStwo', 'CForest', 'APS'
    ]

    f = [
        PRMstar, LazyPRMstar, RRTstar, RRTsharp, RRTXstatic,
        InfRRTstar, BITstar, ABITstar, AITstar, LBTRRT, SST,
        SPARS, SPARStwo, CForest, APS
    ]

    means = []
    stds = []
    for t in f:
        means.append(np.mean(t))
        stds.append(np.std(t))

    f = np.array(f, dtype=object)
    x = np.array(x)
    means = np.array(means, dtype=np.float32)
    means_copy = means

    means, f = zip(*sorted(zip(means, f)))
    means_copy, x = zip(*sorted(zip(means_copy, x)))

    index = 0

    colors = dict(mcolors.BASE_COLORS, **mcolors.TABLEAU_COLORS,
                  **mcolors.CSS4_COLORS)

    for t in f:
        x_axis = np.arange(-0.1, 0.1, 0.001)
        mean = round(np.mean(t), 3)
        std = round(np.std(t), 3)

        legend_label = "{}, μ={}, σ={}".format(x[index],
                                               mean, std)
        color_index = index
        if color_index == 7:  # This is white color so skip it as its not visible
            color_index += 1

        plt.plot(x_axis, norm.pdf(x_axis, mean, std),
                 label=legend_label, marker=index % 11, color=list(colors.values())[color_index])
        plt.legend(loc="best", prop={'size': 16})
        plt.title(
            "Normal Distribution of Path Smoothness, Sorted by Lowest Mean(μ) Values")
        plt.xlabel("Path Smoothness")
        plt.ylabel("Probablity Density")
        index += 1
    plt.show()


def plot_cdf_length():
    # ALL SBO PLANNERS
    PRMstar = np.empty([], dtype=float)
    LazyPRMstar = np.empty([], dtype=float)
    RRTstar = np.empty([], dtype=float)
    RRTsharp = np.empty([], dtype=float)
    RRTXstatic = np.empty([], dtype=float)
    InfRRTstar = np.empty([], dtype=float)
    BITstar = np.empty([], dtype=float)
    ABITstar = np.empty([], dtype=float)
    AITstar = np.empty([], dtype=float)
    LBTRRT = np.empty([], dtype=float)
    SST = np.empty([], dtype=float)
    SPARS = np.empty([], dtype=float)
    SPARStwo = np.empty([], dtype=float)
    FMT = np.empty([], dtype=float)
    CForest = np.empty([], dtype=float)
    APS = np.empty([], dtype=float)

    database = "/home/atas/test/benchmark.db"
    conn = create_connection(database)

    cur = conn.cursor()
    cur.execute("SELECT plannerid, solution_length FROM runs")
    datas = cur.fetchall()

    for data in datas:
        data = list(data)
        if data[1] != None:
            if data[0] == 1:
                PRMstar = np.append(PRMstar, data[1])
            if data[0] == 2:
                LazyPRMstar = np.append(LazyPRMstar, data[1])
            if data[0] == 3:
                RRTstar = np.append(RRTstar, data[1])
            if data[0] == 4:
                RRTsharp = np.append(RRTsharp, data[1])
            if data[0] == 5:
                RRTXstatic = np.append(RRTXstatic, data[1])
            if data[0] == 6:
                InfRRTstar = np.append(InfRRTstar, data[1])
            if data[0] == 7:
                BITstar = np.append(BITstar, data[1])
            if data[0] == 8:
                ABITstar = np.append(ABITstar, data[1])
            if data[0] == 9:
                AITstar = np.append(AITstar, data[1])
            if data[0] == 10:
                LBTRRT = np.append(LBTRRT, data[1])
            if data[0] == 11:
                SST = np.append(SST, data[1])
            if data[0] == 12:
                SPARS = np.append(SPARS, data[1])
            if data[0] == 13:
                SPARStwo = np.append(SPARStwo, data[1])
            if data[0] == 14:
                FMT = np.append(FMT, data[1])
            if data[0] == 15:
                CForest = np.append(CForest, data[1])
            if data[0] == 16:
                APS = np.append(APS, data[1])

    planners = [
        'PRMstar', 'LazyPRMstar', 'RRTstar', 'RRTsharp', 'RRTXstatic',
        'InfRRTstar', 'BITstar', 'ABITstar', 'AITstar', 'LBTRRT', 'SST',
        'SPARS', 'SPARStwo', 'CForest', 'APS'
    ]

    f = [
        PRMstar, LazyPRMstar, RRTstar, RRTsharp, RRTXstatic,
        InfRRTstar, BITstar, ABITstar, AITstar, LBTRRT, SST,
        SPARS, SPARStwo, CForest, APS
    ]

    means = []
    for t in f:
        means.append(np.mean(t))

    f = np.array(f, dtype=object)
    means = np.array(means)
    planners = np.array(planners)
    means_copy = means

    means, f = zip(*sorted(zip(means, f)))
    means_copy, planners = zip(*sorted(zip(means_copy, planners)))

    colors = dict(mcolors.BASE_COLORS, **mcolors.TABLEAU_COLORS,
                  **mcolors.CSS4_COLORS)
    index = 0

    for t in f:
        x = np.sort(t)
        # get the cdf values of y
        y = np.arange(len(t)) / float(len(t))
        mean = round(np.mean(t), 3)
        std = round(np.std(t), 3)

        color_index = index
        if color_index == 7:  # This is white color so skip it as its not visible
            color_index += 1

        legend_label = "{}, μ={}, σ={}".format(planners[index],
                                               mean, std)

        # plotting
        plt.title(
            "Cumulative Distribution of Path Lengths, Sorted by Lowest Mean Values")
        plt.xlabel("Path Length")
        plt.ylabel("Probablity")

        plt.plot(x, y,  label=legend_label, marker=index % 11,
                 color=list(colors.values())[color_index])
        index += 1

        plt.legend(loc="best")

    plt.show()


def plot_cdf_smoothness():
    # ALL SBO PLANNERS
    PRMstar = np.empty([], dtype=float)
    LazyPRMstar = np.empty([], dtype=float)
    RRTstar = np.empty([], dtype=float)
    RRTsharp = np.empty([], dtype=float)
    RRTXstatic = np.empty([], dtype=float)
    InfRRTstar = np.empty([], dtype=float)
    BITstar = np.empty([], dtype=float)
    ABITstar = np.empty([], dtype=float)
    AITstar = np.empty([], dtype=float)
    LBTRRT = np.empty([], dtype=float)
    SST = np.empty([], dtype=float)
    SPARS = np.empty([], dtype=float)
    SPARStwo = np.empty([], dtype=float)
    FMT = np.empty([], dtype=float)
    CForest = np.empty([], dtype=float)
    APS = np.empty([], dtype=float)

    database = "/home/atas/test/benchmark.db"
    conn = create_connection(database)

    cur = conn.cursor()
    cur.execute("SELECT plannerid, solution_smoothness FROM runs")
    datas = cur.fetchall()

    for data in datas:
        data = list(data)
        if data[1] != None:
            if data[0] == 1:
                PRMstar = np.append(PRMstar, data[1])
            if data[0] == 2:
                LazyPRMstar = np.append(LazyPRMstar, data[1])
            if data[0] == 3:
                RRTstar = np.append(RRTstar, data[1])
            if data[0] == 4:
                RRTsharp = np.append(RRTsharp, data[1])
            if data[0] == 5:
                RRTXstatic = np.append(RRTXstatic, data[1])
            if data[0] == 6:
                InfRRTstar = np.append(InfRRTstar, data[1])
            if data[0] == 7:
                BITstar = np.append(BITstar, data[1])
            if data[0] == 8:
                ABITstar = np.append(ABITstar, data[1])
            if data[0] == 9:
                AITstar = np.append(AITstar, data[1])
            if data[0] == 10:
                LBTRRT = np.append(LBTRRT, data[1])
            if data[0] == 11:
                SST = np.append(SST, data[1])
            if data[0] == 12:
                SPARS = np.append(SPARS, data[1])
            if data[0] == 13:
                SPARStwo = np.append(SPARStwo, data[1])
            if data[0] == 14:
                FMT = np.append(FMT, data[1])
            if data[0] == 15:
                CForest = np.append(CForest, data[1])
            if data[0] == 16:
                APS = np.append(APS, data[1])

    planners = ['PRMstar', 'LazyPRMstar', 'RRTstar', 'RRTsharp', 'RRTXstatic',
                'InfRRTstar', 'BITstar', 'ABITstar', 'AITstar', 'LBTRRT', 'SST',
                'SPARS', 'SPARStwo', 'CForest', 'APS']

    f = [PRMstar, LazyPRMstar, RRTstar, RRTsharp, RRTXstatic,
         InfRRTstar, BITstar, ABITstar, AITstar, LBTRRT, SST,
         SPARS, SPARStwo, CForest, APS]

    means = []
    for t in f:
        means.append(np.mean(t))

    f = np.array(f, dtype=object)
    means = np.array(means)
    planners = np.array(planners)
    means_copy = means

    means, f = zip(*sorted(zip(means, f)))
    means_copy, planners = zip(*sorted(zip(means_copy, planners)))

    colors = dict(mcolors.BASE_COLORS,
                  **mcolors.TABLEAU_COLORS,
                  **mcolors.CSS4_COLORS)
    index = 0

    for t in f:
        x = np.sort(t)
        print(x)
        y = np.arange(len(t)) / float(len(t))
        mean = round(np.mean(t), 3)
        std = round(np.std(t), 3)
        color_index = index

        # This is white color so skip it as its not visible
        if color_index == 7:
            color_index += 1

        legend_label = "{}, μ={}, σ={}".format(planners[index], mean, std)

        plt.title(
            "Cumulative Distribution of Path Smoothness, Sorted by Lowest Mean Values", fontsize=18)
        plt.xlabel("Path Smoothness", fontsize=18)
        plt.ylabel("Probablity", fontsize=18)

        plt.plot(x, y, label=legend_label, marker=index % 11,
                 color=list(colors.values())[color_index])
        index += 1
        plt.legend(loc="best", prop={'size': 16})
    plt.xlim([-0.01, 0.8])

    plt.show()


def plot_cdf_best_cost():
    # ALL SBO PLANNERS
    PRMstar = np.empty([], dtype=float)
    LazyPRMstar = np.empty([], dtype=float)
    RRTstar = np.empty([], dtype=float)
    RRTsharp = np.empty([], dtype=float)
    RRTXstatic = np.empty([], dtype=float)
    InfRRTstar = np.empty([], dtype=float)
    BITstar = np.empty([], dtype=float)
    ABITstar = np.empty([], dtype=float)
    AITstar = np.empty([], dtype=float)
    LBTRRT = np.empty([], dtype=float)
    SST = np.empty([], dtype=float)
    SPARS = np.empty([], dtype=float)
    SPARStwo = np.empty([], dtype=float)
    FMT = np.empty([], dtype=float)
    CForest = np.empty([], dtype=float)
    APS = np.empty([], dtype=float)

    database = "/home/atas/test/benchmark.db"
    conn = create_connection(database)

    cur = conn.cursor()
    cur.execute("SELECT plannerid, memory FROM runs")
    datas = cur.fetchall()

    for data in datas:
        data = list(data)
        if data[1] != None:
            if data[0] == 1:
                PRMstar = np.append(PRMstar, data[1])
            if data[0] == 2:
                LazyPRMstar = np.append(LazyPRMstar, data[1])
            if data[0] == 3:
                RRTstar = np.append(RRTstar, data[1])
            if data[0] == 4:
                RRTsharp = np.append(RRTsharp, data[1])
            if data[0] == 5:
                RRTXstatic = np.append(RRTXstatic, data[1])
            if data[0] == 6:
                InfRRTstar = np.append(InfRRTstar, data[1])
            if data[0] == 7:
                BITstar = np.append(BITstar, data[1])
            if data[0] == 8:
                ABITstar = np.append(ABITstar, data[1])
            if data[0] == 9:
                AITstar = np.append(AITstar, data[1])
            if data[0] == 10:
                LBTRRT = np.append(LBTRRT, data[1])
            if data[0] == 11:
                SST = np.append(SST, data[1])
            if data[0] == 12:
                SPARS = np.append(SPARS, data[1])
            if data[0] == 13:
                SPARStwo = np.append(SPARStwo, data[1])
            if data[0] == 14:
                FMT = np.append(FMT, data[1])
            if data[0] == 15:
                CForest = np.append(CForest, data[1])
            if data[0] == 16:
                APS = np.append(APS, data[1])

    planners = ['PRMstar', 'LazyPRMstar', 'RRTstar', 'RRTsharp', 'RRTXstatic',
                'InfRRTstar', 'BITstar', 'ABITstar', 'AITstar', 'LBTRRT', 'SST',
                'SPARS', 'SPARStwo', 'CForest', 'APS']

    f = [PRMstar, LazyPRMstar, RRTstar, RRTsharp, RRTXstatic,
         InfRRTstar, BITstar, ABITstar, AITstar, LBTRRT, SST,
         SPARS, SPARStwo, CForest, APS]

    means = []
    for t in f:
        j = np.mean(t)
        if j < 0.1:
            j = 0
        means.append(j)
        print(j)

    f = np.asarray(f, dtype=object)
    means = np.asarray(means)
    planners = np.asarray(planners)
    means_copy = means

    means, f = zip(*sorted(zip(means, f)))
    means_copy, planners = zip(*sorted(zip(means_copy, planners)))

    colors = dict(mcolors.BASE_COLORS,
                  **mcolors.TABLEAU_COLORS,
                  **mcolors.CSS4_COLORS)
    index = 0

    for t in f:
        x_axis = np.arange(0, 2000, 5)
        mean = round(np.mean(t), 3)
        std = round(np.std(t), 3)

        legend_label = "{}, μ={}, σ={}".format(planners[index],
                                               mean, std)
        color_index = index
        if color_index == 7:  # This is white color so skip it as its not visible
            color_index += 1

        plt.plot(x_axis, norm.pdf(x_axis, mean, std),
                 label=legend_label, marker=index % 11, color=list(colors.values())[color_index])
        plt.legend(loc="best")
        plt.title(
            "Normal Distribution of Path Smoothness, Sorted by Lowest Mean(μ) Values")
        plt.xlabel("Path Smoothness")
        plt.ylabel("Probablity Density")
        index += 1
    plt.show()


def plot_ztest_length():
    # ALL SBO PLANNERS
    PRMstar = np.empty([], dtype=float)
    LazyPRMstar = np.empty([], dtype=float)
    RRTstar = np.empty([], dtype=float)
    RRTsharp = np.empty([], dtype=float)
    RRTXstatic = np.empty([], dtype=float)
    InfRRTstar = np.empty([], dtype=float)
    BITstar = np.empty([], dtype=float)
    ABITstar = np.empty([], dtype=float)
    AITstar = np.empty([], dtype=float)
    LBTRRT = np.empty([], dtype=float)
    SST = np.empty([], dtype=float)
    SPARS = np.empty([], dtype=float)
    SPARStwo = np.empty([], dtype=float)
    FMT = np.empty([], dtype=float)
    CForest = np.empty([], dtype=float)
    APS = np.empty([], dtype=float)

    database = "/home/atas/test/benchmark.db"
    conn = create_connection(database)

    cur = conn.cursor()
    cur.execute("SELECT plannerid, solution_length FROM runs")
    datas = cur.fetchall()

    for data in datas:
        data = list(data)
        if data[1] != None:
            if data[0] == 1:
                PRMstar = np.append(PRMstar, data[1])
            if data[0] == 2:
                LazyPRMstar = np.append(LazyPRMstar, data[1])
            if data[0] == 3:
                RRTstar = np.append(RRTstar, data[1])
            if data[0] == 4:
                RRTsharp = np.append(RRTsharp, data[1])
            if data[0] == 5:
                RRTXstatic = np.append(RRTXstatic, data[1])
            if data[0] == 6:
                InfRRTstar = np.append(InfRRTstar, data[1])
            if data[0] == 7:
                BITstar = np.append(BITstar, data[1])
            if data[0] == 8:
                ABITstar = np.append(ABITstar, data[1])
            if data[0] == 9:
                AITstar = np.append(AITstar, data[1])
            if data[0] == 10:
                LBTRRT = np.append(LBTRRT, data[1])
            if data[0] == 11:
                SST = np.append(SST, data[1])
            if data[0] == 12:
                SPARS = np.append(SPARS, data[1])
            if data[0] == 13:
                SPARStwo = np.append(SPARStwo, data[1])
            if data[0] == 14:
                FMT = np.append(FMT, data[1])
            if data[0] == 15:
                CForest = np.append(CForest, data[1])
            if data[0] == 16:
                APS = np.append(APS, data[1])

    planners = [
        'PRMstar', 'LazyPRMstar', 'RRTstar', 'RRTsharp', 'RRTXstatic',
        'InfRRTstar', 'BITstar', 'ABITstar', 'AITstar', 'LBTRRT', 'SST',
        'SPARS', 'SPARStwo', 'CForest', 'APS'
    ]

    f = [
        PRMstar, LazyPRMstar, RRTstar, RRTsharp, RRTXstatic,
        InfRRTstar, BITstar, ABITstar, AITstar, LBTRRT, SST,
        SPARS, SPARStwo, CForest, APS
    ]

    means = []
    stds = []
    Ns = []
    for t in f:
        means.append(np.mean(t))
        stds.append(np.std(t))
        Ns.append(len(t))

    f = np.asarray(f, dtype=object)
    means = np.asarray(means)
    planners = np.asarray(planners)
    means_copy = means
    means_copy1 = means
    means_copy2 = means

    # means, f = zip(*sorted(zip(means, f)))
    # means_copy, stds = zip(*sorted(zip(means_copy, stds)))
    # means_copy1, Ns = zip(*sorted(zip(means_copy1, Ns)))
    # means_copy2, planners = zip(*sorted(zip(means_copy2, planners)))

    M = []

    for i in range(0, len(means)):
        K = []
        for j in range(0, len(means)):
            sigma_1 = (stds[i] / Ns[i]) ** 2
            sigma_2 = (stds[j] / Ns[j]) ** 2
            Z = (means[i] - means[j]) / math.sqrt(sigma_1 + sigma_2)
            K.append(Z)

        M.append(K)

    M = np.asarray(M)
    M = np.interp(M, (M.min(), M.max()), (-1, 1))
    overall_score = np.round(M.sum(axis=1), 1)

    score_with_planner = []
    for k in range(len(overall_score)):
        string = planners[k] + ": " + str(overall_score[k])
        score_with_planner.append(string)

    df_cm = pd.DataFrame(M, index=[i for i in score_with_planner],
                         columns=[i for i in planners])

    plt.figure(figsize=(7, 7))
    sn.heatmap(df_cm, annot=True, robust=False)

    plt.title(
        "Normalized Z values of one-by-one comparison, This matrix is acquired from distributions given for path lengths", fontsize=18)
    plt.xlabel("Planner", fontsize=18)
    plt.ylabel("Planner", fontsize=18)
    plt.xticks(rotation=45, fontsize=18)
    plt.yticks(rotation=45, fontsize=18)

    plt.show()


def plot_ztest_smooth():
    # ALL SBO PLANNERS
    PRMstar = np.empty([], dtype=float)
    LazyPRMstar = np.empty([], dtype=float)
    RRTstar = np.empty([], dtype=float)
    RRTsharp = np.empty([], dtype=float)
    RRTXstatic = np.empty([], dtype=float)
    InfRRTstar = np.empty([], dtype=float)
    BITstar = np.empty([], dtype=float)
    ABITstar = np.empty([], dtype=float)
    AITstar = np.empty([], dtype=float)
    LBTRRT = np.empty([], dtype=float)
    SST = np.empty([], dtype=float)
    SPARS = np.empty([], dtype=float)
    SPARStwo = np.empty([], dtype=float)
    FMT = np.empty([], dtype=float)
    CForest = np.empty([], dtype=float)
    APS = np.empty([], dtype=float)

    database = "/home/atas/test/benchmark.db"
    conn = create_connection(database)

    cur = conn.cursor()
    cur.execute("SELECT plannerid, solution_smoothness FROM runs")
    datas = cur.fetchall()

    for data in datas:
        data = list(data)
        if data[1] != None:
            if data[0] == 1:
                PRMstar = np.append(PRMstar, data[1])
            if data[0] == 2:
                LazyPRMstar = np.append(LazyPRMstar, data[1])
            if data[0] == 3:
                RRTstar = np.append(RRTstar, data[1])
            if data[0] == 4:
                RRTsharp = np.append(RRTsharp, data[1])
            if data[0] == 5:
                RRTXstatic = np.append(RRTXstatic, data[1])
            if data[0] == 6:
                InfRRTstar = np.append(InfRRTstar, data[1])
            if data[0] == 7:
                BITstar = np.append(BITstar, data[1])
            if data[0] == 8:
                ABITstar = np.append(ABITstar, data[1])
            if data[0] == 9:
                AITstar = np.append(AITstar, data[1])
            if data[0] == 10:
                LBTRRT = np.append(LBTRRT, data[1])
            if data[0] == 11:
                SST = np.append(SST, data[1])
            if data[0] == 12:
                SPARS = np.append(SPARS, data[1])
            if data[0] == 13:
                SPARStwo = np.append(SPARStwo, data[1])
            if data[0] == 14:
                FMT = np.append(FMT, data[1])
            if data[0] == 15:
                CForest = np.append(CForest, data[1])
            if data[0] == 16:
                APS = np.append(APS, data[1])

    planners = [
        'PRMstar', 'LazyPRMstar', 'RRTstar', 'RRTsharp', 'RRTXstatic',
        'InfRRTstar', 'BITstar', 'ABITstar', 'AITstar', 'LBTRRT', 'SST',
        'SPARS', 'SPARStwo', 'CForest', 'APS'
    ]

    f = [
        PRMstar, LazyPRMstar, RRTstar, RRTsharp, RRTXstatic,
        InfRRTstar, BITstar, ABITstar, AITstar, LBTRRT, SST,
        SPARS, SPARStwo, CForest, APS
    ]

    means = []
    stds = []
    Ns = []
    for t in f:
        means.append(np.mean(t))
        stds.append(np.std(t))
        Ns.append(len(t))

    f = np.asarray(f, dtype=object)
    means = np.asarray(means)
    planners = np.asarray(planners)

    M = []

    for i in range(0, len(means)):
        K = []
        for j in range(0, len(means)):
            sigma_1 = (stds[i] / Ns[i]) ** 2
            sigma_2 = (stds[j] / Ns[j]) ** 2
            Z = (means[i] - means[j]) / math.sqrt(sigma_1 + sigma_2)
            K.append(Z)

        M.append(K)

    M = np.asarray(M)
    M = np.interp(M, (M.min(), M.max()), (-1, 1))

    print(M.sum(axis=0))

    df_cm = pd.DataFrame(M, index=[i for i in planners],
                         columns=[i for i in planners])

    plt.figure(figsize=(10, 7))
    sn.heatmap(df_cm, annot=True)
    plt.show()


def score():
    # ALL SBO PLANNERS
    PRMstar_length = np.empty([], dtype=float)
    LazyPRMstar_length = np.empty([], dtype=float)
    RRTstar_length = np.empty([], dtype=float)
    RRTsharp_length = np.empty([], dtype=float)
    RRTXstatic_length = np.empty([], dtype=float)
    InfRRTstar_length = np.empty([], dtype=float)
    BITstar_length = np.empty([], dtype=float)
    ABITstar_length = np.empty([], dtype=float)
    AITstar_length = np.empty([], dtype=float)
    LBTRRT_length = np.empty([], dtype=float)
    SST_length = np.empty([], dtype=float)
    SPARS_length = np.empty([], dtype=float)
    SPARStwo_length = np.empty([], dtype=float)
    FMT_length = np.empty([], dtype=float)
    CForest_length = np.empty([], dtype=float)
    APS_length = np.empty([], dtype=float)

    PRMstar_smooth = np.empty([], dtype=float)
    LazyPRMstar_smooth = np.empty([], dtype=float)
    RRTstar_smooth = np.empty([], dtype=float)
    RRTsharp_smooth = np.empty([], dtype=float)
    RRTXstatic_smooth = np.empty([], dtype=float)
    InfRRTstar_smooth = np.empty([], dtype=float)
    BITstar_smooth = np.empty([], dtype=float)
    ABITstar_smooth = np.empty([], dtype=float)
    AITstar_smooth = np.empty([], dtype=float)
    LBTRRT_smooth = np.empty([], dtype=float)
    SST_smooth = np.empty([], dtype=float)
    SPARS_smooth = np.empty([], dtype=float)
    SPARStwo_smooth = np.empty([], dtype=float)
    FMT_smooth = np.empty([], dtype=float)
    CForest_smooth = np.empty([], dtype=float)
    APS_smooth = np.empty([], dtype=float)

    database = "/home/atas/test/benchmark.db"
    conn = create_connection(database)

    length_connection = conn.cursor()
    length_connection.execute("SELECT plannerid, solution_length FROM runs")
    length_datas = length_connection.fetchall()

    for data in length_datas:
        data = list(data)
        if data[1] != None:
            if data[0] == 1:
                PRMstar_length = np.append(PRMstar_length, data[1])
            if data[0] == 2:
                LazyPRMstar_length = np.append(LazyPRMstar_length, data[1])
            if data[0] == 3:
                RRTstar_length = np.append(RRTstar_length, data[1])
            if data[0] == 4:
                RRTsharp_length = np.append(RRTsharp_length, data[1])
            if data[0] == 5:
                RRTXstatic_length = np.append(RRTXstatic_length, data[1])
            if data[0] == 6:
                InfRRTstar_length = np.append(InfRRTstar_length, data[1])
            if data[0] == 7:
                BITstar_length = np.append(BITstar_length, data[1])
            if data[0] == 8:
                ABITstar_length = np.append(ABITstar_length, data[1])
            if data[0] == 9:
                AITstar_length = np.append(AITstar_length, data[1])
            if data[0] == 10:
                LBTRRT_length = np.append(LBTRRT_length, data[1])
            if data[0] == 11:
                SST_length = np.append(SST_length, data[1])
            if data[0] == 12:
                SPARS_length = np.append(SPARS_length, data[1])
            if data[0] == 13:
                SPARStwo_length = np.append(SPARStwo_length, data[1])
            if data[0] == 14:
                FMT_length = np.append(FMT_length, data[1])
            if data[0] == 15:
                CForest_length = np.append(CForest_length, data[1])
            if data[0] == 16:
                APS_length = np.append(APS_length, data[1])

    smooth_connection = conn.cursor()
    smooth_connection.execute(
        "SELECT plannerid, solution_smoothness FROM runs")
    smooth_datas = smooth_connection.fetchall()

    for data in smooth_datas:
        data = list(data)
        if data[1] != None:
            if data[0] == 1:
                PRMstar_smooth = np.append(PRMstar_smooth, data[1])
            if data[0] == 2:
                LazyPRMstar_smooth = np.append(LazyPRMstar_smooth, data[1])
            if data[0] == 3:
                RRTstar_smooth = np.append(RRTstar_smooth, data[1])
            if data[0] == 4:
                RRTsharp_smooth = np.append(RRTsharp_smooth, data[1])
            if data[0] == 5:
                RRTXstatic_smooth = np.append(RRTXstatic_smooth, data[1])
            if data[0] == 6:
                InfRRTstar_smooth = np.append(InfRRTstar_smooth, data[1])
            if data[0] == 7:
                BITstar_smooth = np.append(BITstar_smooth, data[1])
            if data[0] == 8:
                ABITstar_smooth = np.append(ABITstar_smooth, data[1])
            if data[0] == 9:
                AITstar_smooth = np.append(AITstar_smooth, data[1])
            if data[0] == 10:
                LBTRRT_smooth = np.append(LBTRRT_smooth, data[1])
            if data[0] == 11:
                SST_smooth = np.append(SST_smooth, data[1])
            if data[0] == 12:
                SPARS_smooth = np.append(SPARS_smooth, data[1])
            if data[0] == 13:
                SPARStwo_smooth = np.append(SPARStwo_smooth, data[1])
            if data[0] == 14:
                FMT_smooth = np.append(FMT_smooth, data[1])
            if data[0] == 15:
                CForest_smooth = np.append(CForest_smooth, data[1])
            if data[0] == 16:
                APS_smooth = np.append(APS_smooth, data[1])

    PRMstar = np.empty([], dtype=float)
    LazyPRMstar = np.empty([], dtype=float)
    RRTstar = np.empty([], dtype=float)
    RRTsharp = np.empty([], dtype=float)
    RRTXstatic = np.empty([], dtype=float)
    InfRRTstar = np.empty([], dtype=float)
    BITstar = np.empty([], dtype=float)
    ABITstar = np.empty([], dtype=float)
    AITstar = np.empty([], dtype=float)
    LBTRRT = np.empty([], dtype=float)
    SST = np.empty([], dtype=float)
    SPARS = np.empty([], dtype=float)
    SPARStwo = np.empty([], dtype=float)
    FMT = np.empty([], dtype=float)
    CForest = np.empty([], dtype=float)
    APS = np.empty([], dtype=float)

    cur2 = conn.cursor()
    cur2.execute("SELECT plannerid, avg_solved FROM bestPlannerConfigs")
    datas = cur2.fetchall()

    for data in datas:
        data = list(data)
        if data[1] != None:
            if data[0] == 1:
                PRMstar = np.append(PRMstar, data[1])
            if data[0] == 2:
                LazyPRMstar = np.append(LazyPRMstar, data[1])
            if data[0] == 3:
                RRTstar = np.append(RRTstar, data[1])
            if data[0] == 4:
                RRTsharp = np.append(RRTsharp, data[1])
            if data[0] == 5:
                RRTXstatic = np.append(RRTXstatic, data[1])
            if data[0] == 6:
                InfRRTstar = np.append(InfRRTstar, data[1])
            if data[0] == 7:
                BITstar = np.append(BITstar, data[1])
            if data[0] == 8:
                ABITstar = np.append(ABITstar, data[1])
            if data[0] == 9:
                AITstar = np.append(AITstar, data[1])
            if data[0] == 10:
                LBTRRT = np.append(LBTRRT, data[1])
            if data[0] == 11:
                SST = np.append(SST, data[1])
            if data[0] == 12:
                SPARS = np.append(SPARS, data[1])
            if data[0] == 13:
                SPARStwo = np.append(SPARStwo, data[1])
            if data[0] == 14:
                FMT = np.append(FMT, data[1])
            if data[0] == 15:
                CForest = np.append(CForest, data[1])
            if data[0] == 16:
                APS = np.append(APS, data[1])

    planners = [
        'PRMstar', 'LazyPRMstar', 'RRTstar', 'RRTsharp', 'RRTXstatic',
        'InfRRTstar', 'BITstar', 'ABITstar', 'AITstar', 'LBTRRT', 'SST',
        'SPARS', 'SPARStwo', 'CForest', 'APS'
    ]

    f_length = [
        PRMstar_length, LazyPRMstar_length, RRTstar_length, RRTsharp_length, RRTXstatic_length,
        InfRRTstar_length, BITstar_length, ABITstar_length, AITstar_length, LBTRRT_length, SST_length,
        SPARS_length, SPARStwo_length, CForest_length, APS_length
    ]

    f_smooth = [
        PRMstar_smooth, LazyPRMstar_smooth, RRTstar_smooth, RRTsharp_smooth, RRTXstatic_smooth,
        InfRRTstar_smooth, BITstar_smooth, ABITstar_smooth, AITstar_smooth, LBTRRT_smooth, SST_smooth,
        SPARS_smooth, SPARStwo_smooth, CForest_smooth, APS_smooth
    ]

    f = [
        PRMstar, LazyPRMstar, RRTstar, RRTsharp, RRTXstatic,
        InfRRTstar, BITstar, ABITstar, AITstar, LBTRRT, SST,
        SPARS, SPARStwo, CForest, APS
    ]

    f_length = np.asarray(f_length, dtype=object)
    f_smooth = np.asarray(f_smooth, dtype=object)
    f = np.asarray(f, dtype=object)

    means_length = []
    stds_length = []
    Ns_length = []

    means_smooth = []
    stds_smooth = []
    Ns_smooth = []

    for t in f_length:
        means_length.append(np.mean(t))
        stds_length.append(np.std(t))
        Ns_length.append(len(t))

    for t in f_smooth:
        means_smooth.append(np.mean(t))
        stds_smooth.append(np.std(t))
        Ns_smooth.append(len(t))

    means_length = np.asarray(means_length)
    means_smooth = np.asarray(means_smooth)

    planners = np.asarray(planners)

    M_length = []
    M_smooth = []

    for i in range(0, len(means_length)):
        K_length = []
        K_smooth = []
        for j in range(0, len(means_length)):

            sigma_1 = (stds_length[i] / Ns_length[i]) ** 2
            sigma_2 = (stds_length[j] / Ns_length[j]) ** 2
            Z = (means_length[i] - means_length[j]) / \
                math.sqrt(sigma_1 + sigma_2)
            K_length.append(Z)

            sigma_1 = (stds_smooth[i] / Ns_smooth[i]) ** 2
            sigma_2 = (stds_smooth[j] / Ns_smooth[j]) ** 2
            Z = (means_smooth[i] - means_smooth[j]) / \
                math.sqrt(sigma_1 + sigma_2)

            K_smooth.append(Z)

        M_length.append(K_length)
        M_smooth.append(K_smooth)

    M_length = np.asarray(M_length)
    M_length = np.interp(M_length, (M_length.min(), M_length.max()), (-1, 1))

    M_smooth = np.asarray(M_smooth)
    M_smooth = np.interp(M_smooth, (M_smooth.min(), M_smooth.max()), (-1, 1))

    print(f[:, 1])
    print("===============")
    print(M_length.sum(axis=0))
    print("===============")
    print(M_smooth.sum(axis=0))
    print("===============")
    print(M_length.sum(axis=0) + M_smooth.sum(axis=0))
    print("===============")
    final_score = (0.8 * M_length.sum(axis=0) + 0.2 *
                   M_smooth.sum(axis=0)) * f[:, 1]
    print(final_score)

    final_score, planners = zip(*sorted(zip(final_score, planners)))

    for i in range(len(final_score)-1, -1, -1):
        print(planners[i], ": ", final_score[i])

    df_cm = pd.DataFrame(M_length, index=[i for i in planners],
                         columns=[i for i in planners])

    plt.figure(figsize=(10, 7))
    sn.heatmap(df_cm, annot=True)
    plt.show()


if __name__ == '__main__':
    # plot_status()
    # plot_gaussian_length()
    # plot_gaussian_smoothness()
    # plot_cdf_length()
    # plot_cdf_smoothness()
    # plot_cdf_best_cost()
    plot_ztest_length()
    # plot_ztest_smooth()
    # score()
