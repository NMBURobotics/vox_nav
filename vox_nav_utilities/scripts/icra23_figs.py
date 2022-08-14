import sqlite3
from sqlite3 import Error
from turtle import position
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
import math
from scipy.stats import norm


def normpdf(x, mean, sd):
    return norm.pdf(x, loc=mean, scale=sd)


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


TIMEOUT = 'Timeout'
APPROXIMATE_SOLUTION = 'Approximate'
EXACT_SOLUTION = 'Exact'


def plot_status():
    # ALL SBO PLANNERS
    RRTStarF = []
    LQRRRTStar = []
    SST = []
    EST = []
    KPIECE1 = []

    file1 = open(
        '/home/atas/colcon_ws/src/vox_nav/vox_nav_planning/src/plugins/exp.txt', 'r')
    Lines = file1.readlines()
    count = 0
    for line in Lines:
        count += 1
        result = len(line.split())
        print(line.split()[0])

        if line.split()[0] == 'RRTStarF':
            RRTStarF = np.append(RRTStarF, line.split()[1])
        if line.split()[0] == 'LQRRRTStar':
            LQRRRTStar = np.append(LQRRRTStar, line.split()[1])
        if line.split()[0] == 'SST':
            SST = np.append(SST, line.split()[1])
        if line.split()[0] == 'EST':
            EST = np.append(EST, line.split()[1])
        if line.split()[0] == 'KPIECE1':
            KPIECE1 = np.append(KPIECE1, line.split()[1])

    exact_sol = []
    apprx_sol = []
    timeout = []

    f = [
        RRTStarF,
        LQRRRTStar,
        SST,
        EST,
        KPIECE1
    ]

    x = [
        'RRTStarF',
        'LQRRRTStar',
        'SST',
        'EST',
        'KPIECE1'
    ]

    index = 0
    for t in x:
        unique, counts = np.unique(f[index], return_counts=True)
        D = dict(zip(unique, counts))
        exact_sol.append(D.get(EXACT_SOLUTION, 0))
        apprx_sol.append(D.get(APPROXIMATE_SOLUTION, 0))
        timeout.append(D.get(TIMEOUT, 0))
        index += 1

    exact_sol = np.array(exact_sol)
    apprx_sol = np.array(apprx_sol)
    timeout = np.array(timeout)

    # plot bars in stack manner
    plt.bar(x, exact_sol,  color='g')
    plt.bar(x, apprx_sol, bottom=exact_sol, color='y')
    plt.bar(x, timeout, bottom=exact_sol + apprx_sol, color='b')
    plt.xlabel("Planners")
    plt.ylabel("Total Number of Runs")
    plt.legend(["Exact Solution",
               "Approximate Solution", "Timeout"])
    plt.title(
        "Stacked bar plot representing status of solutions by evaluated planners")
    plt.show()


def plot_status():
    # ALL SBO PLANNERS
    RRTStarF = []
    LQRRRTStar = []
    SST = []
    EST = []
    KPIECE1 = []

    file1 = open(
        '/home/atas/colcon_ws/src/vox_nav/vox_nav_planning/src/plugins/exp.txt', 'r')
    Lines = file1.readlines()
    count = 0
    for line in Lines:
        count += 1
        result = len(line.split())
        print(line.split()[0])

        if line.split()[0] == 'RRTStarF':
            RRTStarF = np.append(RRTStarF, line.split()[1])
        if line.split()[0] == 'LQRRRTStar':
            LQRRRTStar = np.append(LQRRRTStar, line.split()[1])
        if line.split()[0] == 'SST':
            SST = np.append(SST, line.split()[1])
        if line.split()[0] == 'EST':
            EST = np.append(EST, line.split()[1])
        if line.split()[0] == 'KPIECE1':
            KPIECE1 = np.append(KPIECE1, line.split()[1])

    exact_sol = []
    apprx_sol = []
    timeout = []

    f = [
        RRTStarF,
        LQRRRTStar,
        SST,
        EST,
        KPIECE1
    ]

    x = [
        'RRTStarF',
        'LQRRRTStar',
        'SST',
        'EST',
        'KPIECE1'
    ]

    index = 0
    for t in x:
        unique, counts = np.unique(f[index], return_counts=True)
        D = dict(zip(unique, counts))
        exact_sol.append(D.get(EXACT_SOLUTION, 0))
        apprx_sol.append(D.get(APPROXIMATE_SOLUTION, 0))
        timeout.append(D.get(TIMEOUT, 0))
        index += 1

    exact_sol = np.array(exact_sol)
    apprx_sol = np.array(apprx_sol)
    timeout = np.array(timeout)

    # plot bars in stack manner
    plt.bar(x, exact_sol,  color='g')
    plt.bar(x, apprx_sol, bottom=exact_sol, color='y')
    plt.bar(x, timeout, bottom=exact_sol + apprx_sol, color='b')
    plt.xlabel("Planners")
    plt.ylabel("Total Number of Runs")
    plt.legend(["Exact Solution",
               "Approximate Solution", "Timeout"])
    plt.title(
        "Stacked bar plot representing status of solutions by evaluated planners")
    plt.show()


def plot_lenght():
    # ALL SBO PLANNERS
    RRTStarF = []
    LQRRRTStar = []
    SST = []
    EST = []
    KPIECE1 = []

    file1 = open(
        '/home/atas/colcon_ws/src/vox_nav/vox_nav_planning/src/plugins/exp.txt', 'r')
    Lines = file1.readlines()
    count = 0
    for line in Lines:
        count += 1
        result = len(line.split())
        print(line.split()[0])
        res = float(line.split()[-1])

        if line.split()[0] == 'RRTStarF' and line.split()[1] == 'Exact':
            RRTStarF = np.append(RRTStarF, res)
        if line.split()[0] == 'LQRRRTStar' and line.split()[1] == 'Exact':
            LQRRRTStar = np.append(LQRRRTStar, res)
        if line.split()[0] == 'SST' and line.split()[1] == 'Exact':
            SST = np.append(SST, res)
        if line.split()[0] == 'EST' and line.split()[1] == 'Exact':
            EST = np.append(EST, res)
        if line.split()[0] == 'KPIECE1' and line.split()[1] == 'Exact':
            KPIECE1 = np.append(KPIECE1, res)

    exact_sol = []
    apprx_sol = []
    timeout = []

    import plotly.express as px

    data = [
        RRTStarF,
        LQRRRTStar,
        SST,
        EST,
        KPIECE1
    ]

    x = [
        'RRTStarF',
        'LQRRRTStar',
        'SST',
        'EST',
        'KPIECE1'
    ]

    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111)

    bp = ax.boxplot(data, patch_artist=True,
                    notch='True', vert=0, showfliers=False, autorange=True, showbox=True, showmeans=True)

    colors = ['#0000FF', '#00FF00',
              '#FFFF00', '#FF00FF', '#FFF0FF']

    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)

    # changing color and linewidth of
    # whiskers
    for whisker in bp['whiskers']:
        whisker.set(color='#8B008B',
                    linewidth=1.5,
                    linestyle=":")

    # changing color and linewidth of
    # caps
    for cap in bp['caps']:
        cap.set(color='#8B008B',
                linewidth=2)

    # changing color and linewidth of
    # medians
    for median in bp['medians']:
        median.set(color='red',
                   linewidth=3)

    # changing style of fliers
    for flier in bp['fliers']:
        flier.set(marker='D',
                  color='#e7298a',
                  alpha=0.5)

    # x-axis labels
    ax.set_yticklabels(x)

    # Adding title
    plt.title(
        "Means of Path lenghts by each planner computed only by exact solutions")

    # Removing top axes and right axes
    # ticks
    ax.get_xaxis().tick_bottom()
    ax.get_yaxis().tick_left()
    plt.xlabel("Path Lengths")
    plt.ylabel("Planner")
    # show plot
    plt.show()


if __name__ == '__main__':
    # plot_status()
    plot_lenght()
