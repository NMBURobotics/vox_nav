import sqlite3
from sqlite3 import Error
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import matplotlib.patches as mpatches
import plotly.graph_objects as go


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
    unique, counts = np.unique(InfRRTstar, return_counts=True)
    print("InfRRTstar", dict(zip(unique, counts)))
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
    unique, counts = np.unique(APS, return_counts=True)
    print("APS", dict(zip(unique, counts)))


def plot_comparative_metrics(state_spaces,
                             epochs,
                             metric,
                             normalization_value,
                             ground_truth_name,
                             ylim_low=1,
                             ylim_upper=5):

    first_space = []
    second_space = []

    for curr_state_space in range(0, len(state_spaces), 1):
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
        for i in range(0, epochs, 1):
            database = "/home/ros2-foxy/ECMR2021/benchmark_results/" + state_spaces[
                curr_state_space] + "_" + str(i) + ".db"

            conn = create_connection(database)
            cur = conn.cursor()
            cur.execute("SELECT plannerid, " + metric + " FROM runs")
            datas = cur.fetchall()

            cur.execute("SELECT " + ground_truth_name + " FROM experiments")
            gt_val = cur.fetchone()
            print(gt_val[0])
            gt_val = gt_val[0]

            try:
                normalization_coeff = normalization_value / gt_val
            except ZeroDivisionError:
                normalization_coeff = 0

            gt_val = normalization_coeff * gt_val

            for data in datas:
                data = list(data)
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
        if curr_state_space == 0:
            first_space.append(PRMstar)
            first_space.append(LazyPRMstar)
            first_space.append(RRTstar)
            first_space.append(RRTsharp)
            first_space.append(RRTXstatic)
            first_space.append(InfRRTstar)
            first_space.append(BITstar)
            first_space.append(ABITstar)
            first_space.append(AITstar)
            first_space.append(LBTRRT)
            first_space.append(SST)
            first_space.append(SPARS)
            first_space.append(SPARStwo)
            first_space.append(FMT)
            first_space.append(CForest)
            first_space.append(APS)
        else:
            second_space.append(PRMstar)
            second_space.append(LazyPRMstar)
            second_space.append(RRTstar)
            second_space.append(RRTsharp)
            second_space.append(RRTXstatic)
            second_space.append(InfRRTstar)
            second_space.append(BITstar)
            second_space.append(ABITstar)
            second_space.append(AITstar)
            second_space.append(LBTRRT)
            second_space.append(SST)
            second_space.append(SPARS)
            second_space.append(SPARStwo)
            second_space.append(FMT)
            second_space.append(CForest)
            second_space.append(APS)

    ticks = [
        'PRMstar', 'LazyPRMstar', 'RRTstar', 'RRTsharp', 'RRTXstatic',
        'InfRRTstar', 'BITstar', 'ABITstar', 'AITstar', 'LBTRRT', 'SST',
        'SPARS', 'SPARStwo', 'FMT', 'CForest', 'APS'
    ]

    plt.figure()

    bpl = plt.boxplot(first_space,
                      positions=np.array(range(len(first_space))) * 2.0 - 0.4,
                      sym='',
                      widths=0.7,
                      notch=True,
                      patch_artist=True,
                      autorange=True)
    bpr = plt.boxplot(second_space,
                      positions=np.array(range(len(second_space))) * 2.0 + 0.4,
                      sym='',
                      widths=0.7,
                      notch=True,
                      patch_artist=True,
                      autorange=True)
    set_box_color(bpl, '#D7191C')
    set_box_color(bpr, '#2C7BB6')

    # draw temporary red and blue lines and use them to create a legend
    plt.plot([], c='#D7191C', label=state_spaces[0])
    plt.plot([], c='#2C7BB6', label=state_spaces[1])

    # make a legend for both plots
    leg = plt.legend(loc=2, prop={'size': 18}, scatterpoints=1)

    # set the linewidth of each legend object
    for legobj in leg.legendHandles:
        legobj.set_linewidth(10.0)

    plt.xticks(range(0, len(ticks) * 2, 2), ticks, fontsize=18, rotation=45)
    plt.xlim(-2, len(ticks) * 2)
    plt.ylim(ylim_low, ylim_upper)
    plt.tight_layout()
    plt.grid()
    plt.show()


def set_box_color(bp, color):
    plt.setp(bp['boxes'], color=color)
    plt.setp(bp['whiskers'], color=color)
    plt.setp(bp['caps'], color=color)
    plt.setp(bp['medians'], color=color)


if __name__ == '__main__':
    #plot_normalized_path_lengths("SE2", 25)
    #plot_status("SE3", 25)
    #plot_smoothness("SE2", 25)
    plot_comparative_metrics(['DUBINS', 'REEDS'],
                             25,
                             'solution_smoothness',
                             1.0,
                             'gt_path_smoothness',
                             ylim_low=0.0,
                             ylim_upper=0.9)
