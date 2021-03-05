import sqlite3
from sqlite3 import Error
import numpy as np
import matplotlib.pyplot as plt


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


def select_specific_task(conn, priority):
    """
    Query all rows in the tasks table
    :param conn: the Connection object
    :return:
    """
    cur = conn.cursor()
    cur.execute(
        "SELECT plannerid, memory, solution_length, solution_smoothness,id FROM runs"
    )

    datas = cur.fetchall()

    GT = []
    RRTstar = []
    PRMstar = []
    LazyPRMstar = []
    RRTX = []
    FMTstar = []
    BITstar = []
    ABITstar = []
    CFOREST = []

    cur.execute("SELECT gt_path_length FROM experiments")
    gt_path_length = cur.fetchone()
    print(gt_path_length[0])
    gt_path_length = gt_path_length[0]

    range_min = gt_path_length
    range_max = range_min * 1.5

    for data in datas:
        data = list(data)
        if data[2] != None:
            data[2] = normalize(data[2], range_min, range_max)
            if data[0] == 1:
                RRTstar.append(data[2])
            if data[0] == 2:
                PRMstar.append(data[2])
            if data[0] == 3:
                LazyPRMstar.append(data[2])
            if data[0] == 4:
                RRTX.append(data[2])
            if data[0] == 5:
                FMTstar.append(data[2])
            if data[0] == 6:
                BITstar.append(data[2])
            if data[0] == 7:
                ABITstar.append(data[2])
            if data[0] == 8:
                CFOREST.append(data[2])

    plot = [
        RRTstar, PRMstar, LazyPRMstar, RRTX, FMTstar, BITstar, ABITstar,
        CFOREST
    ]

    fig1, ax1 = plt.subplots()
    ax1.set_title('Solution Lengths')
    # x-axis labels
    ax1.set_yticklabels([
        'RRTstar', 'PRMstar', 'LazyPRMstar', ' RRTX', 'FMTstar', 'BITstar',
        'ABITstar', 'CFOREST'
    ])

    ax1.boxplot(plot, vert=0)
    plt.show()


def normalize(value, min, max):
    return (value - min) / (max - min)


if __name__ == '__main__':
    database = "/home/ros2-foxy/ECMR2021/benchmark_results/SE2_0.db"

    conn = create_connection(database)
    with conn:
        print("2. Query all tasks")
        select_specific_task(conn, "memory")
