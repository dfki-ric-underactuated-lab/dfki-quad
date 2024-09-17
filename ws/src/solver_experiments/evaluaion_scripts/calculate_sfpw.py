import load_rosbag

import pandas as pd
import numpy as np


def calculate_sfpw(
    df_s,
    df_p,
    column_name_s: str,
    column_name_p: str,
    solver_name: str,
    architecture: str,
) -> pd.DataFrame:
    # calculate overhang to be removed from df_s
    remain = df_s.shape[0] % df_p.shape[0]
    # get start and end index
    if remain % 2 == 0:
        start_trim = int(remain / 2)
        end_trim = start_trim
    else:
        start_trim = int((remain - 1 / 2))
        end_trim = int((remain + 1 / 2))

    # trim df_s, such that there is no remainder, when calculating the average solve time
    df_s = df_s.iloc[start_trim:-end_trim]
    # reset indices
    df_s.reset_index(drop=True, inplace=True)

    # caluculate bin size for mapping -> 1 df_p data point to --bin_size-- df_s data points
    bin_size = int(df_s.shape[0] / df_p.shape[0])

    # calculate the average solve time
    average_st = pd.DataFrame(
        {"average_st": df_s[column_name_s].groupby(df_s.index // bin_size).mean()}
    )

    # assemble one data frame with all information
    df_sfpw = pd.DataFrame(
        {
            "architecture": architecture,
            "solver": solver_name,
            "average_solve_time": average_st["average_st"],
            "cpu_power_consumption": df_p[column_name_p],
            "sfpw": (1 / average_st["average_st"]) / df_p[column_name_p],
        }
    )

    return df_sfpw
