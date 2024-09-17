from rosbags.rosbag2.reader import Reader
from rosbags.serde import deserialize_cdr
from rosbags.typesys import get_types_from_msg, register_types

import pandas as pd
import numpy as np

import glob
from pathlib import Path

quad_rep_root = "/home/dfki.uni-bremen.de/fstark/Documents/dfki-quad-ros2"


def register_custom_message_types():
    for path in glob.glob(f"{quad_rep_root}/ws/src/interfaces/msg/*.msg"):
        msg_def = Path(path).read_text(encoding="utf-8")
        print(f"register custom message: {msg_def}")
        register_types(get_types_from_msg(msg_def, "interfaces/msg/" + Path(path).stem))


def load_bag(path: str):
    register_custom_message_types()
    bag_data = {"solve_time": [], "wbc_solve_time": [], "cpu_power_consumption": []}
    with Reader(path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            msg = deserialize_cdr(rawdata, connection.msgtype)

        df_st = pd.DataFrame(columns_st)
        df_p = pd.DataFrame(columns_power)
        return df_st, df_p


# bag_path = "/home/jm/#Data/#Uni/DFKI/Experiments/x86_test_20240705/PARTIAL_CONDENSING_OSQP_FULL_all_movements_2"
# bag_path = "/home/jm/PARTIAL_CONDENSING_OSQP_FULL_all_movements_2"
# bag_path = "../x86_test_20240705/PARTIAL_CONDENSING_OSQP_FULL_all_movements_3"
# load_bag(bag_path)
