#!/usr/bin/env python3

from rosbags.rosbag2.reader import Reader
from rosbags.serde import deserialize_cdr
from rosbags.typesys import get_types_from_msg, register_types
import plotnine as p9
import pandas as pd
import numpy as np
from pathlib import Path
import glob
import os
from os.path import dirname, basename
import argparse



# if run from install version
if (basename(dirname(dirname(dirname(dirname(__file__))))) == "install"):
    WS_DIR = dirname(dirname(dirname(dirname(dirname(__file__)))))
# if run from src
else:
    WS_DIR = dirname(dirname(dirname(dirname(__file__))))

parser = argparse.ArgumentParser()
parser.add_argument("-f", "--folder", default=WS_DIR, type=str, help="rosbag2 folder.")
parser.add_argument("-l", "--last", action="store_true", help="Use latest log file in ws or folder if given")
args = parser.parse_args()

if args.last:
    ROSBAG = dirname(sorted(glob.glob(f"{args.folder}/rosbag2*/metadata.yaml"))[-1])
    print("Used rosbag2: ", ROSBAG)
else:
    ROSBAG = os.path.abspath(args.folder)
    if not os.path.exists(os.path.join(ROSBAG, "metadata.yaml")):
        raise Exception("Please use valid rosbag2 folder")

def register_costom_message_types():
    for path in glob.glob(f"{WS_DIR}/src/interfaces/msg/*.msg"):
        msg_def = Path(path).read_text(encoding='utf-8')
        register_types(get_types_from_msg(msg_def,"interfaces/msg/" + Path(path).stem))


def load_bag(path):
    register_costom_message_types()

    columns  = {"time": [], "joint id": [], "joint": [], "leg": [], "topic": [], "name": [],"value": [], "state": []}
    # df = pd.DataFrame(columns=columns)

    with Reader(path) as reader:
        connections = [x for x in reader.connections if x.topic in {'/joint_cmd', "/joint_states", "/trajectory_state"}]
        state = -1
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = deserialize_cdr(rawdata, connection.msgtype)
            # print(msg.position)
            if connection.topic == "/trajectory_state":
                state = msg.state
                continue
            for i in range(12):
                for name in ["position", "velocity", "effort"]:
                    columns["time"].append(timestamp)
                    columns["joint id"].append(i)
                    columns["joint"].append(i%3)
                    columns["leg"].append(np.floor(i/3))
                    columns["topic"].append(connection.topic)
                    columns["name"].append(name)
                    if name == "position":
                        columns["value"].append(msg.position[i]*180/np.pi)
                    elif name == "velocity":
                        columns["value"].append(msg.velocity[i]*180/np.pi)
                    else:
                        columns["value"].append(msg.effort[i])
                    columns["state"].append(state)
    df = pd.DataFrame(columns)
    return df


def plot_data(df):
    df["time"] = (df["time"] - df["time"].min())*1e-9
    df['facet_variable'] = df['name'] + '_' + df['joint'].astype(str)
    df["facet_variable"] = pd.Categorical(df["facet_variable"], categories=
                                          ["position_0", "velocity_0", "effort_0",
                                           "position_1", "velocity_1", "effort_1",
                                           "position_2", "velocity_2", "effort_2"]
                                          , ordered=True)
    df["name"] = pd.Categorical(df["name"], categories=["position", "velocity", "effort"], ordered=True)
    df["topic"] = pd.Categorical(df["topic"], categories=["/joint_states", "/joint_cmd"], ordered=True)
    state_change_positions = df.index[df['state'].diff() != 0][1:]

    print(state_change_positions)

    state_mapping = {
        -1:"invalid",
        0:"init",
        1:"trajectory_replay",
        2:"exertion",
        3:"flight",
        4:"land",
        5:"done",
    }

    state_change_df = pd.DataFrame({
    'state_change_positions': state_change_positions,
    'state': df.loc[state_change_positions, 'state'].map(state_mapping),
    'time': df.loc[state_change_positions, 'time']
    })
    state_change_df["state"] = pd.Categorical(
        state_change_df["state"],
        [state_mapping[i] for i in range(6)
         if state_mapping[i] in state_change_df["state"].tolist()],
        ordered=True)

    titles = ["fl", "fr", "bl","br"]

    exertion_time = state_change_df.loc[state_change_df["state"] == "exertion", "time"].values
    done_time = state_change_df.loc[state_change_df["state"] == "done", "time"].values

    print(exertion_time)
    print(done_time)

    for i in range(4):

        plot: p9.ggplot = (
            p9.ggplot(df[df["leg"]==i], p9.aes(x="time",y="value", color="topic"))
            + p9.geom_line()
            + p9.scale_color_discrete()
            # + p9.facet_grid("joint~name", scales="free")
            + p9.facet_wrap('~facet_variable', scales='free_y')
            + p9.ylab("")
            + p9.xlab("Time in s")
            # + p9.geom_vline(xintercept=state_change_positions, color='red', linetype='dashed')
            + p9.geom_vline(data=state_change_df, mapping=p9.aes(xintercept="time", color="state"),linetype='dashed')
            + p9.ggtitle(f"Leg: {titles[i]}")
            + p9.theme_bw()
        )

        if exertion_time and done_time:
            plot += p9.xlim(exertion_time[0]-0.1, done_time[-1]+0.1)
        print(plot)
        # plot.save(f"{titles[i]}.pdf", "pdf")





if __name__ == "__main__":
    df  = load_bag(Path(f"{ROSBAG}"))
    plot_data(df)
