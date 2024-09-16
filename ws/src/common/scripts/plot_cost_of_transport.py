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
parser.add_argument("-l", "--last", action="store", nargs="?", default=0, const=1, type=int, help="Use latest log file in ws or folder if given. If past with a number i it takes the i th last bag file.")
args = parser.parse_args()

if args.last:
    ROSBAG = dirname(sorted(glob.glob(f"{args.folder}/rosbag2*/metadata.yaml"))[-args.last])
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

    columns  = {"time": [], "joint id": [], "joint": [], "leg": [], "topic": [], "name": [],"value": [], "axis": [], "body": [], "identifier": []}
    # df = pd.DataFrame(columns=columns)

    with Reader(path) as reader:
        connections = [x for x in reader.connections if x.topic in {'/quad_state', "/quad_control_target", '/gait_state'}]
        # control target
        body_velocity_cmd_x = 0
        body_velocity_cmd_y = 0
        body_velocity_cmd_norm = 0
        body_position_cmd_z = 0
        body_twist_cmd_z = 0
        # gait state
        period = 1
        duty_factor = [1,1,1,1]
        phase_offset = [0,0,0,0]
        contact = [1,1,1,1]
        phase = [0,0,0,0]

        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = deserialize_cdr(rawdata, connection.msgtype)
            if connection.topic == "/quad_control_target":
                body_velocity_cmd_x = msg.body_x_dot
                body_velocity_cmd_y = msg.body_y_dot
                body_velocity_cmd_norm = np.sqrt(body_velocity_cmd_x**2 + body_velocity_cmd_y**2)
                body_position_cmd_z = msg.world_z
                body_twist_cmd_z = msg.hybrid_theta_dot
                continue
            if connection.topic == "/gait_state":
                period = msg.period
                for i in range(4):
                    duty_factor[i] = msg.duty_factor[i]
                    phase_offset[i] = msg.phase_offset[i]
                    contact[i] = int(msg.contact[i])
                    phase[i] = msg.phase[i]
                continue
            # quad control target
            for name in ["body_velocity_cmd_x", "body_velocity_cmd_y", "body_velocity_cmd_norm", "body_position_cmd_z", "body_twist_cmd_z"]:
                columns["time"].append(timestamp)
                columns["joint id"].append(-1)
                columns["joint"].append(-1)
                columns["leg"].append(-1)
                columns["body"].append("body")
                columns["topic"].append("/quad_control_target")
                columns["name"].append(name.split("_")[1])
                columns["axis"].append(name.split("_")[-1])
                columns["identifier"].append(name)
                columns["value"].append(eval(name))

            # gait state
            for name in ["period", "duty_factor", "phase_offset", "contact", "phase"]:
                if name == "period":
                    columns["time"].append(timestamp)
                    columns["joint id"].append(-1)
                    columns["joint"].append(-1)
                    columns["leg"].append(-1)
                    columns["body"].append("leg")
                    columns["topic"].append("/gait_state")
                    columns["name"].append(name)
                    columns["axis"].append("b")
                    columns["identifier"].append(f"{name}")
                    columns["value"].append(eval(name))
                    continue
                for i in range(4):
                    columns["time"].append(timestamp)
                    columns["joint id"].append(-1)
                    columns["joint"].append(-1)
                    columns["leg"].append(i)
                    columns["body"].append("leg")
                    columns["topic"].append("/gait_state")
                    columns["name"].append(name)
                    columns["axis"].append("b")
                    columns["identifier"].append(f"{name}_{i}")
                    columns["value"].append(eval(name)[i])

            # for CoT
            mass = 10
            g = 9.81
            v = 0
            cost_of_transport = 0
            # print(msg.position)
            for i in ["w","x","y","z","norm"]:
                for name in ["position", "velocity", "twist", "orientation"]:
                    if ((i == "w" and name != "orientation")
                        or (i == "norm" and name != "velocity")):
                        continue
                    columns["time"].append(timestamp)
                    columns["joint id"].append(-1)
                    columns["joint"].append(-1)
                    columns["leg"].append(-1)
                    columns["body"].append("body")
                    columns["topic"].append(connection.topic)
                    columns["name"].append(name)
                    columns["axis"].append(i)
                    columns["identifier"].append(f"body_{name}_{i}")

                    if name == "position":
                        columns["value"].append(getattr(msg.pose.pose.position,i))
                    elif name == "velocity":
                        if i == "norm":
                            v = np.sqrt(
                                    msg.twist.twist.linear.x**2
                                    + msg.twist.twist.linear.y**2
                                    # + msg.twist.twist.linear.z**2
                                )
                            columns["value"].append(v)
                        else:
                            columns["value"].append(getattr(msg.twist.twist.linear,i))
                    elif name == "twist":
                        columns["value"].append(getattr(msg.twist.twist.angular,i))
                    else:

                        columns["value"].append(getattr(msg.pose.pose.orientation,i))
            for i in range(12):
                # For CoT
                cost_of_transport += abs(msg.joint_state.effort[i]) * abs(msg.joint_state.velocity[i])
                for name in ["position", "velocity", "effort"]:
                    columns["time"].append(timestamp)
                    columns["joint id"].append(i)
                    columns["joint"].append(i%3)
                    columns["body"].append("joint")
                    columns["leg"].append(np.floor(i/3))
                    columns["topic"].append(connection.topic)
                    columns["name"].append(name)
                    columns["axis"].append("j")
                    columns["identifier"].append(f"joint_{name}_{i}")
                    if name == "position":
                        columns["value"].append(msg.joint_state.position[i]*180/np.pi)
                    elif name == "velocity":
                        columns["value"].append(msg.joint_state.velocity[i]*180/np.pi)
                    else:
                        columns["value"].append(msg.joint_state.effort[i])
            # CoT
            if v:
                cost_of_transport/= mass*g*v
                columns["time"].append(timestamp)
                columns["joint id"].append(-1)
                columns["joint"].append(-1)
                columns["body"].append("body")
                columns["leg"].append(-1)
                columns["topic"].append(connection.topic)
                columns["name"].append("CoT")
                columns["axis"].append("b")
                columns["value"].append(cost_of_transport)
                columns["identifier"].append(f"CoT")
                for i in range(4):
                    cot = (abs(msg.joint_state.effort[i*3+0]) * abs(msg.joint_state.velocity[i*3+0])
                           + abs(msg.joint_state.effort[i*3+1]) * abs(msg.joint_state.velocity[i*3+1])
                           + abs(msg.joint_state.effort[i*3+2]) * abs(msg.joint_state.velocity[i*3+2])
                           )/ (mass*g*v)
                    columns["time"].append(timestamp)
                    columns["joint id"].append(-1)
                    columns["joint"].append(-1)
                    columns["body"].append("leg")
                    columns["leg"].append(i)
                    columns["topic"].append(connection.topic)
                    columns["name"].append("CoT")
                    columns["axis"].append("b")
                    columns["value"].append(cot)
                    columns["identifier"].append(f"CoT_{i}")

                    # leg cot percentage
                    columns["time"].append(timestamp)
                    columns["joint id"].append(-1)
                    columns["joint"].append(-1)
                    columns["body"].append("leg")
                    columns["leg"].append(i)
                    columns["topic"].append(connection.topic)
                    columns["name"].append("CoT_percentage")
                    columns["axis"].append("b")
                    columns["value"].append(cot/cost_of_transport)
                    columns["identifier"].append(f"CoT_percentage_{i}")



    df = pd.DataFrame(columns)
    df["time"] = (df["time"] - df["time"].min())*1e-9
    # df["name"] = pd.Categorical(df["name"], categories=["position", "orientation", "velocity","twist", "effort", "CoT", "period", "duty_factor", "phase_offset", "contact", "phase"], ordered=True)
    # df["axis"] = pd.Categorical(df["axis"], categories=["w", "x", "y","z", "j", "b", "norm"], ordered=True)

    # add values for alternative x axis
    df = copy_val_to_column(df,"time","velocity","value","identifier","body_velocity_norm")
    df = copy_val_to_column(df,"time","velocity_cmd","value","identifier","body_velocity_cmd_norm")
    df = copy_val_to_column(df,"time","z_pos","value","identifier","body_position_z")

    # delete repeating contact info to fasten plotting
    indices_to_drop = []
    leg_contact_indices = [[], [], [], []]
    for i in range(4):
        leg_contact_indices[i] = df[(df["name"] == "contact") & (df["leg"] == i)].index
        for j in range(1, len(leg_contact_indices[i]) - 1):
            if (df["value"].loc[leg_contact_indices[i][j]] == df["value"].loc[leg_contact_indices[i][j - 1]]
                    and df["value"].loc[leg_contact_indices[i][j]] == df["value"].loc[leg_contact_indices[i][j + 1]]):
                indices_to_drop.append(leg_contact_indices[i][j])

    df.drop(indices_to_drop, inplace=True)
    df.reset_index(drop=True, inplace=True)


    return df


def copy_val_to_column(df, index_column, new_name, old_column, identifying_column, identifying_value):
    df_new = df[df[identifying_column] == identifying_value]
    df_new[new_name] = df_new[old_column]
    df = pd.merge(df, df_new[[index_column, new_name]], on=index_column, how="left")
    return df

def plot_CoT(df):
    (
        p9.ggplot(df[df["name"]=="CoT"], p9.aes(x="velocity", y="value", color="velocity_cmd"))
        + p9.geom_point()
        # +p9.geom_density()
        + p9.geom_smooth(span=0.0001)
        # + p9.geom_path()
    ).draw(1)

def plot_CoT_time(df):
    (
        p9.ggplot(df[df["name"].isin(("CoT", "CoT_percentage"))], p9.aes(x="time", y="value", color="identifier"))
        + p9.geom_line()
        + p9.facet_wrap("~name")
    ).draw(1)


def plot_velocity_vs_cmd(df):
    (
        # p9.ggplot(df[df["body"] == "body"], p9.aes(x="time", y="value", color="axis"))
        p9.ggplot(df[df["axis"].isin(("norm", "b"))][df["name"].isin(("velocity",))], p9.aes(x="time", y="value", color="identifier"))
        + p9.geom_line()
        + p9.facet_wrap("~name", scales="free_y")
    ).draw(1)


def plot_contact_pattern(df):
    (
        p9.ggplot(df[df["name"].isin({"phase", "duty_factor", "velocity", "CoT_percentage"})][df["body"]!="joint"], p9.aes(x="time", y="value", color="identifier"))
        # + p9.geom_polygon()
        + p9.geom_area(mapping=p9.aes(x="time", y="value"), inherit_aes=False, data=df[df["name"]=="contact"], stat="identity")
        + p9.geom_line()
        + p9.facet_grid("leg~", scales="free_y")
        + p9.xlab("time in s")
    ).draw(1)

def plot(df):
    (
        p9.ggplot(df[df["name"]=="contact"], p9.aes(x="time", y="value", color="factor(leg)"))
        + p9.geom_point()
        + p9.geom_line()

    ).draw(1)



if __name__ == "__main__":
    df  = load_bag(Path(f"{ROSBAG}"))
    # plot(df)
    plot_CoT(df)
    plot_CoT_time(df)
    plot_velocity_vs_cmd(df)
    plot_contact_pattern(df)
