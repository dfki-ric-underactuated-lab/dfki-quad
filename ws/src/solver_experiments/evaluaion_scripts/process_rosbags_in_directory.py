import os

from load_rosbag import register_custom_message_types
from rosbags.rosbag2.reader import Reader as Bagreader
from rosbags.serde import deserialize_cdr
import numpy as np


def process_rosbags_in_directory(
    result_datas: list[dict], path: str, tag_parser_fun, process_rosbag_funs: list
):
    register_custom_message_types()
    assert len(result_datas) == len(process_rosbag_funs)
    # Iterate over experiment directory
    for exp_folder in os.listdir(path):
        # load failed attempts
        failed_file = None
        try:
            failed_file = open(
                os.path.join(path, exp_folder, "failed_attempts.txt"), "r"
            )
        except:
            failed_file = open(os.path.join(path, exp_folder, "failed_attempts"), "r")

        faild_exps = failed_file.readlines()
        failed_file.close()
        # Iterate over experiments in directory
        for exp_bag in os.listdir(os.path.join(path, exp_folder)):
            failed = False
            if os.path.isfile(os.path.join(path, exp_folder, exp_bag)):
                print("skipping meta data file")
                continue
            elif str(faild_exps).__contains__(exp_bag):
                print(
                    f"Mark this experiment as experiment failed: {exp_folder} -> {exp_bag} "
                )
                failed = True
            try:
                with Bagreader(os.path.join(path, exp_folder, exp_bag)) as bag:

                    print(f"reading bag: {exp_folder} -> {exp_bag}")
                    # Read bag
                    bag_time_data = {
                        "wbc_solve_time": {
                            "qp_solve_time": [],
                            "qp_update_time": [],
                            "success": [],
                            "total_time": [],
                        },
                        "solve_time": {
                            "acados_solve_qp_time": [],
                            "acados_condensing_time": [],
                            "acados_interface_time": [],
                            "acados_total_time": [],
                            "acados_num_iter": [],
                            "acados_t_computed": [],
                            "acados_return": [],
                        },
                        "cpu_power_consumption": {"data": []},
                    }
                    # add timestamp fields to every topic
                    for topic in bag_time_data.keys():
                        bag_time_data[topic]["recv_timestamp_ns"] = []
                    # add failed field
                    bag_time_data["failed"] = failed
                    # iterate through messages
                    for connection, timestamp, rawdata in bag.messages():
                        msg = deserialize_cdr(rawdata, connection.msgtype)
                        for topic in bag_time_data.keys():
                            if connection.topic.__contains__(topic):
                                for topic_field in bag_time_data[topic].keys():
                                    if topic_field == "recv_timestamp_ns":
                                        bag_time_data[topic][topic_field].append(
                                            timestamp
                                        )
                                    else:
                                        bag_time_data[topic][topic_field].append(
                                            msg.__dict__[topic_field]
                                        )
                                break
                    # first get the keys:
                    tag_data = tag_parser_fun(exp_folder, exp_bag, bag_time_data)
                    for result_data_out, result_fun in zip(
                        result_datas, process_rosbag_funs
                    ):
                        processed_data = result_fun(exp_folder, exp_bag, bag_time_data)
                        # if data_out is empty create the lists
                        if len(result_data_out) == 0:
                            for key in tag_data.keys():
                                result_data_out[key] = []
                            for key in processed_data.keys():
                                result_data_out[key] = []

                        data_len = len(processed_data[list(processed_data.keys())[0]])
                        for key in processed_data.keys():
                            # check the length of each element in here
                            assert len(processed_data[key]) == data_len
                            # now add data
                            result_data_out[key].extend(processed_data[key])
                        for key in tag_data.keys():
                            # now add tag data
                            result_data_out[key].extend(data_len * [tag_data[key]])

            except Exception as e:
                # raise e
                print(f"!!!!!!FAILURE IN Bag: {e}")


def calculate_sfpw(solve_time: np.ndarray, cpu_measurements: np.ndarray):
    # interpolate cpu_measurements
    cpu_measurements_interp = np.interp(
        solve_time[0, :], cpu_measurements[0, :], cpu_measurements[1, :]
    )
    # get theoretical solve frequency
    solve_freq = 1.0 / solve_time[1, :]
    # get sfpw
    sfpw = solve_freq / cpu_measurements_interp
    return np.vstack((solve_time[0, :], sfpw))
