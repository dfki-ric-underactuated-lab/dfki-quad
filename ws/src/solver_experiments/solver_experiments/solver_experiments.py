#!/usr/bin/env python3
import os.path
import signal
import subprocess
import threading
import time
from math import radians

import rclpy
import rosbag2_py
from ament_index_python import get_package_share_directory
from interfaces.msg import QuadControlTarget
from interfaces.msg import QuadState
from interfaces.srv import ChangeLegDriverMode, KeepJointPositions
from interfaces.srv import ResetSimulation
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_srvs.srv import Trigger


class ExperimentPlayer(Node):
    def __init__(self):
        super().__init__("experiment_player")
        # create publisher
        self.control_target_pub = self.create_publisher(
            QuadControlTarget, "quad_control_target", 10
        )
        self.damping_client = self.create_client(Trigger, "/set_emergency_damping_mode")
        self.param_client = self.create_client(
            SetParameters, "/mit_controller_node/set_parameters"
        )
        self.stop_msg = QuadControlTarget()
        self.stop_msg.body_x_dot = 0.0
        self.stop_msg.body_y_dot = 0.0
        self.stop_msg.world_z = 0.30
        self.stop_msg.hybrid_theta_dot = 0.0
        self.stop_msg.pitch = 0.0
        self.stop_msg.roll = 0.0

        self.playout = False
        self.playout_lock = threading.Lock()
        self.playout_thread = None

    def send_cmd(self, msg: QuadControlTarget):
        self.get_logger().info(f"Sending control command {msg}")
        self.control_target_pub.publish(msg)

    def damp(self):
        trigg = Trigger()
        self.damping_client.call(trigg)

    def playout_experiment_async(self, action_list: list):
        self.playout_lock.acquire()
        if self.playout:
            self.get_logger().info(f"Playout already running")
            self.playout_lock.release()
            return
        self.playout = True
        self.playout_lock.release()
        self.playout_thread = threading.Thread(
            target=self.playout_experiment, args=[action_list]
        )
        self.playout_thread.start()
        self.get_logger().info(f"Started playout async")

    def playout_experiment(self, action_list: list):
        self.playout_lock.acquire()
        if not self.playout:
            self.playout_lock.release()
            self.get_logger().info(f"Playout was immeadiatly stopped")
            return
        self.playout_lock.release()

        for i in range(len(action_list)):
            self.get_logger().info(f"Commanding index in list {i}")
            self.playout_lock.acquire()
            if not self.playout:  # canceled
                self.get_logger().info("Playout was cancelled")
                self.send_cmd(self.stop_msg)
                self.playout_lock.release()
                return
            self.get_logger().info(f"Sleeping for {action_list[i][0]}")
            self.playout_lock.release()
            self.send_cmd(action_list[i][1])
            time.sleep(action_list[i][0])

        self.get_logger().info("Playout end")
        self.send_cmd(self.stop_msg)
        self.playout_lock.acquire()
        self.playout = False
        self.playout_lock.release()

    def isPlaying(self):
        ret = False
        self.playout_lock.acquire()
        ret = self.playout
        self.playout_lock.release()
        return ret

    def cancel_playout(self):
        self.get_logger().info("Cancelling playout")
        self.playout_lock.acquire()
        self.playout = False
        self.playout_lock.release()
        self.playout_thread.join()


class SolverExperiments(Node):
    def __init__(self, working_folder: str):
        super().__init__("solver_experiments")

        self.working_folder = working_folder
        self.quad_state = None
        self.sim_reset_client = self.create_client(ResetSimulation, "/reset_sim")
        self.reset_request = ResetSimulation.Request()
        self.leg_driver_switch_mode_client = self.create_client(
            ChangeLegDriverMode, "/switch_op_mode"
        )
        self.keep_pose_client = self.create_client(
            KeepJointPositions, "/keep_joint_pose"
        )
        self.sim_reset_request = ResetSimulation.Request()
        self.leg_driver_switch_mode_request = ChangeLegDriverMode.Request()
        self.leg_driver_keep_pose_request = KeepJointPositions.Request()
        self.leg_driver_switch_mode_request.target_mode = 2  # KEEP_POSE
        self.leg_driver_keep_pose_request.joint_positions = [
            0.126,
            0.61,
            -1.22,
            -0.126,
            0.61,
            -1.22,
            0.126,
            0.61,
            -1.22,
            -0.126,
            0.61,
            -1.22,
        ]
        self.quad_state_sub = self.create_subscription(
            QuadState, "quad_state", self.quad_state_callback, 1
        )
        self.recording_subprocess = None
        self.playback_subprocess = None
        self.failed_experiments = []
        self.controller_process = None
        self.cpu_process = None
        self.joy_to_target_process = None
        self.leg_driver_process = None
        self.bag_recorder = rosbag2_py.Recorder()
        self.player = ExperimentPlayer()
        config_file = "mit_controller_sim_go2.yaml"
        pkg_controllers = get_package_share_directory("controllers")
        self.cpu_params_file = os.path.join(
            pkg_controllers, "config", "cpu_power_logging.yaml"
        )
        self.controller_params_file = os.path.join(
            pkg_controllers, "config", config_file
        )
        self.get_logger().info("Controller params: " + self.controller_params_file)

        self.start_needed_nodes()

    def STOP(self):
        self.get_logger().info("---> Writing failed attempts file...")
        f = open(os.path.join(self.working_folder, "failed_attempts.txt"), "x")
        f.write(str(self.failed_experiments))
        f.close()
        self.get_logger().info("<--- ")
        self.stop_needed_nodes()
        if self.controller_process is not None:
            self.stop_controller()

        self.player.destroy_node()
        # Write back failed attempts

    def quad_state_callback(self, msg):
        self.quad_state = msg

    def start_playback_rosbag(self, rosbag_file: str):
        if self.playback_subprocess is None:
            self.playback_subprocess = subprocess.Popen(
                ["ros2", "bag", "play", rosbag_file],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                stdin=subprocess.DEVNULL,
            )

    def stop_playout_rosbag(self):
        if self.playback_subprocess is not None:
            self.playback_subprocess.terminate()
            self.playback_subprocess = None

    def robot_fell(self):
        return (
            self.quad_state.pose.pose.position.z < 0.1
            # abad joint angle
            or abs(self.quad_state.joint_state.position[0]) > 0.785
            or abs(self.quad_state.joint_state.position[3]) > 0.785
            or abs(self.quad_state.joint_state.position[6]) > 0.785
            or abs(self.quad_state.joint_state.position[9]) > 0.785
            # inverted knee
            or self.quad_state.joint_state.position[2] > 0.02
            or self.quad_state.joint_state.position[5] > 0.02
            or self.quad_state.joint_state.position[8] > 0.02
            or self.quad_state.joint_state.position[11] > 0.02
        )  # FIXME: add more indicators

    def start_record_bag(self, bag_file: str):
        storage_options = rosbag2_py.StorageOptions(bag_file)
        record_options = rosbag2_py.RecordOptions()
        record_options.topics = [
            "/solve_time",
            "/cpu_power_consumption",
            "/wbc_solve_time",
        ]
        self.recording_subprocess = threading.Thread(
            target=self.bag_recorder.record, args=[storage_options, record_options]
        )
        self.recording_subprocess.start()

    # self.recording_subprocess = subprocess.Popen(
    #     ['ros2', 'bag', 'record', '-o', str(bag_file), '/solve_time', '/cpu_power_consumption', '/wbc_solve_time'],
    #     stdout=subprocess.DEVNULL,
    #     stderr=subprocess.DEVNULL,
    #     stdin=subprocess.DEVNULL
    # )

    def stop_record_bag(self):
        self.bag_recorder.cancel()
        self.recording_subprocess.join()
        # if self.recording_subprocess is not None:
        #    self.recording_subprocess.terminate()
        #    self.recording_subprocess = None

    def start_power_consumption_node(self):
        if self.cpu_process is None:
            self.cpu_process = subprocess.Popen(
                [
                    "ros2",
                    "run",
                    "controllers",
                    "log_cpu_power",
                    "--ros-args",
                    "--params-file",
                    self.cpu_params_file,
                ]
            )

    def stop_power_consumption_node(self):
        if self.cpu_process is not None:
            self.cpu_process.terminate()
            self.cpu_process = None

    def start_joy_node(self):
        if self.joy_to_target_process is None:
            self.joy_to_target_process = subprocess.Popen(
                [
                    "ros2",
                    "run",
                    "controllers",
                    "joy_to_target.py",
                    "--ros-args",
                    "-p",
                    "use_sim_time:=True",
                    "-p",
                    "init_robot_height:=0.30",
                    "-p",
                    "max_robot_height:=0.40",
                    "-p",
                    "max_acceleration:=0.5",
                    "-p",
                    "use_sim_time:=True",
                ]
            )

    def stop_joy_node(self):
        if self.joy_to_target_process is not None:
            self.joy_to_target_process.terminate()
            self.joy_to_target_process = None

    def start_leg_driver(self):
        if self.leg_driver_process is None:
            self.joy_to_target_process = subprocess.Popen(
                ["ros2", "launch", "drivers", "leg_driver_launch.py", "sim:=go2"],
            )

    def stop_leg_driver(self):
        if self.leg_driver_process is not None:
            os.killpg(os.getpgid(self.leg_driver_process.pid), signal.SIGKILL)
            self.leg_driver_process = None

    def run_controller(
        self,
        mpc_solver_name: str,
        mpc_hpipm_mode: str,
        mpc_condensed_size: int,
        wbc_solver_name: str,
        wbc_scene_name: str,
        gs_gait: str,
    ):
        self.controller_process = subprocess.Popen(
            [
                "ros2",
                "run",
                "controllers",
                "mitcontrollernode",
                "--ros-args",
                "--params-file",
                self.controller_params_file,
                "-p",
                "mpc_solver:=" + mpc_solver_name,
                "-p",
                "mpc_condensed_size:=" + str(mpc_condensed_size),
                "-p",
                "mpc_hpipm_mode:=" + mpc_hpipm_mode,
                "-p",
                "wbc.arc_opt.solver:=" + wbc_solver_name,
                "-p",
                "wbc.arc_opt.scene:=" + wbc_scene_name,
                "-p",
                "simple_gait_sequencer.gait:=" + gs_gait,
                "-p",
                "use_sim_time:=True",
            ],
            # stdout=subprocess.PIPE,
            # shell=True,
            preexec_fn=os.setsid,
        )

    def stop_controller(self):
        if self.controller_process is not None:
            os.killpg(os.getpgid(self.controller_process.pid), signal.SIGTERM)
            self.controller_process = None

    def playout_finished(self):
        if self.playback_subprocess is not None:
            poll_ret = self.playback_subprocess.poll()
            self.get_logger().info("status: " + str(poll_ret))
            if poll_ret is None:
                return False
            if poll_ret <= 0:
                self.playback_subprocess = None
                return True
            else:
                return False
        else:
            return True

    def reset_simulation(self):
        self.sim_reset_request.pose.position.x = 0.0
        self.sim_reset_request.pose.position.y = 0.0
        self.sim_reset_request.pose.position.z = 0.35
        self.sim_reset_request.pose.orientation.w = 1.0
        self.sim_reset_request.pose.orientation.x = 0.0
        self.sim_reset_request.pose.orientation.y = 0.0
        self.sim_reset_request.pose.orientation.z = 0.0
        self.sim_reset_request.joint_positions = [
            0.126,
            0.61,
            -1.22,
            -0.126,
            0.61,
            -1.22,
            0.126,
            0.61,
            -1.22,
            -0.126,
            0.61,
            -1.22,
        ]  # TODO: ggf adapt joint positions

        self.sim_reset_client.call(self.sim_reset_request)
        return

    def start_needed_nodes(self):
        # self.get_logger().info("---> Start Joy node")
        # self.start_joy_node()
        # self.get_logger().info("<---")
        self.get_logger().info("---> Start CPU node")
        self.start_power_consumption_node()
        self.get_logger().info("<---")
        self.get_logger().info("---> Start Leg driver node")
        self.start_leg_driver()
        self.get_logger().info("<---")

    def stop_needed_nodes(self):
        # self.get_logger().info("---> Stop Joy node")
        # self.stop_joy_node()
        # self.get_logger().info("<---")
        self.get_logger().info("---> Stop CPU node")
        self.stop_power_consumption_node()
        self.get_logger().info("<---")
        self.get_logger().info("---> Stop Leg driver node")
        self.stop_leg_driver()
        self.get_logger().info("<---")

    def run_experiment(
        self,
        mpc_solver_name: str,
        mpc_hpipm_mode: str,
        mpc_condensed_size: int,
        wbc_solver_name: str,
        wbc_scene_name: str,
        gs_gait: str,
        action_list: list,
        target_bag_file: str,
    ) -> bool:
        # Keep leg pose etc.
        self.get_logger().info("-----------------------")
        self.get_logger().info("Starting new experiment")
        self.get_logger().info("Current filename: " + target_bag_file)
        self.get_logger().info("---> Reset leg driver")
        self.leg_driver_switch_mode_client.wait_for_service()
        self.leg_driver_switch_mode_client.call(self.leg_driver_switch_mode_request)
        self.keep_pose_client.call(self.leg_driver_keep_pose_request)
        self.get_logger().info("<---")
        self.get_logger().info("---> Reset Simulation")
        self.reset_simulation()
        self.get_logger().info("<---")
        self.get_logger().info("---> Start controller")
        self.run_controller(
            mpc_solver_name,
            mpc_hpipm_mode,
            mpc_condensed_size,
            wbc_solver_name,
            wbc_scene_name,
            gs_gait,
        )
        self.get_logger().info("<---")
        self.get_logger().info("---> Sleep 7 secs")
        time.sleep(7)
        self.get_logger().info("<---")
        self.get_logger().info("---> Record bag")
        self.start_record_bag(os.path.join(self.working_folder, target_bag_file))
        self.get_logger().info("<---")
        self.get_logger().info("---> Playout actions")
        self.player.playout_experiment_async(action_list)
        self.get_logger().info("<---")
        exp_run = True
        failed = False
        while exp_run:
            time.sleep(0.5)
            self.get_logger().info("----check status-----")
            if self.robot_fell():
                self.get_logger().info("!!!!!!! ROBOT FELL !!!!!!!")
                self.get_logger().info("---> Stop playout bag")
                self.player.cancel_playout()
                self.get_logger().info("<---")
                self.failed_experiments.append(target_bag_file)
                exp_run = False
                failed = True
            elif not self.player.isPlaying():
                self.get_logger().info("<><><><><> Experiment successfull <><><><><><>")
                exp_run = False
        self.get_logger().info("---> Stop record bag")
        self.stop_record_bag()
        self.get_logger().info("<---")
        self.get_logger().info("---> Stop controller")
        self.stop_controller()
        self.get_logger().info("<---")
        self.get_logger().info("---> Sleep 3 secs")
        time.sleep(3)
        self.get_logger().info("<---")
        return not failed


def main(args=None):
    # Main script
    rclpy.init(args=args)
    solver_experiments_node = SolverExperiments("src/solver_experiments/results")

    def run_func():
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(solver_experiments_node)
        while rclpy.ok():
            solver_experiments_node.get_logger().info("Running spinner")
            executor.spin()
        return

    runner = threading.Thread(target=run_func)
    runner.start()

    experiment_stand = [
        # 5 sec standing:
        [5, QuadControlTarget(world_z=0.3)],
        #  do rp each for 5 sec
        [2, QuadControlTarget(world_z=0.3, pitch=radians(30))],
        [2, QuadControlTarget(world_z=0.3, pitch=radians(-30))],
        [2, QuadControlTarget(world_z=0.3, roll=radians(30))],
        [2, QuadControlTarget(world_z=0.3, roll=radians(-30))],
        [1, QuadControlTarget(world_z=0.3)],
        # now yaw carefully to +-30 degrees
        [2.0, QuadControlTarget(world_z=0.3, hybrid_theta_dot=radians(30))],
        [1, QuadControlTarget(world_z=0.3)],
        [4.0, QuadControlTarget(world_z=0.3, hybrid_theta_dot=radians(-30))],
        [1, QuadControlTarget(world_z=0.3)],
        [2.0, QuadControlTarget(world_z=0.3, hybrid_theta_dot=radians(30))],
        # increase and decrese a bit the hight
        [1.0, QuadControlTarget(world_z=0.4)],
        [1.0, QuadControlTarget(world_z=0.2)],
        [1.0, QuadControlTarget(world_z=0.3)],
    ]

    experiment_trot = [
        # 5 sec trotting and nothing
        [5, QuadControlTarget(world_z=0.3)],
        # 5 sec increase vel forward
        [0.2, QuadControlTarget(world_z=0.3, body_x_dot=0.1)],
        [0.2, QuadControlTarget(world_z=0.3, body_x_dot=0.2)],
        [0.2, QuadControlTarget(world_z=0.3, body_x_dot=0.3)],
        [0.2, QuadControlTarget(world_z=0.3, body_x_dot=0.4)],
        [5.0, QuadControlTarget(world_z=0.3, body_x_dot=0.5)],
        # now little right curve 180 degrees
        [
            4.0,
            QuadControlTarget(
                world_z=0.3, body_x_dot=0.5, hybrid_theta_dot=radians(40)
            ),
        ],
        # now diagonal
        [0.2, QuadControlTarget(world_z=0.3, body_x_dot=0.5, body_y_dot=0.1)],
        [0.2, QuadControlTarget(world_z=0.3, body_x_dot=0.5, body_y_dot=0.2)],
        [0.2, QuadControlTarget(world_z=0.3, body_x_dot=0.5, body_y_dot=0.3)],
        [0.2, QuadControlTarget(world_z=0.3, body_x_dot=0.5, body_y_dot=0.4)],
        [3.0, QuadControlTarget(world_z=0.3, body_x_dot=0.4, body_y_dot=0.4)],
        # now only sidewards
        [0.2, QuadControlTarget(world_z=0.3, body_x_dot=0.4, body_y_dot=0.4)],
        [0.2, QuadControlTarget(world_z=0.3, body_x_dot=0.3, body_y_dot=0.4)],
        [0.2, QuadControlTarget(world_z=0.3, body_x_dot=0.2, body_y_dot=0.4)],
        [0.2, QuadControlTarget(world_z=0.3, body_x_dot=0.1, body_y_dot=0.4)],
        [6.0, QuadControlTarget(world_z=0.3, body_x_dot=0.0, body_y_dot=0.4)],
        # now stop
        [0.2, QuadControlTarget(world_z=0.3, body_x_dot=0.0, body_y_dot=0.4)],
        [0.2, QuadControlTarget(world_z=0.3, body_x_dot=0.0, body_y_dot=0.3)],
        [0.2, QuadControlTarget(world_z=0.3, body_x_dot=0.0, body_y_dot=0.2)],
        [0.2, QuadControlTarget(world_z=0.3, body_x_dot=0.0, body_y_dot=0.1)],
        [0.2, QuadControlTarget(world_z=0.3, body_x_dot=0.0, body_y_dot=0.0)],
        [2.0, QuadControlTarget(world_z=0.3, body_x_dot=0.0, body_y_dot=0.0)],
        # now curve in place
        [
            3.0,
            QuadControlTarget(
                world_z=0.3, hybrid_theta_dot=radians(40), body_x_dot=0.0
            ),
        ],
        # now backwards curve
        [
            0.2,
            QuadControlTarget(
                world_z=0.3, hybrid_theta_dot=radians(40), body_x_dot=-0.1
            ),
        ],
        [
            0.2,
            QuadControlTarget(
                world_z=0.3, hybrid_theta_dot=radians(40), body_x_dot=-0.2
            ),
        ],
        [
            0.2,
            QuadControlTarget(
                world_z=0.3, hybrid_theta_dot=radians(40), body_x_dot=-0.3
            ),
        ],
        [
            0.2,
            QuadControlTarget(
                world_z=0.3, hybrid_theta_dot=radians(40), body_x_dot=-0.4
            ),
        ],
        [
            5.0,
            QuadControlTarget(
                world_z=0.3, hybrid_theta_dot=radians(0), body_x_dot=-0.5
            ),
        ],
        # stop again
        [
            0.2,
            QuadControlTarget(
                world_z=0.3, hybrid_theta_dot=radians(0), body_x_dot=-0.4
            ),
        ],
        [
            0.2,
            QuadControlTarget(
                world_z=0.3, hybrid_theta_dot=radians(0), body_x_dot=-0.3
            ),
        ],
        [
            0.2,
            QuadControlTarget(
                world_z=0.3, hybrid_theta_dot=radians(0), body_x_dot=-0.2
            ),
        ],
        [
            0.2,
            QuadControlTarget(
                world_z=0.3, hybrid_theta_dot=radians(0), body_x_dot=-0.1
            ),
        ],
        [
            3.0,
            QuadControlTarget(world_z=0.3, hybrid_theta_dot=radians(0), body_x_dot=0.0),
        ],
    ]

    mpc_solvers = [
        "PARTIAL_CONDENSING_HPIPM",
        "PARTIAL_CONDENSING_OSQP",
        "FULL_CONDENSING_QPOASES",
        "FULL_CONDENSING_DAQP",
        "FULL_CONDENSING_HPIPM",
    ]
    mpc_hpipm_modes = ["SPEED", "SPEED_ABS", "BALANCE", "ROBUST"]
    wbc_solvers = ["EiquadprogSolver", "ProxQPSolver", "QPOasesSolver", "HPIPMSolver"]
    wbc_scenes = ["AccelerationSceneReducedTSID", "AccelerationSceneTSID"]
    mpc_condensed_sizes = [
        20,
        19,
        18,
        17,
        16,
        15,
        14,
        13,
        12,
        11,
        10,
        9,
        8,
        7,
        6,
        5,
        4,
        3,
        2,
        1,
    ]
    max_trials = 3

    for mpc_solver in mpc_solvers:
        if not mpc_solver.__contains__("PARTIAL"):
            mpc_condensed_sizes = [1]
        if not mpc_solver.__contains__("HPIPM"):
            mpc_hpipm_modes = ["xxxxx"]

        for mpc_hpipm_mode in mpc_hpipm_modes:
            for mpc_condensed_size in mpc_condensed_sizes:
                for wbc_scene in wbc_scenes:
                    for wbc_solver in wbc_solvers:
                        for experiment, experiment_name, experiment_gait in zip(
                            [experiment_stand, experiment_trot],
                            ["stand_exp", "trot_exp"],
                            ["STAND", "TROT"],
                        ):
                            for iteration in range(max_trials):
                                target_bag = (
                                    mpc_solver
                                    + "-"
                                    + mpc_hpipm_mode
                                    + "-"
                                    + str(mpc_condensed_size)
                                    + "-"
                                    + experiment_name
                                    + "-"
                                    + str(iteration)
                                    + "-"
                                    + wbc_solver
                                    + "-"
                                    + wbc_scene
                                )
                                success = solver_experiments_node.run_experiment(
                                    mpc_solver,
                                    mpc_hpipm_mode,
                                    mpc_condensed_size,
                                    wbc_solver,
                                    wbc_scene,
                                    experiment_gait,
                                    experiment,
                                    target_bag,
                                )
                                if success:
                                    print("Experiment successful no more trials")
                                    break
                                else:
                                    print(
                                        "Experiment failed -> if more trials available try again"
                                    )
    print("STOP --->")
    solver_experiments_node.STOP()
    print("<-----")
    print("JOIN RUNNER --->")
    rclpy.shutdown()
    runner.join()
    print("<----- runner stopped")
    solver_experiments_node.destroy_node()
    print("Bye...")


if __name__ == "__main__":
    main()
