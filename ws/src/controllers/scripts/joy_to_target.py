#!/usr/bin/env python3
import numpy as np
import rclpy
from interfaces.msg import QuadControlTarget
from rcl_interfaces.srv import SetParameters
from rclpy import Parameter
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
from unitree_go.msg import WirelessController


# Simple script, that allows to reconfigure the quads pose using mpc controller with all legs in contact
# new trajectories are generated based on joystick inputs
# for now just send static values of the desired state.


class Joy2Target(Node):
    def __init__(self):
        super().__init__("joy2target")

        # general parameters
        self.declare_parameter("update_freq", 20.0)  # update frequency
        self.declare_parameter("init_robot_height", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("max_robot_height", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("max_acceleration", rclpy.Parameter.Type.DOUBLE)

        self.max_acceleration = self.get_parameter("max_acceleration").get_parameter_value().double_value
        self.v = np.array([0.0,0.0])

        # subscribers
        self.joy_sub = self.create_subscription(Joy, "joy", self.joy_sub_callback, 1)
        self.unitree_joy_sub = self.create_subscription(WirelessController, "wirelesscontroller",
                                                        self.wireless_controller_sub_callback, 1)

        # parameter client
        self.param_client = self.create_client(SetParameters, '/mit_controller_node/set_parameters')
        self.emerg_damp_client = self.create_client(Trigger, '/set_emergency_damping_mode')
        self.reset_covar_client = self.create_client(Trigger, '/reset_state_estimation_covariances')

        # publishers
        self.timer_period = 1 / self.get_parameter("update_freq").get_parameter_value().double_value
        self.timer = self.create_timer(self.timer_period,
                                       self.timer_callback)  # INFO: Timer callback is now called in subscriber callback

        self.publisher_ = self.create_publisher(QuadControlTarget, "quad_control_target", 10)

        # zero pose/twist, when no command is sent
        self.offset = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "x": 0.0, "y": 0.0, "z": 0.0}
        self.scaling = {"roll": 0.5, "pitch": -0.5, "yaw": 1.0, "x": 0.5, "y": 0.5, "z": 0.1}

        self.world_z = self.get_parameter("init_robot_height").get_parameter_value().double_value
        self.world_max_z = self.get_parameter("max_robot_height").get_parameter_value().double_value
        self.gait_sequencer = "Simple"

        # zi s not set
        self.js_mapping = {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
        }

        # e.g. llr = left joystick left/right
        self.ax2idx = {}
        self.btn2idx = {}

        # logitech joystick mode D
        self.ax2idx["logitechD"] = {"llr": 0, "lud": 1, "rlr": 2, "rud": 3, "ulr": 4, "uud": 5}
        self.btn2idx["logitechD"] = {"rb": 5, "rt": 7, "lb": 4, "lt": 6, "A": 1, "B": 2, "X": 0, "Y": 3, "back": 8,
                                     "start": 9, "l": 10, "r": 11}

        # logitech joystick mode X
        self.ax2idx["logitechX"] = {"llr": 0, "lud": 1, "rlr": 3, "rud": 4, "ulr": 6, "uud": 7, "rt": 5, "lt": 2}
        self.btn2idx["logitechX"] = {"rb": 5, "rt": None, "lb": 4, "lt": None, "A": 0, "B": 1, "X": 2, "Y": 3,
                                     "back": 6,
                                     "start": 7, "l": 9, "r": 10}

        # xbox360 joystick
        self.ax2idx["xbox360"] = {"llr": 0, "lud": 1, "rlr": 3, "rud": 4, "ulr": 6, "uud": 7, "rt": 5, "lt": 2}
        self.btn2idx["xbox360"] = {"rb": 5, "rt": 7, "lb": 4, "lt": 6, "A": 0, "B": 1, "X": 3, "Y": 2, "back": 8,
                                   "start": 9, "l": 11, "r": 12}  # FIXME: find correct buttons

        self.Abef = False
        self.Bbef = False
        self.Xbef = False
        self.Ybef = False

        self.Upref = False
        self.Downref = False
        self.Leftref = False
        self.Rightref = False
        self.start_ref = False

        self.lt_init = False
        self.rt_init = False

        self.current_swing_height = 0.05

        # deactivate joystick
        self.wireless_active = True
        self.joy_active = True

    def wireless_controller_sub_callback(self, msg):
        left_js_lr = -msg.lx
        left_js_ud = msg.ly
        right_js_lr = -msg.rx
        right_js_ud = msg.ry
        button_values = [int(i) for i in np.binary_repr(msg.keys, 16)]
        upper_js_lr = button_values[2] - button_values[0]
        upper_js_ud = button_values[3] - button_values[1]

        shift_r = button_values[15]
        shift_l = button_values[14]
        alt_r = button_values[11]
        alt_l = button_values[10]
        rt = int(alt_r)
        lt = int(alt_l)

        A = button_values[7]
        B = button_values[6]
        X = button_values[5]
        Y = button_values[4]
        L = button_values[2]  # in this case upper left instead of left joystick
        R = button_values[0]  # in this case upper right instead of right joystick
        start = button_values[13]
        back = button_values[12]


        # detect rising edges
        A_rising = A and not self.Abef
        B_rising = B and not self.Bbef
        X_rising = X and not self.Xbef
        Y_rising = Y and not self.Ybef
        Up_rising = button_values[3] and not self.Upref
        Down_rising = button_values[1] and not self.Downref
        Left_rising = button_values[0] and not self.Leftref
        Right_rising = button_values[2] and not self.Rightref
        start_rising = start and not self.start_ref


        # emergency stop
        if (alt_r or alt_l):
            req = Trigger.Request()
            self.emerg_damp_client.call_async(req)

        if left_js_lr or right_js_lr or left_js_ud or right_js_ud:
            self.joy_active = False
            self.wireless_active = True

        # only emergency stop active when deactivated
        if not self.wireless_active:
            return
        # deactivate joy msg
        # self.joy_active = False

        self.send_control_input(left_js_lr, left_js_ud, right_js_lr, right_js_ud, upper_js_lr, upper_js_ud,rt, lt,
                                shift_r, shift_l, alt_r, alt_l,
                                A, B, X, Y, L, R,
                                A_rising, B_rising, X_rising, Y_rising, Up_rising, Down_rising, Left_rising, Right_rising,
                                start, start_rising)

    def joy_sub_callback(self, msg):
        if len(msg.axes) == 6:
            js = "logitechD"
        elif len(msg.axes) == 8:
            if (len(msg.buttons) == 11):
                js = "logitechX"
            else:
                js = "xbox360"
        else:
            raise Exception("Joystick type not recognized")

        left_js_lr = msg.axes[self.ax2idx[js]["llr"]]
        left_js_ud = msg.axes[self.ax2idx[js]["lud"]]
        right_js_lr = msg.axes[self.ax2idx[js]["rlr"]]
        right_js_ud = msg.axes[self.ax2idx[js]["rud"]]
        upper_js_lr = -msg.axes[self.ax2idx[js]["ulr"]]
        upper_js_ud = msg.axes[self.ax2idx[js]["uud"]]


        shift_r = msg.buttons[self.btn2idx[js]["rb"]]
        shift_l = msg.buttons[self.btn2idx[js]["lb"]]
        if js == "logitechX":
            alt_r = int(msg.axes[self.ax2idx[js]["rt"]] < 0)
            alt_l = int(msg.axes[self.ax2idx[js]["lt"]] < 0)
        else:
            alt_r = msg.buttons[self.btn2idx[js]["rt"]]
            alt_l = msg.buttons[self.btn2idx[js]["lt"]]
        if js == "logitechD":
            rt = int(alt_r)
            lt = int(alt_l)
        else:
            rt = (msg.axes[self.ax2idx[js]["rt"]]-1.0)*-0.5
            lt = (msg.axes[self.ax2idx[js]["lt"]]-1.0)*-0.5
            if not self.rt_init:
                if abs(msg.axes[self.ax2idx[js]["rt"]]) > 0.1:
                    self.rt_init = True
                else:
                    rt = 0
            if not self.lt_init:
                if abs(msg.axes[self.ax2idx[js]["lt"]]) > 0.1:
                    self.lt_init = True
                else:
                    lt = 0
        A = msg.buttons[self.btn2idx[js]["A"]]
        B = msg.buttons[self.btn2idx[js]["B"]]
        X = msg.buttons[self.btn2idx[js]["X"]]
        Y = msg.buttons[self.btn2idx[js]["Y"]]
        L = msg.buttons[self.btn2idx[js]["l"]]  # press on left nob
        R = msg.buttons[self.btn2idx[js]["r"]]  # press on right nob
        start = msg.buttons[self.btn2idx[js]["start"]]

        # detect rising edges
        A_rising = A and not self.Abef
        B_rising = B and not self.Bbef
        X_rising = X and not self.Xbef
        Y_rising = Y and not self.Ybef

        Up_rising = upper_js_ud >= 0.5 and not self.Upref
        Down_rising = upper_js_ud <= -0.5 and not self.Downref
        Left_rising = upper_js_lr <= -0.5 and not self.Leftref
        Right_rising = upper_js_lr >= 0.5 and not self.Rightref

        start_rising = start and not self.start_ref

        # emergency stop
        if (L or R):
            req = Trigger.Request()
            self.emerg_damp_client.call_async(req)

        if left_js_lr or right_js_lr or left_js_ud or right_js_ud:
            self.joy_active = True
            self.wireless_active = False
        # only emergency stop active when deactivated
        if not self.joy_active:
            return

        self.send_control_input(left_js_lr, left_js_ud, right_js_lr, right_js_ud, upper_js_lr, upper_js_ud,rt, lt,
                                shift_r, shift_l, alt_r, alt_l,
                                A, B, X, Y, L, R,
                                A_rising, B_rising, X_rising, Y_rising, Up_rising, Down_rising, Left_rising, Right_rising,
                                start, start_rising)

    def send_control_input(self, left_js_lr, left_js_ud, right_js_lr, right_js_ud, upper_js_lr, upper_js_ud, rt, lt,
                           shift_r, shift_l, alt_r, alt_l,
                           A, B, X, Y, L, R,
                           A_rising, B_rising, X_rising, Y_rising, Up_rising, Down_rising, Left_rising, Right_rising,
                           start, start_rising):
        # fixed velocities for testing
        additional_x = 0.0
        further_additional_x = 0.0
        vel_factor = 1.0
        if shift_l and shift_r:
            if alt_l:
                further_additional_x += 0.4
            if alt_r:
                further_additional_x += 0.8
            if Y:
                additional_x += 0.1 / self.scaling["x"]
            if B:
                additional_x += 0.2 / self.scaling["x"]
            if A:
                additional_x += 0.3 / self.scaling["x"]
            if X:
                additional_x += 0.4 / self.scaling["x"]
            if Y or B or A or X:
                additional_x += further_additional_x / self.scaling["x"]
        if not shift_l and not shift_r:
            vel_factor += (1.0 + lt) * (1.0 + rt)

        self.js_mapping = {
            "roll": -right_js_lr if shift_r and not shift_l else 0.0,
            "pitch": -right_js_ud,
            "yaw": right_js_lr if not (shift_r and not shift_l)  else 0.0,
            "x": left_js_ud * vel_factor + additional_x,
            "y": left_js_lr * vel_factor,
            "z": upper_js_ud if not alt_r else 0.0,
        }

        msg = QuadControlTarget()
        # msg.body_x_dot = self.scaling["x"] * self.js_mapping["x"] + self.offset["x"]
        # msg.body_y_dot = self.scaling["y"] * self.js_mapping["y"] + self.offset["y"]
        msg.body_x_dot, msg.body_y_dot = self.clip_velocity(
            self.scaling["x"] * self.js_mapping["x"] + self.offset["x"],
            self.scaling["y"] * self.js_mapping["y"] + self.offset["y"]
        )
        msg.world_z = self.world_z
        msg.hybrid_theta_dot = self.scaling["yaw"] * self.js_mapping["yaw"] + self.offset["yaw"]
        msg.roll = self.scaling["roll"] * self.js_mapping["roll"] + self.offset["roll"]
        msg.pitch = self.scaling["pitch"] * self.js_mapping["pitch"] + self.offset["pitch"]

        self.publisher_.publish(msg)

        NEW_GAIT = ""
        NEW_GAIT_SEQUENCER = ""
        DELTA_SWING_HEIGHT = 0
        # if (shift_r and X_rising):
        #     NEW_GAIT = "TROT"
        # elif (shift_r and B_rising):
        #     NEW_GAIT = "FLYING_TROT"
        if (shift_l and shift_r):
            pass
        elif (shift_l and X_rising):
            NEW_GAIT = "PACE"
        elif (shift_l and B_rising):
            NEW_GAIT = "BOUND"
        elif (shift_l and A_rising):
            NEW_GAIT = "PRONK"
        elif (shift_l and Y_rising):
            NEW_GAIT_SEQUENCER = "Simple"
        elif (Right_rising or (alt_r and Up_rising)):
            DELTA_SWING_HEIGHT = 0.0125
        elif (Left_rising or (alt_r and Down_rising)):
            DELTA_SWING_HEIGHT = -0.0125
        elif (A_rising):
            NEW_GAIT = "STAND"
        elif (X_rising):
            NEW_GAIT = "WALKING_TROT"
        elif (B_rising):
            NEW_GAIT = "STATIC_WALK"
        elif (Y_rising):
            NEW_GAIT_SEQUENCER = "Adaptive"


        if (NEW_GAIT != ""):
            param_req = SetParameters.Request()
            if self.gait_sequencer == "Simple":
                param_req.parameters = [
                    Parameter(name='simple_gait_sequencer.gait', value=NEW_GAIT).to_parameter_msg()]
                # if self.gait_sequencer != "Simple":
                #     self.gait_sequencer = "Simple"
                #     param_req.parameters.append(
                #         Parameter(name="gait_sequencer", value=self.gait_sequencer).to_parameter_msg()
                #     )
            elif self.gait_sequencer == "Adaptive":
                if NEW_GAIT == "STAND":
                    self.gait_sequencer = "Simple"
                    param_req.parameters = [
                        Parameter(name='simple_gait_sequencer.gait', value=NEW_GAIT).to_parameter_msg(),
                        Parameter(name="gait_sequencer", value=self.gait_sequencer).to_parameter_msg(),
                    ]
                elif "TROT" in NEW_GAIT:
                    param_req.parameters = [
                        Parameter(name='adaptive_gait_sequencer.gait.switch_offsets', value=False).to_parameter_msg(),
                        Parameter(name='adaptive_gait_sequencer.gait.phase_offset',
                                  value=[0.0, 0.5, 0.5, 0.0]).to_parameter_msg(),
                    ]
                elif NEW_GAIT == "STATIC_WALK":
                    param_req.parameters = [
                        Parameter(name='adaptive_gait_sequencer.gait.switch_offsets', value=False).to_parameter_msg(),
                        Parameter(name='adaptive_gait_sequencer.gait.phase_offset',
                                  value=[0.0, 0.5, 0.75, 0.25]).to_parameter_msg(),
                    ]
                elif NEW_GAIT == "PACE":
                    param_req.parameters = [
                        Parameter(name='adaptive_gait_sequencer.gait.switch_offsets', value=False).to_parameter_msg(),
                        Parameter(name='adaptive_gait_sequencer.gait.phase_offset',
                                  value=[0.0, 0.5, 0.0, 0.5]).to_parameter_msg(),
                    ]
                elif NEW_GAIT == "BOUND":
                    param_req.parameters = [
                        Parameter(name='adaptive_gait_sequencer.gait.switch_offsets', value=False).to_parameter_msg(),
                        Parameter(name='adaptive_gait_sequencer.gait.phase_offset',
                                  value=[0.0, 0.0, 0.5, 0.5]).to_parameter_msg(),
                    ]
                elif NEW_GAIT == "PRONK":
                    param_req.parameters = [
                        Parameter(name='adaptive_gait_sequencer.gait.switch_offsets', value=False).to_parameter_msg(),
                        Parameter(name='adaptive_gait_sequencer.gait.phase_offset',
                                  value=[0.0, 0.0, 0.0, 0.0]).to_parameter_msg(),
                    ]

            self.param_client.call_async(param_req)
        if (NEW_GAIT_SEQUENCER not in {"", self.gait_sequencer}):
            self.gait_sequencer = NEW_GAIT_SEQUENCER
            param_req = SetParameters.Request()
            param_req.parameters = [
                Parameter(name="gait_sequencer", value=self.gait_sequencer).to_parameter_msg(),
                Parameter(name='adaptive_gait_sequencer.gait.switch_offsets', value=True).to_parameter_msg(),
            ]
            self.param_client.call_async(param_req)

        if DELTA_SWING_HEIGHT:
            self.current_swing_height = np.clip(self.current_swing_height + DELTA_SWING_HEIGHT, 0.0,
                                                self.world_z - 0.05)
            param_req = SetParameters.Request()
            param_req.parameters = [
                Parameter(name="slc_swing_height", value=self.current_swing_height).to_parameter_msg(),
            ]
            self.param_client.call_async(param_req)

        if start_rising:
            req = Trigger.Request()
            self.reset_covar_client.call_async(req)

        self.Abef = A
        self.Bbef = B
        self.Xbef = X
        self.Ybef = Y
        self.Upref = upper_js_ud >= 0.5
        self.Downref = upper_js_ud <= -0.5
        self.Leftref = upper_js_lr <= -0.5
        self.Rightref = upper_js_lr >= 0.5
        self.start_ref = start

    def timer_callback(self):
        self.world_z += (self.scaling["z"] * self.js_mapping["z"] + self.offset["z"]) * self.timer_period
        self.world_z = np.clip(self.world_z, 0.0, self.world_max_z)


    def clip_velocity(self, vx, vy):
        if self.max_acceleration == 0.0:
            self.v[0] = vx
            self.v[1] = vy
            return vx, vy
        v_new = np.array([vx,vy], dtype=np.float64)
        v_dif = (v_new - self.v)
        v_dif_norm = np.linalg.norm(v_dif)
        acc = v_dif_norm/self.timer_period
        if (abs(acc) > abs(self.max_acceleration)):
            v_new = self.v + (v_dif * abs(self.max_acceleration/acc))
        self.v = v_new
        return v_new[0], v_new[1]


def main(args=None):
    rclpy.init(args=args)
    joy2ee_node = Joy2Target()
    rclpy.spin(joy2ee_node)
    joy2ee_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
