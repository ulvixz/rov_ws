#!/usr/bin/env python3

import math

import rclpy

from geometry_msgs.msg import Twist
from mavros_msgs.msg import ManualControl, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from sensor_msgs.msg import Joy


# ============================================================
# EXPO:
#   1.0 = Linear response
#   1.5 = Mild exponential response
#   2.0 = Strong exponential response
#   3.0 = Very soft response around the joystick center
#
# Axis scales:
#   1.0 = 100% output
#   0.5 = 50% output
#   0.2 = 20% output
# ============================================================

EXPO = 1.8

FORWARD_SCALE = 1.0
VERTICAL_SCALE = 1.0

YAW_SCALE = 0.8
PITCH_SCALE = 0.3
ROLL_SCALE = 0.3


# ============================================================
#                  CONTROLLER SETTINGS
# ============================================================
#
# Current PS5 mapping:
#
#   OPTIONS  = ARM
#   CREATE   = DISARM
#
# Proposed mode mapping:
#
#   SQUARE   = MANUAL
#   TRIANGLE = STABILIZE
#   CIRCLE   = ALT_HOLD / Depth Hold
#
# Verify the button indices with:
#
#   ros2 topic echo /joy_rov --field buttons
#
# Change only the values below if your controller uses
# different button indices.
# ============================================================

JOY_TOPIC = "/joy_rov"

ARM_BUTTON = 9
DISARM_BUTTON = 8

MANUAL_MODE_BUTTON = 3
STABILIZE_MODE_BUTTON = 2
DEPTH_HOLD_MODE_BUTTON = 1


# ============================================================


class TwistToManualControl(Node):
    """Convert normalized ROS 2 Twist commands to MAVROS ManualControl."""

    def __init__(self) -> None:
        super().__init__("twist_to_manual_control")

        # Subscribe to autonomous movement commands.
        self.twist_subscriber = self.create_subscription(
            Twist,
            "/rov/autonomy/cmd_vel",
            self.twist_callback,
            10,
        )

        # Subscribe directly to joystick buttons.
        self.joy_subscriber = self.create_subscription(
            Joy,
            JOY_TOPIC,
            self.joy_callback,
            10,
        )

        # Subscribe to MAVROS vehicle state.
        # This allows us to track the actual mode and armed state.
        self.state_subscriber = self.create_subscription(
            State,
            "/mavros/state",
            self.state_callback,
            10,
        )

        # Publish converted commands to the MAVROS manual-control plugin.
        self.manual_control_publisher = self.create_publisher(
            ManualControl,
            "/mavros/manual_control/send",
            10,
        )

        # Create the MAVROS ARM / DISARM service client.
        self.arming_client = self.create_client(
            CommandBool,
            "/mavros/cmd/arming",
        )

        # Create the MAVROS flight-mode service client.
        self.set_mode_client = self.create_client(
            SetMode,
            "/mavros/set_mode",
        )

        # Store previous button states for rising-edge detection.
        self.previous_button_states = {}

        # Store the latest state reported by MAVROS.
        self.current_mode = ""
        self.current_armed = False

        self.get_logger().info(
            "Twist to ManualControl converter started."
        )

        self.get_logger().info(
            f"Control settings: "
            f"expo={EXPO}, "
            f"forward_scale={FORWARD_SCALE}, "
            f"vertical_scale={VERTICAL_SCALE}, "
            f"yaw_scale={YAW_SCALE}, "
            f"pitch_scale={PITCH_SCALE}, "
            f"roll_scale={ROLL_SCALE}"
        )

        self.get_logger().info(
            f"Controller buttons: "
            f"ARM={ARM_BUTTON}, "
            f"DISARM={DISARM_BUTTON}, "
            f"MANUAL={MANUAL_MODE_BUTTON}, "
            f"STABILIZE={STABILIZE_MODE_BUTTON}, "
            f"ALT_HOLD={DEPTH_HOLD_MODE_BUTTON}"
        )

    @staticmethod
    def clamp(value: float, minimum: float, maximum: float) -> float:
        """Limit a value to the specified range."""
        return max(minimum, min(value, maximum))

    @staticmethod
    def shape_axis(value: float, scale: float) -> float:
        """
        Apply exponential response and output scaling to an axis.

        The sign of the original command is preserved.
        Full joystick deflection still reaches the configured scale limit.
        """

        # Limit the raw input to the normalized range.
        value = max(-1.0, min(1.0, value))

        # Apply exponential response while preserving direction.
        value = math.copysign(
            abs(value) ** EXPO,
            value,
        )

        # Apply the configured output scale.
        value *= scale

        # Keep the final value inside the normalized range.
        return max(-1.0, min(1.0, value))

    @staticmethod
    def get_button(
        joy_message: Joy,
        button_index: int,
    ) -> int:
        """Safely read a joystick button."""

        if button_index < 0:
            return 0

        if button_index >= len(joy_message.buttons):
            return 0

        return int(
            joy_message.buttons[button_index]
        )

    def button_pressed_once(
        self,
        joy_message: Joy,
        button_index: int,
    ) -> bool:
        """
        Detect a single button press.

        The function returns True only when the button changes
        from released to pressed.
        """

        current_state = self.get_button(
            joy_message,
            button_index,
        )

        previous_state = self.previous_button_states.get(
            button_index,
            0,
        )

        self.previous_button_states[button_index] = current_state

        return (
            current_state == 1
            and previous_state == 0
        )

    def joy_callback(
        self,
        joy_message: Joy,
    ) -> None:
        """Handle ARM, DISARM, and flight-mode buttons."""

        # --------------------------------------------------------
        # ARM
        # --------------------------------------------------------

        if self.button_pressed_once(
            joy_message,
            ARM_BUTTON,
        ):
            self.send_arm_command(True)

        # --------------------------------------------------------
        # DISARM
        # --------------------------------------------------------

        if self.button_pressed_once(
            joy_message,
            DISARM_BUTTON,
        ):
            self.send_arm_command(False)

        # --------------------------------------------------------
        # MANUAL MODE
        # --------------------------------------------------------

        if self.button_pressed_once(
            joy_message,
            MANUAL_MODE_BUTTON,
        ):
            self.send_mode_command(
                "MANUAL"
            )

        # --------------------------------------------------------
        # STABILIZE MODE
        # --------------------------------------------------------

        if self.button_pressed_once(
            joy_message,
            STABILIZE_MODE_BUTTON,
        ):
            self.send_mode_command(
                "STABILIZE"
            )

        # --------------------------------------------------------
        # DEPTH HOLD MODE
        # --------------------------------------------------------
        #
        # ArduSub uses ALT_HOLD as the MAVLink mode name
        # for Depth Hold.
        # --------------------------------------------------------

        if self.button_pressed_once(
            joy_message,
            DEPTH_HOLD_MODE_BUTTON,
        ):
            self.send_mode_command(
                "ALT_HOLD"
            )

    def state_callback(
        self,
        state_message: State,
    ) -> None:
        """
        Track the actual flight mode and armed state
        reported by MAVROS.
        """

        # Log only when the flight mode changes.
        if state_message.mode != self.current_mode:
            self.current_mode = state_message.mode

            self.get_logger().info(
                f"Vehicle mode changed to: "
                f"{self.current_mode}"
            )

        # Log only when the armed state changes.
        if state_message.armed != self.current_armed:
            self.current_armed = state_message.armed

            if self.current_armed:
                arm_state = "ARMED"
            else:
                arm_state = "DISARMED"

            self.get_logger().info(
                f"Vehicle state changed to: "
                f"{arm_state}"
            )

    def send_arm_command(
        self,
        arm: bool,
    ) -> None:
        """Send an ARM or DISARM request through MAVROS."""

        action = (
            "ARM"
            if arm
            else "DISARM"
        )

        # Do not block the node if the MAVROS service is unavailable.
        if not self.arming_client.service_is_ready():

            self.get_logger().warning(
                f"{action} request ignored: "
                "MAVROS arming service is not available."
            )

            return

        request = CommandBool.Request()

        request.value = arm

        future = self.arming_client.call_async(
            request
        )

        future.add_done_callback(
            lambda completed_future:
            self.arming_response_callback(
                completed_future,
                arm,
            )
        )

    def arming_response_callback(
        self,
        future,
        arm: bool,
    ) -> None:
        """Process the MAVROS ARM / DISARM response."""

        action = (
            "ARM"
            if arm
            else "DISARM"
        )

        try:
            response = future.result()

            if response.success:

                self.get_logger().info(
                    f"{action} command accepted."
                )

            else:

                self.get_logger().warning(
                    f"{action} command rejected. "
                    f"MAV_RESULT={response.result}"
                )

        except Exception as error:

            self.get_logger().error(
                f"{action} service call failed: "
                f"{error}"
            )

    def send_mode_command(
        self,
        mode_name: str,
    ) -> None:
        """Send a flight-mode change request through MAVROS."""

        # Avoid sending another request if the vehicle
        # is already in the requested mode.
        if self.current_mode == mode_name:

            self.get_logger().info(
                f"Vehicle is already in "
                f"{mode_name} mode."
            )

            return

        # Do not block the node if the MAVROS service is unavailable.
        if not self.set_mode_client.service_is_ready():

            self.get_logger().warning(
                f"{mode_name} request ignored: "
                "MAVROS set-mode service is not available."
            )

            return

        request = SetMode.Request()

        # base_mode is zero because an ArduSub custom
        # mode name is provided.
        request.base_mode = 0

        request.custom_mode = mode_name

        future = self.set_mode_client.call_async(
            request
        )

        future.add_done_callback(
            lambda completed_future:
            self.mode_response_callback(
                completed_future,
                mode_name,
            )
        )

    def mode_response_callback(
        self,
        future,
        mode_name: str,
    ) -> None:
        """Process the MAVROS flight-mode response."""

        try:
            response = future.result()

            if response.mode_sent:

                self.get_logger().info(
                    f"{mode_name} mode request sent."
                )

            else:

                self.get_logger().warning(
                    f"{mode_name} mode request "
                    "was rejected by MAVROS."
                )

        except Exception as error:

            self.get_logger().error(
                f"{mode_name} mode service call failed: "
                f"{error}"
            )

    def twist_callback(
        self,
        twist_message: Twist,
    ) -> None:
        """Convert an incoming Twist message to ManualControl."""

        # Apply exponential response and scaling
        # to each control axis.
        forward = self.shape_axis(
            twist_message.linear.x,
            FORWARD_SCALE,
        )

        vertical = self.shape_axis(
            twist_message.linear.z,
            VERTICAL_SCALE,
        )

        roll = self.shape_axis(
            -twist_message.angular.x,
            ROLL_SCALE,
        )

        pitch = self.shape_axis(
            twist_message.angular.y,
            PITCH_SCALE,
        )

        yaw = self.shape_axis(
            twist_message.angular.z,
            YAW_SCALE,
        )

        # Create the MAVROS ManualControl message.
        manual_message = ManualControl()

        manual_message.header.stamp = (
            self.get_clock().now().to_msg()
        )

        # --------------------------------------------------------
        # FORWARD / BACKWARD
        # --------------------------------------------------------
        #
        # -1.0 ... +1.0 becomes -1000 ... +1000.
        # --------------------------------------------------------

        manual_message.x = (
            forward * 1000.0
        )

        # The current six-thruster vehicle
        # cannot move laterally.
        manual_message.y = 0.0

        # --------------------------------------------------------
        # VERTICAL
        # --------------------------------------------------------
        #
        # ArduSub vertical command:
        #
        # -1.0 -> 0
        #  0.0 -> 500 neutral
        # +1.0 -> 1000
        # --------------------------------------------------------

        manual_message.z = (
            500.0
            + vertical * 500.0
        )

        # --------------------------------------------------------
        # ROTATIONAL COMMANDS
        # --------------------------------------------------------
        #
        # -1.0 ... +1.0 becomes -1000 ... +1000.
        # --------------------------------------------------------

        manual_message.r = (
            yaw * 1000.0
        )

        manual_message.s = (
            pitch * 1000.0
        )

        manual_message.t = (
            roll * 1000.0
        )

        # MANUAL_CONTROL button bitfields are not used here.
        #
        # ARM / DISARM and mode selection are handled
        # through MAVROS services instead.
        manual_message.buttons = 0
        manual_message.buttons2 = 0

        # Bit 0 enables pitch field "s".
        # Bit 1 enables roll field "t".
        manual_message.enabled_extensions = (
            0b00000011
        )

        # Auxiliary axes are not used.
        manual_message.aux1 = 0.0
        manual_message.aux2 = 0.0
        manual_message.aux3 = 0.0
        manual_message.aux4 = 0.0
        manual_message.aux5 = 0.0
        manual_message.aux6 = 0.0

        # Publish the converted message.
        self.manual_control_publisher.publish(
            manual_message
        )

        self.get_logger().info(
            "ManualControl published: "
            f"x={manual_message.x:.0f}, "
            f"y={manual_message.y:.0f}, "
            f"z={manual_message.z:.0f}, "
            f"r={manual_message.r:.0f}, "
            f"s={manual_message.s:.0f}, "
            f"t={manual_message.t:.0f}"
        )


def main(args=None) -> None:

    rclpy.init(
        args=args
    )

    node = TwistToManualControl()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        node.get_logger().info(
            "Node stopped by the user."
        )

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
