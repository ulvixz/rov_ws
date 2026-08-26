import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyTriggerMixer(Node):

    def __init__(self):
        super().__init__('joy_trigger_mixer')

        # Subscribe to the raw joystick data published by joy_node
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Publish the modified joystick message
        # with an additional virtual axis for vertical motion
        self.publisher = self.create_publisher(
            Joy,
            '/joy_rov',
            10
        )

        self.get_logger().info(
            'Joy Trigger Mixer started: '
            'virtual axis 6 = (L2 - R2) / 2'
        )

    def joy_callback(self, msg):

        # The controller must provide at least axes 0 through 5
        if len(msg.axes) < 6:
            self.get_logger().warn(
                'Expected at least 6 joystick axes.'
            )
            return

        # DualSense trigger axes
        l2 = msg.axes[2]
        r2 = msg.axes[5]

        # On the DualSense controller:
        # released trigger     = +1
        # fully pressed trigger = -1
        #
        # Therefore:
        # no trigger pressed -> 0
        # L2 pressed         -> -1
        # R2 pressed         -> +1
        #
        # This creates a single virtual axis
        # for vertical ROV movement.
        vertical = (l2 - r2) / 2.0

        # Create a new Joy message
        output_msg = Joy()

        # Preserve the original timestamp/header
        output_msg.header = msg.header

        # Keep only the controller axes used by the ROV
        # axis 0 = Left X
        # axis 1 = Left Y
        # axis 2 = L2
        # axis 3 = Right X
        # axis 4 = Right Y
        # axis 5 = R2
        output_msg.axes = list(msg.axes[:6])

        # Preserve all controller buttons
        output_msg.buttons = list(msg.buttons)

        # Add the calculated vertical motion as virtual axis 6
        output_msg.axes.append(vertical)

        # Publish the modified joystick message
        self.publisher.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)

    node = JoyTriggerMixer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
