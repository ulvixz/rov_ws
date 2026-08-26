import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_share = get_package_share_directory('rov_autonomy')

    config_file = os.path.join(
        package_share,
        'config',
        'ps5_rov.yaml'
    )

    # --------------------------------------------------
    # 1. PS5 Controller -> /joy
    # --------------------------------------------------
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[
            {
                'deadzone': 0.08,
                'autorepeat_rate': 20.0
            }
        ]
    )

    # --------------------------------------------------
    # 2. L2 + R2 -> virtual axis 6
    #
    # /joy -> /joy_rov
    # --------------------------------------------------
    trigger_mixer_node = Node(
        package='rov_autonomy',
        executable='joy_trigger_mixer',
        name='joy_trigger_mixer',
        output='screen'
    )

    # --------------------------------------------------
    # 3. /joy_rov -> Twist
    # --------------------------------------------------
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',

        parameters=[
            config_file
        ],

        remappings=[
            ('joy', '/joy_rov'),
            ('cmd_vel', '/rov/autonomy/cmd_vel')
        ]
    )

    # --------------------------------------------------
    # 4. Twist -> MAVROS MANUAL_CONTROL
    # --------------------------------------------------
    manual_control_node = Node(
        package='rov_autonomy',
        executable='twist_to_manual_control',
        name='twist_to_manual_control',
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        trigger_mixer_node,
        teleop_node,
        manual_control_node
    ])
