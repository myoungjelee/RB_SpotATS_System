from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='head_follow_controller',
            executable='head_follow_controller',
            name='head_follow_controller',
            output='screen',
            parameters=[
                {'cmd_topic': '/cmd_vel'},
                {'tf_base_frame': 'body'},
                {'tf_camera_frame': 'Camera'},
                {'kp_align': 1.2},
                {'wz_cap': 2.5},
                {'yaw_align_sign': -1.0},  # 반대로 돌면 +1.0 로 변경
                {'yaw_deadband_deg': 1.0},
                {'vx_max': 2.5},
                {'vx_min': 0.0},
                {'vx_curve': 'cos'},
                {'hz': 20.0},
                {'log_period': 0.5},
            ]
        )
    ])
