import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    orb_dvl2_share = get_package_share_directory('aqua_slam_ros2')

    vocab_path = os.path.join(orb_dvl2_share, 'Vocabulary', 'ORBvoc.txt')
    config_path = os.path.join(orb_dvl2_share, 'data', 'underwater_orbslam3_blue_gx5_short.yaml')
    default_model = os.path.join(orb_dvl2_share, 'urdf', 'bluerov.urdf')
    rviz_config = os.path.join(orb_dvl2_share, 'launch', 'falcon_tightly.rviz')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model,
        description='URDF model file'
    )

    robot_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': open(default_model).read()
        }]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world2odom',
        arguments=[
            "0", "0", "0",
            "0", "0", "0", "1",
            "odom", "orb_slam"
        ]
    )

    stereo_dvl = Node(
        package='aqua_slam_ros2',
        executable='stereo_dvl2',
        name='stereo_dvl',
        output='screen',
        arguments=[vocab_path, config_path, "false"],
        parameters=[{
            'ORBSLAM3_tightly.out_path': '/home/da/project/ros/orb_dvl2_ws/src/dvl2/orb3_result/',
            'ORBSLAM3_tightly.traj_path': '/home/da/project/ros/orb_dvl2_ws/src/dvl2/dvl2_results/stamped_traj_estimate0.txt',
            'ORBSLAM3_tightly.map_file': '/home/da/project/ros/orb_dvl2_ws/src/dvl2/orb3_result/Altlas.osa',
            'ORBSLAM3_tightly.is_load_map': False,
            'ros__parameters': {'log_level': 'DEBUG'}
        }]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        static_tf,
        stereo_dvl,
        robot_description,
        rviz
    ])
