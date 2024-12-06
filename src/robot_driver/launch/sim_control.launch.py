from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # MuJoCo simulation node
    mujoco_sim_node = Node(
        package='mujoco_sim',
        executable='mujoco_sim_node',
        name='mujoco_sim',
        output='screen'
    )

    # Simulation driver node
    sim_driver_node = Node(
        package='robot_driver',
        executable='sim_driver_node',
        name='sim_driver',
        output='screen'
    )

    # Trajopt node
    trajopt_node = Node(
        package='trajopt',
        executable='trajopt_node',
        name='trajopt',
        arguments=["/home/a2rlab/Indy7/indy7-control/src/trajopt/trajectories/figure8_traj_eePos_meters.csv"],
        output='screen'
    )

    return LaunchDescription([
        mujoco_sim_node,
        sim_driver_node,
        trajopt_node
    ]) 