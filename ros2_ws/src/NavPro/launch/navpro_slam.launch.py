import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    compiled = os.environ.get('need_compile', 'False')

    if compiled == 'True':
        slam_pkg = get_package_share_directory('slam')
    else:
        slam_pkg = '/home/ubuntu/ros2_ws/src/slam'

    sim = LaunchConfiguration('sim').perform(context)

    # Include only slam_base — robot base nodes are already running via bringup
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, 'launch', 'include', 'slam_base.launch.py')),
        launch_arguments={
            'use_sim_time': 'true' if sim == 'true' else 'false',
            'enable_save': 'true',
        }.items(),
    )

    return [slam_launch]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false',
                              description='Use simulation clock'),
        OpaqueFunction(function=launch_setup),
    ])


if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
