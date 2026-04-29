import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction


def launch_setup(context):
    compiled = os.environ.get('need_compile', 'False')

    if compiled == 'True':
        slam_pkg = get_package_share_directory('slam')
    else:
        slam_pkg = '/home/ubuntu/ros2_ws/src/slam'

    sim = LaunchConfiguration('sim').perform(context)

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_pkg, 'launch', 'slam.launch.py')),
        launch_arguments={
            'sim': sim,
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
