import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    compiled = os.environ.get('need_compile', 'False')

    if compiled == 'True':
        navigation_pkg = get_package_share_directory('navigation')
        xf_pkg = get_package_share_directory('xf_mic_asr_offline')
        navpro_pkg = get_package_share_directory('navpro')
    else:
        navigation_pkg = '/home/ubuntu/ros2_ws/src/navigation'
        xf_pkg = '/home/ubuntu/ros2_ws/src/xf_mic_asr_offline'
        navpro_pkg = '/home/ubuntu/ros2_ws/src/NavPro'

    robot_name = LaunchConfiguration('robot_name',
                                     default=os.environ.get('HOST', '/')).perform(context)
    master_name = LaunchConfiguration('master_name',
                                      default=os.environ.get('MASTER', '/')).perform(context)
    map_name = LaunchConfiguration('map_name', default='map_01').perform(context)

    frame_prefix = '' if robot_name == '/' else f'{robot_name}/'
    map_frame = (f'{frame_prefix}map' if robot_name == master_name
                 else f'{master_name}/map')

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_pkg, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'map': map_name,
            'master_name': master_name,
            'robot_name': robot_name,
        }.items(),
    )

    mic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(xf_pkg, 'launch', 'mic_init.launch.py')),
    )

    navpro_node = Node(
        package='navpro',
        executable='navpro_node',
        name='navpro_node',
        output='screen',
        parameters=[
            os.path.join(navpro_pkg, 'config', 'navpro_params.yaml'),
            {'map_frame': map_frame},
        ],
    )

    # Delay NavPro node startup until nav2 stack is active
    navpro_delayed = TimerAction(period=15.0, actions=[navpro_node])

    return [
        navigation_launch,
        mic_launch,
        navpro_delayed,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='map_01',
                              description='Map file name (without .yaml extension)'),
        DeclareLaunchArgument('robot_name', default_value='/'),
        DeclareLaunchArgument('master_name', default_value='/'),
        OpaqueFunction(function=launch_setup),
    ])


if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
