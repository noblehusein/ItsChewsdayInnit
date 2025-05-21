import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch argument for headless mode
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run without Gazebo if true'
    )
    
    # Get headless configuration
    headless = LaunchConfiguration('headless')
    
    # Include the Turtlebot3 Gazebo simulation launch file only if NOT headless
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'empty_world.launch.py')
        ),
        condition=UnlessCondition(headless)  # Only launch Gazebo if headless=false
    )

    # Motion controller node
    motion_controller = Node(
        package='motion_controller',
        executable='motion_controller_node',
        name='motion_controller',
        output='screen',
        remappings=[
            ('/vel_cmd', '/cmd_vel'),
        ]
    )

    # Target position issuer node
    target_pos_issuer = Node(
        package='target_pos_issuer',
        executable='target_pos_issuer_node',
        name='target_pos_issuer',
        output='screen'
    )

    return LaunchDescription([
        headless_arg,
        turtlebot_launch,
        motion_controller,
        target_pos_issuer
    ])