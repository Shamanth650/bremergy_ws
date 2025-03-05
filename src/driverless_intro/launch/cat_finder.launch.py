from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    track_file_path = PathJoinSubstitution([
        FindPackageShare('driverless_intro'),
        'tracks',
        LaunchConfiguration('track_file')
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'track_file',
            default_value='FSG.csv',  # Default track file
            description='Name of the track CSV file (in src/driverless_intro/tracks)'
        ),

        # Launch RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),

        # Node to visualize cones in RViz
        Node(
            package='driverless_intro',
            executable='cone_visualizer.py',
            output='screen'
        ),

        # Node to publish cone positions from the track file
        Node(
            package='driverless_intro',
            executable='cat_visualizer.py',
            output='screen'
        ),

        # Node to publish cone positions from the track file
        Node(
            package='driverless_intro',
            executable='cone_publisher',
            arguments=[track_file_path],
            output='screen'
        ),

        # Node to publish cone positions from the track file
        Node(
            package='driverless_intro',
            executable='nearest_cone',
            arguments=[track_file_path],
            output='screen'
        ),

        # Node to publish the random cat position
        Node(
            package='driverless_intro',
            executable='cat_position.py',
            arguments=[LaunchConfiguration('track_file')],
            output='screen'
        ),
    ])