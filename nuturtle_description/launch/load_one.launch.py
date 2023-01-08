from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import Shutdown
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_rviz', 
            default_value='true', 
            choices=['true', 'false'],
            description='Flag to enable rviz'
        ),
        DeclareLaunchArgument(
            name='use_jsp', 
            default_value='true', 
            choices=['true', 'false'],
            description='Flag to enable joint_state_publisher'
        ),
        DeclareLaunchArgument(
            name='model', 
            default_value=str(get_package_share_path('nuturtle_description') / 
                              'turtlebot3_burger.urdf.xacro'),
            description='Absolute path to robot urdf file'
        ),
        DeclareLaunchArgument(
            name='color',
            default_value='purple',
            choices=['purple', 'black', 'dark', 'light_black', 'blue', 'green', 
                    'grey', 'orange', 'brown', 'red', 'white'],
            description='Color of the robot. Default is purple.'
        ),
        Node(
            package='joint_state_publisher', 
            executable='joint_state_publisher', 
            condition=LaunchConfigurationEquals('use_jsp', 'true'),
            on_exit=Shutdown()
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': 
                Command([ExecutableInPackage("xacro", "xacro"), " ",
                         PathJoinSubstitution(
                        [FindPackageShare("nuturtle_description"), 
                        "urdf/turtlebot3_burger.urdf.xacro"]),
                        " color:=", LaunchConfiguration('color')])}],
            on_exit=Shutdown()
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution(
                            [FindPackageShare("nuturtle_description"), 
                            "config/basic_purple.rviz"])],
            condition=LaunchConfigurationEquals('use_rviz', 'true'),
            on_exit=Shutdown()
        )
    ])