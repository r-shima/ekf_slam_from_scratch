from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, SetLaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, \
                                 TextSubstitution

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
            name='color',
            default_value='purple',
            choices=['purple', 'red', 'green', 'blue'],
            description='Color of the robot. Default is purple.'
        ),
        SetLaunchConfiguration(name='rvizconfig',
                               value=[FindPackageShare('nuturtle_description'),
                               TextSubstitution(text='/config/basic_'),
                               LaunchConfiguration('color'),
                               TextSubstitution(text='.rviz')]),
        Node(
            package='joint_state_publisher',
            namespace=LaunchConfiguration('color'),
            executable='joint_state_publisher',
            condition=LaunchConfigurationEquals('use_jsp', 'true'),
            on_exit=Shutdown()
        ),
        Node(
            package='robot_state_publisher',
            namespace=LaunchConfiguration('color'),
            executable='robot_state_publisher',
            parameters=[
                {'frame_prefix': PathJoinSubstitution([LaunchConfiguration('color'), '']),
                 'robot_description':
                 Command([ExecutableInPackage("xacro", "xacro"), " ",
                          PathJoinSubstitution(
                         [FindPackageShare("nuturtle_description"),
                         "urdf/turtlebot3_burger.urdf.xacro"]),
                         " color:=", LaunchConfiguration('color')])}],
            on_exit=Shutdown()
        ),
        Node(
            package='rviz2',
            namespace=LaunchConfiguration('color'),
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution(
                            [FindPackageShare('nuturtle_description'),
                            LaunchConfiguration('rvizconfig')])],
            condition=LaunchConfigurationEquals('use_rviz', 'true'),
            on_exit=Shutdown()
        )
    ])