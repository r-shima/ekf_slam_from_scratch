from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import Shutdown
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import Command, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare


def generate_launch_description():
    # path = get_package_share_path('nuturtle_description')
    # default_model_path = path / 'turtlebot3_burger.urdf.xacro'
    # default_rviz_config_path = path / 'basic_purple.rviz'

    # use_rviz = DeclareLaunchArgument(name='use_rviz', default_value='true', choices=['true', 'false'],
    #                                 description='Flag to enable rviz')
    # use_jsp = DeclareLaunchArgument(name='use_jsp', default_value='true', choices=['true', 'false'],
    #                                 description='Flag to enable joint_state_publisher')
    # model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
    #                                   description='Absolute path to robot urdf file')
    # rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
    #                                  description='Absolute path to rviz config file')

    # robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
    #                                    value_type=str)

    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     parameters=[{'robot_description': robot_description}]
    # )

    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     condition=LaunchConfigurationEquals('use_jsp', 'true')
    # )

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', LaunchConfiguration('rvizconfig')],
    #     condition=LaunchConfigurationEquals('use_rviz', 'true')
    # )

    return LaunchDescription([
        # use_rviz,
        # use_jsp,
        # model_arg,
        # rviz_arg,
        # joint_state_publisher_node,
        # robot_state_publisher_node,
        # rviz_node
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
        # DeclareLaunchArgument(
        #     name='rvizconfig', 
        #     default_value=str(get_package_share_path('nuturtle_description') / 
        #                       'basic_purple.rviz'),
        #     description='Absolute path to rviz config file'
        # ),
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
                        "urdf/turtlebot3_burger.urdf.xacro"])])}],
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