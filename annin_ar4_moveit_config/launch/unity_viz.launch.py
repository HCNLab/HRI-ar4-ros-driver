import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ar_model_arg = DeclareLaunchArgument("ar_model", default_value="mk3", description="Model of AR4")
    ar_model_config = LaunchConfiguration("ar_model")
    tf_prefix_arg = DeclareLaunchArgument("tf_prefix", default_value="", description="Prefix for AR4 tf_tree")
    tf_prefix = LaunchConfiguration("tf_prefix")

    # 1. Robot Description (URDF)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("annin_ar4_description"), "urdf", "ar.urdf.xacro"]), " ",
        "ar_model:=", ar_model_config, " ",
        "tf_prefix:=", tf_prefix,
    ])
    robot_description = {"robot_description": robot_description_content}

    # 2. Joint State Publisher (Unity 입력 수신)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'source_list': ['/unity_joint_states'], # Unity에서 오는 토픽 수신
            'rate': 30
        }]
    )

    # # 테스트용 GUI 실행 (Unity 대신)
    # joint_state_publisher = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui'
    # )

    # 3. Robot State Publisher (TF 발행)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # 4. Unity Bridge
    unity_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('annin_ar4_driver'), 'launch', 'unity_bridge.launch.py')
        ])
    )

    # 5. RViz
    rviz_config_file = os.path.join(get_package_share_directory('annin_ar4_moveit_config'), 'rviz', 'basic.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[robot_description]
    )

    return LaunchDescription([
        ar_model_arg,
        tf_prefix_arg,
        joint_state_publisher,
        robot_state_publisher,
        unity_bridge,
        rviz_node
    ])
