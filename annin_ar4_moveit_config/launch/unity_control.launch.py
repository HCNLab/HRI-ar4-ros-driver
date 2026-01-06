from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ar", package_name="annin_ar4_moveit_config").to_moveit_configs()
    
    # 기본 데모 런치 파일 생성
    # source_list를 추가하여 외부 토픽(/unity_joint_states)을 받아들이도록 설정
    launch_description = generate_demo_launch(moveit_config)
    
    # joint_state_publisher 노드 찾아서 파라미터 수정 (source_list 추가)
    # 하지만 generate_demo_launch로 생성된 노드를 직접 수정하기 어려우므로,
    # 별도의 joint_state_publisher를 실행하고 기존 것을 덮어쓰거나,
    # 간단하게 Unity Bridge만 추가하여 실행합니다.
    
    # Unity Bridge 실행
    unity_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('annin_ar4_driver'), 'launch', 'unity_bridge.launch.py')
        ])
    )
    
    # joint_state_publisher_gui 대신 Unity 입력을 받는 joint_state_publisher 실행
    # 주의: demo.launch.py는 기본적으로 gui를 실행하므로, 충돌할 수 있습니다.
    # 따라서 여기서는 demo.launch.py를 사용하지 않고 필요한 노드만 직접 구성하는 것이 좋습니다.
    # 하지만 복잡도를 줄이기 위해, 사용자가 demo.launch.py를 실행한 상태에서
    # Unity가 /joint_states를 직접 발행하지 않고 /unity_joint_states를 발행하면
    # ROS 쪽에서 이를 합쳐주는 노드가 필요합니다.
    
    return LaunchDescription([
        unity_bridge,
        # 여기에 추가적인 설정이 필요할 수 있습니다.
    ])

# 위 방식보다는, 기존 demo.launch.py를 실행할 때 파라미터를 변경하는 것이 좋습니다.
# 하지만 demo.launch.py 내부 구조를 모르므로, 
# 가장 확실한 방법은 'joint_state_publisher'의 'source_list' 파라미터를 사용하는 것입니다.
