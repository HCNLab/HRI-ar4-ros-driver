#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import math

class VRTeleop(Node):

    def __init__(self):
        super().__init__('vr_teleop')
        
        # 1. Unity에서 오는 목표 위치 구독
        self.create_subscription(PoseStamped, '/vr_target_pose', self.target_callback, 10)
        
        # 2. IK 서비스 클라이언트 (MoveIt)
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # 3. 로봇 제어 액션 클라이언트
        self.traj_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        
        # 4. Visualization marker publisher for RViz
        self.marker_pub = self.create_publisher(Marker, '/vr_target_marker', 10)
        
        # 현재 관절 상태 구독 (IK 초기값용)
        self.current_joints = JointState()
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        # 5. VR Hand Tracking 그리퍼 제어
        self.create_subscription(Float32, '/vr/gripper_command', self.gripper_callback, 10)
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        self.gripper_open_pos = 0.014
        self.gripper_close_pos = -0.02
        
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3', 
            'joint_4', 'joint_5', 'joint_6'
        ]
        
        # Counter for debugging
        self.msg_count = 0
        self.ik_success_count = 0
        self.ik_fail_count = 0
        
        # Wait for IK service
        self.get_logger().info('Waiting for IK service...')
        self.ik_client.wait_for_service(timeout_sec=5.0)
        self.gripper_client.wait_for_server(timeout_sec=5.0)
        self.get_logger().info('VR Teleop Ready! Hand tracking + gripper control enabled 🎮')

    def joint_state_callback(self, msg):
        self.current_joints = msg

    def gripper_callback(self, msg):
        """Handle VR hand pinch commands for gripper control"""
        # msg.data: 0.0 = close, 1.0 = open
        if msg.data > 0.5:
            position = self.gripper_open_pos
            action = "OPEN"
        else:
            position = self.gripper_close_pos
            action = "CLOSE"
        
        self.get_logger().info(f'VR Gripper Command: {action}')
        
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = 100.0
        
        # Send goal without waiting
        self.gripper_client.send_goal_async(goal_msg)

    def target_callback(self, msg):
        self.msg_count += 1
        
        # Publish visualization marker
        self.publish_marker(msg)
        
        # Log every 50th message to avoid spam
        if self.msg_count % 50 == 0:
            self.get_logger().info(
                f'Received {self.msg_count} poses. IK Success: {self.ik_success_count}, Fail: {self.ik_fail_count}'
            )
            self.get_logger().info(
                f'Target position: x={msg.pose.position.x:.3f}, '
                f'y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}'
            )
        
        # Check if we have current joint states
        if not self.current_joints.name:
            if self.msg_count == 1:
                self.get_logger().warn('No joint states received yet!')
            return
        
        # IK 요청 생성
        request = GetPositionIK.Request()
        request.ik_request.group_name = "ar_manipulator"
        request.ik_request.robot_state.joint_state = self.current_joints
        request.ik_request.avoid_collisions = False  # Allow free movement for VR
        request.ik_request.pose_stamped = msg
        request.ik_request.timeout.sec = 0
        request.ik_request.timeout.nanosec = 50000000  # 50ms timeout for fast response
        
        # Call IK service
        future = self.ik_client.call_async(request)
        future.add_done_callback(self.ik_response_callback)

    def publish_marker(self, pose_stamped):
        """Publish visualization marker for RViz"""
        marker = Marker()
        marker.header = pose_stamped.header
        marker.ns = "vr_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose_stamped.pose
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        self.marker_pub.publish(marker)

    def ik_response_callback(self, future):
        try:
            response = future.result()
            if response.error_code.val == response.error_code.SUCCESS:
                self.ik_success_count += 1
                # IK 성공! 관절 각도 추출
                target_joints = response.solution.joint_state.position
                # 로봇 이동
                self.move_robot(target_joints)
            else:
                self.ik_fail_count += 1
                # Log first few failures for debugging
                if self.ik_fail_count <= 3:
                    self.get_logger().warn(f'IK Failed with error code: {response.error_code.val}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def move_robot(self, positions):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        # IK 결과에는 그리퍼 등 다른 관절도 포함될 수 있으므로, 이름에 맞는 것만 필터링하거나 순서대로 매핑해야 함.
        # MoveIt IK 결과는 보통 정의된 순서대로 나옴.
        # 안전을 위해 앞의 6개만 사용 (AR4는 6축)
        point.positions = list(positions[:6])
        point.time_from_start = Duration(sec=0, nanosec=200000000) # 0.2초 안에 이동 (더 빠른 반응)
        
        goal_msg.trajectory.points = [point]
        
        # 비동기 전송 (결과 기다리지 않음 - 연속 제어 위해)
        self.traj_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VRTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
