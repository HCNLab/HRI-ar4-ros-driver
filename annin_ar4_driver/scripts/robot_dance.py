#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import math

class RobotDancer(Node):

    def __init__(self):
        super().__init__('robot_dancer')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3', 
            'joint_4', 'joint_5', 'joint_6'
        ]
        
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server found! Let\'s dance! 💃')

    def send_goal(self, points):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = points
        
        self.get_logger().info('Sending dance move...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted! Moving...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Move finished! Next move coming up...')
        # 여기서 다음 동작을 트리거할 수도 있습니다.

    def create_dance_trajectory(self):
        points = []
        
        # Move 1: Home
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point1.time_from_start = Duration(sec=2, nanosec=0)
        points.append(point1)

        # Move 2: Wave Left
        point2 = JointTrajectoryPoint()
        point2.positions = [0.5, -0.2, -0.5, 0.0, 0.0, 0.0]
        point2.time_from_start = Duration(sec=4, nanosec=0)
        points.append(point2)

        # Move 3: Wave Right
        point3 = JointTrajectoryPoint()
        point3.positions = [-0.5, -0.2, -0.5, 0.0, 0.0, 0.0]
        point3.time_from_start = Duration(sec=6, nanosec=0)
        points.append(point3)
        
        # Move 4: Look Up
        point4 = JointTrajectoryPoint()
        point4.positions = [0.0, -0.5, -1.0, 0.0, -1.0, 0.0]
        point4.time_from_start = Duration(sec=8, nanosec=0)
        points.append(point4)

        # Move 5: Back Home
        point5 = JointTrajectoryPoint()
        point5.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point5.time_from_start = Duration(sec=10, nanosec=0)
        points.append(point5)

        return points

def main(args=None):
    rclpy.init(args=args)
    dancer = RobotDancer()

    # 춤 동작 생성
    dance_moves = dancer.create_dance_trajectory()
    
    # 춤 시작
    dancer.send_goal(dance_moves)

    # ROS 노드 실행 유지
    try:
        rclpy.spin(dancer)
    except KeyboardInterrupt:
        pass
    
    dancer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
