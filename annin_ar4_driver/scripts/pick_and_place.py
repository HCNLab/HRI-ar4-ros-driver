#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool, String
import time

class PickAndPlace(Node):

    def __init__(self):
        super().__init__('pick_and_place')

        # 1. 팔 컨트롤러 클라이언트
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

        # 2. 그리퍼 컨트롤러 클라이언트
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

        # 3. Unity grasp feedback subscribers
        self.grasp_confirmed = False
        self.last_feedback = ""
        self.grasp_sub = self.create_subscription(
            Bool, '/unity/grasp_success',
            self.on_grasp_feedback, 10
        )
        self.feedback_sub = self.create_subscription(
            String, '/unity/grasp_feedback',
            self.on_feedback_message, 10
        )

        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]

        # Configuration parameters
        self.grasp_confirmation_timeout = 5.0  # seconds
        self.physics_settle_delay = 1.0        # seconds
        self.use_unity_feedback = False         # Enable/disable Unity feedback

        self.get_logger().info('Waiting for action servers...')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info('Ready for Pick and Place with Unity feedback! 📦')

    def move_arm(self, positions, duration_sec=2.0):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info(f'Moving arm to {positions}...')
        future = self.arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        # 동작 완료 대기
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Arm goal rejected')
            return
            
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Arm movement finished.')

    def control_gripper(self, position, max_effort=100.0):
        # position: 0.0 (닫힘) ~ 0.04 (열림, 4cm)
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        self.get_logger().info(f'Gripper moving to {position}...')
        future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Gripper action finished.')

    def on_grasp_feedback(self, msg):
        """Callback for Unity grasp success/failure"""
        self.grasp_confirmed = msg.data
        if msg.data:
            self.get_logger().info('✓ Unity confirmed grasp success')
        else:
            self.get_logger().warn('✗ Unity reported grasp failure')

    def on_feedback_message(self, msg):
        """Callback for detailed Unity feedback"""
        self.last_feedback = msg.data
        self.get_logger().info(f'Unity feedback: {msg.data}')

    def wait_for_grasp_confirmation(self, timeout=None):
        """Wait for Unity to confirm grasp success or failure"""
        if not self.use_unity_feedback:
            return True  # Skip if Unity feedback disabled

        if timeout is None:
            timeout = self.grasp_confirmation_timeout

        self.grasp_confirmed = False
        start_time = time.time()

        self.get_logger().info(f'Waiting for Unity grasp confirmation (timeout: {timeout}s)...')

        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.grasp_confirmed:
                self.get_logger().info('✓ Grasp confirmed by Unity')
                return True

        self.get_logger().error(f'✗ Grasp confirmation timeout after {timeout}s')
        return False

    def run_scenario(self):
        """Run full pick-and-place scenario with Unity feedback"""
        self.get_logger().info('=== Starting Pick-and-Place Scenario ===')

        # 1. Home Position
        self.get_logger().info('[1/9] Moving to home position...')
        self.move_arm([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 2.0)
        time.sleep(self.physics_settle_delay)

        # 2. Open Gripper
        self.get_logger().info('[2/9] Opening gripper...')
        self.control_gripper(0.014)  # 14mm open
        time.sleep(self.physics_settle_delay)

        # 3. Approach Pick Location (위에서 접근)
        self.get_logger().info('[3/9] Approaching pick location from above...')
        pick_pose = [-1.637, 1.028, 0.536, 0.0, 0.0, 0.0]  # 실제 물체 위치
        pick_above_pose = [pick_pose[0], pick_pose[1] - 0.3, pick_pose[2] + 0.3, 0.0, 0.0, 0.0]  # 위에서 접근
        self.move_arm(pick_above_pose, 5.0)
        time.sleep(self.physics_settle_delay)

        # 4. Descend to Pick (내려와서 잡기 위치)
        self.get_logger().info('[4/9] Descending to pick position...')
        self.move_arm(pick_pose, 3.0)
        time.sleep(self.physics_settle_delay)

        # 5. Grasp with Unity confirmation
        self.get_logger().info('[5/9] Attempting grasp...')
        self.control_gripper(0.0)  # Close gripper

        if self.use_unity_feedback:
            if not self.wait_for_grasp_confirmation():
                self.get_logger().error('Grasp failed! Aborting scenario.')
                self.move_arm([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 3.0)
                return False
        else:
            time.sleep(1.0)  # Fallback delay without Unity feedback

        # 6. Lift Object (위로 들어올리기)
        self.get_logger().info('[6/9] Lifting object...')
        lift_pose = [pick_pose[0], pick_pose[1] - 0.3, pick_pose[2] + 0.3, 0.0, 0.0, 0.0]  # pick 위로
        self.move_arm(lift_pose, 2.0)
        time.sleep(self.physics_settle_delay)  

        # 7. Transit to Place Location (위에서 접근)
        self.get_logger().info('[7/9] Approaching place location from above...')
        place_pose = [1.554, 1.028, 0.536, 0.0, 0.0, 0.0]  # 실제 놓을 위치
        place_above_pose = [place_pose[0], place_pose[1] - 0.3, place_pose[2] + 0.3, 0.0, 0.0, 0.0]  # 위에서 접근
        self.move_arm(place_above_pose, 4.0)
        time.sleep(self.physics_settle_delay)

        # 8. Descend to Place (내려와서 놓기 위치)
        self.get_logger().info('[8/9] Descending to place position...')
        self.move_arm(place_pose, 3.0)
        time.sleep(self.physics_settle_delay)

        # 9. Release Object
        self.get_logger().info('[9/9] Releasing object...')
        self.control_gripper(0.014)  # Open gripper
        time.sleep(1.0)

        # 10. Lift from Place (위로 들어올리기)
        self.get_logger().info('[10] Lifting from place...')
        self.move_arm(place_above_pose, 2.0)
        time.sleep(self.physics_settle_delay)

        # 9. Return Home (첫 번째 사이클 완료)
        self.get_logger().info('[9/9] Returning home...')
        self.move_arm([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 3.0)
        time.sleep(self.physics_settle_delay)

        self.get_logger().info('=== First Cycle Complete! Starting Reverse Cycle... ===')

        # ========== 역방향 사이클 (Place → Pick) ==========
        
        # 10. Open Gripper
        self.get_logger().info('[R1] Opening gripper...')
        self.control_gripper(0.014)
        time.sleep(self.physics_settle_delay)

        # 11. Approach Place Location from above (이제 여기서 Pick)
        self.get_logger().info('[R2] Approaching place location from above...')
        self.move_arm(place_above_pose, 4.0)
        time.sleep(self.physics_settle_delay)

        # 12. Descend to Place Location to Pick
        self.get_logger().info('[R3] Descending to pick object...')
        self.move_arm(place_pose, 3.0)
        time.sleep(self.physics_settle_delay)

        # 13. Grasp
        self.get_logger().info('[R4] Grasping object...')
        self.control_gripper(0.0)
        time.sleep(1.0)

        # 14. Lift from Place
        self.get_logger().info('[R5] Lifting object...')
        self.move_arm(place_above_pose, 2.0)
        time.sleep(self.physics_settle_delay)

        # 15. Move to above Original Pick Location
        self.get_logger().info('[R6] Moving to original pick location...')
        self.move_arm(pick_above_pose, 4.0)
        time.sleep(self.physics_settle_delay)

        # 16. Descend to Pick Location to Place
        self.get_logger().info('[R7] Descending to place object...')
        self.move_arm(pick_pose, 3.0)
        time.sleep(self.physics_settle_delay)

        # 17. Release Object
        self.get_logger().info('[R8] Releasing object...')
        self.control_gripper(0.014)
        time.sleep(1.0)

        # 18. Lift from Pick
        self.get_logger().info('[R9] Lifting from pick...')
        self.move_arm(pick_above_pose, 2.0)
        time.sleep(self.physics_settle_delay)

        # 16. Return Home
        self.get_logger().info('[16/17] Returning home...')
        self.move_arm([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 3.0)
        
        # 17. Close Gripper
        self.get_logger().info('[17/17] Closing gripper...')
        self.control_gripper(0.0)

        self.get_logger().info('=== Full Pick-and-Place Cycle Complete! ===')
        return True

def main(args=None):
    rclpy.init(args=args)
    pnp = PickAndPlace()
    
    try:
        pnp.run_scenario()
    except KeyboardInterrupt:
        pass
    
    pnp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
