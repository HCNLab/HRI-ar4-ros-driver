#!/usr/bin/env python3
"""
Unity Pick & Place Integration Node

Receives pick/place positions from Unity and executes Pick & Place automatically.

Topics:
    Subscribed:
        /unity/pick_pose (PoseStamped) - Pick position from Unity
        /unity/place_pose (PoseStamped) - Place position from Unity
        /unity/execute_pnp (Bool) - Execute command
    
    Published:
        /unity/pnp_status (String) - Status feedback to Unity
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
import time
import threading


class UnityPickPlace(Node):

    def __init__(self):
        super().__init__('unity_pick_place')

        # Received poses from Unity
        self.pick_pose = None
        self.place_pose = None
        
        # Current joint state
        self.current_joints = JointState()

        # Subscribers
        self.create_subscription(PoseStamped, '/unity/pick_pose', self.pick_pose_callback, 10)
        self.create_subscription(PoseStamped, '/unity/place_pose', self.place_pose_callback, 10)
        self.create_subscription(Bool, '/unity/execute_pnp', self.execute_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Publishers
        self.status_pub = self.create_publisher(String, '/unity/pnp_status', 10)

        # Action clients
        self.arm_client = ActionClient(self, FollowJointTrajectory, 
                                        '/joint_trajectory_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        
        # IK service client
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        # Configuration
        self.gripper_open = 0.014
        self.gripper_close = 0.0
        self.approach_offset = 0.1  # 10cm above target
        
        self.get_logger().info('Waiting for action servers...')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.ik_client.wait_for_service()
        
        self.get_logger().info('Unity Pick & Place Ready! 🤖')
        self.publish_status('READY', 'Waiting for pick/place positions from Unity')

    def joint_state_callback(self, msg):
        self.current_joints = msg

    def pick_pose_callback(self, msg):
        # Ensure frame_id is set
        if not msg.header.frame_id:
            msg.header.frame_id = "base_link"
        # Set default orientation (gripper pointing down)
        self.set_default_orientation(msg)
        self.pick_pose = msg
        self.get_logger().info(f'Received Pick pose: ({msg.pose.position.x:.3f}, '
                               f'{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}) in {msg.header.frame_id}')
        self.publish_status('PICK_RECEIVED', f'Pick position received')

    def place_pose_callback(self, msg):
        # Ensure frame_id is set
        if not msg.header.frame_id:
            msg.header.frame_id = "base_link"
        # Set default orientation (gripper pointing down)
        self.set_default_orientation(msg)
        self.place_pose = msg
        self.get_logger().info(f'Received Place pose: ({msg.pose.position.x:.3f}, '
                               f'{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}) in {msg.header.frame_id}')
        self.publish_status('PLACE_RECEIVED', f'Place position received')

    def set_default_orientation(self, pose_stamped):
        """Set gripper orientation to point downward (Z-down)"""
        # Gripper pointing down: rotate 180 degrees around X axis
        # Quaternion for this rotation: (w=0, x=1, y=0, z=0) or similar
        # Common "tool down" orientation for pick and place
        pose_stamped.pose.orientation.x = 1.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 0.0

    def execute_callback(self, msg):
        if msg.data:
            self.get_logger().info('Received execute command!')
            if self.pick_pose is None:
                self.publish_status('ERROR', 'Pick pose not set')
                return
            if self.place_pose is None:
                self.publish_status('ERROR', 'Place pose not set')
                return
            
            # Execute pick and place in separate thread
            # This allows ROS callbacks to continue processing
            thread = threading.Thread(target=self.execute_pick_and_place)
            thread.start()

    def publish_status(self, status, message):
        msg = String()
        msg.data = f'{status}|{message}'
        self.status_pub.publish(msg)
        self.get_logger().info(f'Status: {status} - {message}')

    def compute_ik(self, pose_stamped):
        """Compute IK for a given pose - manual timeout loop"""
        if not self.current_joints.name:
            self.get_logger().error('No current joint state available')
            return None
            
        request = GetPositionIK.Request()
        request.ik_request.group_name = "ar_manipulator"
        request.ik_request.robot_state.joint_state = self.current_joints
        request.ik_request.avoid_collisions = False  # Faster IK
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.timeout.sec = 0
        request.ik_request.timeout.nanosec = 500000000  # 0.5s
        
        # Use async with manual timeout
        future = self.ik_client.call_async(request)
        
        timeout_sec = 5.0
        start_time = time.time()
        while not future.done():
            time.sleep(0.1)
            if time.time() - start_time > timeout_sec:
                self.get_logger().error('IK service timeout')
                return None
        
        try:
            response = future.result()
            if response is None:
                self.get_logger().error('IK response is None')
                return None
            if response.error_code.val == response.error_code.SUCCESS:
                return list(response.solution.joint_state.position[:6])
            else:
                self.get_logger().error(f'IK failed with error code: {response.error_code.val}')
                return None
        except Exception as e:
            self.get_logger().error(f'IK exception: {e}')
            return None

    def move_arm(self, positions, duration_sec=3.0):
        """Move arm to joint positions - fire and wait with sleep"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        goal_msg.trajectory.points = [point]
        
        # Send goal without waiting
        self.arm_client.send_goal_async(goal_msg)
        self.get_logger().info(f'Moving arm, waiting {duration_sec}s...')
        
        # Wait for movement to complete using sleep
        time.sleep(duration_sec + 1.0)
        return True

    def control_gripper(self, position):
        """Control gripper - fire and forget"""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = 100.0
        
        # Just send the goal, don't wait for result
        self.gripper_client.send_goal_async(goal_msg)
        self.get_logger().info(f'Gripper command sent: {position}')
        return True

    def create_approach_pose(self, pose_stamped, z_offset=0.05):
        """Create approach pose (above target) - reduced offset"""
        import copy
        approach = PoseStamped()
        approach.header.frame_id = pose_stamped.header.frame_id or "base_link"
        approach.pose.position.x = pose_stamped.pose.position.x
        approach.pose.position.y = pose_stamped.pose.position.y
        approach.pose.position.z = pose_stamped.pose.position.z + z_offset
        approach.pose.orientation = pose_stamped.pose.orientation
        return approach

    def execute_pick_and_place(self):
        """Execute full pick and place sequence"""
        self.publish_status('EXECUTING', 'Starting Pick & Place sequence')
        
        try:
            # 1. Open gripper
            self.publish_status('STEP_1', 'Opening gripper')
            self.control_gripper(self.gripper_open)
            time.sleep(0.5)
            
            # 2. Move to above pick position
            self.publish_status('STEP_2', 'Moving to pick approach')
            pick_approach = self.create_approach_pose(self.pick_pose, 0.1)
            joints = self.compute_ik(pick_approach)
            if joints is None:
                self.publish_status('ERROR', 'IK failed for pick approach')
                return
            self.move_arm(joints, 4.0)
            time.sleep(0.5)
            
            # 3. Descend to pick
            self.publish_status('STEP_3', 'Descending to pick')
            joints = self.compute_ik(self.pick_pose)
            if joints is None:
                self.publish_status('ERROR', 'IK failed for pick position')
                return
            self.move_arm(joints, 2.0)
            time.sleep(0.5)
            
            # 4. Close gripper
            self.publish_status('STEP_4', 'Grasping object')
            self.control_gripper(self.gripper_close)
            time.sleep(1.0)
            
            # 5. Lift
            self.publish_status('STEP_5', 'Lifting object')
            joints = self.compute_ik(pick_approach)
            if joints:
                self.move_arm(joints, 2.0)
            time.sleep(0.5)
            
            # 6. Move to above place position
            self.publish_status('STEP_6', 'Moving to place approach')
            place_approach = self.create_approach_pose(self.place_pose, 0.1)
            joints = self.compute_ik(place_approach)
            if joints is None:
                self.publish_status('ERROR', 'IK failed for place approach')
                return
            self.move_arm(joints, 4.0)
            time.sleep(0.5)
            
            # 7. Descend to place
            self.publish_status('STEP_7', 'Descending to place')
            joints = self.compute_ik(self.place_pose)
            if joints is None:
                self.publish_status('ERROR', 'IK failed for place position')
                return
            self.move_arm(joints, 2.0)
            time.sleep(0.5)
            
            # 8. Release
            self.publish_status('STEP_8', 'Releasing object')
            self.control_gripper(self.gripper_open)
            time.sleep(1.0)
            
            # 9. Lift from place
            self.publish_status('STEP_9', 'Lifting from place')
            joints = self.compute_ik(place_approach)
            if joints:
                self.move_arm(joints, 2.0)
            time.sleep(0.5)
            
            # 10. Return to Home position
            self.publish_status('STEP_10', 'Returning to home position')
            home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.move_arm(home_joints, 3.0)
            
            # 11. Close gripper
            self.publish_status('STEP_11', 'Closing gripper')
            self.control_gripper(self.gripper_close)
            time.sleep(0.5)
            
            self.publish_status('COMPLETE', 'Pick & Place completed successfully!')
            
        except Exception as e:
            self.publish_status('ERROR', f'Exception: {str(e)}')
            self.get_logger().error(f'Pick & Place failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UnityPickPlace()
    
    # Use MultiThreadedExecutor to allow callbacks to process while executing
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
