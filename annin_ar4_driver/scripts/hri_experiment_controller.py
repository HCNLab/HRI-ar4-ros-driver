#!/usr/bin/env python3
"""
HRI Experiment Controller

Controls robot for HRI experiment. Supports simulation mode (--sim).

Usage:
    python3 hri_experiment_controller.py         # With MoveIt
    python3 hri_experiment_controller.py --sim   # Simulation only
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import threading
import sys
import random


class HRIExperimentController(Node):
    def __init__(self, simulation_mode=False):
        super().__init__('hri_experiment_controller')
        
        self.simulation_mode = simulation_mode
        
        # ROS2 Subscriptions
        self.create_subscription(String, '/unity/robot_command', self.command_callback, 10)
        
        # ROS2 Publishers
        self.status_pub = self.create_publisher(String, '/unity/robot_status', 10)
        self.eeg_marker_pub = self.create_publisher(String, '/unity/eeg_marker', 10)
        
        # Joint names
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        # Predefined positions (radians)
        self.positions = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'cylinder': [-0.12, 1.25, -1.19, 0.0, 1.45, 0.0],
            'cube': [-0.26, 0.9, -0.32, 0.0, 1.0, 0.0],
            'press_offset': [0.0, 0.15, -0.15, 0.0, 0.0, 0.0],
            'press_miss_offset': [0.25, 0.15, -0.15, 0.0, 0.0, 0.0], # 5cm offset in X
            'press_air_offset': [0.0, 0.01, 0.01, 0.0, 0.0, 0.0],  # Shallow press (Air)
        }
        
        # Timing settings (optimized for high trial count paradigm)
        self.timing = {
            'move_home_to_target': 1.0,
            'move_target_to_target': 0.8,
            'press_down': 0.3,
            'hold': 0.4,
            'press_up': 0.3,
            'move_target_to_home': 1.0,
        }
        
        if simulation_mode:
            self.get_logger().info("=" * 50)
            self.get_logger().info("SIMULATION MODE - No robot movement")
            self.get_logger().info("=" * 50)
        else:
            # Action clients for MoveIt
            self.arm_client = ActionClient(self, FollowJointTrajectory, 
                                          '/joint_trajectory_controller/follow_joint_trajectory')
            self.gripper_client = ActionClient(self, GripperCommand, 
                                              '/gripper_controller/gripper_cmd')
            
            self.get_logger().info("Waiting for action servers...")
            if not self.arm_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().warn("Arm action server not available!")
            if not self.gripper_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().warn("Gripper action server not available!")
        
        self.get_logger().info("Python markers use ROS relay only. Unity LSL relay is authoritative.")

        # Sequence counter for cross-process timing joins (Python -> Unity)
        self.marker_seq = 0
        self.marker_seq_lock = threading.Lock()

        self.get_logger().info("HRI Experiment Controller Ready!")

    def next_marker_seq(self):
        with self.marker_seq_lock:
            self.marker_seq += 1
            return self.marker_seq

    def command_callback(self, msg):
        command = msg.data.strip().upper()
        self.get_logger().info(f"Received command: {command}")
        
        thread = threading.Thread(target=self.execute_command, args=(command,))
        thread.start()

    def execute_command(self, full_command):
        try:
            parts = full_command.split(':')
            command = parts[0]
            target_arg = parts[1] if len(parts) > 1 else None

            if command == "PRESS_CORRECT":
                self.execute_press_sequence(mode="CORRECT")
            elif command == "PRESS_WRONG":
                self.execute_press_sequence(mode="WRONG")
            elif command == "PRESS_MISS":
                self.execute_press_sequence(mode="MISS", target_arg=target_arg)
            elif command == "PRESS_AIR":
                self.execute_press_sequence(mode="AIR", target_arg=target_arg)
            elif command == "PRESS_FREEZE":
                self.execute_press_sequence(mode="FREEZE")
            elif command == "PRESS_DOUBLE":
                self.execute_press_sequence(mode="DOUBLE")
            elif command == "PRESS_ABORT":
                self.execute_press_sequence(mode="ABORT")
            elif command == "GO_HOME":
                duration = self.timing['move_target_to_home']
                self.get_logger().info(f"GO_HOME: returning to home [{duration:.2f}s]")
                self.move_to_position('home', duration)
                self.publish_status("COMPLETE")
            else:
                self.get_logger().warn(f"Unknown command: {command}")
                return
            
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            self.publish_status("ERROR")

    def execute_press_sequence(self, mode="CORRECT", target_arg=None):
        """Execute the press sequence"""
        
        # Marker codes (matching Unity LSLManager)
        # Robot normal: 20-29, Robot error: 120-129
        MARKER_TRIAL_START = 1
        MARKER_ROBOT_PRESS_CYL = 21
        MARKER_ROBOT_PRESS_CUBE = 22
        MARKER_ROBOT_PRESS_WRONG = 121
        MARKER_ROBOT_PRESS_MISS = 122
        MARKER_ROBOT_PRESS_AIR = 124   # New: Air press
        MARKER_ROBOT_PRESS_FREEZE = 125 # New: Freeze
        MARKER_ROBOT_PRESS_DOUBLE = 126 # New: Double tap
        MARKER_ROBOT_PRESS_ABORT = 127  # New: Abort
        
        # Define sequence based on mode
        if mode == "CORRECT" or mode == "AIR" or mode == "FREEZE" or mode == "DOUBLE" or mode == "ABORT":
            self.get_logger().info(f"{mode} sequence: Cylinder -> Cube")
            sequence = ['cylinder', 'cube']
        elif mode == "MISS":
            self.get_logger().info("MISS sequence: Cylinder (Miss) -> Cube")
            sequence = ['cylinder', 'cube']
        else: # WRONG
            self.get_logger().info("WRONG sequence: Cube -> Cylinder")
            sequence = ['cube', 'cylinder']

        # Determine which step will have the error (0 or 1)
        error_step_idx = 0
        if mode in ["MISS", "AIR", "FREEZE", "DOUBLE", "ABORT"]:
            if target_arg:
                # Unity specified which target to mess up
                target_lower = target_arg.lower()
                if target_lower in sequence:
                    error_step_idx = sequence.index(target_lower)
                    self.get_logger().info(f"Target specified from Unity: {target_lower} (Step {error_step_idx})")
                else:
                    self.get_logger().warn(f"Specified target '{target_lower}' not in sequence! Fallback to random.")
                    error_step_idx = random.choice([0, 1])
            else:
                error_step_idx = random.choice([0, 1])
                self.get_logger().info(f"Random Error Step Selected: {error_step_idx} ({sequence[error_step_idx]})")
            
        for idx, target in enumerate(sequence):
            step_num = idx + 1
            if idx == 0:
                move_segment = "home_to_target"
                duration = self.timing['move_home_to_target']
            else:
                move_segment = "target_to_target"
                duration = self.timing['move_target_to_target']
            self.get_logger().info(
                f"Step {step_num}: Moving to {target} [{move_segment}, {duration:.2f}s]"
            )
            
            # FREEZE ERROR logic: Pause mid-movement
            if mode == "FREEZE" and idx == error_step_idx:
                self.get_logger().info("Applying FREEZE (Hesitation)...")
                # Move halfway? (Simplified: just pause before move)
                time.sleep(1.5) 
            
            # 1. Move to position
            self.move_to_position(target, duration)
            time.sleep(duration)  # Wait for movement

            # ABORT ERROR Logic: Skip press and return
            if mode == "ABORT" and idx == error_step_idx:
                self.get_logger().info("Applying ABORT (Mission Abandon)...")
                time.sleep(1.0) # Hesitate then abort
                self.send_marker(MARKER_ROBOT_PRESS_ABORT)
                continue # Skip press, go to next target or home
            
            # 2. Press down
            # Determine offset type
            if mode == "MISS" and idx == error_step_idx:
                offset = self.positions['press_miss_offset']
                self.get_logger().info("Applying SPATIAL DEVIATION (MISS)")
                # Tell Unity to ignore press detection for this target
                self.publish_eeg_marker(f"IGNORE:{target}")
            elif mode == "AIR" and idx == error_step_idx:
                offset = self.positions['press_air_offset']
                self.get_logger().info("Applying AIR PRESS")
                # Tell Unity to ignore press detection for this target
                self.publish_eeg_marker(f"IGNORE:{target}")
            else:
                offset = self.positions['press_offset']
                
            press_pos = [
                self.positions[target][i] + offset[i]
                for i in range(6)
            ]
            
            # Marker logic:
            # Send marker immediately before each press segment.
            if mode == "CORRECT":
                marker_code = MARKER_ROBOT_PRESS_CYL if target == 'cylinder' else MARKER_ROBOT_PRESS_CUBE
            elif mode == "MISS" and idx == error_step_idx:
                marker_code = MARKER_ROBOT_PRESS_MISS
            elif mode == "AIR" and idx == error_step_idx:
                marker_code = MARKER_ROBOT_PRESS_AIR
            elif mode == "FREEZE" and idx == error_step_idx:
                marker_code = MARKER_ROBOT_PRESS_FREEZE
            elif mode == "DOUBLE" and idx == error_step_idx:
                marker_code = MARKER_ROBOT_PRESS_DOUBLE
            elif mode == "WRONG" and idx == 0:
                # Error marker only on first press (wrong order detection point)
                marker_code = MARKER_ROBOT_PRESS_WRONG
            else:
                # Normal press for other steps of error trials
                marker_code = MARKER_ROBOT_PRESS_CYL if target == 'cylinder' else MARKER_ROBOT_PRESS_CUBE

            self.send_marker(marker_code)
            
            # Press down (after marker sent)
            duration = self.timing['press_down']
            self.move_to_joint_position(press_pos, duration)
            time.sleep(duration)  # Wait for press
            
            # 3. Hold
            time.sleep(self.timing['hold'])
            
            # 4. Move back up
            duration = self.timing['press_up']
            self.move_to_position(target, duration)
            time.sleep(duration)  # Wait for return

            # DOUBLE TAP ERROR Logic
            if mode == "DOUBLE" and idx == error_step_idx:
                self.get_logger().info("Applying DOUBLE TAP...")
                # Press down again
                self.move_to_joint_position(press_pos, duration/2) # Faster re-press
                time.sleep(0.1)
                # Up again
                self.move_to_position(target, duration/2)
                time.sleep(0.1)
        
        # Publish status BEFORE return-to-home so Unity can emit COMPLETE marker
        # and start post-action observation from the same event.
        self.publish_status("COMPLETE")
        self.get_logger().info("COMPLETE sent, returning home (overlapping with post-action)...")

        # 5. Return home (overlaps with Unity post-action observation period)
        home_duration = self.timing['move_target_to_home']
        self.get_logger().info(f"Return segment: target_to_home [{home_duration:.2f}s]")
        self.move_to_position('home', home_duration)
        time.sleep(home_duration)

        self.get_logger().info("Sequence complete")

    def move_to_position(self, position_name, duration=3.0):
        """Move to a predefined position"""
        if position_name not in self.positions:
            self.get_logger().error(f"Unknown position: {position_name}")
            return False
        return self.move_to_joint_position(self.positions[position_name], duration)

    def move_to_joint_position(self, joint_positions, duration=2.0):
        """Move arm to specified joint positions"""
        if self.simulation_mode:
            time.sleep(duration)
            return True
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        
        goal.trajectory.points = [point]
        
        # Send goal
        future = self.arm_client.send_goal_async(goal)
        
        # Wait for result
        timeout = duration + 5.0
        start = time.time()
        while time.time() - start < timeout:
            if future.done():
                break
            time.sleep(0.1)
        
        return future.done()

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f"Status: {status}")

    def send_marker(self, code):
        """Publish a marker payload for Unity immediate relay only."""
        seq = self.next_marker_seq()
        py_mono_ns = time.perf_counter_ns()
        py_wall_ns = time.time_ns()

        path = "relay"
        payload = (
            f"{code}|seq={seq}|py_mono_ns={py_mono_ns}|"
            f"py_wall_ns={py_wall_ns}|path={path}"
        )
        self.publish_eeg_marker(payload)

    def publish_eeg_marker(self, marker):
        msg = String()
        msg.data = marker
        self.eeg_marker_pub.publish(msg)
        self.get_logger().info(f">>> EEG Marker: {marker}")


def main(args=None):
    rclpy.init(args=args)
    
    simulation_mode = '--sim' in sys.argv
    node = HRIExperimentController(simulation_mode=simulation_mode)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
