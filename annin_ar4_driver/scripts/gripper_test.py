#!/usr/bin/env python3
"""
Gripper Test Script
Tests gripper open/close functionality after reassembly
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
import time


class GripperTester(Node):

    def __init__(self):
        super().__init__('gripper_tester')
        
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd'
        )
        
        # Gripper position values
        self.positions = {
            'full_close': -0.02,
            'close': 0.0,
            'half': 0.007,
            'open': 0.014,      # 원래 값으로 복원
            'full_open': 0.02
        }
        
        self.get_logger().info('Waiting for gripper controller...')
        self.gripper_client.wait_for_server()
        self.get_logger().info('Gripper Tester Ready! 🔧')

    def move_gripper(self, position, name=""):
        """Move gripper to specified position"""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = 100.0
        
        self.get_logger().info(f'Moving gripper to {name} ({position}m)...')
        
        future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            self.get_logger().info(f'  ✅ {name} complete')
            return True
        else:
            self.get_logger().error(f'  ❌ {name} rejected')
            return False

    def run_test(self):
        """Run complete gripper test sequence"""
        self.get_logger().info('='*50)
        self.get_logger().info('Starting Gripper Test Sequence')
        self.get_logger().info('='*50)
        
        tests = [
            ('FULL OPEN', self.positions['full_open']),
            ('OPEN', self.positions['open']),
            ('HALF OPEN', self.positions['half']),
            ('CLOSE', self.positions['close']),
            ('FULL CLOSE', self.positions['full_close']),
            ('OPEN', self.positions['open']),
            ('CLOSE', self.positions['close']),
        ]
        
        for name, pos in tests:
            self.move_gripper(pos, name)
            time.sleep(1.5)  # Wait between movements
        
        self.get_logger().info('='*50)
        self.get_logger().info('Gripper Test Complete! ✅')
        self.get_logger().info('='*50)

    def interactive_test(self):
        """Interactive gripper test - manual control"""
        self.get_logger().info('='*50)
        self.get_logger().info('Interactive Gripper Test')
        self.get_logger().info('Commands: o=open, c=close, h=half, f=full_close, q=quit')
        self.get_logger().info('='*50)
        
        while True:
            try:
                cmd = input('Enter command (o/c/h/f/q): ').strip().lower()
                
                if cmd == 'o':
                    self.move_gripper(self.positions['open'], 'OPEN')
                elif cmd == 'c':
                    self.move_gripper(self.positions['close'], 'CLOSE')
                elif cmd == 'h':
                    self.move_gripper(self.positions['half'], 'HALF')
                elif cmd == 'f':
                    self.move_gripper(self.positions['full_close'], 'FULL CLOSE')
                elif cmd == 'e':
                    self.move_gripper(self.positions['full_open'], 'FULL OPEN')
                elif cmd == 'q':
                    self.get_logger().info('Exiting...')
                    break
                else:
                    # Try to parse as float
                    try:
                        pos = float(cmd)
                        self.move_gripper(pos, f'CUSTOM ({pos})')
                    except ValueError:
                        self.get_logger().warn('Unknown command. Use o/c/h/f/q or enter a number')
                        
            except EOFError:
                break
            except KeyboardInterrupt:
                self.get_logger().info('Interrupted')
                break


def main(args=None):
    rclpy.init(args=args)
    tester = GripperTester()
    
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == '-i':
        # Interactive mode
        tester.interactive_test()
    else:
        # Auto test sequence
        tester.run_test()
    
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
