#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class StraightMove(Node):
    def __init__(self):
        super().__init__('straight_move')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def move_straight(self, distance=1.0, speed=0.2):
        """直行指定距离"""
        duration = distance / speed
        
        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = 0.0
        
        self.get_logger().info(f'开始直行 {distance} 米')
        
        # 持续发布
        start_time = time.time()
        while time.time() - start_time < duration:
            self.pub.publish(cmd)
            time.sleep(0.05)
        
        # 停止
        cmd.linear.x = 0.0
        self.pub.publish(cmd)
        self.get_logger().info('完成')

def main():
    import sys
    rclpy.init()
    node = StraightMove()
    
    distance = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
    node.move_straight(distance)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()