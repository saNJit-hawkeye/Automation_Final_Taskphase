#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class FollowPath(Node):  
    def __init__(self):  
        super().__init__('follow_path')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)  

        self.timer_period = 0.1 
        self.timer = self.create_timer(self.timer_period, self.movement)
        self.x = 0
        self.y = 0
        self.heading = 0  #0=north, 1=east, 2=south, 3=west
        self.rotation_time = 0.0
        self.move_time = 0.0
        self.target_heading = 0
        self.linear_speed = 0.5        
        self.move_duration = 4          

        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.closest_obstacle = float('inf')
        self.object_detected = False
        self.safe_distance = 0.7
            
    def lidar_callback(self, msg):
        self.closest_object = float('inf')
        front_index = len(msg.ranges) // 2 + 90
        right_index = front_index - 90  
        left_index = front_index + 90  
        front_values = msg.ranges[front_index - 25: front_index + 25]
        right_values = msg.ranges[right_index - 10: right_index + 10]
        left_values = msg.ranges[left_index - 10: left_index + 10]
        def get_closest(valid_ranges):
            return min([r for r in valid_ranges if msg.range_min < r < msg.range_max], default=float('inf'))
        self.front_distance = get_closest(front_values)
        self.right_distance = get_closest(right_values)
        self.left_distance = get_closest(left_values)
        self.object_detected_front = self.front_distance<self.safe_distance
        self.object_detected_right = self.right_distance<self.safe_distance
        self.object_detected_left = self.left_distance<self.safe_distance
        
    def movement(self):
        msg=Twist()
        self.kp=1.0
        msg.linear.x=1.0
        self.distfromwall=1.5
        self.left_error=self.distfromwall-self.left_distance
        self.right_error=self.distfromwall-self.right_distance
        self.error=self.right_error-self.left_error
        msg.angular.z=self.kp*self.error
        self.publisher_publish(msg)
        
def main(args=None):  
    rclpy.init(args=args)  
    node = FollowPath()  
    rclpy.spin(node)  
    node.destroy_node()  
    rclpy.shutdown()  

if __name__ == '__main__':  
    main()