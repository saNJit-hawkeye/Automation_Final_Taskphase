import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from srvpkg.srv import Waypoint
from sensor_msgs.msg import LaserScan
import math

class FollowPath(Node):  
    def __init__(self):  
        super().__init__('follow_path')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)  
        self.cli = self.create_client(Waypoint, 'next_waypoint')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for waypoint service...')

        self.timer_period = 0.1 
        self.timer = self.create_timer(self.timer_period, self.movement)
        self.x = 0
        self.y = 0
        self.heading = 0  #0=north, 1=east, 2=south, 3=west
        self.targetx = None
        self.targety = None
        self.state = "request"
        self.pause_counter = 0
        self.rotation_time = 0.0
        self.move_time = 0.0
        self.target_heading = 0
        self.angular_speed = 22 / (4*7)  
        self.linear_speed = 0.5          
        self.rotation_duration =  22/ (0.8*7) / self.angular_speed  
        self.move_duration = 4.0          
        
        self.waypoints_done = 0
        self.max_waypoints = 3

        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.closest_obstacle = float('inf')
        self.object_detected = False
        self.safe_distance = 1.0

        self.avoid_step = 0
        self.avoid_timer = 0.0
        
    def lidar_callback(self,msg):
        self.get_logger().info(f"Angle min: {msg.angle_min:.2f}, max: {msg.angle_max:.2f}, increment: {msg.angle_increment:.4f}, length: {len(msg.ranges)}")
        self.closest_object = float('inf')
        middle=len(msg.ranges)//2
        front_values=msg.ranges[middle-25:middle+25]
        for i in front_values:
            if msg.range_min<i<msg.range_max:
                if i<self.closest_object:
                    self.closest_object=i
        self.object_detected=self.closest_object<self.safe_distance
        self.get_logger().debug("LiDAR callback triggered. Sample range: {:.2f}".format(msg.ranges[len(msg.ranges)//2]))
                    
    
    def request_waypoint(self):
        request = Waypoint.Request()
        request.x = float(self.x)
        request.y = float(self.y)
        self.get_logger().info(f"Requesting waypoint from:({self.x},{self.y})...")
        future = self.cli.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            self.targetx = int(response.next_x)
            self.targety = int(response.next_y)
            self.get_logger().info(f"Received waypoint {self.waypoints_done+1}: ({self.targetx},{self.targety})")
            self.state = 'move_y'
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.state = 'done'

    def movement(self):
        msg = Twist()
        if self.state == 'request':
            if self.waypoints_done < self.max_waypoints:
                self.request_waypoint()
            else:
                self.state = 'done'

        elif self.state == 'move_y':
            desired_heading = 0 if self.targety > self.y else 2
            if self.y == self.targety:
                self.state = 'move_x'  
            elif self.heading != desired_heading:
                self.target_heading = desired_heading
                self.rotation_time = 0.0
                self.state = 'rotate'
            elif self.object_detected:
                self.get_logger().info("Object detected, dodging it")
                self.target_heading=(self.heading+1)%4
                self.rotation_time=0.0
                self.state='turn_avoid_obj'
            else:
                msg.linear.x = self.linear_speed
                self.move_time += self.timer_period
                if self.move_time >= self.move_duration:
                    self.y += 1 if self.targety > self.y else -1
                    self.move_time = 0.0
                    print(f"Step complete  Position: ({self.x}, {self.y})")

        elif self.state == 'move_x':
            desired_heading = 1 if self.targetx > self.x else 3
            if self.x == self.targetx:
                self.state = 'done_check'
            elif self.heading != desired_heading:
                self.target_heading = desired_heading
                self.rotation_time = 0.0
                self.state = 'rotate'
            elif self.object_detected:
                self.get_logger().info("Object detected, dodging it")
                self.target_heading=(self.heading+1)%4
                msg.linear.x=0.0
                self.rotation_time=0.0
                self.state='turn_avoid_obj'
            else:
                msg.linear.x = self.linear_speed
                self.move_time += self.timer_period
                if self.move_time >= self.move_duration:
                    self.x += 1 if self.targetx > self.x else -1
                    self.move_time = 0.0
                    print(f"Step complete  Position: ({self.x}, {self.y})")

        elif self.state == 'done_check':
                print(f"Destination reached at ({self.x}, {self.y})")
                self.state = 'request'
                self.waypoints_done += 1

        elif self.state == 'rotate':
            if self.rotation_time < self.rotation_duration:
                msg.angular.z = self.get_rotation_direction() * self.angular_speed
                self.rotation_time += self.timer_period
            else:
                self.heading = self.target_heading
                self.state = 'move_y' if self.y != self.targety else 'move_x'
                msg.angular.z = 0.0
                
        elif self.state=='turn_avoid_obj':
            if self.rotation_time < self.rotation_duration:
                msg.angular.z = -self.angular_speed
                self.rotation_time += self.timer_period
            else:
                self.heading=self.target_heading
                self.rotation_time=0.0
                self.move_time=0.0
                self.state='move_forward'
                
        elif self.state=='move_forward':
            msg.linear.x=self.linear_speed
            self.move_time+=self.timer_period
            if self.move_time >= self.move_duration:
                if self.heading==0:
                    self.y+=1
                elif self.heading==1:
                    self.x+=1
                elif self.heading == 2:
                    self.y-=1
                elif self.heading == 3:
                    self.x-=1
                self.move_time=0.0
                self.get_logger().info(f"Avoided to ({self.x},{self.y})")
                if self.y!=self.targety:
                    self.state='move_y'
                else:
                    self.state='move_x'
                            
        elif self.state == 'done':
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            print("All waypoints reached.")

        self.publisher_.publish(msg)
        
    def get_rotation_direction(self):
        diff = (self.target_heading - self.heading) % 4
        if diff == 1 or diff == -3:
            return -1  
        elif diff == 3 or diff == -1:
            return +1  
        elif diff == 2:
            return 2  
        return 0

def main(args=None):  
    rclpy.init(args=args)  
    node = FollowPath()  
    rclpy.spin(node)  
    node.destroy_node()  
    rclpy.shutdown()  

if __name__ == '__main__':  
    main()



