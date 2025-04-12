import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Navigation(Node):
    def __init__(self):
        super().__init__('move')
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 0.1
        self.create_timer(self.timer_period, self.move_to_target)

        self.x = 0
        self.y = 0
        self.heading = 0  #0=north, 1=east, 2=south, 3=west
        self.target_x = 5
        self.target_y = 1
        self.rotation_time = 0.0
        self.move_time = 0.0
        self.state='move_in_y'
        self.target_heading = 0
        self.angular_speed = 22 / (4*7)  
        self.linear_speed = 0.5          
        self.rotation_duration =  22/ (0.8*7) / self.angular_speed  
        self.move_duration = 4.0       

    def move_to_target(self):
        msg = Twist()
        if self.state == 'move_in_y':
            desired_heading = 0 if self.target_y > self.y else 2
            if self.y == self.target_y:
                self.state = 'move_in_x'
            elif self.heading != desired_heading:
                self.target_heading = desired_heading
                self.rotation_time = 0.0
                self.state = 'rotate'
            else:
                msg.linear.x = self.linear_speed
                self.move_time += self.timer_period
                if self.move_time >= self.move_duration:
                    self.y += 1 if self.target_y > self.y else -1
                    self.move_time = 0.0
                    print(f"Moved to:({self.x}, {self.y})")

        elif self.state == 'move_in_x':
            desired_heading = 1 if self.target_x > self.x else 3
            if self.x == self.target_x:
                if self.y == self.target_y:
                    print(f"Destination reached at ({self.x}, {self.y})")
                else:
                    self.state = 'move_in_y'
            elif self.heading != desired_heading:
                self.target_heading = desired_heading
                self.rotation_time = 0.0
                self.state = 'rotate'
            else:
                msg.linear.x = self.linear_speed
                self.move_time += self.timer_period
                if self.move_time >= self.move_duration:
                    self.x += 1 if self.target_x > self.x else -1
                    self.move_time = 0.0
                    print(f"Moved to: ({self.x}, {self.y})")

        elif self.state == 'rotate':
            if self.rotation_time < self.rotation_duration:
                msg.angular.z = self.get_rotation_direction() * self.angular_speed
                self.rotation_time += self.timer_period
            elif (self.target_heading - self.heading) % 4 == 2 and self.rotation_time < 2 * self.rotation_duration:
                msg.angular.z = self.get_rotation_direction() * self.angular_speed
                self.rotation_time += self.timer_period
            else:
                self.heading = self.target_heading
                self.state = 'move_in_y' if self.y != self.target_y else 'move_in_x'
                msg.angular.z = 0.0
                print(f"Rotated to heading {self.heading}")

        self.pub_vel.publish(msg)

    def get_rotation_direction(self):
        dh = (self.target_heading - self.heading) % 4
        if dh == 1:
            return -1  
        elif dh == 3:
            return +1  
        elif dh == 2:
            return 2   
        return 0

def main(args=None):
    rclpy.init(args=args)
    node = Navigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
