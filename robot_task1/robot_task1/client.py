
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from srvpkg.srv import Waypoint

class ClientAsync(Node):  
    def __init__(self):  
        super().__init__('client_async')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)  
        self.cli = self.create_client(Waypoint, 'next_waypoint')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for waypoint service...')

        self.timer = self.create_timer(0.5, self.movement)

        # Robot state
        self.x = 0
        self.y = 0
        self.targetx = None
        self.targety = None
        self.state = 'request'
        self.current_heading = 0 

        self.time_remaining = 0
        self.action_timer = None

    def request_waypoint(self):
        request = Waypoint.Request()
        request.x = float(self.x)
        request.y = float(self.y)
        self.get_logger().info(f"Requesting next waypoint from ({self.x}, {self.y})")
        future = self.cli.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            self.targetx = int(response.next_x)
            self.targety = int(response.next_y)
            self.get_logger().info(f"Received next waypoint: ({self.targetx}, {self.targety})")
            self.state = 'decide'
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.state = 'done'

    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.state = 'decide'

    def move_forward(self, duration=3):
        msg = Twist()
        msg.linear.x = 0.5
        self.publisher_.publish(msg)
        print(f"Moving forward for {duration} seconds.")
        self.time_remaining = duration
        self.action_timer = self.create_timer(1.0, self.update_move)

    def rotate_to(self, target_heading):
        diff = (target_heading - self.current_heading) % 4
        if diff == 0:
            return  
        elif diff == 1:
            speed = -0.5  # Left turn
        elif diff == 3:
            speed = 0.5   # Right turn
        elif diff == 2:
            speed = 0.5   # 180° turn (choose right for consistency)

        duration = 5.8 * (1 if diff in [1, 3] else 2)

        msg = Twist()
        msg.angular.z = speed
        self.publisher_.publish(msg)
        self.get_logger().info(f"Rotating from {self.current_heading} to {target_heading}")
        
        self.time_remaining = duration
        self.next_heading = target_heading
        self.action_timer = self.create_timer(1.0, self.update_rotate)



    def update_move(self):
        self.time_remaining -= 1
        if self.time_remaining <= 0:
            self.action_timer.cancel()
            self.update_position()

            if (self.x, self.y) == (self.targetx, self.targety):
                self.get_logger().info(f"Reached FINAL waypoint at ({self.x}, {self.y})")
                self.stop()
                self.state = 'done'  # Stop everything — mission complete
            else:
                self.state = 'decide'  # Continue to next cell

            

    def update_rotate(self):
        self.time_remaining -= 1
        if self.time_remaining <= 0:
            self.action_timer.cancel()
            self.current_heading = self.next_heading

            # After rotation, go directly to move forward
            self.move_forward()


    def update_position(self):
        if self.current_heading == 0:
            self.y += 1
        elif self.current_heading == 1:
            self.x += 1
        elif self.current_heading == 2:
            self.y -= 1
        elif self.current_heading == 3:
            self.x -= 1
        print(f"Now at ({self.x}, {self.y}) heading {self.current_heading}")

    def movement(self):
        if self.state == 'request':
            self.request_waypoint()

        elif self.state == 'decide':
            if (self.x, self.y) == (self.targetx, self.targety):
                print("Reached current waypoint.")
                self.state = 'request'  # Ask for the next waypoint
                return

            # Decide step direction
            if self.y < self.targety:
                desired_heading = 0  # North
            elif self.y > self.targety:
                desired_heading = 2  # South
            elif self.x < self.targetx:
                desired_heading = 1  # East
            elif self.x > self.targetx:
                desired_heading = 3  # West
            else:
                self.state = 'request'
                return

            if self.current_heading != desired_heading:
                self.rotate_to(desired_heading)
            else:
                self.move_forward()

            self.state = 'wait'

        elif self.state == 'wait':
            # Waiting for move/rotate to finish
            pass

        elif self.state == 'done':
            self.stop()

def main(args=None):  
    rclpy.init(args=args)  
    node = ClientAsync()  
    rclpy.spin(node)  
    node.destroy_node()  
    rclpy.shutdown()  

if __name__ == '__main__':  
    main()




