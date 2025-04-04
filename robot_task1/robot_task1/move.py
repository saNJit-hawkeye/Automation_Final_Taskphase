import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Navigation(Node):
    def __init__(self):
        super().__init__('move')
        self.pub_vel = self.create_publisher(Twist,'/cmd_vel',10)
        self.current_x = 0
        self.current_y = 0
        self.target_x = 5
        self.target_y = 1
        self.move_x = True
        self.rotate = False
        self.move_y = False
        self.rotation_times = 0  
        self.create_timer(0.1, self.move_to_target)  

    def move_to_target(self):
        moving_position_x=self.target_x-self.current_x
        moving_position_y=self.target_y-self.current_y
        velocity=Twist()

        if self.move_x:
            if abs(moving_position_x)>0.1:
                velocity.linear.x=0.5
                self.pub_vel.publish(velocity)
                print(f"the position is {self.current_x,0}")
                self.current_x+=velocity.linear.x*0.1
            else:
                velocity.linear.x=0.0
                self.pub_vel.publish(velocity)
                print("X Target Reached")
                self.move_x=False
                self.rotate=True 

        elif self.rotate:
            if self.rotation_times<16:  
                velocity.angular.z=0.7
                self.pub_vel.publish(velocity)
                self.rotation_times+=1
            else:
                velocity.angular.z=0.0
                self.pub_vel.publish(velocity)
                self.rotate=False
                self.move_y=True  

        elif self.move_y:
            if abs(moving_position_y)>0.1:
                velocity.linear.x=0.5  
                self.pub_vel.publish(velocity)
                print(f"the position is {0,self.current_y}")
                self.current_y+=velocity.linear.x * 0.1
            else:
                velocity.linear.x=0.0
                self.pub_vel.publish(velocity)
                print("Target coordinates reached")
                rclpy.shutdown()  

def main(args=None):
    rclpy.init(args=args)
    node=Navigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
