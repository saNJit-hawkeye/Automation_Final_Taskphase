import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from srvpkg.srv import Waypoint  

class ClientAsync(Node):  
    def __init__(self):  
        super().__init__('client_async') 
        self.publisher_ = self.create_publisher(Twist,'/cmd_vel',10)  
        self.cli = self.create_client(Waypoint, 'next_waypoint')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            print('Waiting for waypoint service...')
        self.timer_period = 0.5  
        self.timer=self.create_timer(self.timer_period,self.coordinates)
        self.x,self.y=0,0   
        self.targetx,self.targety = None,None
        self.state='request'
        self.rotation_times=0
        self.max_rotation_ticks=4   
        self.waypoint_count=0
        self.heading='north' 
        self.next_heading=None
        self.rotation_direction=None  
        
    def request_waypoint(self):
        request=Waypoint.Request()
        request.x=float(self.x)
        request.y=float(self.y)
        print(f"Sending request with x={self.x}, y={self.y}")
        future=self.cli.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response=future.result()
            self.targetx=response.next_x
            self.targety=response.next_y
            print(f"Received waypoint {self.waypoint_count + 1}: ({self.targetx}, {self.targety})")
            if self.y!=self.targety:
                self.next_heading='north' if self.targety>self.y else 'south'
            elif self.x!=self.targetx:
                self.next_heading='east' if self.targetx > self.x else 'west'
            else:
                self.state='done'
                return
            self.rotation_direction = self.get_rotation_direction(self.heading,self.next_heading)
            self.state='rotate'
            self.rotation_times=0
            self.waypoint_count+=1
        except Exception as e:
            print(f"Service call failed: {e}")
            self.state='done'

    def get_rotation_direction(self, current, target):
        directions=['north', 'east', 'south', 'west']
        curr_idx=directions.index(current)
        target_idx=directions.index(target)
        diff=(target_idx - curr_idx) % 4
        return 'right' if diff==1  else 'left'

    def coordinates(self):
        msg=Twist()
        print(f"Pos:({self.x:.2f},{self.y:.2f}) Target:({self.targetx},{self.targety}) ")
        
        if self.state=='request':
            self.request_waypoint()
            
        elif self.state=='rotate':
            if self.heading!=self.next_heading:
                msg.angular.z=-1.57 if self.rotation_direction=='right' else 1.57
                self.rotation_times+=1
                if self.rotation_times>=self.max_rotation_ticks:
                    self.heading=self.next_heading
                    self.rotation_times=0
                    msg.angular.z=0.0
                    if self.heading in ['north','south']:
                        self.state='move_y'
                    else:
                        self.state='move_x'
            else:
                self.state='move_y' if self.heading in ['north','south'] else 'move_x'

        elif self.state=='move_y':
            if self.heading=='north' and self.y<self.targety:
                msg.linear.x=0.5
                self.y+=msg.linear.x*self.timer_period
            elif self.heading=='south' and self.y > self.targety:
                msg.linear.x=0.5
                self.y-=msg.linear.x*self.timer_period
            else:
                msg.linear.x=0.0
                if self.x!=self.targetx:
                    self.next_heading='east' if self.targetx>self.x else 'west'
                    self.rotation_direction=self.get_rotation_direction(self.heading,self.next_heading)
                    self.state='rotate'
                else:
                    self.state='request'

        elif self.state=='move_x':
            if self.heading=='east' and self.x < self.targetx:
                msg.linear.x=0.5
                self.x+=msg.linear.x*self.timer_period
            elif self.heading=='west' and self.x>self.targetx:
                msg.linear.x=0.5
                self.x-=msg.linear.x*self.timer_period
            else:
                msg.linear.x=0.0
                self.state='request'

        elif self.state=='done':
            msg.linear.x=0.0
            msg.angular.z=0.0

        self.publisher_.publish(msg)

def main(args=None):  
    rclpy.init(args=args)  
    node=ClientAsync()  
    rclpy.spin(node)  
    node.destroy_node()  
    rclpy.shutdown()  

if __name__ == '__main__':  
    main()

