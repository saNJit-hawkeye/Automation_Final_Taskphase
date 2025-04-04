import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
import tf2_ros
import math
import time

class CustomOdometry(Node):
    def __init__(self):
        super().__init__('custom_odometry')
        self.odom_pub=self.create_publisher(Odometry,'/odom',10)
        self.tf_broadcaster=tf2_ros.TransformBroadcaster(self)
        self.wheel_radius=0.195
        self.wheel_sep=0.89
        self.x=0.0
        self.y=0.0
        self.theta=0.0
        self.v=0.0
        self.omega=0.0
        self.last_time=self.get_clock().now()
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_timer(0.05,self.update_odometry)

    def cmd_vel_callback(self,msg):
        self.v=msg.linear.x
        self.omega=msg.angular.z

    def update_odometry(self):
        current_time=self.get_clock().now()
        dt=(current_time-self.last_time).nanoseconds*1e-9
        self.last_time=current_time
        self.x+=self.v*math.cos(self.theta)*dt
        self.y+=self.v*math.sin(self.theta)*dt
        self.theta+=self.omega*dt
        odom_msg=Odometry()
        odom_msg.header.stamp=current_time.to_msg()
        odom_msg.pose.pose.position.x=self.x
        odom_msg.pose.pose.position.y=self.y
        odom_msg.pose.pose.orientation.z=math.sin(self.theta/2)
        odom_msg.pose.pose.orientation.w=math.cos(self.theta/2)
        odom_msg.twist.twist.linear.x=self.v
        odom_msg.twist.twist.angular.z=self.omega
        self.odom_pub.publish(odom_msg)
        t = TransformStamped()
        t.header.stamp=current_time.to_msg()
        t.header.frame_id="odom"
        t.child_frame_id="base_link"
        t.transform.translation.x=self.x
        t.transform.translation.y=self.y
        t.transform.rotation.z=math.sin(self.theta/2)
        t.transform.rotation.w=math.cos(self.theta/2)
        self.tf_broadcaster.sendTransform(t)
    
def main(args=None):
    rclpy.init(args=args)
    node = CustomOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


