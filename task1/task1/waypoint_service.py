import rclpy
from rclpy.node import Node
from srvpkg.srv import Waypoint


class WaypointService(Node):
    def __init__(self):
        super().__init__('waypoint_service')
        self.srv = self.create_service(Waypoint,'next_waypoint',self.next_waypoint)
        self.waypoints=[(5,5),(10,5),(10,10)]
        self.i=0

    def next_waypoint(self, request, response): 
        if self.i<len(self.waypoints):
            next_wp=self.waypoints[self.i]
            response.next_x=float(next_wp[0])
            response.next_y=float(next_wp[1])
            print(f"Current: ({request.x},{request.y}),Next:{next_wp}")
            self.i+=1
        else:
            response.next_x=float(request.x)
            response.next_y=float(request.y)
            print("All waypoints sent. Returning current position.")
        return response


def main(args=None):
    rclpy.init(args=args)
    node=WaypointService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




