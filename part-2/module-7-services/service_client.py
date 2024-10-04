import rclpy

from rclpy.node import Node
from custom_service.srv import ExampleSrv
from geometry_msgs.msg import Point
import random
import math

class PositionClient(Node):
    def __init__(self):
        super().__init__('position_client')
        self.client = self.create_client(ExampleSrv, 'position_srv')
        self.req = ExampleSrv.Request()
    def distance(self, a, b):
        return math.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2 + (b[2]-a[2])**2)
    def send_request(self):
        locations = [[random.uniform(-20.0, 20.0) for x in range(0,3)] for x in range(0,5)]
        for i in range(0,4):
            self.req.distances.append(self.distance(locations[4],locations[i]))
            self.req.locations.append(Point())
            self.req.locations[i].x = locations[i][0]
            self.req.locations[i].y = locations[i][1]
            self.req.locations[i].z = locations[i][2]
        self.get_logger().info(f"Real position is {locations[4]}")
        return self.client.call_async(self.req)

def main():
    rclpy.init()
    position_client = PositionClient()
    future = position_client.send_request()
    rclpy.spin_until_future_complete(position_client, future)
    response = future.result()
    position_client.get_logger().info(f"Result {response.position}")
    position_client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()