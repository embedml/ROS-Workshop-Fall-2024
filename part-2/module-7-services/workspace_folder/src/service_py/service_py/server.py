#Daniel Yanke
#10/3/2024

import rclpy
from rclpy.node import Node

from custom_service.srv import ExampleSrv
from geometry_msgs.msg import Point

import localization as lx

class PositionService(Node):
    def __init__(self):
        super().__init__("position_service")
        self.srv = self.create_service(ExampleSrv, 'position_srv', self.service_callback)
    def service_callback(self, request, response):
        count =0
        localizer = lx.Project(mode="3D",solver="LSE")
        for anchor in request.locations:
            localizer.add_anchor(f"Anchor {count}", [anchor.x, anchor.y, anchor.z])
            count += 1
        target, label = localizer.add_target()
        count = 0
        for distance in request.distances:
            target.add_measure(f"Anchor {count}",distance)
        localizer.solve()
        response.position = Point()
        response.position.x = target.loc.x
        response.position.y = target.loc.y
        response.position.z = target.loc.z
        return response

def main(args = None):
    rclpy.init()
    position_service = PositionService()
    rclpy.spin(position_service)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
