import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
import time
import math

class TurtleWriter(Node):

    def __init__(self):
        super().__init__('turtle_writer')

        self.spawn_cli = self.create_client(Spawn, '/spawn')
        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        self.turtles = []
        self.spawn_turtles()
        time.sleep(2)
        self.draw()

    def spawn_turtles(self):
        positions = [
            (2, 8), (3, 8), (4, 8),   # R
            (6, 8), (7, 8),           # O
            (9, 8), (10, 8),          # B
            (12, 8),                 # O
            (14, 8), (15, 8),         # T
            (17, 8),                 # S
        ]

        for i, (x, y) in enumerate(positions):
            req = Spawn.Request()
            req.x = float(x)
            req.y = float(y)
            req.theta = 0.0
            req.name = f'turtle{i+2}'
            self.spawn_cli.call_async(req)
            self.turtles.append(req.name)

    def move(self, turtle, linear, angular, duration):
        pub = self.create_publisher(Twist, f'/{turtle}/cmd_vel', 10)
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular

        end = time.time() + duration
        while time.time() < end:
            pub.publish(msg)
            time.sleep(0.05)

        pub.publish(Twist())

    def draw(self):
        # VERY simple strokes (example only)
        for turtle in self.turtles:
            self.move(turtle, 1.0, 0.0, 1.2)
            self.move(turtle, 0.0, 1.5, 1.0)
            self.move(turtle, 1.0, 0.0, 0.6)

def main():
    rclpy.init()
    node = TurtleWriter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
