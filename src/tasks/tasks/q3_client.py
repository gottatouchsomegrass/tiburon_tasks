#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
from tasks_msgs.srv import DrawShape 

class ShapeClient(Node):
    def __init__(self):
        super().__init__('shape_client')
        self.client = self.create_client(DrawShape, '/draw_shape')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.request = DrawShape.Request()

    def send_request(self, shape, size):
        self.request.shape = shape
        self.request.size = float(size)
        
        self.future = self.client.call_async(self.request)
        self.get_logger().info(f'Calling service to draw a {shape} of size {size}...')
        rclpy.spin_until_future_complete(self, self.future)
        
        try:
            response = self.future.result()
            if response.success:
                self.get_logger().info('Shape drawn successfully')
            else:
                self.get_logger().warn('Service call failed')
        except Exception as e:
            self.get_logger().error(f'Service error: {e}')

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 3:
        print("Eg: ros2 run tasks shape_client <shape_name> <size> (in decimal pls)")
        sys.exit(1)
        
    shape = sys.argv[1]
    size = sys.argv[2]
    
    client_node = ShapeClient()
    client_node.send_request(shape, size)
    
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
