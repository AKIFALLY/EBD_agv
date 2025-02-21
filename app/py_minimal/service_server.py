import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger  # 假設使用 Trigger 服務

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        #self.srv = self.create_service(Trigger, 'my_service')
        self.srv = self.create_service(
            Trigger, 
            'my_service', 
            self.handle_request, 
            qos_profile=rclpy.qos.QoSProfile(depth=1000, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
        )
        self.index = 1

    def handle_request(self, request, response):
        #self.get_logger().info(f'Received request{self.index}')
        self.index += 1
        # 這裡處理您的邏輯
        response.success = True
        response.message = 'Service response message'
        return response

def main(args=None):
    rclpy.init(args=args)
    service_server = ServiceServer()
    rclpy.spin(service_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
