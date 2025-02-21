import rclpy
from rclpy.node import Node
from interfaces.srv import ForceOn, ForceOff, ReadData, WriteData, ReadContinuousData, WriteContinuousData

class PlcClient(Node):
    def __init__(self, node_name='plc_client'):
        if not rclpy.ok():  # 檢查是否已初始化
            rclpy.init()
        super().__init__(node_name)
        self.client_force_on = self.create_client(ForceOn, 'force_on')
        self.client_force_off = self.create_client(ForceOff, 'force_off')
        self.client_read = self.create_client(ReadData, 'read_data')
        self.client_write = self.create_client(WriteData, 'write_data')
        self.client_read_continuous = self.create_client(ReadContinuousData, 'read_continuous_data')
        self.client_write_continuous = self.create_client(WriteContinuousData, 'write_continuous_data')

        #這裡先等一個Read服務啟動就好
        while not self.client_read.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for PLC service...')

    def force_on(self, device_type, address):
        while not self.client_force_on.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        request = ForceOn.Request()
        request.device_type = device_type
        request.address = address
        future = self.client_force_on.call_async(request)
        rclpy.spin_until_future_complete(self, future,None,timeout_sec=1.0)
        return future.result()

    def force_off(self, device_type, address):
        while not self.client_force_off.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        request = ForceOff.Request()
        request.device_type = device_type
        request.address = address
        future = self.client_force_off.call_async(request)
        rclpy.spin_until_future_complete(self, future,None,timeout_sec=1.0)
        return future.result()
    
    def read_data(self, device_type, address):
        while not self.client_read.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.get_logger().info('read_data...')
        request = ReadData.Request()
        request.device_type = device_type
        request.address = address
        future = self.client_read.call_async(request)
        rclpy.spin_until_future_complete(self, future,None,timeout_sec=1.0)
        if future.result():
            return future.result()
        else:
            self.get_logger().error('Failed to call service read_data')
            return None

    def write_data(self, device_type, address, value):
        while not self.client_write.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        request = WriteData.Request()
        request.device_type = device_type
        request.address = address
        request.value = value
        future = self.client_write.call_async(request)
        rclpy.spin_until_future_complete(self, future,None,timeout_sec=1.0)
        return future.result()

    def read_continuous_data(self, device_type, start_address, count):
        while not self.client_read_continuous.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        request = ReadContinuousData.Request()        
        request.device_type = device_type
        request.start_address = start_address
        request.count = count
        future = self.client_read_continuous.call_async(request)
        rclpy.spin_until_future_complete(self, future,None,timeout_sec=1.0)
        if future.result():
            return future.result()
        else:
            self.get_logger().error('Failed to call service read_continuous_data')
            return None

    def write_continuous_data(self, device_type, start_address, values):
        while not self.client_write_continuous.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        request = WriteContinuousData.Request()
        request.device_type = device_type
        request.start_address = start_address
        request.values = values
        future = self.client_write_continuous.call_async(request)
        rclpy.spin_until_future_complete(self, future,None,timeout_sec=1.0)
        return future.result()
