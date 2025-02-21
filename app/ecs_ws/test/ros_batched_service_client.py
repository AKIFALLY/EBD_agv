import rclpy
from rclpy.node import Node
from interfaces.srv import ReadContinuousData  # 引入適當的服務
import time  # 用於計算時間

class BatchedServiceClient(Node):
    def __init__(self):
        super().__init__('batched_service_client')
        self.client = self.create_client(
            ReadContinuousData, 
            'read_continuous_data',  # 更新為 read_continuous_data 服務名稱
            qos_profile=rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.start_time = time.time()  # 記錄開始時間
        self.response_times = []  # 儲存回應時間
        self.completed_requests = 0  # 完成的請求數量
        self.total_requests = 10000  # 總共請求數
        self.batch_size = 10  # 每次發送的請求數量
        self.pending_futures = []  # 儲存所有的future物件

        self.send_next_batch()  # 發送第一批請求

    def send_next_batch(self):
        """發送下一批請求，每批最多 10 個請求"""
        batch_start = self.completed_requests
        batch_end = min(self.completed_requests + self.batch_size, self.total_requests)

        for i in range(batch_start, batch_end):
            req = ReadContinuousData.Request()  # 創建 ReadContinuousData 請求
            req.device_type = 'DM'  # 設定設備類型
            req.start_address = "5000"  # 設定開始地址
            req.count = 200  # 設定請求數量
            request_time = time.time()  # 記錄發送請求的時間
            future = self.client.call_async(req)
            future.add_done_callback(lambda f, req_time=request_time, idx=i: self.callback(f, req_time, idx))
            self.pending_futures.append(future)

    def callback(self, future, request_time, idx):
        """處理回應並記錄時間"""
        try:
            response = future.result()
            #print(len(response.values))
            #self.get_logger().info(f'Response {response}')
            response_time = time.time()
            time_taken_ms = (response_time - request_time) * 1000  # 計算回應時間（毫秒）
            self.response_times.append(time_taken_ms)
            self.completed_requests += 1
        except Exception as e:
            self.get_logger().error(f'Response {idx+1} failed: {e}')
        finally:
            self.pending_futures.remove(future)
            if not self.pending_futures:  # 當前批次的請求都處理完畢
                if self.completed_requests < self.total_requests:
                    self.send_next_batch()  # 發送下一批請求
                else:
                    self.calculate_stats()  # 計算統計數據
                    rclpy.shutdown()

    def calculate_stats(self):
        """計算統計數據"""
        if not self.response_times:
            self.get_logger().warn('No response times recorded.')
            return
        
        fastest = min(self.response_times)
        slowest = max(self.response_times)
        average = sum(self.response_times) / len(self.response_times)
        self.get_logger().info(f'Sum response time: {sum(self.response_times)} ms')
        self.get_logger().info(f'len response : {len(self.response_times)}')
        
        self.get_logger().info(f'Fastest response time: {fastest:.2f} ms')
        self.get_logger().info(f'Slowest response time: {slowest:.2f} ms')
        self.get_logger().info(f'Average response time: {average:.2f} ms')
        total_time = (time.time() - self.start_time) * 1000
        self.get_logger().info(f'Total time to complete all requests: {total_time:.2f} ms')

def main(args=None):
    rclpy.init(args=args)
    client = BatchedServiceClient()

    try:
        rclpy.spin(client)  # 保持節點運行以處理回調
    except Exception as e:
        client.get_logger().error(f'Error occurred: {e}')
    finally:
        if rclpy.ok():
            client.destroy_node()  # 修正為 client.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
