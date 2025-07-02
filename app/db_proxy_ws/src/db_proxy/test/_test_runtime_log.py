import rclpy
from rclpy.node import Node
from db_proxy.agvc_logger_pub import RosLogger  # 確保這是你放 RosLogger 類別的模組名稱


class TestLoggerNode(Node):
    def __init__(self):
        super().__init__('test_logger_node')
        self.logger = RosLogger(self)

        # 測試輸出不同等級的 log
        self.logger.debug("這是 DEBUG 訊息")
        self.logger.info("這是 INFO 訊息")
        self.logger.warn("這是 WARN 訊息")
        self.logger.error("這是 ERROR 訊息")
        self.logger.fatal("這是 FATAL 訊息")


def main(args=None):
    rclpy.init(args=args)
    node = TestLoggerNode()
    rclpy.spin_once(node, timeout_sec=0.5)  # 執行一下讓 log 有機會送出
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
