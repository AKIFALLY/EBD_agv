import inspect
import logging
from rclpy.node import Node
from rcl_interfaces.msg import Log
from builtin_interfaces.msg import Time


class RosLogger:
    # ROS log levels
    DEBUG = Log.DEBUG
    INFO = Log.INFO
    WARN = Log.WARN
    ERROR = Log.ERROR
    FATAL = Log.FATAL

    def __init__(self, node: Node):
        self.node = node
        self.publisher = node.create_publisher(Log, "/runtime_log", 30)

    def _log(self, level: int, message: str):
        # 自動擷取 caller 的 file、function、line
        frame = inspect.stack()[2]
        file_name = frame.filename
        function_name = frame.function
        line_number = frame.lineno
        # ROS console log
        if level == self.DEBUG:
            self.node.get_logger().debug(message)
        elif level == self.INFO:
            self.node.get_logger().info(message)
        elif level == self.WARN:
            self.node.get_logger().warn(message)
        elif level == self.ERROR:
            self.node.get_logger().error(message)
        elif level == self.FATAL:
            self.node.get_logger().fatal(message)

        # Publish log to topic
        log_msg = Log()
        log_msg.level = level
        log_msg.name = self.node.get_name()
        log_msg.msg = message
        log_msg.stamp = self.node.get_clock().now().to_msg()
        log_msg.file = file_name
        log_msg.function = function_name
        log_msg.line = line_number
        self.publisher.publish(log_msg)

    def debug(self, message: str):
        self._log(self.DEBUG, message)

    def info(self, message: str):
        self._log(self.INFO, message)

    def warn(self, message: str):
        self._log(self.WARN, message)

    def error(self, message: str):
        self._log(self.ERROR, message)

    def fatal(self, message: str):
        self._log(self.FATAL, message)
