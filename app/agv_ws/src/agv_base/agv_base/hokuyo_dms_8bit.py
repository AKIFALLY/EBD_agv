import rclpy
from rclpy.node import Node
from plc_proxy.plc_client import PlcClient
import yaml
from pathlib import Path


class HokuyoDMS8Bit():

    def __init__(self,  node: Node, config_path: str, config_key: str):
        self.node = node
        self.plc_client = PlcClient(node)

        # 讀取 YAML 配置檔案
        config_file = Path(config_path)
        if not config_file.exists():
            raise FileNotFoundError(f"Config file not found: {config_path}")

        with open(config_file, 'r') as file:
            self.config = yaml.safe_load(file)
        self.node.get_logger().info(f"Configuration loaded: {self.config}")

        self.config_key = config_key

        # 從配置中取得設備類型與地址
        if self.config_key not in self.config:
            self.node.get_logger().error(
                f"Config key '{self.config_key}' not found in configuration.")
            raise ValueError(f"Config key '{self.config_key}' not found in configuration.")

        self.device_config = self.config[self.config_key]
        self.device_type = self.device_config['device']
        self.address = self.device_config['address']
        self.node.get_logger().info(
            f"Config Key:{self.config_key}, Device type: {self.device_type}, Address: {self.address}")

        self.valid_response = None
        self.valid_success = False
        self.valid_failed = False
        self.valid_step = 0
        self.port_number_response = None
        self.port_number_success = False
        self.port_number_failed = False
        self.port_number_step = 0
        self.tr_req_response = None
        self.tr_req_success = False
        self.tr_req_failed = False
        self.tr_req_step = 0
        self.busy_response = None
        self.busy_success = False
        self.busy_failed = False
        self.busy_step = 0
        self.complete_response = None
        self.complete_success = False
        self.complete_failed = False
        self.complete_step = 0
        self.hokuyo_input_response = None
        self.hokuyo_input_success = False
        self.hokuyo_input_failed = False
        self.hokuyo_input_step = 0

        self.load_req = False
        self.unload_req = False
        self.ready = False

    def valid_callback(self, response):
        self.node.get_logger().info(f"Read Valid value: {response}")
        self.valid_response = response
        self.valid_success = response.success
        self.valid_failed = not response.success
        self.valid_step = 0

    def port_number_callback(self, response):
        self.node.get_logger().info(f"Read Port Number value: {response}")
        self.port_number_response = response
        self.port_number_success = response.success
        self.port_number_failed = not response.success
        self.port_number_step = 0

    def tr_req_callback(self, response):
        self.node.get_logger().info(f"Read TR_REQ value: {response}")
        self.tr_req_response = response
        self.tr_req_success = response.success
        self.tr_req_failed = not response.success
        self.tr_req_step = 0

    def busy_callback(self, response):
        self.node.get_logger().info(f"Read Busy value: {response}")
        self.busy_response = response
        self.busy_success = response.success
        self.busy_failed = not response.success
        self.busy_step = 0

    def complete_callback(self, response):
        self.node.get_logger().info(f"Read Complete value: {response}")
        self.complete_response = response
        self.complete_success = response.success
        self.complete_failed = not response.success
        self.complete_step = 0

    def hokuyo_input_callback(self, response):
        self.node.get_logger().info(f"Read Hokuyo Input value: {response}")
        self.hokuyo_input_response = response
        self.hokuyo_input_success = response.success
        self.hokuyo_input_failed = not response.success
        self.hokuyo_input_step = 0
        if self.hokuyo_input_success:
            self.load_req = bool(int(self.hokuyo_input_response.values[0]))
            self.unload_req = bool(int(self.hokuyo_input_response.values[1]))
            self.ready = bool(int(self.hokuyo_input_response.values[3]))

    def write_valid(self, data: str) -> bool:
        match self.valid_step:
            case 0:
                self.node.get_logger().info("Send 寫入Valid")
                self.plc_client.async_write_data(
                    device_type=self.device_type, address=self.address, value=data, callback=self.valid_callback)
                self.valid_step = 1
            case 1:
                self.node.get_logger().info("等待寫入Valid")

    def write_port_number(self, port_number: int) -> bool:
        # 將 port_number 轉換為 8 位二進位並存入 string[] 變數
        binary_values = list(format(port_number, '04b'))  # 轉換為 4 位二進位字串
        binary_strings = [str(bit) for bit in reversed(
            binary_values)]  # 將每個位元轉為字串，並從左至右排列
        self.node.get_logger().info(
            f"Port number {port_number} converted to binary (1 byte, left-to-right): {binary_strings}")
        port_address = str(int(self.address) + 1)
        match self.port_number_step:
            case 0:
                self.node.get_logger().info("Send 寫入Port Number")
                self.plc_client.async_write_continuous_data(
                    device_type=self.device_type, start_address=port_address, values=binary_strings, callback=self.port_number_callback)
                self.port_number_step = 1
            case 1:
                self.node.get_logger().info("等待寫入Port Number")

    def write_tr_req(self, data: str) -> bool:
        tr_req_address = str(int(self.address) + 5)
        match self.tr_req_step:
            case 0:
                self.node.get_logger().info("Send 寫入TR_REQ")
                self.plc_client.async_write_data(
                    device_type=self.device_type, address=tr_req_address, value=data, callback=self.tr_req_callback)
                self.tr_req_step = 1
            case 1:
                self.node.get_logger().info("等待寫入TR_REQ")

    def write_busy(self, data: str) -> bool:
        busy_address = str(int(self.address) + 6)
        match self.busy_step:
            case 0:
                self.node.get_logger().info("Send 寫入Busy")
                self.plc_client.async_write_data(
                    device_type=self.device_type, address=busy_address, value=data, callback=self.busy_callback)
                self.busy_step = 1
            case 1:
                self.node.get_logger().info("等待寫入Busy")

    def write_complete(self, data: str) -> bool:
        complete_address = str(int(self.address) + 7)
        match self.complete_step:
            case 0:
                self.node.get_logger().info("Send 寫入Complete")
                self.plc_client.async_write_data(
                    device_type=self.device_type, address=complete_address, value=data, callback=self.complete_callback)
                self.complete_step = 1
            case 1:
                self.node.get_logger().info("等待寫入Complete")

    def update_hokuyo_input(self):
        hokuyo_input_address = str(int(self.address) + 8)
        match self.hokuyo_input_step:
            case 0:
                self.node.get_logger().info("Send 讀取Hokuyo Input")
                self.plc_client.async_read_continuous_data(
                    device_type=self.device_type, start_address=hokuyo_input_address, count=8, callback=self.hokuyo_input_callback)
                self.hokuyo_input_step = 1
            case 1:
                self.node.get_logger().info("等待讀取Hokuyo Input")
