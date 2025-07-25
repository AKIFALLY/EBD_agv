from plc_interfaces.srv import (
    ForceOn,
    ForceOff,
    ReadData,
    WriteData,
    ReadContinuousData,
    WriteContinuousData,
    ReadContinuousByte,
    WriteContinuousByte,
)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile


class PlcClient:
    def __init__(self, node: Node):
        self.node = node

        # 設定 depth（例如 50）
        self.qos = QoSProfile(depth=100)

        # 存儲命名空間
        ns = node.get_namespace()
        self.namespace = '' if ns == '/' else ns


        # 使用 namespace 構建每個服務的名稱
        node._logger.info(f"🧭Using namespace: {self.namespace}")
        self.client_force_on = None
        self.client_force_off = None
        self.client_read = None
        self.client_write = None
        self.client_read_continuous = None
        self.client_write_continuous = None
        self.client_read_continuous_byte = None
        self.client_write_continuous_byte = None

    def force_on(self, device_type, address):
        if self.client_force_on is None:
            self.client_force_on = self.node.create_client(
                ForceOn, f"{self.namespace}/force_on" if self.namespace else "force_on", qos_profile=self.qos)
        return self._call_sync(self.client_force_on, ForceOn.Request(device_type=device_type, address=address))

    def force_off(self, device_type, address):
        if self.client_force_off is None:
            self.client_force_off = self.node.create_client(
                ForceOff, f"{self.namespace}/force_off" if self.namespace else "force_off", qos_profile=self.qos)
        return self._call_sync(self.client_force_off, ForceOff.Request(device_type=device_type, address=address))

    def read_data(self, device_type, address) -> ReadData.Response | None:
        if self.client_read is None:
            self.client_read = self.node.create_client(
                ReadData, f"{self.namespace}/read_data" if self.namespace else "read_data", qos_profile=self.qos)
        return self._call_sync(self.client_read, ReadData.Request(device_type=device_type, address=address))

    def write_data(self, device_type, address, value):
        if self.client_write is None:
            self.client_write = self.node.create_client(
                WriteData, f"{self.namespace}/write_data" if self.namespace else "write_data", qos_profile=self.qos)
        return self._call_sync(self.client_write, WriteData.Request(device_type=device_type, address=address, value=value))

    def read_continuous_data(self, device_type, start_address, count) -> ReadContinuousData.Response | None:
        if self.client_read_continuous is None:
            self.client_read_continuous = self.node.create_client(
                ReadContinuousData, f"{self.namespace}/read_continuous_data" if self.namespace else "read_continuous_data", qos_profile=self.qos)
        return self._call_sync(self.client_read_continuous, ReadContinuousData.Request(
            device_type=device_type, start_address=start_address, count=count))

    def write_continuous_data(self, device_type, start_address, values):
        if self.client_write_continuous is None:
            self.client_write_continuous = self.node.create_client(
                WriteContinuousData, f"{self.namespace}/write_continuous_data" if self.namespace else "write_continuous_data", qos_profile=self.qos)
        return self._call_sync(self.client_write_continuous, WriteContinuousData.Request(
            device_type=device_type, start_address=start_address, values=values))

    def read_continuous_byte(self, device_type, start_address, count):
        if self.client_read_continuous_byte is None:
            self.client_read_continuous_byte = self.node.create_client(
                ReadContinuousByte, f"{self.namespace}/read_continuous_byte" if self.namespace else "read_continuous_byte", qos_profile=self.qos)
        return self._call_sync(self.client_read_continuous_byte, ReadContinuousByte.Request(
            device_type=device_type, start_address=start_address, count=count))

    def write_continuous_byte(self, device_type, start_address, values):
        if self.client_write_continuous_byte is None:
            self.client_write_continuous_byte = self.node.create_client(
                WriteContinuousByte, f"{self.namespace}/write_continuous_byte" if self.namespace else "write_continuous_byte", qos_profile=self.qos)
        return self._call_sync(self.client_write_continuous_byte, WriteContinuousByte.Request(
            device_type=device_type, start_address=start_address, values=values))

    # 非同步方法（與之前相同）
    def async_force_on(self, device_type, address, callback):
        if self.client_force_on is None:
            self.client_force_on = self.node.create_client(
                ForceOn, f"{self.namespace}/force_on" if self.namespace else "force_on", qos_profile=self.qos)
        self._call_async(self.client_force_on, ForceOn.Request(
            device_type=device_type, address=address), callback)

    def async_force_off(self, device_type, address, callback):
        if self.client_force_off is None:
            self.client_force_off = self.node.create_client(
                ForceOff, f"{self.namespace}/force_off" if self.namespace else "force_off", qos_profile=self.qos)
        self._call_async(self.client_force_off, ForceOff.Request(
            device_type=device_type, address=address), callback)

    def async_read_data(self, device_type, address, callback):
        if self.client_read is None:
            self.client_read = self.node.create_client(
                ReadData, f"{self.namespace}/read_data" if self.namespace else "read_data", qos_profile=self.qos)
        self._call_async(self.client_read, ReadData.Request(
            device_type=device_type, address=address), callback)

    def async_write_data(self, device_type, address, value, callback):
        if self.client_write is None:
            self.client_write = self.node.create_client(
                WriteData, f"{self.namespace}/write_data" if self.namespace else "write_data", qos_profile=self.qos)
        self._call_async(self.client_write, WriteData.Request(
            device_type=device_type, address=address, value=value), callback)

    def async_read_continuous_data(self, device_type, start_address, count, callback):
        if self.client_read_continuous is None:
            self.client_read_continuous = self.node.create_client(
                ReadContinuousData, f"{self.namespace}/read_continuous_data" if self.namespace else "read_continuous_data", qos_profile=self.qos)
        self._call_async(self.client_read_continuous, ReadContinuousData.Request(
            device_type=device_type, start_address=start_address, count=count), callback)

    def async_write_continuous_data(self, device_type, start_address, values, callback):
        if self.client_write_continuous is None:
            self.client_write_continuous = self.node.create_client(
                WriteContinuousData, f"{self.namespace}/write_continuous_data" if self.namespace else "write_continuous_data", qos_profile=self.qos)
        self._call_async(self.client_write_continuous, WriteContinuousData.Request(
            device_type=device_type, start_address=start_address, values=values), callback)

    def async_read_continuous_byte(self, device_type, start_address, count, callback):
        if self.client_read_continuous_byte is None:
            self.client_read_continuous_byte = self.node.create_client(
                ReadContinuousByte, f"{self.namespace}/read_continuous_byte" if self.namespace else "read_continuous_byte", qos_profile=self.qos)
        self._call_async(self.client_read_continuous_byte, ReadContinuousByte.Request(
            device_type=device_type, start_address=start_address, count=count), callback)

    def async_write_continuous_byte(self, device_type, start_address, values, callback):
        if self.client_write_continuous_byte is None:
            self.client_write_continuous_byte = self.node.create_client(
                WriteContinuousByte, f"{self.namespace}/write_continuous_byte" if self.namespace else "write_continuous_byte", qos_profile=self.qos)
        self._call_async(self.client_write_continuous_byte, WriteContinuousByte.Request(
            device_type=device_type, start_address=start_address, values=values), callback)

    # 共用封裝
    def _call_sync(self, client, request, timeout_sec=1.0):
        wait_for_service = client.wait_for_service(timeout_sec=1.0)
        if not wait_for_service:
            self.node.get_logger().warn(
                f"Service {client.srv_type.__name__} not available")
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=timeout_sec)
        if future.done():
            return future.result()
        else:
            self.node.get_logger().error(
                f"Service {client.srv_type.__name__} call timed out after {timeout_sec} seconds")
            return None

    def _call_async(self, client, request, callback):
        if not client.service_is_ready():
            self.node.get_logger().warn(
                f"Service {client.srv_type.__name__} not ready")
            return
        future = client.call_async(request)
        future.add_done_callback(lambda f: callback(
            f.result() if f.done() else None) if callback else None)

    def destroy(self):
        if self.client_force_on:
            self.node.destroy_client(self.client_force_on)
        if self.client_force_off:
            self.node.destroy_client(self.client_force_off)
        if self.client_read:
            self.node.destroy_client(self.client_read)
        if self.client_write:
            self.node.destroy_client(self.client_write)
        if self.client_read_continuous:
            self.node.destroy_client(self.client_read_continuous)
        if self.client_write_continuous:
            self.node.destroy_client(self.client_write_continuous)
        if self.client_read_continuous_byte:
            self.node.destroy_client(self.client_read_continuous_byte)
        if self.client_write_continuous_byte:
            self.node.destroy_client(self.client_write_continuous_byte)
        self.client_force_on = None
        self.client_force_off = None
        self.client_read = None
        self.client_write = None
        self.client_read_continuous = None
        self.client_write_continuous = None
        self.client_read_continuous_byte = None
        self.client_write_continuous_byte = None
