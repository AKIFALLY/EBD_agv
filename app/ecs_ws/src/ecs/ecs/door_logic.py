from typing import Dict, Callable, Union
from ecs.door_controller_config import DoorControllerConfig
from plc_proxy.plc_client import PlcClient
from plc_proxy.plc_client_node import PlcClientNode
from dataclasses import dataclass


@dataclass
class DoorState:
    door_id: int
    state: str


class DoorLogic:
    def __init__(self, plc_client: Union[PlcClient, PlcClientNode], config: DoorControllerConfig):
        """
        初始化 DoorLogic，並傳入 PLC 客戶端和門設定。
        :param plc_client: 用來與 PLC 通訊的 PlcClient 或 PlcClientNode 實例
        :param config: DoorControllerConfig 實例，提供門控制配置
        """
        self.plc_client = plc_client
        self.config = config
        
        # Get the logger based on the type of plc_client
        # PlcClient has a 'node' attribute, PlcClientNode IS a Node
        if hasattr(plc_client, 'node'):
            # It's a PlcClient
            self._logger_node = plc_client.node
        else:
            # It's a PlcClientNode (or other Node-based class)
            self._logger_node = plc_client

    def async_control_door(self, door_id: int, is_open: bool) -> Dict:
        """
        根據門 ID 和開關狀態控制門開關。
        :param door_id: 門的識別 ID
        :param is_open: 是否開門，True 為開門，False 為關門
        :return: 控制結果字典
        """
        cfg = self.config.get_config(door_id)
        if not cfg:
            # self.plc_client.get_logger().warning(f"無法找到門設定: doorId={door_id}")
            raise ValueError(f"Unknown doorId: {door_id}")

        # 根據設定控制門
        if is_open:
            response = self.plc_client.async_force_on(
                cfg["mr_type"], cfg["mr_address"], self.force_callback)
        else:
            response = self.plc_client.async_force_off(
                cfg["mr_type"], cfg["mr_address"], self.force_callback)

    def force_callback(self, response):
        """
        控制門的回呼函數。
        :param response: PLC 回應
        """
        logger = self._logger_node.get_logger()
        
        if response is None:
            logger.error("未收到 PLC 的回應")
            return
        if not response.success:
            logger.error("控制門失敗")
            return
        logger.info("控制門成功")

    def async_state_door(self, door_id: int, callback: Callable[[Dict], None]) -> None:
        cfg = self.config.get_config(door_id)
        if not cfg:
            # self.plc_client.get_logger().warning(f"無法找到門設定: doorId={door_id}")
            raise ValueError(f"Unknown doorId: {door_id}")

        # self.plc_client.get_logger().info(f"查詢門狀態: doorId={door_id}, dm_type={cfg['dm_type']}, dm_address={cfg['dm_address']}")

        def handle_response(response):
            if response is None:
                # self.plc_client.get_logger().error(f"未收到門 {door_id} 的回應")
                callback({"doorId": door_id, "state": "UNKNOWN",
                         "isOpen": False, "success": False})
                return

            bit = bool(response.values[0])
            door_state = "OPEN" if bit else "CLOSE"
            result = {
                "doorId": door_id,
                "state": door_state,
                "isOpen": bit,
                "success": response.success
            }
            callback(result)

        self.plc_client.async_read_continuous_byte(
            cfg["dm_type"], cfg["dm_address"], 1, handle_response)

    def state_door(self, door_id: int) -> Dict:
        """
        同步查詢門的狀態。
        :param door_id: 門的識別 ID
        :return: 門的狀態字典
        """
        cfg = self.config.get_config(door_id)
        if not cfg:
            raise ValueError(f"Unknown doorId: {door_id}")

        response = self.plc_client.read_continuous_byte(
            cfg["dm_type"], cfg["dm_address"], 1)
        if response is None or not response.success:
            return {"doorId": door_id, "state": "UNKNOWN", "isOpen": False, "success": False}

        bit = bool(response.values[0])
        door_state = "OPEN" if bit else "CLOSE"
        return {
            "doorId": door_id,
            "state": door_state,
            "isOpen": bit,
            "success": response.success
        }

    def batch_control(self, door_ids: list, is_open: bool) -> Dict:
        """
        批量控制門開關
        :param door_ids: 門的識別 ID 列表
        :param is_open: 是否開門，True 為開門，False 為關門
        :return: 控制結果字典
        """
        result = {}
        for door_id in door_ids:
            result[door_id] = self.control_door(door_id, is_open)
        return result
