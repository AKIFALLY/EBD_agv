"""
交通管制 HTTP 客戶端（簡化版）

負責與 web_api 的 /traffic/acquire 和 /traffic/release 端點通訊
"""

import requests
from typing import Dict, Any
import rclpy
from rclpy.node import Node


class TrafficClient:
    """
    交通管制 HTTP 客戶端

    只負責與 web_api 的 /traffic/acquire 和 /traffic/release 通訊
    """

    def __init__(self, node: Node, api_base_url: str = "http://agvc.webapi"):
        """
        Args:
            node: ROS2 節點（用於日誌記錄）
            api_base_url: web_api 的基礎 URL
        """
        self.node = node
        self.api_base_url = api_base_url
        self.timeout = 5.0  # HTTP 請求超時時間（秒）- 增加到 5 秒

    def acquire_traffic_zone(self, traffic_id: int, agv_id: int) -> Dict[str, Any]:
        """
        請求交管區控制權

        Args:
            traffic_id: 交管區 ID（例如：1 表示 room2）
            agv_id: AGV ID

        Returns:
            {
                "success": bool,
                "isAllow": bool,      # 是否允許通行
                "message": str,
                "owner_agv_id": int   # (可選) 當前擁有者
            }
        """
        url = f"{self.api_base_url}/traffic/acquire"
        payload = {
            "trafficId": str(traffic_id),
            "agvId": str(agv_id)
        }

        # 記錄 request body
        self.node.get_logger().info(
            f"[交管] 📤 發送請求: POST {url}, body={payload}"
        )

        try:
            response = requests.post(url, json=payload, timeout=self.timeout)
            response.raise_for_status()
            result = response.json()

            # 記錄 response body
            self.node.get_logger().info(
                f"[交管] 📥 收到回應: status={response.status_code}, body={result}"
            )

            if result.get("isAllow", False):
                self.node.get_logger().info(
                    f"[交管] ✅ 允許通行交管區 {traffic_id}"
                )
            else:
                owner_id = result.get("owner_agv_id", "未知")
                self.node.get_logger().warn(
                    f"[交管] ⛔ 拒絕通行：交管區 {traffic_id} 被 AGV {owner_id} 佔用"
                )

            return result

        except requests.exceptions.Timeout:
            self.node.get_logger().error(f"[交管] 請求超時: {url}")
            return {"success": False, "isAllow": False, "message": "請求超時"}

        except requests.exceptions.RequestException as e:
            self.node.get_logger().error(f"[交管] HTTP 請求失敗: {e}")
            return {"success": False, "isAllow": False, "message": str(e)}

    def release_traffic_zone(self, traffic_id: int, agv_id: int) -> bool:
        """
        釋放交管區控制權

        Args:
            traffic_id: 交管區 ID
            agv_id: AGV ID

        Returns:
            bool: 是否成功釋放
        """
        url = f"{self.api_base_url}/traffic/release"
        payload = {
            "trafficId": str(traffic_id),
            "agvId": str(agv_id)
        }

        # 記錄 request body
        self.node.get_logger().info(
            f"[交管] 📤 發送釋放請求: POST {url}, body={payload}"
        )

        try:
            response = requests.post(url, json=payload, timeout=self.timeout)
            response.raise_for_status()
            result = response.json()

            # 記錄 response body
            self.node.get_logger().info(
                f"[交管] 📥 收到釋放回應: status={response.status_code}, body={result}"
            )

            success = result.get("success", False)
            if success:
                self.node.get_logger().info(f"[交管] ✅ 成功釋放交管區 {traffic_id}")
            else:
                self.node.get_logger().warn(f"[交管] ⚠️ 釋放失敗: {traffic_id}")

            return success

        except requests.exceptions.RequestException as e:
            self.node.get_logger().error(f"[交管] 釋放請求失敗: {e}")
            return False
