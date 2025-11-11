"""
OPUI 专用 KUKA Container 同步服务（简化版）
只实现容器入场和出场功能，位置更新由 RCS 负责

使用场景：
1. 派车任务产生前 → sync_container_entry()（入场）
2. HMI 移除 rack → sync_container_exit()（出场）
"""
import uuid as uuid_module
import logging
from typing import Optional, Dict, Any
from kuka_fleet_adapter.kuka_api_client import KukaApiClient
from db_proxy.models.agvc_location import Location
from db_proxy.models.agvc_kuka import KukaNode

logger = logging.getLogger(__name__)


class OpuiKukaContainerSync:
    """OPUI 专用的 KUKA Container 同步服务（简化版）"""

    DEFAULT_BASE_URL = "http://192.168.10.3:10870"
    DEFAULT_USERNAME = "admin"
    DEFAULT_PASSWORD = "Admin"

    def __init__(self):
        """初始化 KUKA API Client"""
        try:
            self.kuka_client = KukaApiClient(
                base_url=self.DEFAULT_BASE_URL,
                username=self.DEFAULT_USERNAME,
                password=self.DEFAULT_PASSWORD
            )
            logger.info(f"✅ KUKA API Client 初始化成功: {self.DEFAULT_BASE_URL}")
        except Exception as e:
            logger.error(f"❌ KUKA API Client 初始化失败: {str(e)}")
            self.kuka_client = None

    def sync_container_entry(self, rack, location_id: int, session) -> Dict[str, Any]:
        """
        容器入场同步

        Args:
            rack: Rack 对象
            location_id: 目标 Location ID
            session: 数据库会话

        Returns:
            {"success": bool, "message": str, "kuka_uuid": str}
        """
        try:
            # 检查 KUKA Client
            if not self.kuka_client:
                return {"success": False, "message": "KUKA API Client 未初始化"}

            # 获取 KUKA UUID
            kuka_uuid = self._get_kuka_node_uuid(location_id, session)
            if not kuka_uuid:
                error_msg = f"无法获取 Location(id={location_id}) 的 KUKA UUID"
                logger.warning(f"⚠️ {error_msg}")
                return {"success": False, "message": error_msg}

            # 🆕 1. 先更新 rack.direction = 0（入场时重置角度）
            old_direction = rack.direction
            rack.direction = 0
            session.add(rack)
            session.commit()
            logger.info(f"📐 重置料架角度: {rack.name} | direction {old_direction}° → 0° (入场前)")

            # 🆕 2. 构建 container_in DTO（添加 enterOrientation）
            container_dto = {
                "requestId": str(uuid_module.uuid4()),
                "containerCode": rack.name,
                "position": kuka_uuid,
                "enterOrientation": "0",  # 🔑 明确指定入场角度为 0
                "isNew": False
            }

            logger.info(f"📥 KUKA 容器入场: {rack.name} → Location(id={location_id}, uuid={kuka_uuid}) | enterOrientation=0°")

            # 调用 KUKA API
            result = self.kuka_client.container_in(container_dto)

            # 检查结果
            if result.get("success"):
                logger.info(f"✅ KUKA 入场成功: {rack.name} → {kuka_uuid} | 已重置 direction=0°")
                return {
                    "success": True,
                    "message": "容器入场成功",
                    "kuka_uuid": kuka_uuid
                }
            else:
                error_msg = result.get("message", "未知错误")
                logger.warning(f"⚠️ KUKA 入场失败: {rack.name} | 错误: {error_msg}")
                return {
                    "success": False,
                    "message": f"KUKA API 返回失败: {error_msg}"
                }

        except Exception as e:
            error_msg = f"容器入场异常: {str(e)}"
            logger.error(f"❌ {error_msg}")
            return {"success": False, "message": error_msg}

    def sync_container_exit(self, rack) -> Dict[str, Any]:
        """
        容器出场同步

        Args:
            rack: Rack 对象

        Returns:
            {"success": bool, "message": str}
        """
        try:
            # 检查 KUKA Client
            if not self.kuka_client:
                return {"success": False, "message": "KUKA API Client 未初始化"}

            # 构建 container_out DTO
            container_dto = {
                "requestId": str(uuid_module.uuid4()),
                "containerCode": rack.name
            }

            logger.info(f"📤 KUKA 容器出场: {rack.name}")

            # 调用 KUKA API
            result = self.kuka_client.container_out(container_dto)

            # 检查结果
            if result.get("success"):
                logger.info(f"✅ KUKA 出场成功: {rack.name}")
                return {
                    "success": True,
                    "message": "容器出场成功"
                }
            else:
                error_msg = result.get("message", "未知错误")
                logger.warning(f"⚠️ KUKA 出场失败: {rack.name} | 错误: {error_msg}")
                return {
                    "success": False,
                    "message": f"KUKA API 返回失败: {error_msg}"
                }

        except Exception as e:
            error_msg = f"容器出场异常: {str(e)}"
            logger.error(f"❌ {error_msg}")
            return {"success": False, "message": error_msg}

    def _get_kuka_node_uuid(self, location_id: int, session) -> Optional[str]:
        """
        Location ID → KUKA Node UUID 映射

        查询流程：
        1. 从 Location 表获取 node_id
        2. 从 KukaNode 表获取 uuid (where id = node_id)

        Args:
            location_id: Location ID
            session: 数据库会话

        Returns:
            KUKA Node UUID 或 None
        """
        if location_id is None:
            logger.debug("Location ID 为 None，无法映射 KUKA UUID")
            return None

        try:
            # 查询 Location
            location = session.get(Location, location_id)
            if not location:
                logger.debug(f"Location(id={location_id}) 不存在")
                return None

            if not location.node_id:
                logger.debug(f"Location(id={location_id}) 无对应的 node_id")
                return None

            # 查询 KukaNode
            kuka_node = session.get(KukaNode, location.node_id)
            if not kuka_node:
                logger.debug(f"KukaNode(id={location.node_id}) 不存在")
                return None

            if not kuka_node.uuid:
                logger.debug(f"KukaNode(id={location.node_id}) 无 UUID")
                return None

            logger.debug(f"Location(id={location_id}) → KukaNode(id={location.node_id}, uuid={kuka_node.uuid})")
            return kuka_node.uuid

        except Exception as e:
            logger.error(f"❌ Location UUID 映射异常: {str(e)}")
            return None
