"""
KUKA Container Synchronization Service

此模組負責將 RosAGV 系統的 Rack 變更同步到 KUKA Fleet Manager。
當 Rack 的位置或 is_in_map 狀態變更時，自動調用對應的 KUKA Container API。

功能：
- 智能判斷同步動作（入場/出場/位置更新）
- Location ID → KUKA Node UUID 映射
- KUKA Fleet API 調用封裝
- 錯誤處理和日誌記錄

使用範例：
    from agvcui.services.kuka_sync_service import KukaContainerSyncService

    # 建立服務實例
    sync_service = KukaContainerSyncService()

    # 同步 rack 變更
    result = sync_service.sync_rack_to_kuka(old_rack, new_rack)
    if result["success"]:
        logger.info(f"KUKA 同步成功: {result}")
    else:
        logger.error(f"KUKA 同步失敗: {result['error']}")
"""

import uuid as uuid_module
import logging
from typing import Optional, Dict, Any
from sqlmodel import select

# KUKA Fleet API Client
from kuka_fleet_adapter.kuka_api_client import KukaApiClient

# Database Models
from db_proxy.models.agvc_location import Location
from db_proxy.models.agvc_kuka import KukaNode

# Logger
logger = logging.getLogger(__name__)


class KukaContainerSyncService:
    """KUKA Container 同步服務"""

    # KUKA Fleet Manager 配置
    DEFAULT_BASE_URL = "http://192.168.10.3:10870"
    DEFAULT_USERNAME = "admin"
    DEFAULT_PASSWORD = "Admin"

    def __init__(
        self,
        base_url: str = None,
        username: str = None,
        password: str = None,
        auto_login: bool = True
    ):
        """
        初始化 KUKA 同步服務

        Args:
            base_url: KUKA Fleet Manager API URL（預設: http://192.168.10.3:10870）
            username: KUKA API 登入帳號（預設: admin）
            password: KUKA API 登入密碼（預設: Admin）
            auto_login: 是否自動登入（預設: True）
        """
        self.base_url = base_url or self.DEFAULT_BASE_URL
        self.username = username or self.DEFAULT_USERNAME
        self.password = password or self.DEFAULT_PASSWORD

        # 建立 KUKA API Client（如果提供了認證資訊則自動登入）
        if auto_login:
            self.kuka_client = KukaApiClient(
                base_url=self.base_url,
                username=self.username,
                password=self.password
            )
        else:
            self.kuka_client = KukaApiClient(base_url=self.base_url)

    def sync_rack_to_kuka(
        self,
        old_rack: Optional[Any],
        new_rack: Any,
        session: Any
    ) -> Dict[str, Any]:
        """
        同步 Rack 變更到 KUKA Fleet Manager

        智能判斷同步動作：
        - is_in_map: 0→1 → container_in（入場）
        - is_in_map: 1→0 → container_out（出場）
        - is_in_map=1 且 location_id 變更 → update_container（位置更新）
        - 其他情況 → 不同步

        Args:
            old_rack: 更新前的 Rack 物件（新增時為 None）
            new_rack: 更新後的 Rack 物件
            session: SQLModel session（用於查詢 Location 和 KukaNode）

        Returns:
            Dict 包含:
                - success: bool - 同步是否成功
                - action: str - 執行的動作 ("entry", "exit", "update", "skip")
                - result: dict - KUKA API 回應（成功時）
                - error: str - 錯誤訊息（失敗時）
        """
        try:
            # 判斷需要執行的同步動作
            action = self._determine_sync_action(old_rack, new_rack)

            if action == "skip":
                logger.debug(
                    f"[KUKA_SYNC] Rack(id={new_rack.id}, name={new_rack.name}) | "
                    f"Action: skip | Reason: 無需同步"
                )
                return {
                    "success": True,
                    "action": "skip",
                    "message": "No sync required"
                }

            # 執行對應的同步動作
            if action == "entry":
                result = self._call_container_entry(new_rack, session)
            elif action == "exit":
                result = self._call_container_exit(new_rack, session)
            elif action == "update":
                result = self._call_container_update(new_rack, session)
            else:
                return {
                    "success": False,
                    "action": "unknown",
                    "error": f"Unknown action: {action}"
                }

            return result

        except Exception as e:
            error_msg = f"KUKA sync exception: {str(e)}"
            logger.error(
                f"[KUKA_SYNC] Rack(id={new_rack.id if new_rack else 'N/A'}) | "
                f"Action: error | Error: {error_msg}",
                exc_info=True
            )
            return {
                "success": False,
                "action": "error",
                "error": error_msg
            }

    def _determine_sync_action(
        self,
        old_rack: Optional[Any],
        new_rack: Any
    ) -> str:
        """
        智能判斷需要執行的同步動作

        Args:
            old_rack: 更新前的 Rack（新增時為 None）
            new_rack: 更新後的 Rack

        Returns:
            str: "entry", "exit", "update", "skip"
        """
        # 新增 rack 的情況
        if old_rack is None:
            # 如果新增時 is_in_map=1，視為入場
            if new_rack.is_in_map == 1:
                return "entry"
            else:
                return "skip"

        # 檢查 is_in_map 狀態變化
        old_in_map = old_rack.is_in_map
        new_in_map = new_rack.is_in_map

        # 入場：0 → 1
        if old_in_map != 1 and new_in_map == 1:
            return "entry"

        # 出場：1 → 0
        if old_in_map == 1 and new_in_map != 1:
            return "exit"

        # 位置更新：is_in_map=1 且 location_id 變更
        if new_in_map == 1:
            old_location = getattr(old_rack, 'location_id', None)
            new_location = getattr(new_rack, 'location_id', None)

            if old_location != new_location and new_location is not None:
                logger.debug(
                    f"Rack {new_rack.name} location changed while in_map: "
                    f"{old_location} → {new_location}"
                )
                return "update"
        else:
            # 不在地图中时，即使 location_id 变更也不同步
            old_location = getattr(old_rack, 'location_id', None) if old_rack else None
            new_location = getattr(new_rack, 'location_id', None)
            if old_location != new_location and new_location is not None:
                logger.debug(
                    f"Rack {new_rack.name} location changed but not in_map (is_in_map={new_in_map}), "
                    f"skipping KUKA update: {old_location} → {new_location}"
                )

        # 其他情況不需要同步
        return "skip"

    def _call_container_entry(
        self,
        rack: Any,
        session: Any
    ) -> Dict[str, Any]:
        """
        調用 KUKA Container Entry API

        Args:
            rack: Rack 物件
            session: SQLModel session

        Returns:
            Dict 包含 success, action, result/error
        """
        try:
            # 取得 KUKA Node UUID
            kuka_uuid = self._get_kuka_node_uuid(rack.location_id, session)

            if not kuka_uuid:
                error_msg = f"無法取得 location_id={rack.location_id} 對應的 KUKA Node UUID"
                logger.warning(
                    f"[KUKA_SYNC] Rack(id={rack.id}, name={rack.name}) | "
                    f"Action: entry | Status: failed | Error: {error_msg}"
                )
                return {
                    "success": False,
                    "action": "entry",
                    "error": error_msg
                }

            # 建立 Container Entry DTO
            container_dto = {
                "requestId": str(uuid_module.uuid4()),
                "containerCode": rack.name,  # rack.name 對應 KUKA container code
                "position": kuka_uuid,
                "isNew": False  # 假設是既有容器
            }

            # 調用 KUKA API
            api_result = self.kuka_client.container_in(container_dto)

            # 檢查回應
            if api_result.get("success"):
                logger.info(
                    f"[KUKA_SYNC] Rack(id={rack.id}, name={rack.name}) | "
                    f"Action: entry | Status: success | "
                    f"Position: {kuka_uuid}"
                )
                return {
                    "success": True,
                    "action": "entry",
                    "result": api_result
                }
            else:
                error_msg = api_result.get("message", "Unknown error")
                logger.error(
                    f"[KUKA_SYNC] Rack(id={rack.id}, name={rack.name}) | "
                    f"Action: entry | Status: failed | Error: {error_msg}"
                )
                return {
                    "success": False,
                    "action": "entry",
                    "error": error_msg
                }

        except Exception as e:
            error_msg = f"Container entry exception: {str(e)}"
            logger.error(
                f"[KUKA_SYNC] Rack(id={rack.id}, name={rack.name}) | "
                f"Action: entry | Status: error | Error: {error_msg}",
                exc_info=True
            )
            return {
                "success": False,
                "action": "entry",
                "error": error_msg
            }

    def _call_container_exit(
        self,
        rack: Any,
        session: Any
    ) -> Dict[str, Any]:
        """
        調用 KUKA Container Exit API

        Args:
            rack: Rack 物件
            session: SQLModel session

        Returns:
            Dict 包含 success, action, result/error
        """
        try:
            # 建立 Container Exit DTO
            container_dto = {
                "requestId": str(uuid_module.uuid4()),
                "containerCode": rack.name  # rack.name 對應 KUKA container code
            }

            # 調用 KUKA API
            api_result = self.kuka_client.container_out(container_dto)

            # 檢查回應
            if api_result.get("success"):
                logger.info(
                    f"[KUKA_SYNC] Rack(id={rack.id}, name={rack.name}) | "
                    f"Action: exit | Status: success"
                )
                return {
                    "success": True,
                    "action": "exit",
                    "result": api_result
                }
            else:
                error_msg = api_result.get("message", "Unknown error")
                logger.error(
                    f"[KUKA_SYNC] Rack(id={rack.id}, name={rack.name}) | "
                    f"Action: exit | Status: failed | Error: {error_msg}"
                )
                return {
                    "success": False,
                    "action": "exit",
                    "error": error_msg
                }

        except Exception as e:
            error_msg = f"Container exit exception: {str(e)}"
            logger.error(
                f"[KUKA_SYNC] Rack(id={rack.id}, name={rack.name}) | "
                f"Action: exit | Status: error | Error: {error_msg}",
                exc_info=True
            )
            return {
                "success": False,
                "action": "exit",
                "error": error_msg
            }

    def _call_container_update(
        self,
        rack: Any,
        session: Any
    ) -> Dict[str, Any]:
        """
        調用 KUKA Container Update API

        Args:
            rack: Rack 物件
            session: SQLModel session

        Returns:
            Dict 包含 success, action, result/error
        """
        try:
            # 取得 KUKA Node UUID
            kuka_uuid = self._get_kuka_node_uuid(rack.location_id, session)

            if not kuka_uuid:
                error_msg = f"無法取得 location_id={rack.location_id} 對應的 KUKA Node UUID"
                logger.warning(
                    f"[KUKA_SYNC] Rack(id={rack.id}, name={rack.name}) | "
                    f"Action: update | Status: failed | Error: {error_msg}"
                )
                return {
                    "success": False,
                    "action": "update",
                    "error": error_msg
                }

            # 建立 Container Update DTO
            update_dto = {
                "containerCode": rack.name,  # rack.name 對應 KUKA container code
                "position": kuka_uuid
                # 可選：orientation, containerSize 等
            }

            # 調用 KUKA API
            api_result = self.kuka_client.update_container(update_dto)

            # 檢查回應
            if api_result.get("success"):
                logger.info(
                    f"[KUKA_SYNC] Rack(id={rack.id}, name={rack.name}) | "
                    f"Action: update | Status: success | "
                    f"Position: {kuka_uuid}"
                )
                return {
                    "success": True,
                    "action": "update",
                    "result": api_result
                }
            else:
                error_msg = api_result.get("message", "Unknown error")
                logger.error(
                    f"[KUKA_SYNC] Rack(id={rack.id}, name={rack.name}) | "
                    f"Action: update | Status: failed | Error: {error_msg}"
                )
                return {
                    "success": False,
                    "action": "update",
                    "error": error_msg
                }

        except Exception as e:
            error_msg = f"Container update exception: {str(e)}"
            logger.error(
                f"[KUKA_SYNC] Rack(id={rack.id}, name={rack.name}) | "
                f"Action: update | Status: error | Error: {error_msg}",
                exc_info=True
            )
            return {
                "success": False,
                "action": "update",
                "error": error_msg
            }

    def _get_kuka_node_uuid(
        self,
        location_id: Optional[int],
        session: Any
    ) -> Optional[str]:
        """
        Location ID → KUKA Node UUID 映射

        查詢流程：
        1. 從 Location 取得 node_id
        2. 從 KukaNode 取得 uuid (where id = node_id)

        Args:
            location_id: Location ID
            session: SQLModel session

        Returns:
            str: KUKA Node UUID，查詢失敗返回 None
        """
        if location_id is None:
            return None

        try:
            # 查詢 Location
            location = session.get(Location, location_id)
            if not location or not location.node_id:
                logger.debug(
                    f"[KUKA_SYNC] Location(id={location_id}) 無對應的 node_id"
                )
                return None

            # 查詢 KukaNode
            kuka_node = session.get(KukaNode, location.node_id)
            if not kuka_node or not kuka_node.uuid:
                logger.debug(
                    f"[KUKA_SYNC] KukaNode(id={location.node_id}) 無 UUID"
                )
                return None

            return kuka_node.uuid

        except Exception as e:
            logger.error(
                f"[KUKA_SYNC] Location ID 映射錯誤: location_id={location_id}, error={str(e)}",
                exc_info=True
            )
            return None
