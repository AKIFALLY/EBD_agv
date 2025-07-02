"""
KUKA 容器管理模組
處理 KUKA 容器狀態更新和相關操作
KUKA 中的 container 對應到資料庫中的 Rack 表
"""
from db_proxy.models import Rack, ModifyLog
from sqlmodel import select


class KukaContainer:
    """KUKA 容器管理類別"""

    def __init__(self, rcs_core_node):
        """
        初始化 KUKA 容器管理器

        Args:
            rcs_core_node: RCS Core 節點實例，用於日誌記錄和資料庫存取
        """
        self.rcs_core = rcs_core_node
        self.logger = rcs_core_node.get_logger()

    def on_container_update(self, containers: list):
        """
        當 KukaFleetAdapter 查詢到容器狀態時，此方法會被呼叫。
        更新資料庫中的 Rack 資料 (KUKA container 對應 Rack 表)。

        Args:
            containers: 容器狀態列表
        """
        # self.logger.info(
        #    f"--- [KUKA Container] 接收到 {len(containers)} 個 KUKA 容器狀態更新 ---")

        # 處理容器資料並更新 Rack 資料庫
        self.update_container_database(containers)

    def get_rack_status(self, rack_name: str):
        """
        取得指定 Rack 的狀態

        Args:
            rack_name: Rack 名稱 (對應 KUKA containerCode)

        Returns:
            dict: Rack 狀態資訊，如果找不到則返回 None
        """
        if not self.rcs_core.db_pool:
            self.logger.error("資料庫連線池不可用，無法查詢 Rack 狀態。")
            return None

        try:
            with self.rcs_core.db_pool.get_session() as session:
                rack = self._get_rack_by_name(session, rack_name)
                if rack:
                    return self._rack_to_dict(rack)
                return None
        except Exception as e:
            self.logger.error(f"查詢 Rack 狀態時發生錯誤: {e}")
            return None

    def get_rack_by_id(self, rack_id: int):
        """
        根據 ID 取得指定 Rack 的狀態

        Args:
            rack_id: Rack ID

        Returns:
            dict: Rack 狀態資訊，如果找不到則返回 None
        """
        if not self.rcs_core.db_pool:
            self.logger.error("資料庫連線池不可用，無法查詢 Rack 狀態。")
            return None

        try:
            with self.rcs_core.db_pool.get_session() as session:
                rack = session.exec(
                    select(Rack).where(Rack.id == rack_id)
                ).first()

                if rack:
                    return self._rack_to_dict(rack)
                return None
        except Exception as e:
            self.logger.error(f"查詢 Rack ID {rack_id} 時發生錯誤: {e}")
            return None

    def _rack_to_dict(self, rack: Rack):
        """
        將 Rack 物件轉換為字典

        Args:
            rack: Rack 物件

        Returns:
            dict: Rack 資訊字典
        """
        return {
            "id": rack.id,
            "name": rack.name,
            "agv_id": rack.agv_id,
            "location_id": rack.location_id,
            "product_id": rack.product_id,
            "is_carry": rack.is_carry,
            "is_in_map": rack.is_in_map,
            "is_docked": rack.is_docked,
            "status_id": rack.status_id,
            "direction": rack.direction
        }

    def get_carrying_racks(self):
        """
        取得所有正在被搬運的 Rack

        Returns:
            list: 正在被搬運的 Rack 列表
        """
        if not self.rcs_core.db_pool:
            self.logger.error("資料庫連線池不可用，無法查詢搬運中的 Rack。")
            return []

        try:
            with self.rcs_core.db_pool.get_session() as session:
                carrying_racks = session.exec(
                    select(Rack).where(Rack.is_carry == 1)
                ).all()

                return [
                    {
                        "id": rack.id,
                        "name": rack.name,
                        "agv_id": rack.agv_id,
                        "location_id": rack.location_id,
                        "is_carry": rack.is_carry,
                        "is_in_map": rack.is_in_map
                    }
                    for rack in carrying_racks
                ]
        except Exception as e:
            self.logger.error(f"查詢搬運中的 Rack 時發生錯誤: {e}")
            return []

    def get_in_map_racks(self):
        """
        取得所有在地圖中的 Rack

        Returns:
            list: 在地圖中的 Rack 列表
        """
        if not self.rcs_core.db_pool:
            self.logger.error("資料庫連線池不可用，無法查詢地圖中的 Rack。")
            return []

        try:
            with self.rcs_core.db_pool.get_session() as session:
                in_map_racks = session.exec(
                    select(Rack).where(Rack.is_in_map == 1)
                ).all()

                return [
                    {
                        "id": rack.id,
                        "name": rack.name,
                        "location_id": rack.location_id,
                        "is_carry": rack.is_carry,
                        "is_in_map": rack.is_in_map,
                        "agv_id": rack.agv_id
                    }
                    for rack in in_map_racks
                ]
        except Exception as e:
            self.logger.error(f"查詢地圖中的 Rack 時發生錯誤: {e}")
            return []

    def get_racks_by_ids(self, rack_ids: list):
        """
        根據 ID 列表批量取得 Rack 狀態

        Args:
            rack_ids: Rack ID 列表

        Returns:
            list: Rack 狀態資訊列表
        """
        if not rack_ids:
            return []

        if not self.rcs_core.db_pool:
            self.logger.error("資料庫連線池不可用，無法查詢 Rack 狀態。")
            return []

        try:
            with self.rcs_core.db_pool.get_session() as session:
                racks = session.exec(
                    select(Rack).where(Rack.id.in_(rack_ids))
                ).all()

                return [self._rack_to_dict(rack) for rack in racks]

        except Exception as e:
            self.logger.error(f"批量查詢 Rack 時發生錯誤: {e}")
            return []

    def get_all_racks(self):
        """
        取得所有 Rack 的狀態

        Returns:
            list: 所有 Rack 狀態資訊列表
        """
        if not self.rcs_core.db_pool:
            self.logger.error("資料庫連線池不可用，無法查詢 Rack 狀態。")
            return []

        try:
            with self.rcs_core.db_pool.get_session() as session:
                racks = session.exec(select(Rack)).all()
                return [self._rack_to_dict(rack) for rack in racks]

        except Exception as e:
            self.logger.error(f"查詢所有 Rack 時發生錯誤: {e}")
            return []

    def update_container_database(self, containers: list):
        """
        更新容器資料到資料庫 (KUKA container 對應 Rack 表)

        Args:
            containers: 容器狀態列表
        """
        if not self._validate_database_connection():
            return

        if not containers:
            return

        try:
            with self.rcs_core.db_pool.get_session() as session:
                updated_count = 0

                for container in containers:
                    if self._process_single_container(session, container):
                        updated_count += 1

                # 標記 Rack 資料已更新
                if updated_count > 0:
                    ModifyLog.mark(session, "rack")
                    self.logger.debug(f"成功更新 {updated_count} 個 Rack 狀態")

        except Exception as e:
            self.logger.error(f"更新容器狀態時發生資料庫錯誤: {e}", exc_info=True)

    def _validate_database_connection(self):
        """驗證資料庫連線是否可用"""
        if not self.rcs_core.db_pool:
            self.logger.error("資料庫連線池不可用，無法更新容器狀態。")
            return False
        return True

    def _process_single_container(self, session, container_data: dict):
        """
        處理單個容器的資料更新

        Args:
            session: 資料庫 session
            container_data: 容器資料字典

        Returns:
            bool: 是否成功處理
        """
        container_code = container_data.get("containerCode")
        if not container_code:
            self.logger.warning("收到的容器資料缺少 'containerCode'，跳過此筆更新")
            return False

        try:
            # 驗證容器資料
            if not self._validate_container_data(container_data):
                return False

            # 根據 containerCode (name) 查找 Rack
            rack = self._get_rack_by_name(session, container_code)
            # rack = self._get_rack_by_id(session, container_code)  # 也可以用id來查詢
            if not rack:
                # self.logger.warning(f"找不到名稱為 {container_code} 的 Rack，跳過更新")
                return False

            # self.logger.info(f"正在處理容器 {container_code}")
            # 更新 Rack 資料
            self._update_rack_data(rack, container_data)

            # 提交變更
            session.commit()

            self.logger.debug(f"已成功更新 Rack {container_code} 的狀態")
            return True

        except Exception as e:
            self.logger.error(f"處理容器 {container_code} 時發生錯誤: {e}")
            return False

    def _validate_container_data(self, container_data: dict):
        """
        驗證容器資料的完整性

        Args:
            container_data: 容器資料字典

        Returns:
            bool: 資料是否有效
        """
        if not container_data:
            self.logger.warning("容器資料為空")
            return False

        required_fields = ["containerCode"]

        for field in required_fields:
            if field not in container_data or container_data[field] is None:
                self.logger.warning(f"容器資料缺少必要欄位: {field}")
                return False

        return True

    def _get_rack_by_name(self, session, rack_name: str):
        """
        根據名稱查找 Rack

        Args:
            session: 資料庫 session
            rack_name: Rack 名稱 (對應 KUKA containerCode)

        Returns:
            Rack: Rack 物件，如果不存在則返回 None
        """
        try:
            rack = session.exec(
                select(Rack).where(Rack.name == rack_name)
            ).first()
            return rack
        except Exception as e:
            self.logger.error(f"查詢 Rack {rack_name} 時發生錯誤: {e}")
            return None

    def _update_rack_data(self, rack: Rack, container_data: dict):
        """
        更新 Rack 物件的資料

        Args:
            rack: Rack 物件
            container_data: 容器資料字典
        """
        # 更新 is_carry 狀態 (是否被搬運)
        is_carry = container_data.get("isCarry")
        if is_carry is not None:
            rack.is_carry = 1 if is_carry else 0

        # 更新 is_in_map 狀態 (是否入場)
        is_in_map = container_data.get("inMapStatus")
        if is_in_map is not None:
            rack.is_in_map = 1 if is_in_map else 0

        # 根據搬運狀態更新 AGV 關聯
        # 如果正在被搬運，可能需要設定 agv_id
        # 如果沒有被搬運，可能需要清除 agv_id
        # 這部分邏輯可能需要根據實際業務需求調整

        self.logger.debug(
            f"更新 Rack {rack.name}: is_carry={rack.is_carry}, is_in_map={rack.is_in_map}"
        )
