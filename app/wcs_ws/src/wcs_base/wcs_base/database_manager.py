"""
資料庫管理器模組
負責處理所有資料表的操作和查詢功能
"""

import rclpy
from rclpy.node import Node
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import Task, Work, TaskStatus, ProcessSettings, Product, TrafficZone, Rack, Location, Eqp, AGV, Carrier, EqpSignal, ModifyLog, AGVContext, KukaNode
from db_proxy.models import Node as NodeTable
from db_proxy.models import TaskCondition, TaskConditionHistory, TaskConditionCache
from db_proxy.crud.location_crud import location_crud
from db_proxy.crud.rack_crud import rack_crud
from sqlmodel import SQLModel, select, text
from sqlalchemy import cast, Integer
from typing import List, Optional, Any
from datetime import datetime, timezone


class DatabaseManager:
    """
    資料庫管理器類別
    負責管理所有資料表的操作和查詢功能
    """

    def __init__(self, logger, db_url_agvc: str):
        """
        初始化資料庫管理器

        Args:
            logger: ROS2 logger 實例
            db_url_agvc: 資料庫連接URL
        """
        self.logger = logger
        
        # 初始化資料庫連線池
        self.pool_agvc = ConnectionPoolManager(db_url_agvc)
        self.logger.info(f"✅ DatabaseManager 初始化完成，使用資料庫 URL: {db_url_agvc}")
        
        # 初始化資料表快取
        self.node_table = None  # node_table查詢資料
        self.work_table = None  # work_table查詢資料
        self.location_table = None  # 查詢location資料表
        self.kuka_node_table = None  # kuka_node_table查詢資料
        self.task_table = []  # 查詢task_table資料
        self.agv_context_table = []  # 查詢AGV的context資料表
        self.carrier_table = []  # 查詢carrier資料表
        self.rack_table = []  # 查詢rack資料表
        self.product_table = []  # 查詢product資料表

        # 初始化任務相關變數
        self.task_id_list = []  # task裡面的任務id列表
        self.parent_task_id_list = []  # task裡面的父任務id列表
        self.node_id_list = []  # task裡面的node_id列表
        self.rack_location_id_list = []  # rack裡面的location_id列表
        self.task_parameters_list = []  # task裡面的parameters列表

    def get_session(self):
        """獲取資料庫會話"""
        return self.pool_agvc.get_session()

    def check_node_table(self):
        """檢查node_table是否有資料,沒有則查詢一次"""
        if self.node_table is None:
            try:
                with self.pool_agvc.get_session() as session:
                    self.node_table = session.exec(select(NodeTable)).all()
            except Exception as e:
                self.logger.error(f"❌ 查詢 node_table 失敗: {e}")

    def check_work_table(self):
        """檢查work_table是否有資料,沒有則查詢一次"""
        if self.work_table is None:
            try:
                with self.pool_agvc.get_session() as session:
                    self.work_table = session.exec(select(Work)).all()
            except Exception as e:
                self.logger.error(f"❌ 查詢 work_table 失敗: {e}")

    def read_location_table(self):
        """讀取location資料表"""
        try:
            with self.pool_agvc.get_session() as session:
                self.location_table = session.exec(select(Location)).all()
        except Exception as e:
            self.logger.error(f"❌ 查詢 location_table 失敗: {e}")

    def read_kuka_node_table(self):
        """讀取kuka_node_table資料"""
        try:
            with self.pool_agvc.get_session() as session:
                self.kuka_node_table = session.exec(select(KukaNode)).all()
        except Exception as e:
            self.logger.error(f"❌ 查詢 kuka_node_table 失敗: {e}")

    def read_agv_context(self):
        """讀取AGV的context資料"""
        try:
            with self.pool_agvc.get_session() as session:
                self.agv_context_table = session.exec(select(AGVContext)).all()
        except Exception as e:
            self.logger.error(f"❌ 讀取AGV的context資料失敗: {e}")

    def read_task_table(self):
        """讀取任務資料"""
        try:
            with self.pool_agvc.get_session() as session:
                self.task_table = session.exec(select(Task)).all()
                self.task_id_list = [task.id for task in self.task_table]  # 取得所有task_id
                self.parent_task_id_list = [task.parent_task_id for task in self.task_table]  # 取得所有parent_task_id
                self.node_id_list = [task.node_id for task in self.task_table]  # 取得所有node_id
                self.task_parameters_list = [task.parameters for task in self.task_table]  # 取得所有parameters
        except Exception as e:
            self.logger.error(f"❌ 讀取任務資料失敗: {e}")

    def read_carrier_table(self):
        """讀取carrier資料"""
        try:
            with self.pool_agvc.get_session() as session:
                self.carrier_table = session.exec(select(Carrier)).all()
        except Exception as e:
            self.logger.error(f"❌ 讀取carrier資料失敗: {e}")

    def read_rack_table(self):
        """讀取rack資料表"""
        try:
            with self.pool_agvc.get_session() as session:
                self.rack_table = session.exec(select(Rack)).all()
                self.rack_location_id_list = [
                    rack.location_id for rack in self.rack_table]  # 取得locator_id
        except Exception as e:
            self.logger.error(f"❌ 讀取rack資料表失敗: {e}")

    def read_product_table(self):
        """讀取product資料表"""
        try:
            with self.pool_agvc.get_session() as session:
                self.product_table = session.exec(select(Product)).all()
        except Exception as e:
            self.logger.error(f"❌ 讀取product資料表失敗: {e}")

    def refresh_all_tables(self):
        """刷新所有資料表"""
        self.check_node_table()
        self.check_work_table()
        self.read_agv_context()
        self.read_task_table()
        self.read_location_table()
        self.read_carrier_table()
        self.read_rack_table()
        self.read_product_table()
        self.read_kuka_node_table()  # 添加缺失的 kuka_node_table 讀取

    def refresh_periodic_tables(self):
        """刷新需要定期更新的資料表"""
        self.read_kuka_node_table()

    def has_all_data(self) -> bool:
        """檢查是否有所有必要的資料"""
        return all([
            self.task_table,
            self.task_id_list,
            self.work_table,
            self.location_table,
            self.kuka_node_table,
            self.carrier_table,
            self.rack_table,
            self.product_table
        ])

    def get_uuid(self, node_id) -> str | None:
        """取得uuid"""
        try:
            for n in self.kuka_node_table:
                if n.id == node_id:
                    return n.uuid
        except Exception as e:
            self.logger.error(f"❌ 取得uuid失敗: {e}")
        return None

    def read_location_by_node_id(self, node_id):
        """根據node_id讀取location資料"""
        try:
            with self.pool_agvc.get_session() as session:
                location = session.exec(select(Location).where(Location.node_id == node_id)).first()
                return location
        except Exception as e:
            self.logger.error(f"❌ 讀取location資料失敗: {e}")
        return None

    # 提供對外訪問資料表的接口
    @property
    def nodes(self) -> List:
        """獲取node資料表"""
        return self.node_table or []

    @property
    def works(self) -> List:
        """獲取work資料表"""
        return self.work_table or []

    @property
    def locations(self) -> List:
        """獲取location資料表"""
        return self.location_table or []

    @property
    def kuka_nodes(self) -> List:
        """獲取kuka_node資料表"""
        return self.kuka_node_table or []

    @property
    def tasks(self) -> List:
        """獲取task資料表"""
        return self.task_table

    @property
    def agv_contexts(self) -> List:
        """獲取agv_context資料表"""
        return self.agv_context_table

    @property
    def carriers(self) -> List:
        """獲取carrier資料表"""
        return self.carrier_table

    @property
    def racks(self) -> List:
        """獲取rack資料表"""
        return self.rack_table

    @property
    def task_ids(self) -> List:
        """獲取task_id列表"""
        return self.task_id_list

    @property
    def parent_task_ids(self) -> List:
        """獲取parent_task_id列表"""
        return self.parent_task_id_list

    @property
    def node_ids(self) -> List:
        """獲取node_id列表"""
        return self.node_id_list

    @property
    def rack_location_ids(self) -> List:
        """獲取rack_location_id列表"""
        return self.rack_location_id_list

    @property
    def task_parameters(self) -> List:
        """獲取task_parameters列表"""
        return self.task_parameters_list

    def location_status_process(self):
        """處理位置狀態"""
        try:
            import config.config as CONFIG

            # 查詢task表裡面的node_id有無此location
            for nlocation in self.locations:
                in_task1 = False  # 在task的node_id
                in_task2 = False  # 在parameter的nodes裡面

                if nlocation.id in self.node_ids:
                    in_task1 = True

                # Location id轉換為uuid
                uuid = self.get_uuid(nlocation.id)
                # 查詢task表裡面的parameters有無此uuid
                for nparameter in self.task_parameters:
                    if nparameter and uuid and uuid in nparameter.get("nodes", []):
                        in_task2 = True
                        break
                #任務占用中的判斷
                if (in_task1 or in_task2) and nlocation.location_status_id != CONFIG.LOCATION_STATUS["任務占用中"]:
                    with self.pool_agvc.get_session() as session:
                        # 更新位置狀態
                        nlocation.location_status_id = CONFIG.LOCATION_STATUS["任務占用中"]
                        location_crud.create_or_update(session, nlocation)
                        self.logger.info(f"✅位置狀態更新成功-位置id:{nlocation.id}-->任務占用中")
                        continue
                #任務占用中以外的判斷
                if (not in_task1 and not in_task2) :
                    #檢查rack table裡面有沒有rack的location_id是占用的,若是則修改為"占用中"
                    if nlocation.id in self.rack_location_ids and nlocation.location_status_id != CONFIG.LOCATION_STATUS["占用中"]:
                        #修改位置狀態
                        with self.pool_agvc.get_session() as session:
                            nlocation.location_status_id = CONFIG.LOCATION_STATUS["占用中"]
                            location_crud.create_or_update(session, nlocation)
                            self.logger.info(f"✅位置狀態更新成功-位置id:{nlocation.id}--?占用中")
                            continue
                    if nlocation.id not in self.rack_location_ids and nlocation.location_status_id != CONFIG.LOCATION_STATUS["無佔用"]:
                        #修改位置狀態
                        with self.pool_agvc.get_session() as session:
                            nlocation.location_status_id = CONFIG.LOCATION_STATUS["無佔用"]
                            location_crud.create_or_update(session, nlocation)
                            self.logger.info(f"✅位置狀態更新成功-位置id:{nlocation.id}-->無佔用")
                            continue
                        
                

        except Exception as e:
            self.logger.error(f"❌位置狀態處理失敗: {e}")

    def rack_status_process(self):
        """定期更新 rack 資料表中的 rack_status 欄位"""
        try:
            if not self.has_all_data():
                self.logger.warning("⚠️ 資料表尚未完全載入，跳過 rack_status 更新")
                return

            # 遍歷所有 rack 記錄
            for rack in self.rack_table:
                if rack.id is None:
                    continue

                # 統計該 rack 對應的 carrier 數量
                carrier_count = sum(1 for carrier in self.carrier_table if carrier.rack_id == rack.id)

                # 取得 rack 對應的 product 資訊
                product = None
                if rack.product_id:
                    product = next((p for p in self.product_table if p.id == rack.product_id), None)

                # 計算新的 rack_status
                new_status_id = self._calculate_rack_status(rack, carrier_count, product)

                # 只有當新狀態與目前狀態不同時才更新
                if new_status_id != rack.status_id:
                    self._update_rack_status(rack.id, new_status_id)
                    rack.status_id = new_status_id  # 更新記憶體中的資料

        except Exception as e:
            self.logger.error(f"❌ rack_status_process 處理失敗: {e}")

    def _calculate_rack_status(self, rack, carrier_count: int, product) -> int:
        """計算 rack 的新狀態"""
        try:
            # 空架判斷 (ID 1)
            if carrier_count == 0:
                return 1

            # 如果沒有 product 資訊，無法判斷滿料架狀態，預設為未滿架
            if not product:
                return 4  # 預設為未滿架-32

            product_size = product.size

            # 滿料架-32判斷 (ID 2)
            if carrier_count == 32 and product_size == "S":
                return 2

            # 滿料架-16判斷 (ID 3)
            if carrier_count == 16 and product_size == "L":
                return 3

            # 未滿料-無carrier判斷 (ID 6)
            if carrier_count > 0 and carrier_count != 32 and carrier_count != 16:
                # 檢查該 rack.room_id 對應的所有 carrier 數量
                if rack.room_id:
                    room_carrier_count = sum(1 for carrier in self.carrier_table if carrier.room_id == rack.room_id)
                    if room_carrier_count == 0:
                        return 6

            # 未滿架-32判斷 (ID 4)
            if carrier_count != 32 and carrier_count > 0 and product_size == "S":
                return 4

            # 未滿架-16判斷 (ID 5)
            if carrier_count != 16 and carrier_count > 0 and product_size == "L":
                return 5

            # 預設返回空架狀態
            return 1

        except Exception as e:
            self.logger.error(f"❌ 計算 rack_status 失敗: {e}")
            return 1  # 預設返回空架狀態

    def _update_rack_status(self, rack_id: int, new_status_id: int):
        """更新資料庫中的 rack status"""
        try:
            with self.get_session() as session:
                # 查詢要更新的 rack
                rack = session.exec(select(Rack).where(Rack.id == rack_id)).first()
                if rack:
                    rack.status_id = new_status_id
                    session.add(rack)
                    session.commit()
                    self.logger.info(f"✅ 更新 rack_id:{rack_id} 狀態為:{new_status_id}")
                else:
                    self.logger.warning(f"⚠️ 找不到 rack_id:{rack_id}")

        except Exception as e:
            self.logger.error(f"❌ 更新 rack_status 失敗 rack_id:{rack_id}, error:{e}")
