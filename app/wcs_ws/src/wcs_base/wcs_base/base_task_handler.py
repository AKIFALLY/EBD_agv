import rclpy
from rclpy.node import Node
from db_proxy.models import Task, Location
from db_proxy.crud.task_crud import task_crud
from db_proxy.crud.location_crud import location_crud
from datetime import datetime, timezone
from abc import ABC, abstractmethod
from typing import List, Tuple, Dict, Any, Optional


class BaseTaskHandler(ABC):
    """
    任務處理器基類
    所有任務處理器都應該繼承此類並實現抽象方法
    """

    def __init__(self, node):
        """
        初始化任務處理器

        Args:
            node: WCSBaseNode 實例
        """
        from wcs_base.wcs_base_node import WCSBaseNode
        self.node: WCSBaseNode = node
        self.db_manager = node.db_manager  # 獲取資料庫管理器的引用

        # 任務狀態標記
        self.find_task = False  # 找到任務標記
        self.task_inserted = False  # 插入任務中

        # 初始化日誌
        self.node.get_logger().info(f"✅{self.__class__.__name__} 初始化完成")

    def execute(self):
        """
        執行任務處理流程
        這是標準的模板方法模式，定義了處理任務的標準流程
        """
        if not self.find_task:  # 檢查任務條件
            self.check_condition()
        if self.find_task and not self.task_inserted:  # 插入任務
            self.insert_task()
        if self.task_inserted:  # 檢查任務插入是否成功
            self.check_insert_done()

    @abstractmethod
    def check_condition(self) -> bool:
        """
        檢查任務成立條件

        Returns:
            bool: 條件是否成立
        """
        pass

    @abstractmethod
    def insert_task(self) -> bool:
        """
        插入任務

        Returns:
            bool: 是否成功插入
        """
        pass

    @abstractmethod
    def check_insert_done(self) -> bool:
        """
        檢查任務插入是否成功

        Returns:
            bool: 是否插入成功
        """
        pass



    def create_task(self, session, task_data: Task) -> Optional[Task]:
        """
        創建任務的通用方法

        Args:
            session: 數據庫會話
            task_data: 任務數據

        Returns:
            Optional[Task]: 創建的任務或None（如果失敗）
        """
        try:
            task = task_crud.create(session, task_data)
            self.node.get_logger().info(f"🔄任務插入中......任務id:{task.id}")
            return task
        except Exception as e:
            self.node.get_logger().error(f"❌插入任務失敗: {e}")
            return None
        
    def delete_task(self, session, task_id: int) -> bool:
        """
        刪除任務的通用方法

        Args:
            session: 數據庫會話
            task_id: 任務ID

        Returns:
            bool: 是否成功刪除
        """
        try:
            task_crud.delete(session, task_id)
            self.node.get_logger().info(f"✅任務刪除成功-任務id:{task_id}")
            return True
        except Exception as e:
            msg = str(e)
            if "foreign key" in msg.lower():
                self.node.get_logger().error(f"❌ 你嘗試 刪除或更新 task 資料表中 ID 為 {task_id} 的資料，但這筆資料仍被其他 task 資料表中的資料當作 parent_task_id 外鍵所參照，所以資料庫不允許你這樣做。")


    #def update_location_status(self, session, location_id: int, location_status_id: int) -> bool:
    #    """
    #    更新位置狀態的通用方法
#
    #    Args:
    #        session: 數據庫會話
    #        location_id: 位置ID
    #        location_status_id: 新的位置狀態ID
    #
    #    Returns:
    #        bool: 是否成功更新
    #    """
    #    try:
    #        location_crud.create_or_update(session, location_id, location_status_id)
    #        self.node.get_logger().info(f"✅位置狀態更新成功-位置id:{location_id}-->{CONFIG.LOCATION_STATUS.get(location_status_id)}")
    #        return True
    #    except Exception as e:
    #        self.node.get_logger().error(f"❌位置狀態更新失敗: {e}")
    #        return False
    
    def update_location_status(self, session, location_id: int, location_status_id: int) -> bool:
        """
        更新位置狀態的通用方法

        Args:
            session: 數據庫會話
            location_id: 位置ID
            location_status_id: 新的位置狀態ID

        Returns:
            bool: 是否成功更新
        """
        try:
            # 先獲取現有的 location 物件
            location = location_crud.get_by_id(session, location_id)
            if not location:
                # 如果不存在，創建新的 location 物件
                location = Location(id=location_id, location_status_id=location_status_id)
            else:
                # 如果存在，更新狀態
                location.location_status_id = location_status_id

            # 使用 create_or_update 方法更新
            location_crud.create_or_update(session, location)
            try:
                import config.config as CONFIG
                nlocation_status = {v: k for k, v in CONFIG.LOCATION_STATUS.items()}
                status_name = nlocation_status.get(location_status_id)
            except (ImportError, AttributeError):
                status_name = f"狀態{location_status_id}"
            self.node.get_logger().info(f"✅位置狀態更新成功-位置id:{location_id}-->{status_name}({location_status_id})")
            return True
        except Exception as e:
            self.node.get_logger().error(f"❌位置狀態更新失敗: {e}")
            return False





    def get_task_by_id(self, session, task_id: int) -> Optional[Task]:
        """
        通過ID獲取任務

        Args:
            session: 數據庫會話
            task_id: 任務ID

        Returns:
            Optional[Task]: 任務或None（如果不存在）
        """
        try:
            return task_crud.get_by_id(session, task_id)
        except Exception as e:
            self.node.get_logger().error(f"❌獲取任務失敗: {e}")
            return None



    # 取得rack位置
    def get_rack_location(self, rack_id):
        for rack in self.db_manager.racks:
            if rack.id == rack_id:
                return rack.location_id
        return None

    # 取得uuid

    def get_uuid(self, node_id) -> str | None:
        """取得uuid - 委託給 DatabaseManager"""
        return self.db_manager.get_uuid(node_id)


    # node_id 換算

    def convert_to_node_id(self, room_id=None, description: str = ""):
        eqp = 0
        act = 0
        if description.count("入口") > 0:
            eqp = 1
        if description.count("出口") > 0:
            eqp = 2
        if description.count("取") > 0:
            act = 1
        if description.count("放") > 0:
            act = 2
        nnode_id = room_id*10000+eqp
        return nnode_id

    # 檢查kuka是否有相同的任務
    def check_kuka_task_doing(self, node_id):
        import config.config as CONFIG
        result = False
        # 如果有node_id
        for nTask in self.db_manager.tasks:
            if nTask.work_id == CONFIG.KUKA_RACK_MOVE and node_id == nTask.node_id:
                result = True
                break
        return result
    
    def read_location_by_node_id(self, node_id):
        """根據node_id讀取location資料 - 委託給 DatabaseManager"""
        return self.db_manager.read_location_by_node_id(node_id)



    # 檢查NG回收區狀態 (保留，因為新系統中尚未實作)
    def ng_recycle_area_status(self):
        import config.config as CONFIG
        empty_count = 0
        place_count = 0
        empty_list = []
        place_list = []

        for n in self.db_manager.locations:
            if n.id in CONFIG.NG_RECYCLE_AREA and n.location_status_id == 2:  # 未佔用
                empty_count += 1
                empty_list.append(n.id)
            if n.id in CONFIG.NG_RECYCLE_AREA and n.location_status_id == 3:  # 佔用
                place_count += 1
                place_list.append(n.id)
        return empty_count, empty_list, place_count, place_list

    # 檢查系統準備區狀態
    def system_ready_area_status(self):
        """檢查系統準備區(11-18)的狀態"""
        import config.config as CONFIG
        empty_count = 0
        place_count = 0
        empty_list = []
        place_list = []

        for n in self.db_manager.locations:
            if n.id in CONFIG.SYSTEM_READY_AREA:
                if n.location_status_id == 2:  # 未佔用
                    empty_count += 1
                    empty_list.append(n.id)
                elif n.location_status_id == 3:  # 佔用
                    place_count += 1
                    place_list.append(n.id)
                elif n.location_status_id == 4:  # 任務佔用中
                    place_count += 1
                    place_list.append(n.id)
        return empty_count, empty_list, place_count, place_list
    



    