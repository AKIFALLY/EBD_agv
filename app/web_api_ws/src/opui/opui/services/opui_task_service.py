# opui/services/opui_task_service.py
"""
OPUI 特定的任務業務邏輯服務
"""

from typing import Dict, Any, Tuple, Optional
from sqlmodel import Session
from db_proxy.services import MachineService, ProductService
from db_proxy.models import Task, Rack, Machine
from db_proxy.crud.base_crud import BaseCRUD


class OpuiTaskService:
    """OPUI 特定的任務業務邏輯"""
    
    def __init__(self, db_pool):
        self.db_pool = db_pool
        
        # 使用 db_proxy 的通用 Service
        self.machine_service = MachineService()
        self.product_service = ProductService()
        
        # 基本 CRUD 操作
        self.task_crud = BaseCRUD(Task, id_column="id")
        self.rack_crud = BaseCRUD(Rack, id_column="id")
    
    def create_call_empty_task(self, machine_id: int, space_num: int, session: Session) -> Tuple[bool, str, Optional[Task]]:
        """
        創建叫空車任務
        OPUI 特定的叫空車業務邏輯
        
        Args:
            machine_id: 機器ID
            space_num: 停車格編號 (1 或 2)
            session: 資料庫 session
            
        Returns:
            (是否成功, 訊息, 任務物件)
        """
        try:
            # 檢查機器是否存在且啟用
            if not self.machine_service.is_machine_enabled(machine_id, session):
                return False, f"機器 {machine_id} 不存在或未啟用", None
            
            # 檢查停車格是否可用
            if not self.machine_service.is_parking_available(machine_id, space_num, session):
                return False, f"停車格 {space_num} 不可用", None
            
            # 獲取停車格對應的節點ID
            machine = self.machine_service.crud.get_by_id(session, machine_id)
            if space_num == 1:
                node_id = machine.parking_space_1
            elif space_num == 2:
                node_id = machine.parking_space_2
            else:
                return False, "無效的停車格編號", None
            
            if not node_id:
                return False, f"停車格 {space_num} 沒有配置節點", None
            
            # 創建任務
            task_data = {
                "task_type": "call_empty",
                "machine_id": machine_id,
                "space_num": space_num,
                "node_id": node_id
            }
            
            task = Task(
                node_id=node_id,
                status_id=1,  # 待執行
                parameters=task_data,
                priority=5
            )
            
            created_task = self.task_crud.create(session, task)
            
            # 更新停車格狀態為任務進行中
            self.machine_service.update_parking_status(
                machine_id, space_num, Machine.PARKING_TASK_ACTIVE, session
            )
            
            return True, "叫空車任務創建成功", created_task
            
        except Exception as e:
            return False, f"創建叫空車任務時發生錯誤: {str(e)}", None
    
    def create_dispatch_full_task(self, rack_id: int, room_id: int, session: Session) -> Tuple[bool, str, Optional[Task]]:
        """
        創建派車任務
        OPUI 特定的派車業務邏輯
        
        Args:
            rack_id: 料架ID
            room_id: 房間ID
            session: 資料庫 session
            
        Returns:
            (是否成功, 訊息, 任務物件)
        """
        try:
            # 檢查料架是否存在
            rack = self.rack_crud.get_by_id(session, rack_id)
            if not rack:
                return False, f"料架 {rack_id} 不存在", None
            
            # 檢查料架是否有產品
            if not rack.product_id:
                return False, f"料架 {rack_id} 沒有產品", None
            
            # 創建任務
            task_data = {
                "task_type": "dispatch_full",
                "rack_id": rack_id,
                "room": room_id,
                "node_id": None  # 由 WCS 決定具體位置
            }
            
            task = Task(
                room_id=room_id,
                status_id=1,  # 待執行
                parameters=task_data,
                priority=3
            )
            
            created_task = self.task_crud.create(session, task)
            
            return True, "派車任務創建成功", created_task
            
        except Exception as e:
            return False, f"創建派車任務時發生錯誤: {str(e)}", None
    
    def cancel_task_by_parking(self, machine_id: int, space_num: int, session: Session) -> Tuple[bool, str]:
        """
        根據停車格取消任務
        OPUI 特定的任務取消邏輯
        
        Args:
            machine_id: 機器ID
            space_num: 停車格編號
            session: 資料庫 session
            
        Returns:
            (是否成功, 訊息)
        """
        try:
            # 獲取停車格對應的節點ID
            machine = self.machine_service.crud.get_by_id(session, machine_id)
            if not machine:
                return False, f"機器 {machine_id} 不存在"
            
            if space_num == 1:
                node_id = machine.parking_space_1
            elif space_num == 2:
                node_id = machine.parking_space_2
            else:
                return False, "無效的停車格編號"
            
            # 查找相關的待執行任務
            from sqlmodel import select
            stmt = select(Task).where(
                Task.node_id == node_id,
                Task.status_id == 1  # 待執行
            )
            tasks = session.exec(stmt).all()
            
            cancelled_count = 0
            for task in tasks:
                # 檢查是否為叫空車任務
                params = task.parameters or {}
                if (params.get("task_type") == "call_empty" and 
                    params.get("machine_id") == machine_id and 
                    params.get("space_num") == space_num):
                    
                    # 取消任務
                    task.status_id = 3  # 已取消
                    self.task_crud.update(session, task.id, task)
                    cancelled_count += 1
            
            # 重置停車格狀態
            self.machine_service.update_parking_status(
                machine_id, space_num, Machine.PARKING_AVAILABLE, session
            )
            
            if cancelled_count > 0:
                return True, f"成功取消 {cancelled_count} 個任務"
            else:
                return True, "沒有找到相關任務，已重置停車格狀態"
                
        except Exception as e:
            return False, f"取消任務時發生錯誤: {str(e)}"
    
    def confirm_delivery(self, machine_id: int, space_num: int, session: Session) -> Tuple[bool, str]:
        """
        確認送達
        OPUI 特定的送達確認邏輯
        
        Args:
            machine_id: 機器ID
            space_num: 停車格編號
            session: 資料庫 session
            
        Returns:
            (是否成功, 訊息)
        """
        try:
            # 檢查停車格狀態是否為任務完成
            if not self.machine_service.is_parking_task_completed(machine_id, space_num, session):
                return False, "停車格狀態不是任務完成，無法確認送達"
            
            # 重置停車格狀態為可用
            self.machine_service.update_parking_status(
                machine_id, space_num, Machine.PARKING_AVAILABLE, session
            )
            
            return True, "送達確認成功"
            
        except Exception as e:
            return False, f"確認送達時發生錯誤: {str(e)}"
    
    def get_machine_parking_status(self, machine_id: int, session: Session) -> Dict[str, Any]:
        """
        獲取機器停車格狀態資訊
        OPUI 特定的狀態查詢邏輯
        
        Args:
            machine_id: 機器ID
            session: 資料庫 session
            
        Returns:
            停車格狀態資訊字典
        """
        try:
            machine = self.machine_service.crud.get_by_id(session, machine_id)
            if not machine:
                return {"error": f"機器 {machine_id} 不存在"}
            
            space1_status = machine.parking_space_1_status or 0
            space2_status = machine.parking_space_2_status or 0
            
            return {
                "machine_id": machine_id,
                "machine_name": machine.name,
                "space_1": {
                    "status": space1_status,
                    "status_info": self.machine_service.get_parking_status_info(space1_status),
                    "is_available": space1_status == Machine.PARKING_AVAILABLE,
                    "is_task_active": space1_status == Machine.PARKING_TASK_ACTIVE,
                    "is_task_completed": space1_status == Machine.PARKING_TASK_COMPLETED
                },
                "space_2": {
                    "status": space2_status,
                    "status_info": self.machine_service.get_parking_status_info(space2_status),
                    "is_available": space2_status == Machine.PARKING_AVAILABLE,
                    "is_task_active": space2_status == Machine.PARKING_TASK_ACTIVE,
                    "is_task_completed": space2_status == Machine.PARKING_TASK_COMPLETED
                }
            }
            
        except Exception as e:
            return {"error": f"獲取停車格狀態時發生錯誤: {str(e)}"}
