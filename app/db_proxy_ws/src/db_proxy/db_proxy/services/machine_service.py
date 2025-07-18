# db_proxy/services/machine_service.py
"""
機器相關的通用業務邏輯服務
"""

from typing import Optional, List, Dict
from sqlmodel import Session
from db_proxy.models import Machine
from db_proxy.crud.base_crud import BaseCRUD


class MachineService:
    """機器相關的通用業務邏輯"""
    
    def __init__(self):
        self.crud = BaseCRUD(Machine, id_column="id")
    
    def is_parking_available(self, machine_id: int, space_num: int, session: Session) -> bool:
        """
        檢查停車格是否可用
        
        Args:
            machine_id: 機器ID
            space_num: 停車格編號 (1 或 2)
            session: 資料庫 session
            
        Returns:
            True 如果停車格可用，False 如果不可用或機器不存在
        """
        machine = self.crud.get_by_id(session, machine_id)
        if not machine:
            return False
        
        if space_num == 1:
            status = machine.parking_space_1_status or 0
        elif space_num == 2:
            status = machine.parking_space_2_status or 0
        else:
            return False
        
        return status == Machine.PARKING_AVAILABLE
    
    def is_parking_task_active(self, machine_id: int, space_num: int, session: Session) -> bool:
        """
        檢查停車格是否有進行中的任務
        
        Args:
            machine_id: 機器ID
            space_num: 停車格編號 (1 或 2)
            session: 資料庫 session
            
        Returns:
            True 如果有進行中的任務，False 如果沒有或機器不存在
        """
        machine = self.crud.get_by_id(session, machine_id)
        if not machine:
            return False
        
        if space_num == 1:
            status = machine.parking_space_1_status or 0
        elif space_num == 2:
            status = machine.parking_space_2_status or 0
        else:
            return False
        
        return status == Machine.PARKING_TASK_ACTIVE
    
    def is_parking_task_completed(self, machine_id: int, space_num: int, session: Session) -> bool:
        """
        檢查停車格任務是否已完成
        
        Args:
            machine_id: 機器ID
            space_num: 停車格編號 (1 或 2)
            session: 資料庫 session
            
        Returns:
            True 如果任務已完成，False 如果未完成或機器不存在
        """
        machine = self.crud.get_by_id(session, machine_id)
        if not machine:
            return False
        
        if space_num == 1:
            status = machine.parking_space_1_status or 0
        elif space_num == 2:
            status = machine.parking_space_2_status or 0
        else:
            return False
        
        return status == Machine.PARKING_TASK_COMPLETED
    
    def get_parking_status_info(self, status_id: int) -> Dict[str, str]:
        """
        根據狀態 ID 獲取停車格狀態資訊
        
        Args:
            status_id: 狀態ID
            
        Returns:
            狀態資訊字典
        """
        return Machine.get_parking_status_info(status_id)
    
    def get_parking_status_name(self, status_id: int) -> str:
        """
        根據狀態 ID 獲取停車格狀態名稱
        
        Args:
            status_id: 狀態ID
            
        Returns:
            狀態名稱
        """
        return Machine.get_parking_status_name(status_id)
    
    def update_parking_status(self, machine_id: int, space_num: int, status: int, session: Session) -> Optional[Machine]:
        """
        更新停車格狀態
        
        Args:
            machine_id: 機器ID
            space_num: 停車格編號 (1 或 2)
            status: 新的狀態
            session: 資料庫 session
            
        Returns:
            更新後的機器物件，如果機器不存在則回傳 None
        """
        machine = self.crud.get_by_id(session, machine_id)
        if not machine:
            return None
        
        if space_num == 1:
            machine.parking_space_1_status = status
        elif space_num == 2:
            machine.parking_space_2_status = status
        else:
            return None
        
        return self.crud.update(session, machine_id, machine)
    
    def get_available_parking_spaces(self, machine_id: int, session: Session) -> List[int]:
        """
        取得機器的可用停車格列表
        
        Args:
            machine_id: 機器ID
            session: 資料庫 session
            
        Returns:
            可用停車格編號列表
        """
        machine = self.crud.get_by_id(session, machine_id)
        if not machine:
            return []
        
        available_spaces = []
        if self.is_parking_available(machine_id, 1, session):
            available_spaces.append(1)
        if self.is_parking_available(machine_id, 2, session):
            available_spaces.append(2)
        
        return available_spaces
    
    def is_machine_enabled(self, machine_id: int, session: Session) -> bool:
        """
        檢查機器是否啟用
        
        Args:
            machine_id: 機器ID
            session: 資料庫 session
            
        Returns:
            True 如果機器啟用，False 如果機器停用或不存在
        """
        machine = self.crud.get_by_id(session, machine_id)
        return machine.enable == 1 if machine else False
    
    def get_all_machines(self, session: Session) -> List[Machine]:
        """
        取得所有機器列表
        
        Args:
            session: 資料庫 session
            
        Returns:
            所有機器列表
        """
        return self.crud.get_all(session)
