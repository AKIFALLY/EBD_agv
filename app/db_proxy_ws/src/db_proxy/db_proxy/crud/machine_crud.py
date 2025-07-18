from typing import Optional, List, Dict
from sqlmodel import Session, select
from db_proxy.models.machine import Machine
from db_proxy.crud.base_crud import BaseCRUD


class MachineCRUD(BaseCRUD):
    """Machine 專用的 CRUD 類別，提供額外的便利方法"""
    
    def get_parking_status_info(self, status_id: int) -> Dict[str, str]:
        """根據狀態 ID 獲取停車格狀態資訊"""
        return Machine.get_parking_status_info(status_id)
    
    def get_parking_status_name(self, status_id: int) -> str:
        """根據狀態 ID 獲取停車格狀態名稱"""
        return Machine.get_parking_status_name(status_id)
    
    def is_parking_available(self, status_id: int) -> bool:
        """檢查停車格是否可用"""
        return Machine.is_parking_available(status_id)
    
    def is_parking_task_active(self, status_id: int) -> bool:
        """檢查停車格是否有進行中的任務"""
        return Machine.is_parking_task_active(status_id)
    
    def is_parking_task_completed(self, status_id: int) -> bool:
        """檢查停車格任務是否已完成"""
        return Machine.is_parking_task_completed(status_id)
    
    def get_machine_by_name(self, session: Session, machine_name: str) -> Optional[Machine]:
        """根據機台名稱查詢機台"""
        stmt = select(Machine).where(Machine.name == machine_name)
        return session.exec(stmt).first()
    
    def get_enabled_machines(self, session: Session) -> List[Machine]:
        """取得所有啟用的機台"""
        stmt = select(Machine).where(Machine.enable == 1)
        return list(session.exec(stmt).all())
    
    def get_machines_with_available_parking(self, session: Session) -> List[Machine]:
        """取得有可用停車格的機台"""
        machines = self.get_enabled_machines(session)
        available_machines = []
        for machine in machines:
            if machine.get_available_parking_spaces():
                available_machines.append(machine)
        return available_machines
    
    def update_parking_space_status(self, session: Session, machine_id: int, 
                                   space_number: int, status: int) -> bool:
        """
        更新停車格狀態
        
        Args:
            session: 資料庫 session
            machine_id: 機台ID
            space_number: 停車格編號 (1 或 2)
            status: 新的狀態值
            
        Returns:
            True 如果更新成功，False 如果失敗
        """
        try:
            machine = self.get_by_id(session, machine_id)
            if not machine:
                return False
            
            if space_number == 1:
                machine.parking_space_1_status = status
            elif space_number == 2:
                machine.parking_space_2_status = status
            else:
                return False
            
            session.add(machine)
            session.commit()
            return True
        except Exception:
            session.rollback()
            return False
    
    def get_machine_parking_info(self, session: Session, machine_id: int) -> Optional[Dict]:
        """
        取得機台的停車格資訊
        
        Args:
            session: 資料庫 session
            machine_id: 機台ID
            
        Returns:
            包含停車格資訊的字典
        """
        machine = self.get_by_id(session, machine_id)
        if not machine:
            return None
        
        return {
            "machine_id": machine.id,
            "machine_name": machine.name,
            "parking_space_1": {
                "node_id": machine.parking_space_1,
                "status": machine.parking_space_1_status,
                "status_name": machine.get_parking_space_1_status_name(),
                "is_available": machine.is_parking_space_1_available()
            },
            "parking_space_2": {
                "node_id": machine.parking_space_2,
                "status": machine.parking_space_2_status,
                "status_name": machine.get_parking_space_2_status_name(),
                "is_available": machine.is_parking_space_2_available()
            },
            "available_spaces": machine.get_available_parking_spaces()
        }
    
    def is_machine_enabled(self, session: Session, machine_id: int) -> bool:
        """檢查機台是否啟用"""
        machine = self.get_by_id(session, machine_id)
        return machine.enable == 1 if machine else False


# 創建 Machine CRUD 實例
machine_crud = MachineCRUD(Machine, id_column="id")
