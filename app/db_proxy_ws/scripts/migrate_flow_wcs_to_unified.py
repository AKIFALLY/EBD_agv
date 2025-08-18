#!/usr/bin/env python3
"""
資料遷移腳本：將 flow_wcs 資料庫遷移到統一的 db_proxy 模型
Migration script: Migrate flow_wcs database to unified db_proxy models
"""

import sys
import os
from datetime import datetime
from typing import Optional, Dict, Any

# 添加路徑以導入模組
sys.path.append('/app/db_proxy_ws/src/db_proxy')
sys.path.append('/app/flow_wcs_ws/src/flow_wcs')

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, Session

# 導入 db_proxy 統一模型
from db_proxy.models.agvc_location import Location as UnifiedLocation, LocationStatus
from db_proxy.models.rack import Rack as UnifiedRack
from db_proxy.models.agvc_task import Task as UnifiedTask, Work as UnifiedWork, FlowLog as UnifiedFlowLog
from db_proxy.models.agvc_rcs import AGV as UnifiedAGV
from db_proxy.models.carrier import Carrier as UnifiedCarrier

# 導入 flow_wcs 原始模型
from flow_wcs.database import (
    Location as FlowLocation,
    Rack as FlowRack,
    Task as FlowTask,
    AGV as FlowAGV,
    FlowLog as FlowFlowLog,
    Base as FlowBase
)

# 資料庫連線設定
DB_URL = os.getenv(
    'DATABASE_URL',
    'postgresql://agvc:password@192.168.100.254:5432/agvc'
)

class DataMigration:
    """資料遷移管理器"""
    
    def __init__(self, db_url: str = DB_URL):
        """初始化遷移管理器"""
        self.engine = create_engine(db_url)
        self.SessionLocal = sessionmaker(bind=self.engine)
        self.stats = {
            'locations': {'migrated': 0, 'failed': 0},
            'racks': {'migrated': 0, 'failed': 0},
            'tasks': {'migrated': 0, 'failed': 0},
            'agvs': {'migrated': 0, 'failed': 0},
            'flow_logs': {'migrated': 0, 'failed': 0}
        }
    
    def migrate_locations(self, session: Session) -> None:
        """遷移 Location 資料"""
        print("開始遷移 Location 資料...")
        
        # 查詢 flow_wcs locations
        flow_locations = session.query(FlowLocation).all()
        
        for flow_loc in flow_locations:
            try:
                # 檢查是否已存在
                existing = session.query(UnifiedLocation).filter(
                    UnifiedLocation.id == flow_loc.id
                ).first()
                
                if existing:
                    # 更新現有記錄
                    existing.type = flow_loc.type
                    existing.x = flow_loc.x
                    existing.y = flow_loc.y
                    existing.rack_id = int(flow_loc.rack_id) if flow_loc.rack_id and flow_loc.rack_id.isdigit() else None
                    
                    # 轉換 status 字串到 location_status_id
                    if flow_loc.status:
                        status_id = self.get_location_status_id(session, flow_loc.status)
                        existing.location_status_id = status_id
                else:
                    # 創建新記錄
                    unified_loc = UnifiedLocation(
                        id=flow_loc.id,
                        name=flow_loc.name,
                        type=flow_loc.type,
                        room_id=flow_loc.room_id,
                        x=flow_loc.x,
                        y=flow_loc.y,
                        rack_id=int(flow_loc.rack_id) if flow_loc.rack_id and flow_loc.rack_id.isdigit() else None,
                        location_status_id=self.get_location_status_id(session, flow_loc.status) if flow_loc.status else None,
                        created_at=flow_loc.created_at,
                        updated_at=flow_loc.updated_at
                    )
                    session.add(unified_loc)
                
                self.stats['locations']['migrated'] += 1
            except Exception as e:
                print(f"遷移 Location {flow_loc.id} 失敗: {e}")
                self.stats['locations']['failed'] += 1
        
        session.commit()
        print(f"Location 遷移完成: {self.stats['locations']}")
    
    def migrate_racks(self, session: Session) -> None:
        """遷移 Rack 資料"""
        print("開始遷移 Rack 資料...")
        
        flow_racks = session.query(FlowRack).all()
        
        for flow_rack in flow_racks:
            try:
                # rack_id 是字串，但實際上是整數 ID
                rack_id = int(flow_rack.rack_id) if flow_rack.rack_id and flow_rack.rack_id.isdigit() else flow_rack.id
                
                # 檢查是否已存在
                existing = session.query(UnifiedRack).filter(
                    UnifiedRack.id == rack_id
                ).first()
                
                if existing:
                    # 更新現有記錄
                    existing.location_id = flow_rack.location_id
                    existing.side_a_status = flow_rack.side_a_status
                    existing.side_b_status = flow_rack.side_b_status
                    existing.rotation_angle = flow_rack.rotation_angle
                    existing.last_rotation = flow_rack.last_rotation
                else:
                    # 創建新記錄
                    unified_rack = UnifiedRack(
                        id=rack_id,
                        name=f"Rack_{rack_id}",
                        location_id=flow_rack.location_id,
                        side_a_status=flow_rack.side_a_status,
                        side_b_status=flow_rack.side_b_status,
                        rotation_angle=flow_rack.rotation_angle,
                        last_rotation=flow_rack.last_rotation,
                        created_at=flow_rack.created_at,
                        updated_at=flow_rack.updated_at
                    )
                    session.add(unified_rack)
                
                # 更新 Carrier 關聯（如果有側面載具 ID）
                if flow_rack.side_a_carrier_id:
                    self.update_carrier_side(session, flow_rack.side_a_carrier_id, rack_id, 'a')
                if flow_rack.side_b_carrier_id:
                    self.update_carrier_side(session, flow_rack.side_b_carrier_id, rack_id, 'b')
                
                self.stats['racks']['migrated'] += 1
            except Exception as e:
                print(f"遷移 Rack {flow_rack.id} 失敗: {e}")
                self.stats['racks']['failed'] += 1
        
        session.commit()
        print(f"Rack 遷移完成: {self.stats['racks']}")
    
    def migrate_tasks(self, session: Session) -> None:
        """遷移 Task 資料"""
        print("開始遷移 Task 資料...")
        
        flow_tasks = session.query(FlowTask).all()
        
        for flow_task in flow_tasks:
            try:
                # 檢查是否已存在 (使用 task_id 字串)
                existing = session.query(UnifiedTask).filter(
                    UnifiedTask.task_id == flow_task.task_id
                ).first()
                
                if existing:
                    # 更新現有記錄
                    existing.type = flow_task.type
                    existing.priority = flow_task.priority
                    existing.parameters = flow_task.task_metadata
                    existing.completed_at = flow_task.completed_at
                else:
                    # 創建新記錄
                    unified_task = UnifiedTask(
                        task_id=flow_task.task_id,
                        name=flow_task.type or "Unknown Task",
                        type=flow_task.type,
                        work_id=self.get_or_create_work_id(session, flow_task.work_id),
                        location_id=flow_task.location_id,
                        rack_id=int(flow_task.rack_id) if flow_task.rack_id and flow_task.rack_id.isdigit() else None,
                        agv_id=self.get_agv_id_from_string(session, flow_task.agv_id),
                        priority=flow_task.priority,
                        parameters=flow_task.task_metadata,
                        status_id=self.get_task_status_id(session, flow_task.status),
                        created_at=flow_task.created_at,
                        updated_at=flow_task.updated_at,
                        completed_at=flow_task.completed_at
                    )
                    session.add(unified_task)
                
                self.stats['tasks']['migrated'] += 1
            except Exception as e:
                print(f"遷移 Task {flow_task.id} 失敗: {e}")
                self.stats['tasks']['failed'] += 1
        
        session.commit()
        print(f"Task 遷移完成: {self.stats['tasks']}")
    
    def migrate_agvs(self, session: Session) -> None:
        """遷移 AGV 資料"""
        print("開始遷移 AGV 資料...")
        
        flow_agvs = session.query(FlowAGV).all()
        
        for flow_agv in flow_agvs:
            try:
                # 檢查是否已存在 (使用 agv_id 字串)
                existing = session.query(UnifiedAGV).filter(
                    UnifiedAGV.agv_id == flow_agv.agv_id
                ).first()
                
                if existing:
                    # 更新現有記錄
                    existing.x = flow_agv.x
                    existing.y = flow_agv.y
                    existing.heading = flow_agv.theta
                    existing.battery = flow_agv.battery_level
                    existing.current_location = flow_agv.current_location
                    existing.current_task_id = flow_agv.current_task_id
                else:
                    # 創建新記錄
                    model_map = {
                        'cargo': 'Cargo',
                        'loader': 'Loader',
                        'unloader': 'Unloader'
                    }
                    model = model_map.get(flow_agv.type, flow_agv.type.title())
                    
                    unified_agv = UnifiedAGV(
                        agv_id=flow_agv.agv_id,
                        name=flow_agv.agv_id,
                        model=model,
                        x=flow_agv.x or 0.0,
                        y=flow_agv.y or 0.0,
                        heading=flow_agv.theta or 0.0,
                        battery=flow_agv.battery_level,
                        current_location=flow_agv.current_location,
                        current_task_id=flow_agv.current_task_id,
                        status_id=self.get_agv_status_id(session, flow_agv.status),
                        created_at=flow_agv.created_at,
                        updated_at=flow_agv.updated_at
                    )
                    session.add(unified_agv)
                
                self.stats['agvs']['migrated'] += 1
            except Exception as e:
                print(f"遷移 AGV {flow_agv.id} 失敗: {e}")
                self.stats['agvs']['failed'] += 1
        
        session.commit()
        print(f"AGV 遷移完成: {self.stats['agvs']}")
    
    def migrate_flow_logs(self, session: Session) -> None:
        """遷移 FlowLog 資料"""
        print("開始遷移 FlowLog 資料...")
        
        flow_logs = session.query(FlowFlowLog).all()
        
        for flow_log in flow_logs:
            try:
                unified_log = UnifiedFlowLog(
                    flow_id=flow_log.flow_id,
                    flow_name=flow_log.flow_name,
                    work_id=self.get_or_create_work_id(session, flow_log.work_id),
                    section=flow_log.section,
                    step_id=flow_log.step_id,
                    function=flow_log.function,
                    params=flow_log.params,
                    result=flow_log.result,
                    status=flow_log.status,
                    error_message=flow_log.error_message,
                    duration=flow_log.duration,
                    created_at=flow_log.created_at
                )
                session.add(unified_log)
                
                self.stats['flow_logs']['migrated'] += 1
            except Exception as e:
                print(f"遷移 FlowLog {flow_log.id} 失敗: {e}")
                self.stats['flow_logs']['failed'] += 1
        
        session.commit()
        print(f"FlowLog 遷移完成: {self.stats['flow_logs']}")
    
    # 輔助方法
    def get_location_status_id(self, session: Session, status_name: str) -> Optional[int]:
        """根據狀態名稱取得 location_status_id"""
        status_map = {
            'unknown': LocationStatus.UNKNOWN,
            'unoccupied': LocationStatus.UNOCCUPIED,
            'occupied': LocationStatus.OCCUPIED
        }
        return status_map.get(status_name.lower() if status_name else None)
    
    def get_task_status_id(self, session: Session, status_name: str) -> Optional[int]:
        """根據狀態名稱取得 task_status_id"""
        from db_proxy.models.agvc_task import TaskStatus
        status_map = {
            'pending': TaskStatus.PENDING,
            'assigned': TaskStatus.READY_TO_EXECUTE,
            'executing': TaskStatus.EXECUTING,
            'completed': TaskStatus.COMPLETED,
            'failed': TaskStatus.ERROR
        }
        return status_map.get(status_name.lower() if status_name else None)
    
    def get_agv_status_id(self, session: Session, status_name: str) -> Optional[int]:
        """根據狀態名稱取得 agv_status_id"""
        status_map = {
            'idle': 1,
            'busy': 2,
            'charging': 3,
            'error': 4,
            'maintenance': 5
        }
        return status_map.get(status_name.lower() if status_name else None)
    
    def get_or_create_work_id(self, session: Session, work_id_str: str) -> Optional[int]:
        """根據字串 work_id 取得或創建整數 work_id"""
        if not work_id_str:
            return None
        
        # 先嘗試查找現有的 Work
        work = session.query(UnifiedWork).filter(
            UnifiedWork.work_code == work_id_str
        ).first()
        
        if not work:
            # 創建新的 Work
            work = UnifiedWork(
                work_code=work_id_str,
                name=f"Work {work_id_str}",
                work_type="flow_wcs_migrated"
            )
            session.add(work)
            session.flush()  # 取得 ID
        
        return work.id
    
    def get_agv_id_from_string(self, session: Session, agv_id_str: str) -> Optional[int]:
        """根據字串 agv_id 取得整數 agv.id"""
        if not agv_id_str:
            return None
        
        agv = session.query(UnifiedAGV).filter(
            UnifiedAGV.agv_id == agv_id_str
        ).first()
        
        return agv.id if agv else None
    
    def update_carrier_side(self, session: Session, carrier_id_str: str, rack_id: int, side: str) -> None:
        """更新 Carrier 的架台側面資訊"""
        try:
            carrier_id = int(carrier_id_str) if carrier_id_str.isdigit() else None
            if carrier_id:
                carrier = session.query(UnifiedCarrier).filter(
                    UnifiedCarrier.id == carrier_id
                ).first()
                
                if carrier:
                    carrier.rack_id = rack_id
                    carrier.rack_side = side
        except Exception as e:
            print(f"更新 Carrier {carrier_id_str} 側面資訊失敗: {e}")
    
    def run_migration(self) -> None:
        """執行完整遷移"""
        print("=" * 50)
        print("開始資料遷移...")
        print("=" * 50)
        
        with self.SessionLocal() as session:
            # 依序遷移各個表
            self.migrate_locations(session)
            self.migrate_racks(session)
            self.migrate_tasks(session)
            self.migrate_agvs(session)
            self.migrate_flow_logs(session)
        
        print("=" * 50)
        print("資料遷移完成！")
        print("遷移統計:")
        for table, stats in self.stats.items():
            print(f"  {table}: 成功 {stats['migrated']}, 失敗 {stats['failed']}")
        print("=" * 50)


def main():
    """主程式"""
    import argparse
    
    parser = argparse.ArgumentParser(description='遷移 flow_wcs 資料到統一的 db_proxy 模型')
    parser.add_argument('--db-url', default=DB_URL, help='資料庫連線 URL')
    parser.add_argument('--dry-run', action='store_true', help='測試執行（不實際寫入）')
    
    args = parser.parse_args()
    
    if args.dry_run:
        print("測試模式：不會實際寫入資料庫")
        # TODO: 實作測試模式邏輯
    else:
        migration = DataMigration(args.db_url)
        migration.run_migration()


if __name__ == "__main__":
    main()