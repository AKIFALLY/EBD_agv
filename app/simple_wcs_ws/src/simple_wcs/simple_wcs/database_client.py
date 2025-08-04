"""
Simple WCS Database Client
直接使用 SQLModel 和 PostgreSQL 的資料庫客戶端，基於 0-360度 rack.direction
"""

import logging
from typing import Optional, List, Dict, Any
import sys
import os
from sqlmodel import Session, select, text
from contextlib import contextmanager

# 添加 db_proxy 路徑以使用 SQLModel 和連接池
sys.path.append('/app/db_proxy_ws/src')

# 導入必要的資料庫組件
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import Rack, Carrier, CarrierStatus, Task, TaskStatus, Room, RackStatus, Work, Location, LocationStatus


class DatabaseClient:
    """直接使用 SQLModel 的資料庫客戶端"""
    
    def __init__(self):
        self.logger = logging.getLogger('simple_wcs.database_client')
        self._init_database()
    
    def _init_database(self):
        """初始化 SQLModel 資料庫連接"""
        try:
            # 使用與現有系統相同的 PostgreSQL 連接配置 - 連接到 agvc 資料庫
            db_url = "postgresql://agvc:password@postgres:5432/agvc"
            
            # 建立連接池管理器
            self.pool_manager = ConnectionPoolManager(db_url)
            
            self.logger.info("✅ SQLModel 資料庫連接初始化成功")
        except Exception as e:
            self.logger.error(f"資料庫連接初始化失敗: {e}")
            # 資料庫連接失敗 - 無法繼續運行
            raise Exception(f"Simple WCS 無法連接資料庫，系統無法啟動: {e}")
    
    @contextmanager
    def get_session(self):
        """獲取資料庫 session 的上下文管理器"""
        if not self.pool_manager:
            self.logger.error("資料庫連接不可用")
            yield None
            return
            
        session = self.pool_manager.get_session()
        try:
            yield session
        except Exception as e:
            session.rollback()
            self.logger.error(f"資料庫操作失敗: {e}")
            raise
        finally:
            session.close()
    
    def execute_raw_query(self, query: str, params: Optional[Dict] = None) -> List[Dict]:
        """執行原始 SQL 查詢 (僅用於複雜查詢或除錯)"""
        try:
            with self.get_session() as session:
                if session is None:
                    return []
                    
                # 修正 SQLModel/SQLAlchemy 語法
                if params:
                    result = session.exec(text(query), params)
                else:
                    result = session.exec(text(query))
                    
                # 將結果轉換為字典列表
                columns = result.keys()
                return [dict(zip(columns, row)) for row in result.fetchall()]
                
        except Exception as e:
            self.logger.error(f"執行 SQL 查詢失敗: {e}")
            return []
    
    # === 統計和分析方法 ===
    
    def get_rack_statistics(self) -> Dict[str, Any]:
        """獲取 Rack 統計資訊"""
        try:
            with self.get_session() as session:
                if session is None:
                    return {}
                    
                # 使用 SQLModel statement 獲取統計
                from sqlalchemy import func
                
                # 總數統計
                total_racks = session.exec(
                    select(func.count(Rack.id))
                ).first() or 0
                
                # 按方向統計
                direction_stats = session.exec(
                    select(Rack.direction, func.count(Rack.id).label('count'))
                    .group_by(Rack.direction)
                ).all()
                
                # 按狀態統計
                status_stats = session.exec(
                    select(RackStatus.name, func.count(Rack.id).label('count'))
                    .join(RackStatus, Rack.status_id == RackStatus.id, isouter=True)
                    .group_by(RackStatus.name)
                ).all()
                
                return {
                    'total_racks': total_racks,
                    'by_direction': {row.direction: row.count for row in direction_stats},
                    'by_status': {row.name or 'Unknown': row.count for row in status_stats}
                }
                
        except Exception as e:
            self.logger.error(f"獲取 Rack 統計失敗: {e}")
            return {}
    
    # === Rack 旋轉相關方法 (基於 rack.direction 0-360度) ===
    
    def rack_at_location_exists(self, location_id: int) -> bool:
        """檢查指定位置是否有 Rack"""
        try:
            with self.get_session() as session:
                if session is None:
                    return False
                    
                # 真實資料庫查詢：檢查指定位置是否有 Rack
                statement = select(Rack).where(
                    Rack.location_id == location_id,
                    Rack.status_id.is_not(None)  # status_id 不為 None 表示 Rack 存在且有效
                ).limit(1)
                
                result = session.exec(statement).first()
                return result is not None
                
        except Exception as e:
            self.logger.error(f"檢查位置 {location_id} Rack 失敗: {e}")
            return False
    
    def get_rack_at_location(self, location_id: int) -> Optional[Dict]:
        """獲取指定位置的 Rack 資訊"""
        try:
            with self.get_session() as session:
                if session is None:
                    return None
                    
                # 真實資料庫查詢：獲取指定位置的 Rack 資訊
                statement = select(Rack).where(
                    Rack.location_id == location_id,
                    Rack.status_id != None
                ).limit(1)
                
                rack = session.exec(statement).first()
                if rack:
                    return {
                        'id': rack.id,
                        'location_id': rack.location_id,
                        'direction': rack.direction,
                        'status_id': rack.status_id,
                        'room_id': rack.room_id
                    }
                return None
                
        except Exception as e:
            self.logger.error(f"獲取位置 {location_id} Rack 資訊失敗: {e}")
            return None
    
    def rack_facing_direction(self, rack_id: int) -> int:
        """獲取 Rack 當前朝向角度 (0-360度)"""
        try:
            with self.get_session() as session:
                if session is None:
                    return 0
                    
                statement = select(Rack.direction).where(Rack.id == rack_id)
                result = session.exec(statement).first()
                return result if result is not None else 0
                
        except Exception as e:
            self.logger.error(f"獲取 Rack {rack_id} 朝向失敗: {e}")
            return 0
    
    def rack_side_completed(self, rack_id: int, side: str) -> bool:
        """檢查 Rack 的 A面 或 B面 是否已完成
        
        業務邏輯：
        - A面完成：A面沒有任何待作業的 carrier (status_id != 2)
        - B面完成：B面沒有任何待作業的 carrier (status_id != 2)
        """
        try:
            with self.get_session() as session:
                if session is None:
                    return False
                
                # 使用 SQLModel statement 查詢特定面的待作業 Carrier
                if side.upper() == 'A':
                    # A面 carrier (rack_index 1-16) - 檢查是否有待作業的 carrier
                    statement = (
                        select(Carrier)
                        .where(
                            Carrier.rack_id == rack_id,
                            Carrier.rack_index >= 1,
                            Carrier.rack_index <= 16,
                            Carrier.status_id == 2  # 2 = 待作業
                        )
                    ).limit(1)  # 只需要知道是否存在
                elif side.upper() == 'B':
                    # B面 carrier (rack_index 17-32) - 檢查是否有待作業的 carrier
                    statement = (
                        select(Carrier)
                        .where(
                            Carrier.rack_id == rack_id,
                            Carrier.rack_index >= 17,
                            Carrier.rack_index <= 32,
                            Carrier.status_id == 2  # 2 = 待作業
                        )
                    ).limit(1)  # 只需要知道是否存在
                else:
                    return False
                
                waiting_carriers = session.exec(statement).all()
                
                # A面或B面完成 = 沒有任何待作業的 carrier
                return len(waiting_carriers) == 0
                
        except Exception as e:
            self.logger.error(f"檢查 Rack {rack_id} {side}面完成狀態失敗: {e}")
            return False
    
    def rack_has_b_side_work(self, rack_id: int) -> bool:
        """檢查 Rack 是否有 B面 的工作需要處理
        
        業務邏輯：
        - B面有工作：B面還有 carrier 存在於該 Rack 上 (rack_id 仍然是該 Rack ID)
        - 不需要檢查狀態，只要還在 Rack 上就表示有工作要處理
        """
        try:
            with self.get_session() as session:
                if session is None:
                    return False
                    
                # 使用 SQLModel statement 查詢 B面是否有 Carrier 存在
                statement = (
                    select(Carrier)
                    .where(
                        Carrier.rack_id == rack_id,
                        Carrier.rack_index >= 17,
                        Carrier.rack_index <= 32
                    )
                ).limit(1)  # 只需要知道是否存在
                
                result = session.exec(statement).first()
                return result is not None
                
        except Exception as e:
            self.logger.error(f"檢查 Rack {rack_id} B面工作失敗: {e}")
            return False
    
    def rack_needs_rotation_for_b_side(self, rack_id: int, location_type: str) -> bool:
        """判斷 Rack 是否需要旋轉以處理 B面"""
        try:
            # 首先確認 Rack 存在
            with self.get_session() as session:
                if session is None:
                    return False
                    
                rack = session.exec(select(Rack).where(Rack.id == rack_id)).first()
                if not rack:
                    self.logger.debug(f"Rack {rack_id} 不存在，無需旋轉")
                    return False
                    
                current_direction = rack.direction or 0
                
                if location_type == 'room_inlet':
                    # 入口：A面朝向0度，需要轉到180度處理B面
                    return current_direction == 0
                elif location_type == 'room_outlet':
                    # 出口：A面朝向180度，需要轉到0度處理B面  
                    return current_direction == 180
                
                return False
                
        except Exception as e:
            self.logger.error(f"判斷 Rack {rack_id} 旋轉需求失敗: {e}")
            return False
    
    def rack_has_a_side_work(self, rack_id: int) -> bool:
        """檢查 Rack A面是否有待處理工作 (還有 carrier 在該 Rack 上)"""
        try:
            with self.get_session() as session:
                if session is None:
                    return False
                    
                # A面 carrier (rack_index 1-16) - 檢查是否還有 carrier 存在
                a_side_carriers = session.exec(
                    select(Carrier)
                    .where(
                        Carrier.rack_id == rack_id,
                        Carrier.rack_index >= 1,
                        Carrier.rack_index <= 16
                    )
                ).all()
                
                has_a_work = len(a_side_carriers) > 0
                self.logger.debug(f"Rack {rack_id} A面工作檢查: 有 {len(a_side_carriers)} 個 carrier ({'有工作' if has_a_work else '無工作'})")
                
                return has_a_work
                
        except Exception as e:
            self.logger.error(f"檢查 Rack {rack_id} A面工作失敗: {e}")
            return False
    
    def rack_needs_rotation_for_a_side(self, rack_id: int, location_type: str) -> bool:
        """判斷 Rack 是否需要旋轉以處理 A面 (從 180度 → 0度)"""
        try:
            with self.get_session() as session:
                if session is None:
                    return False
                    
                rack = session.exec(select(Rack).where(Rack.id == rack_id)).first()
                if not rack:
                    self.logger.debug(f"Rack {rack_id} 不存在，無需旋轉")
                    return False
                    
                current_direction = rack.direction or 0
                
                if location_type == 'room_exit':
                    # 出口：B面完成後(180度)，需要轉回0度處理A面
                    needs_rotation = current_direction == 180
                    self.logger.debug(f"Rack {rack_id} 出口旋轉檢查: 當前朝向 {current_direction}°, {'需要' if needs_rotation else '不需要'}旋轉到A面")
                    return needs_rotation
                
                return False
                
        except Exception as e:
            self.logger.error(f"判斷 Rack {rack_id} A面旋轉需求失敗: {e}")
            return False
    
    def transfer_exit_has_full_rack(self, location_id: int) -> bool:
        """檢查傳送箱出口是否有滿料架
        
        Args:
            location_id: 傳送箱出口位置ID
            
        Returns:
            True: 該位置有滿料架
            False: 該位置沒有滿料架
        """
        try:
            with self.get_session() as session:
                if session is None:
                    return False
                    
                # 真實資料庫查詢：檢查指定位置是否有 Rack
                statement = select(Rack).where(
                    Rack.location_id == location_id,
                    Rack.status_id != None
                ).limit(1)
                
                rack = session.exec(statement).first()
                if rack:
                    # 檢查該 Rack 是否滿載
                    return self.rack_is_full(rack.id)
                
                return False
                
        except Exception as e:
            self.logger.error(f"檢查傳送箱出口 {location_id} 滿料架失敗: {e}")
            return False
    
    def rack_is_full(self, rack_id: int) -> bool:
        """檢查 Rack 是否滿載
        
        Args:
            rack_id: Rack ID
            
        Returns:
            True: Rack 已滿載
            False: Rack 未滿載
        """
        try:
            with self.get_session() as session:
                if session is None:
                    return False
                    
                # 統計該 Rack 上的 Carrier 數量
                from sqlalchemy import func
                
                total_carriers = session.exec(
                    select(func.count(Carrier.id))
                    .where(Carrier.rack_id == rack_id)
                ).first() or 0
                
                # Rack 容量配置 (A面16個 + B面16個 = 32個)
                # TODO: 未來可以從配置檔案或 Rack 相關資料表中獲取實際容量
                MAX_CAPACITY = 32
                
                is_full = total_carriers >= MAX_CAPACITY
                self.logger.debug(f"Rack {rack_id} 載量: {total_carriers}/{MAX_CAPACITY} ({'滿載' if is_full else '未滿'})")
                
                return is_full
                
        except Exception as e:
            self.logger.error(f"檢查 Rack {rack_id} 滿載狀態失敗: {e}")
            return False
    
    def find_available_manual_location(self) -> int:
        """找到第一個可用的人工收料區位置
        
        Returns:
            可用的位置ID，如果沒有可用位置則返回 0
        """
        try:
            with self.get_session() as session:
                if session is None:
                    return 0  # 資料庫不可用時返回 0
                    
                # 從資料庫查詢人工收料區位置 (基於名稱或描述包含"人工收料區"的位置)
                manual_locations = session.exec(
                    select(Location).where(
                        (Location.name.like('%人工收料區%')) |
                        (Location.description.like('%人工收料區%')) |
                        (Location.name.like('%manual%collection%'))
                    ).order_by(Location.id)
                ).all()
                
                if not manual_locations:
                    self.logger.warning("資料庫中沒有找到人工收料區位置")
                    return 0
                
                for location in manual_locations:
                    location_id = location.id
                    # 檢查該位置是否有 Rack 佔用
                    rack_exists = session.exec(
                        select(Rack)
                        .where(
                            Rack.location_id == location_id,
                            Rack.status_id != None
                        )
                    ).first()
                    
                    if not rack_exists:
                        # 再檢查是否有進行中的任務要送到這個位置
                        if self.no_active_task_to_specific_location(location_id):
                            self.logger.debug(f"找到可用的人工收料區位置: {location_id} (名稱: {location.name})")
                            return location_id
                        else:
                            self.logger.debug(f"位置 {location_id} 有進行中的任務")
                    else:
                        self.logger.debug(f"位置 {location_id} 已被 Rack 佔用")
                
                self.logger.warning("所有人工收料區位置都被佔用")
                return 0  # 沒有可用位置
                
        except Exception as e:
            self.logger.error(f"尋找可用人工收料區位置失敗: {e}")
            # 資料庫查詢失敗時，基於安全考量，返回 0 表示無可用位置
            return 0

    def no_active_task_to_specific_location(self, target_location: int) -> bool:
        """檢查指定具體位置是否沒有進行中的運輸任務
        
        Args:
            target_location: 目標位置ID (如 30001, 30002...)
            
        Returns:
            True: 該位置沒有衝突任務，可以建立新任務
            False: 該位置有衝突任務，不應建立新任務
        """
        try:
            with self.get_session() as session:
                if session is None:
                    return True
                    
                # 檢查任務名稱模式：*_to_location_30001, *_to_location_30002 等
                task_name_pattern = f"%_to_location_{target_location}"
                
                # 查詢進行中的任務
                statement = select(Task).where(
                    Task.name.like(task_name_pattern),
                    Task.status_id.in_([
                        TaskStatus.PENDING,
                        TaskStatus.READY_TO_EXECUTE, 
                        TaskStatus.EXECUTING
                    ])
                ).limit(1)
                
                active_task = session.exec(statement).first()
                result = active_task is None
                
                if not result and active_task:
                    self.logger.info(f"🚫 位置 {target_location} 有進行中的任務: {active_task.name} (狀態ID: {active_task.status_id})")
                else:
                    self.logger.debug(f"✅ 位置 {target_location} 無衝突任務")
                
                return result
                
        except Exception as e:
            self.logger.error(f"檢查位置 {target_location} 任務衝突失敗: {e}")
            # 資料庫查詢失敗時，基於安全考量，假設有衝突，拒絕任務執行
            return False

    def manual_collection_area_available(self) -> bool:
        """檢查人工收料區是否有可用空間
        
        Returns:
            True: 人工收料區有空間
            False: 人工收料區已滿或不可用
        """
        try:
            # 簡化為檢查是否能找到可用位置
            available_location = self.find_available_manual_location()
            available = available_location > 0
            
            self.logger.debug(f"人工收料區可用性: {'可用' if available else '已滿'} (下一個可用位置: {available_location if available else '無'})")
            return available
                
        except Exception as e:
            self.logger.error(f"檢查人工收料區可用性失敗: {e}")
            # 資料庫查詢失敗時，基於安全考量，假設不可用，拒絕任務執行
            return False
    
    def no_active_task(self, work_id: str, location: int) -> bool:
        """檢查指定工作和位置是否沒有進行中的任務
        
        Args:
            work_id: 工作ID，用於區分不同類型的任務
            location: 位置ID，用於檢查特定位置的任務衝突
            
        Returns:
            True: 沒有衝突任務，可以建立新任務
            False: 有衝突任務，不應建立新任務
        """
        try:
            with self.get_session() as session:
                if session is None:
                    return True
                    
                # 根據 work_id 確定任務類型和檢查策略
                if work_id == "220001":  # Rack 旋轉任務
                    # 檢查是否有相同位置的旋轉任務正在進行
                    # 基於位置的精確匹配，避免跨房間衝突
                    
                    # 從 location 資料表獲取真實的 room_id
                    from db_proxy.models.agvc_location import Location
                    location_info = session.exec(
                        select(Location).where(Location.id == location)
                    ).first()
                    room_id = location_info.room_id if location_info else 0
                    
                    # 精確匹配特定房間的旋轉任務
                    task_name_pattern = f"rack_rotation_%_room_{room_id}"
                    
                    statement = (
                        select(Task)
                        .where(
                            Task.name.like(task_name_pattern),
                            Task.status_id.in_([  # 檢查活躍狀態
                                TaskStatus.PENDING,
                                TaskStatus.READY_TO_EXECUTE, 
                                TaskStatus.EXECUTING
                            ])
                        )
                    ).limit(1)
                    
                    active_task = session.exec(statement).first()
                    result = active_task is None
                    
                    if not result and active_task:
                        self.logger.info(f"🚫 找到進行中的任務: {active_task.name} (狀態ID: {active_task.status_id}) - 房間{room_id}位置{location}")
                    
                    return result
                    
                else:
                    # 對於其他類型的任務，使用通用檢查邏輯
                    # 檢查相同 work_id 和位置的任務
                    statement = (
                        select(Task)
                        .where(
                            Task.work_id == work_id,
                            Task.status_id.in_([
                                TaskStatus.PENDING,
                                TaskStatus.READY_TO_EXECUTE, 
                                TaskStatus.EXECUTING
                            ])
                        )
                    ).limit(1)
                    
                    active_task = session.exec(statement).first()
                    result = active_task is None
                    
                    if not result and active_task:
                        self.logger.info(f"🚫 找到進行中的任務: WorkID={work_id}, TaskID={active_task.id}, 狀態={active_task.status_id}")
                    
                    return result
                
        except Exception as e:
            self.logger.error(f"檢查WorkID={work_id}位置={location}任務衝突失敗: {e}")
            # 資料庫查詢失敗時，基於安全考量，假設有衝突，拒絕任務執行
            return False
    
    def no_active_task_to_destination(self, destination_type: str, work_id: str = None) -> bool:
        """檢查指定目的地是否沒有進行中的運輸任務
        
        Args:
            destination_type: 目的地類型 ('manual_collection_area', 'storage_area', etc.)
            work_id: 工作ID，用於進一步過濾任務類型
            
        Returns:
            True: 目的地沒有衝突任務，可以建立新任務
            False: 目的地有衝突任務，不應建立新任務
        """
        try:
            with self.get_session() as session:
                if session is None:
                    return True
                    
                # 根據目的地類型構建任務名稱模式
                if destination_type == "manual_collection_area":
                    # 檢查所有到人工收料區的運輸任務
                    task_name_pattern = "%_to_manual_collection_area"
                elif destination_type == "storage_area":
                    # 檢查所有到儲存區的運輸任務
                    task_name_pattern = "%_to_storage_area"
                else:
                    # 通用目的地檢查
                    task_name_pattern = f"%_to_{destination_type}"
                
                # 查詢進行中的任務
                query_conditions = [
                    Task.name.like(task_name_pattern),
                    Task.status_id.in_([
                        TaskStatus.PENDING,
                        TaskStatus.READY_TO_EXECUTE, 
                        TaskStatus.EXECUTING
                    ])
                ]
                
                # 如果指定了 work_id，進一步過濾
                if work_id:
                    # 查詢真實的 work.id 來過濾任務
                    work_record = session.exec(
                        select(Work).where(Work.name == work_id)
                    ).first()
                    if work_record:
                        query_conditions.append(Task.work_id == work_record.id)
                
                statement = select(Task).where(*query_conditions).limit(1)
                active_task = session.exec(statement).first()
                result = active_task is None
                
                if not result and active_task:
                    self.logger.info(f"🚫 目的地 '{destination_type}' 有進行中的任務: {active_task.name} (狀態ID: {active_task.status_id})")
                else:
                    self.logger.debug(f"✅ 目的地 '{destination_type}' 無衝突任務")
                
                return result
                
        except Exception as e:
            self.logger.error(f"檢查目的地 '{destination_type}' 任務衝突失敗: {e}")
            # 資料庫查詢失敗時，基於安全考量，假設有衝突，拒絕任務執行
            return False

    def no_active_task_from_source(self, source_type: str, source_location: int = None, work_id: str = None) -> bool:
        """檢查指定來源地是否沒有進行中的運輸任務
        
        Args:
            source_type: 來源地類型 ('transfer_exit', 'room_inlet', etc.)
            source_location: 具體來源位置ID
            work_id: 工作ID，用於進一步過濾任務類型
            
        Returns:
            True: 來源地沒有衝突任務，可以建立新任務
            False: 來源地有衝突任務，不應建立新任務
        """
        try:
            with self.get_session() as session:
                if session is None:
                    return True
                    
                # 根據來源地類型構建任務名稱模式
                if source_type == "transfer_exit" and source_location:
                    # 檢查特定傳送箱出口的運輸任務
                    task_name_pattern = f"transport_from_transfer_exit_{source_location}_%"
                elif source_type == "room_inlet" and source_location:
                    # 檢查特定房間入口的運輸任務
                    task_name_pattern = f"transport_from_room_inlet_{source_location}_%"
                else:
                    # 通用來源地檢查
                    if source_location:
                        task_name_pattern = f"transport_from_{source_type}_{source_location}_%"
                    else:
                        task_name_pattern = f"transport_from_{source_type}_%"
                
                # 查詢進行中的任務
                query_conditions = [
                    Task.name.like(task_name_pattern),
                    Task.status_id.in_([
                        TaskStatus.PENDING,
                        TaskStatus.READY_TO_EXECUTE, 
                        TaskStatus.EXECUTING
                    ])
                ]
                
                statement = select(Task).where(*query_conditions).limit(1)
                active_task = session.exec(statement).first()
                result = active_task is None
                
                if not result and active_task:
                    self.logger.info(f"🚫 來源地 '{source_type}' (位置: {source_location}) 有進行中的任務: {active_task.name} (狀態ID: {active_task.status_id})")
                else:
                    self.logger.debug(f"✅ 來源地 '{source_type}' (位置: {source_location}) 無衝突任務")
                
                return result
                
        except Exception as e:
            self.logger.error(f"檢查來源地 '{source_type}' (位置: {source_location}) 任務衝突失敗: {e}")
            # 資料庫查詢失敗時，基於安全考量，假設有衝突，拒絕任務執行
            return False

    def no_active_task_by_type(self, task_type: str, **criteria) -> bool:
        """通用的任務衝突檢查方法
        
        Args:
            task_type: 任務類型 ('rack_rotation', 'transport', 'maintenance', etc.)
            **criteria: 檢查條件，如 room_id, location_id, agv_id 等
            
        Returns:
            True: 沒有衝突任務，可以建立新任務
            False: 有衝突任務，不應建立新任務
        """
        try:
            with self.get_session() as session:
                if session is None:
                    return True
                
                # 根據任務類型構建不同的檢查邏輯
                if task_type == "rack_rotation":
                    # Rack 旋轉任務：檢查特定房間是否有旋轉任務
                    room_id = criteria.get('room_id')
                    if not room_id:
                        return True
                        
                    task_name_pattern = f"rack_rotation_%_room_{room_id}"
                    
                elif task_type == "agv_transport":
                    # AGV 運輸任務：檢查特定 AGV 是否有運輸任務
                    agv_id = criteria.get('agv_id')
                    if not agv_id:
                        return True
                        
                    task_name_pattern = f"transport_agv_{agv_id}_%"
                    
                elif task_type == "maintenance":
                    # 維護任務：檢查特定設備是否有維護任務
                    equipment_id = criteria.get('equipment_id')
                    if not equipment_id:
                        return True
                        
                    task_name_pattern = f"maintenance_{equipment_id}_%"
                    
                elif task_type == "cleaning":
                    # 清潔任務：檢查特定區域是否有清潔任務
                    area_id = criteria.get('area_id')
                    if not area_id:
                        return True
                        
                    task_name_pattern = f"cleaning_area_{area_id}_%"
                    
                else:
                    # 未知任務類型，使用通用檢查
                    work_id = criteria.get('work_id')
                    if not work_id:
                        return True
                        
                    statement = (
                        select(Task)
                        .where(
                            Task.work_id == work_id,
                            Task.status_id.in_([
                                TaskStatus.PENDING,
                                TaskStatus.READY_TO_EXECUTE, 
                                TaskStatus.EXECUTING
                            ])
                        )
                    ).limit(1)
                    
                    active_task = session.exec(statement).first()
                    result = active_task is None
                    
                    if not result:
                        self.logger.info(f"🚫 找到進行中的{task_type}任務: WorkID={work_id}")
                    
                    return result
                
                # 對於有 task_name_pattern 的情況
                statement = (
                    select(Task)
                    .where(
                        Task.name.like(task_name_pattern),
                        Task.status_id.in_([
                            TaskStatus.PENDING,
                            TaskStatus.READY_TO_EXECUTE, 
                            TaskStatus.EXECUTING
                        ])
                    )
                ).limit(1)
                
                active_task = session.exec(statement).first()
                result = active_task is None
                
                if not result:
                    self.logger.info(f"🚫 找到進行中的{task_type}任務: {active_task.name} (狀態ID: {active_task.status_id})")
                
                return result
                
        except Exception as e:
            self.logger.error(f"檢查{task_type}任務衝突失敗: {e}")
            # 資料庫查詢失敗時，基於安全考量，假設有衝突，拒絕任務執行
            return False
    
    def create_rack_rotation_task(self, rack_id: int, room_id: int, 
                                 location_type: str, nodes: List[int]) -> Dict:
        """建立 Rack 旋轉任務"""
        try:
            with self.get_session() as session:
                if session is None:
                    return {'status': 'error', 'message': 'Database connection not available'}
                    
                target_direction = 180 if location_type == 'room_inlet' else 0
                
                # 建立任務參數
                parameters = {
                    'function': 'rack_move',
                    'model': 'KUKA400i',
                    'api': 'submit_mission',
                    'missionType': 'RACK_MOVE',
                    'nodes': nodes,
                    'rack_id': rack_id,
                    'rotation_type': location_type,
                    'target_direction': target_direction,
                    'task_category': f'rack_rotation_{location_type}'
                }
                
                # 查詢真實的 work_id (WorkID: 220001)
                work_record = session.exec(
                    select(Work).where(Work.name == "220001")
                ).first()
                actual_work_id = work_record.id if work_record else None
                
                # 查詢真實的 room_id
                room_record = session.exec(
                    select(Room).where(Room.id == room_id)
                ).first()
                actual_room_id = room_record.id if room_record else None
                
                # 使用 SQLModel 建立任務實例
                new_task = Task(
                    work_id=actual_work_id,
                    name=f"rack_rotation_{location_type}_room_{room_id}",
                    description=f"Rack旋轉：房間{room_id}{location_type} A面→B面 (WorkID: 220001)",
                    priority=100,
                    room_id=actual_room_id,
                    node_id=None,  # 可能不需要指定 node，待確認資料結構
                    status_id=TaskStatus.PENDING,  # 待處理狀態
                    parameters=parameters
                )
                
                session.add(new_task)
                session.commit()
                session.refresh(new_task)
                
                self.logger.info(f"✅ 成功建立旋轉任務: ID={new_task.id}")
                return {'id': new_task.id, 'status': 'created'}
                
        except Exception as e:
            self.logger.error(f"建立 Rack 旋轉任務失敗: {e}")
            return {'status': 'error', 'message': str(e)}
    
    def create_rack_transport_task(self, rack_id: int, source_location: int, 
                                 destination_type: str, target_location: int = None, 
                                 nodes: List[int] = None) -> Dict:
        """建立 Rack 運輸任務
        
        Args:
            rack_id: Rack ID
            source_location: 來源位置
            destination_type: 目的地類型 ('manual_collection_area', 'storage_area', etc.)
            target_location: 具體目標位置 (如 30001)，如果為 None 則自動尋找
            nodes: 路徑節點列表，如果為 None 則自動生成
            
        Returns:
            任務建立結果
        """
        try:
            with self.get_session() as session:
                if session is None:
                    return {'status': 'error', 'message': 'Database connection not available'}
                
                # 如果沒有指定目標位置，自動尋找可用位置
                if target_location is None:
                    if destination_type == "manual_collection_area":
                        target_location = self.find_available_manual_location()
                        if target_location == 0:
                            return {'status': 'error', 'message': 'No available manual collection location'}
                    else:
                        return {'status': 'error', 'message': 'Target location required for non-manual destinations'}
                
                # 如果沒有指定路徑節點，自動生成
                if nodes is None:
                    nodes = [source_location, target_location]
                    
                # 建立任務參數 (符合 KUKA API 格式)
                parameters = {
                    'function': 'rack_move',
                    'model': 'KUKA400i',
                    'api': 'submit_mission',
                    'missionType': 'RACK_MOVE',
                    'nodes': nodes,  # KUKA API 需要的路徑節點
                    'rack_id': rack_id,
                    'source_location': source_location,
                    'destination_type': destination_type,
                    'destination_location': target_location,  # 具體目的地位置
                    'task_category': f'rack_transport_to_{destination_type}'
                }
                
                # 使用新的任務名稱格式：包含具體位置
                if destination_type == "manual_collection_area":
                    task_name = f"transport_from_transfer_exit_{source_location}_to_location_{target_location}"
                    description = f"運輸滿料架：傳送箱出口{source_location} → 人工收料區位置{target_location} (WorkID: 220001)"
                else:
                    task_name = f"transport_from_{source_location}_to_location_{target_location}"
                    description = f"運輸 Rack：位置{source_location} → 位置{target_location} (WorkID: 220001)"
                
                # 查詢真實的 work_id (WorkID: 220001)
                work_record = session.exec(
                    select(Work).where(Work.name == "220001")
                ).first()
                actual_work_id = work_record.id if work_record else None
                
                # 查詢來源位置對應的 room_id
                source_location_record = session.exec(
                    select(Location).where(Location.id == source_location)
                ).first()
                actual_room_id = source_location_record.room_id if source_location_record else None
                
                # 使用 SQLModel 建立任務實例
                new_task = Task(
                    work_id=actual_work_id,
                    name=task_name,
                    description=description,
                    priority=80,  # 根據業務流程文檔，滿料架到人工收料區是 Priority 80
                    room_id=actual_room_id,  # 使用來源位置對應的房間
                    node_id=None,  # 可能不需要指定 node，待確認資料結構
                    status_id=TaskStatus.PENDING,  # 待處理狀態
                    parameters=parameters
                )
                
                session.add(new_task)
                session.commit()
                session.refresh(new_task)
                
                self.logger.info(f"✅ 成功建立運輸任務: ID={new_task.id}, 名稱={task_name}")
                self.logger.info(f"   📍 路徑: {source_location} → {target_location}, 節點: {nodes}")
                return {
                    'id': new_task.id, 
                    'status': 'created', 
                    'name': task_name,
                    'target_location': target_location,
                    'nodes': nodes
                }
                
        except Exception as e:
            self.logger.error(f"建立 Rack 運輸任務失敗: {e}")
            return {'status': 'error', 'message': str(e)}