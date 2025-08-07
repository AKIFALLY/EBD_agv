"""
WCS Functions - WCS 業務邏輯函數
實現所有流程中使用的檢查、邏輯和動作函數
"""

import logging
import random
from typing import Dict, Any, List, Optional, Tuple
from datetime import datetime


class WCSFunctions:
    """WCS 業務邏輯函數集合"""
    
    def __init__(self, database_client=None, location_manager=None):
        self.logger = logging.getLogger('simple_wcs.functions')
        self.db = database_client
        self.location_manager = location_manager
        
        # 模擬資料（實際應從資料庫讀取）
        self.racks = {}
        self.tasks = {}
        self.rooms = {
            1: {'inlet': 10001, 'outlet': 10011, 'rotation_inlet': 10021, 'rotation_outlet': 10031},
            2: {'inlet': 10002, 'outlet': 10012, 'rotation_inlet': 10022, 'rotation_outlet': 10032},
            3: {'inlet': 10003, 'outlet': 10013, 'rotation_inlet': 10023, 'rotation_outlet': 10033},
            4: {'inlet': 10004, 'outlet': 10014, 'rotation_inlet': 10024, 'rotation_outlet': 10034},
            5: {'inlet': 10005, 'outlet': 10015, 'rotation_inlet': 10025, 'rotation_outlet': 10035},
        }
    
    # === 條件檢查函數 ===
    
    def check_rack_side_status(self, side: str, check_type: str, **kwargs) -> bool:
        """檢查料架側面狀態
        
        Args:
            side: 'A' 或 'B'
            check_type: 'completion' (完成) 或 'full_capacity' (滿載)
        """
        self.logger.debug(f"檢查料架 {side} 面狀態: {check_type}")
        
        # 模擬檢查邏輯
        if check_type == 'completion':
            # 檢查該面是否已完成處理
            result = random.random() > 0.6  # 40% 機率完成
        elif check_type == 'full_capacity':
            # 檢查該面是否已滿載
            result = random.random() > 0.7  # 30% 機率滿載
        else:
            result = False
        
        self.logger.info(f"料架 {side} 面 {check_type}: {'✓' if result else '✗'}")
        return result
    
    def check_rack_has_carrier(self, side: str, **kwargs) -> bool:
        """檢查料架側面是否有載具
        
        Args:
            side: 'A' 或 'B'
        """
        self.logger.debug(f"檢查料架 {side} 面是否有載具")
        
        # 模擬檢查邏輯
        result = random.random() > 0.5  # 50% 機率有載具
        
        self.logger.info(f"料架 {side} 面載具: {'有' if result else '無'}")
        return result
    
    def check_rack_has_space(self, side: str, **kwargs) -> bool:
        """檢查料架側面是否有空間
        
        Args:
            side: 'B' 側
        """
        self.logger.debug(f"檢查料架 {side} 面是否有空間")
        
        # 模擬檢查邏輯
        result = random.random() > 0.4  # 60% 機率有空間
        
        self.logger.info(f"料架 {side} 面空間: {'有' if result else '無'}")
        return result
    
    def check_task_not_exists(self, task_type: str, location_id: int, **kwargs) -> bool:
        """檢查指定位置是否沒有相同類型的任務
        
        Args:
            task_type: 任務類型（如 'rotation'）
            location_id: 位置ID
        """
        self.logger.debug(f"檢查位置 {location_id} 是否沒有 {task_type} 任務")
        
        # 檢查任務列表
        for task_id, task in self.tasks.items():
            if task.get('type') == task_type and task.get('location_id') == location_id:
                if task.get('status') in ['pending', 'executing']:
                    self.logger.info(f"位置 {location_id} 已有 {task_type} 任務")
                    return False
        
        self.logger.info(f"位置 {location_id} 沒有 {task_type} 任務")
        return True
    
    def check_agv_available(self, agv_id: str, **kwargs) -> bool:
        """檢查 AGV 是否可用
        
        Args:
            agv_id: AGV 識別碼
        """
        # 模擬檢查 - 70% 機率可用
        available = random.random() > 0.3
        self.logger.info(f"檢查 AGV {agv_id} 可用性 -> {'可用' if available else '不可用'}")
        return available
    
    def check_task_pending(self, task_type: str, **kwargs) -> bool:
        """檢查是否有待處理的任務
        
        Args:
            task_type: 任務類型
        """
        # 模擬檢查 - 60% 機率有任務
        has_task = random.random() > 0.4
        self.logger.info(f"檢查待處理任務 ({task_type}) -> {'有任務' if has_task else '無任務'}")
        return has_task
    
    def check_battery_level(self, agv_id: str, min_level: int = 30, **kwargs) -> bool:
        """檢查電池電量是否足夠
        
        Args:
            agv_id: AGV 識別碼
            min_level: 最低電量要求 (預設 30%)
        """
        # 模擬電池電量 (0-100)
        battery_level = random.randint(20, 100)
        sufficient = battery_level >= min_level
        self.logger.info(f"檢查 AGV {agv_id} 電池電量: {battery_level}% (最低需求: {min_level}%) -> {'足夠' if sufficient else '不足'}")
        return sufficient
    
    def check_all_rooms(self, position_type: str, **kwargs) -> List[int]:
        """檢查所有房間的指定位置
        
        Args:
            position_type: 'inlet' 或 'outlet'
        
        Returns:
            需要處理的房間ID列表
        """
        self.logger.debug(f"檢查所有房間的 {position_type} 位置")
        
        rooms_to_process = []
        for room_id, positions in self.rooms.items():
            location_id = positions.get(position_type)
            if location_id:
                # 檢查該位置是否需要處理
                if self._check_location_needs_processing(location_id, position_type):
                    rooms_to_process.append(room_id)
        
        self.logger.info(f"需要處理的房間: {rooms_to_process}")
        return rooms_to_process
    
    def _check_location_needs_processing(self, location_id: int, position_type: str) -> bool:
        """檢查位置是否需要處理"""
        # 模擬檢查邏輯
        return random.random() > 0.7  # 30% 機率需要處理
    
    # === 邏輯處理函數 ===
    
    def and_gate(self, input1: bool, input2: bool, **kwargs) -> bool:
        """AND 邏輯閘"""
        result = bool(input1 and input2)
        self.logger.debug(f"AND: {input1} & {input2} = {result}")
        return result
    
    def or_gate(self, input1: bool, input2: bool, **kwargs) -> bool:
        """OR 邏輯閘"""
        result = bool(input1 or input2)
        self.logger.debug(f"OR: {input1} | {input2} = {result}")
        return result
    
    def not_gate(self, input: bool, **kwargs) -> bool:
        """NOT 邏輯閘"""
        result = not bool(input)
        self.logger.debug(f"NOT: !{input} = {result}")
        return result
    
    def get_rotation_location(self, room_id: int, rotation_type: str, **kwargs) -> Optional[int]:
        """取得旋轉位置
        
        Args:
            room_id: 房間ID
            rotation_type: 'inlet' 或 'outlet'
        
        Returns:
            旋轉位置ID
        """
        room = self.rooms.get(room_id)
        if room:
            location_id = room.get(f'rotation_{rotation_type}')
            self.logger.debug(f"房間 {room_id} 的 {rotation_type} 旋轉位置: {location_id}")
            return location_id
        return None
    
    # === 動作執行函數 ===
    
    def create_rotation_task(self, location_id: int, **kwargs) -> Dict[str, Any]:
        """建立旋轉任務
        
        Args:
            location_id: 旋轉位置ID
        """
        task_id = f"TASK_{datetime.now().strftime('%Y%m%d%H%M%S')}_{random.randint(1000, 9999)}"
        
        task = {
            'task_id': task_id,
            'type': 'rotation',
            'location_id': location_id,
            'work_id': kwargs.get('work_id', '220001'),
            'priority': kwargs.get('priority', 100),
            'status': 'pending',
            'created_at': datetime.now().isoformat(),
            'parameters': {
                'rotation_angle': 180,
                'rotation_speed': 'normal'
            }
        }
        
        # 儲存任務
        self.tasks[task_id] = task
        
        self.logger.info(f"✅ 建立旋轉任務: {task_id} at 位置 {location_id}")
        return task
    
    def update_rack_status(self, location_id: int, status: str, **kwargs) -> bool:
        """更新料架狀態
        
        Args:
            location_id: 位置ID
            status: 新狀態
        """
        if location_id not in self.racks:
            self.racks[location_id] = {
                'location_id': location_id,
                'side_a_status': 'empty',
                'side_b_status': 'empty',
                'last_updated': None
            }
        
        rack = self.racks[location_id]
        rack['status'] = status
        rack['last_updated'] = datetime.now().isoformat()
        
        self.logger.info(f"更新料架狀態 - 位置: {location_id}, 狀態: {status}")
        return True
    
    def log_event(self, event_type: str, message: str, **kwargs) -> bool:
        """記錄事件
        
        Args:
            event_type: 事件類型
            message: 事件訊息
        """
        self.logger.info(f"[{event_type}] {message}")
        return True
    
    def assign_task_to_agv(self, agv_id: str, task_type: str, **kwargs) -> Dict[str, Any]:
        """分配任務給 AGV
        
        Args:
            agv_id: AGV 識別碼
            task_type: 任務類型
        """
        task_id = f"TASK_{datetime.now().strftime('%Y%m%d%H%M%S')}_{agv_id}"
        
        task = {
            'task_id': task_id,
            'type': task_type,
            'agv_id': agv_id,
            'status': 'assigned',
            'created_at': datetime.now().isoformat(),
            'parameters': kwargs
        }
        
        self.tasks[task_id] = task
        self.logger.info(f"✅ 分配任務 {task_id} 給 AGV {agv_id} (類型: {task_type})")
        return task
    
    def send_agv_to_charging(self, agv_id: str, station_id: str, **kwargs) -> bool:
        """送 AGV 去充電站
        
        Args:
            agv_id: AGV 識別碼
            station_id: 充電站識別碼
        """
        self.logger.info(f"🔋 送 AGV {agv_id} 到充電站 {station_id}")
        # 模擬發送充電指令
        return True
    
    def update_agv_status(self, agv_id: str, status: str, **kwargs) -> bool:
        """更新 AGV 狀態
        
        Args:
            agv_id: AGV 識別碼
            status: 新狀態 (idle, busy, charging, error)
        """
        self.logger.info(f"更新 AGV {agv_id} 狀態: {status}")
        # 實際應更新到資料庫
        return True
    
    # === 資料庫相關函數 ===
    
    def get_rack_info(self, location_id: int) -> Optional[Dict[str, Any]]:
        """取得料架資訊"""
        if self.db:
            # 實際從資料庫查詢
            pass
        else:
            # 返回模擬資料
            return self.racks.get(location_id)
    
    def get_pending_tasks(self, task_type: str = None) -> List[Dict[str, Any]]:
        """取得待處理任務"""
        pending_tasks = []
        for task_id, task in self.tasks.items():
            if task.get('status') == 'pending':
                if task_type is None or task.get('type') == task_type:
                    pending_tasks.append(task)
        return pending_tasks
    
    def update_task_status(self, task_id: str, status: str) -> bool:
        """更新任務狀態"""
        if task_id in self.tasks:
            self.tasks[task_id]['status'] = status
            self.tasks[task_id]['updated_at'] = datetime.now().isoformat()
            self.logger.info(f"更新任務 {task_id} 狀態: {status}")
            return True
        return False


# 函數註冊表
FUNCTION_REGISTRY = {
    # 條件檢查
    'check_rack_side_status': 'check_rack_side_status',
    'check_rack_has_carrier': 'check_rack_has_carrier', 
    'check_rack_has_space': 'check_rack_has_space',
    'check_task_not_exists': 'check_task_not_exists',
    'check_all_rooms': 'check_all_rooms',
    'check_agv_available': 'check_agv_available',
    'check_task_pending': 'check_task_pending',
    'check_battery_level': 'check_battery_level',
    
    # 邏輯處理
    'and_gate': 'and_gate',
    'or_gate': 'or_gate',
    'not_gate': 'not_gate',
    'get_rotation_location': 'get_rotation_location',
    
    # 動作執行
    'create_rotation_task': 'create_rotation_task',
    'update_rack_status': 'update_rack_status',
    'log_event': 'log_event',
    'assign_task_to_agv': 'assign_task_to_agv',
    'send_agv_to_charging': 'send_agv_to_charging',
    'update_agv_status': 'update_agv_status',
}


def create_wcs_functions(database_client=None, location_manager=None) -> WCSFunctions:
    """建立 WCS 函數實例"""
    return WCSFunctions(database_client, location_manager)


def register_functions_to_executor(executor, wcs_functions: WCSFunctions):
    """將 WCS 函數註冊到執行器"""
    for func_name, method_name in FUNCTION_REGISTRY.items():
        if hasattr(wcs_functions, method_name):
            method = getattr(wcs_functions, method_name)
            executor.register_function(func_name, method)
            
    logging.getLogger('simple_wcs.functions').info(
        f"註冊了 {len(FUNCTION_REGISTRY)} 個 WCS 函數到執行器"
    )