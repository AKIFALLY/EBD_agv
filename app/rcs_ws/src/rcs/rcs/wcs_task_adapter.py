"""
WCS 任務適配器
負責將 WCS 的智能決策任務轉換為 KUKA 可執行的任務格式
實現 WCS 四級優先度系統與 KUKA 任務的無縫對接
"""

from typing import Dict, List, Optional, Any, Tuple
from enum import Enum, IntEnum
from dataclasses import dataclass
from datetime import datetime, timezone


class WCSTaskType(Enum):
    """WCS 任務類型定義"""
    ROTATION = "rotation"              # 旋轉任務
    RACK_MOVE = "rack_move"           # Rack搬運
    EMPTY_DELIVERY = "empty_delivery"  # 空車派送
    FULL_COLLECTION = "full_collection"  # 滿車收集
    RECOVERY = "recovery"             # 空車回收


class WCSTaskPriority(IntEnum):
    """WCS 四級優先度定義"""
    ROTATION = 100      # 🔴 轉架高優先
    OUTLET = 80         # 🟡 房間出口中優先  
    INLET = 60          # 🟢 房間入口低優先
    MANUAL = 40         # 🔵 人員低優先


class KukaWorkType:
    """KUKA 工作類型常數 (對應 RCS KukaManager)"""
    KUKA_MOVE = 210001          # kuka-移動
    KUKA_RACK_MOVE = 220001     # kuka-移動貨架  
    KUKA_WORKFLOW = 230001      # kuka-workflow


@dataclass
class WCSTask:
    """WCS 任務資料結構"""
    task_id: str
    task_type: WCSTaskType
    priority: WCSTaskPriority
    rack_id: int
    source_location: int
    target_location: int
    nodes: List[int]
    room_id: Optional[int] = None
    parameters: Dict[str, Any] = None
    created_at: datetime = None
    
    def __post_init__(self):
        if self.created_at is None:
            self.created_at = datetime.now(timezone.utc)
        if self.parameters is None:
            self.parameters = {}


@dataclass
class KukaTask:
    """轉換後的 KUKA 任務格式"""
    work_id: int
    priority: int
    parameters: Dict[str, Any]
    mission_code: Optional[str] = None
    agv_id: Optional[int] = None
    room_id: Optional[int] = None
    
    def to_db_format(self) -> Dict[str, Any]:
        """轉換為資料庫 Task 表格式"""
        return {
            'work_id': self.work_id,
            'priority': self.priority,
            'parameters': self.parameters,
            'mission_code': self.mission_code,
            'agv_id': self.agv_id,
            'room_id': self.room_id,
            'status_id': 1,  # 待執行狀態
        }


class WCSTaskAdapter:
    """WCS 任務適配器"""
    
    def __init__(self, logger=None):
        self.logger = logger
        self._log_info('WCS 任務適配器初始化完成')
        
        # WCS 任務類型到 KUKA 工作類型的映射
        self.task_type_mapping = {
            WCSTaskType.ROTATION: KukaWorkType.KUKA_RACK_MOVE,      # 旋轉任務 -> KUKA貨架移動
            WCSTaskType.RACK_MOVE: KukaWorkType.KUKA_RACK_MOVE,     # 貨架搬運 -> KUKA貨架移動
            WCSTaskType.EMPTY_DELIVERY: KukaWorkType.KUKA_RACK_MOVE, # 空車派送 -> KUKA貨架移動
            WCSTaskType.FULL_COLLECTION: KukaWorkType.KUKA_RACK_MOVE, # 滿車收集 -> KUKA貨架移動
            WCSTaskType.RECOVERY: KukaWorkType.KUKA_RACK_MOVE,      # 空車回收 -> KUKA貨架移動
        }
        
        # 任務統計
        self.conversion_stats = {
            'total_converted': 0,
            'rotation_tasks': 0,
            'rack_move_tasks': 0,
            'empty_delivery_tasks': 0,
            'full_collection_tasks': 0,
            'recovery_tasks': 0,
            'conversion_errors': 0
        }
    
    def _log_info(self, message: str):
        """記錄資訊日誌"""
        if self.logger:
            self.logger.info(f"[WCSTaskAdapter] {message}")
    
    def _log_warning(self, message: str):
        """記錄警告日誌"""
        if self.logger:
            self.logger.warning(f"[WCSTaskAdapter] {message}")
    
    def _log_error(self, message: str):
        """記錄錯誤日誌"""
        if self.logger:
            self.logger.error(f"[WCSTaskAdapter] {message}")
    
    def convert_wcs_task_to_kuka(self, wcs_task: WCSTask) -> Optional[KukaTask]:
        """
        將 WCS 任務轉換為 KUKA 任務格式
        
        Args:
            wcs_task: WCS 任務物件
            
        Returns:
            KukaTask: 轉換後的 KUKA 任務，失敗時返回 None
        """
        try:
            # 先驗證 WCS 任務有效性
            is_valid, error_msg = self.validate_wcs_task(wcs_task)
            if not is_valid:
                self._log_error(f"WCS 任務驗證失敗: {error_msg}")
                self.conversion_stats['conversion_errors'] += 1
                return None
            
            # 獲取對應的 KUKA 工作類型
            kuka_work_id = self.task_type_mapping.get(wcs_task.task_type)
            if not kuka_work_id:
                self._log_error(f"不支援的 WCS 任務類型: {wcs_task.task_type}")
                self.conversion_stats['conversion_errors'] += 1
                return None
            
            # 構建 KUKA 任務參數
            kuka_parameters = self._build_kuka_parameters(wcs_task)
            if not kuka_parameters:
                self._log_error(f"構建 KUKA 任務參數失敗: {wcs_task.task_id}")
                self.conversion_stats['conversion_errors'] += 1
                return None
            
            # 創建 KUKA 任務
            kuka_task = KukaTask(
                work_id=kuka_work_id,
                priority=int(wcs_task.priority),
                parameters=kuka_parameters,
                room_id=wcs_task.room_id
            )
            
            # 更新統計
            self._update_conversion_stats(wcs_task.task_type)
            
            self._log_info(
                f"成功轉換 WCS 任務 {wcs_task.task_id} "
                f"({wcs_task.task_type.value}) -> KUKA 任務 "
                f"(work_id: {kuka_work_id}, priority: {wcs_task.priority})"
            )
            
            return kuka_task
            
        except Exception as e:
            self._log_error(f"轉換 WCS 任務失敗: {e}")
            self.conversion_stats['conversion_errors'] += 1
            return None
    
    def _build_kuka_parameters(self, wcs_task: WCSTask) -> Optional[Dict[str, Any]]:
        """
        構建 KUKA 任務參數
        
        Args:
            wcs_task: WCS 任務物件
            
        Returns:
            Dict: KUKA 任務參數，失敗時返回 None
        """
        try:
            # 基礎參數
            kuka_params = {
                'model': 'KUKA400i',
                'function': 'rack_move',
                'nodes': wcs_task.nodes,
                'rack_id': wcs_task.rack_id,
                'source_location': wcs_task.source_location,
                'target_location': wcs_task.target_location,
                'wcs_task_id': wcs_task.task_id,
                'wcs_task_type': wcs_task.task_type.value,
                'wcs_priority': int(wcs_task.priority)
            }
            
            # 根據任務類型添加特定參數
            if wcs_task.task_type == WCSTaskType.ROTATION:
                kuka_params.update(self._build_rotation_parameters(wcs_task))
            elif wcs_task.task_type == WCSTaskType.RACK_MOVE:
                kuka_params.update(self._build_rack_move_parameters(wcs_task))
            elif wcs_task.task_type == WCSTaskType.EMPTY_DELIVERY:
                kuka_params.update(self._build_empty_delivery_parameters(wcs_task))
            elif wcs_task.task_type == WCSTaskType.FULL_COLLECTION:
                kuka_params.update(self._build_full_collection_parameters(wcs_task))
            elif wcs_task.task_type == WCSTaskType.RECOVERY:
                kuka_params.update(self._build_recovery_parameters(wcs_task))
            
            # 添加 WCS 額外參數
            if wcs_task.parameters:
                kuka_params.update({
                    'wcs_extra_params': wcs_task.parameters
                })
            
            return kuka_params
            
        except Exception as e:
            self._log_error(f"構建 KUKA 參數失敗: {e}")
            return None
    
    def _build_rotation_parameters(self, wcs_task: WCSTask) -> Dict[str, Any]:
        """構建旋轉任務特定參數"""
        return {
            'task_subtype': 'rotation',
            'rotation_angle': 180,  # Rack A面轉B面
            'rotation_type': 'rack_flip',
            'description': f'Rack {wcs_task.rack_id} 旋轉任務 - A面轉B面'
        }
    
    def _build_rack_move_parameters(self, wcs_task: WCSTask) -> Dict[str, Any]:
        """構建貨架搬運任務特定參數"""
        return {
            'task_subtype': 'rack_transport',
            'transport_type': 'normal_move',
            'description': f'Rack {wcs_task.rack_id} 搬運任務 - 從位置 {wcs_task.source_location} 到 {wcs_task.target_location}'
        }
    
    def _build_empty_delivery_parameters(self, wcs_task: WCSTask) -> Dict[str, Any]:
        """構建空車派送任務特定參數"""
        return {
            'task_subtype': 'empty_delivery',
            'delivery_type': 'empty_rack',
            'description': f'空 Rack {wcs_task.rack_id} 派送任務 - 派送到位置 {wcs_task.target_location}'
        }
    
    def _build_full_collection_parameters(self, wcs_task: WCSTask) -> Dict[str, Any]:
        """構建滿車收集任務特定參數"""
        return {
            'task_subtype': 'full_collection',
            'collection_type': 'full_rack',
            'description': f'滿載 Rack {wcs_task.rack_id} 收集任務 - 從位置 {wcs_task.source_location} 收集'
        }
    
    def _build_recovery_parameters(self, wcs_task: WCSTask) -> Dict[str, Any]:
        """構建空車回收任務特定參數"""
        return {
            'task_subtype': 'recovery',
            'recovery_type': 'empty_rack_return',
            'description': f'空 Rack {wcs_task.rack_id} 回收任務 - 回收到停車區'
        }
    
    def _update_conversion_stats(self, task_type: WCSTaskType):
        """更新轉換統計"""
        self.conversion_stats['total_converted'] += 1
        
        if task_type == WCSTaskType.ROTATION:
            self.conversion_stats['rotation_tasks'] += 1
        elif task_type == WCSTaskType.RACK_MOVE:
            self.conversion_stats['rack_move_tasks'] += 1
        elif task_type == WCSTaskType.EMPTY_DELIVERY:
            self.conversion_stats['empty_delivery_tasks'] += 1
        elif task_type == WCSTaskType.FULL_COLLECTION:
            self.conversion_stats['full_collection_tasks'] += 1
        elif task_type == WCSTaskType.RECOVERY:
            self.conversion_stats['recovery_tasks'] += 1
    
    def batch_convert_wcs_tasks(self, wcs_tasks: List[WCSTask]) -> List[KukaTask]:
        """
        批量轉換 WCS 任務
        
        Args:
            wcs_tasks: WCS 任務列表
            
        Returns:
            List[KukaTask]: 成功轉換的 KUKA 任務列表
        """
        kuka_tasks = []
        
        for wcs_task in wcs_tasks:
            kuka_task = self.convert_wcs_task_to_kuka(wcs_task)
            if kuka_task:
                kuka_tasks.append(kuka_task)
        
        self._log_info(f"批量轉換完成：{len(wcs_tasks)} 個 WCS 任務 -> {len(kuka_tasks)} 個 KUKA 任務")
        
        return kuka_tasks
    
    def sort_tasks_by_priority(self, kuka_tasks: List[KukaTask]) -> List[KukaTask]:
        """
        依優先度排序 KUKA 任務
        
        Args:
            kuka_tasks: KUKA 任務列表
            
        Returns:
            List[KukaTask]: 按優先度排序的任務列表 (高優先度在前)
        """
        sorted_tasks = sorted(kuka_tasks, key=lambda task: task.priority, reverse=True)
        
        self._log_info(f"任務優先度排序完成：{len(sorted_tasks)} 個任務")
        
        # 記錄排序結果
        for i, task in enumerate(sorted_tasks[:5], 1):  # 只記錄前5個
            self._log_info(f"  #{i}: Priority {task.priority}, WorkID {task.work_id}")
        
        return sorted_tasks
    
    def get_conversion_statistics(self) -> Dict[str, Any]:
        """取得轉換統計資訊"""
        stats = self.conversion_stats.copy()
        stats['success_rate'] = (
            (stats['total_converted'] / (stats['total_converted'] + stats['conversion_errors']) * 100)
            if (stats['total_converted'] + stats['conversion_errors']) > 0 else 0
        )
        return stats
    
    def reset_statistics(self):
        """重置統計資訊"""
        self.conversion_stats = {
            'total_converted': 0,
            'rotation_tasks': 0,
            'rack_move_tasks': 0,
            'empty_delivery_tasks': 0,
            'full_collection_tasks': 0,
            'recovery_tasks': 0,
            'conversion_errors': 0
        }
        self._log_info("轉換統計資訊已重置")
    
    def validate_wcs_task(self, wcs_task: WCSTask) -> Tuple[bool, str]:
        """
        驗證 WCS 任務有效性
        
        Args:
            wcs_task: WCS 任務物件
            
        Returns:
            Tuple[bool, str]: (是否有效, 錯誤訊息)
        """
        try:
            # 檢查必要欄位
            if not wcs_task.task_id:
                return False, "任務 ID 不能為空"
            
            if not wcs_task.nodes or len(wcs_task.nodes) < 2:
                return False, "任務路徑節點不足 (至少需要2個節點)"
            
            if wcs_task.rack_id is None or wcs_task.rack_id <= 0:
                return False, "無效的 Rack ID"
            
            if wcs_task.source_location is None or wcs_task.target_location is None:
                return False, "來源或目標位置不能為空"
            
            # 檢查任務類型
            if wcs_task.task_type not in self.task_type_mapping:
                return False, f"不支援的任務類型: {wcs_task.task_type}"
            
            # 檢查優先度
            if wcs_task.priority not in [p.value for p in WCSTaskPriority]:
                return False, f"無效的優先度: {wcs_task.priority}"
            
            # 旋轉任務特殊檢查
            if wcs_task.task_type == WCSTaskType.ROTATION:
                if len(wcs_task.nodes) != 3:
                    return False, "旋轉任務需要3個節點 (起點->移出點->回到起點)"
                if wcs_task.nodes[0] != wcs_task.nodes[2]:
                    return False, "旋轉任務的起點和終點必須相同"
            
            return True, ""
            
        except Exception as e:
            return False, f"驗證過程發生錯誤: {str(e)}"