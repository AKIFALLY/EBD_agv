"""
WCS 四級優先度任務調度器
實現基於 WCS 智能決策的四級優先度任務調度系統

優先度階層：
🔴 第1級：轉架高優先 (Priority: 100) - Rack旋轉任務
🟡 第2級：房間出口中優先 (Priority: 80) - 出口Rack調度  
🟢 第3級：房間入口低優先 (Priority: 60) - 入口Rack調度
🔵 第4級：人員低優先 (Priority: 40) - 手動請求與回收
"""

from typing import List, Dict, Any, Optional, Tuple
from enum import IntEnum
from dataclasses import dataclass
import json
from datetime import datetime, timezone


class WCSPriorityLevel(IntEnum):
    """WCS 四級優先度定義"""
    ROTATION = 100      # 🔴 轉架高優先 - 緊急旋轉需求
    OUTLET = 80         # 🟡 房間出口中優先 - 出口空間管理
    INLET = 60          # 🟢 房間入口低優先 - 入口任務調度
    MANUAL = 40         # 🔵 人員低優先 - 手動請求和維護


@dataclass
class TaskScheduleInfo:
    """任務調度資訊"""
    task_id: int
    original_priority: int
    calculated_priority: int
    priority_level: WCSPriorityLevel
    priority_boost: float = 0.0
    schedule_reason: str = ""
    estimated_duration: int = 0  # 預估執行時間 (秒)
    dependencies: List[int] = None  # 依賴的任務ID
    
    def __post_init__(self):
        if self.dependencies is None:
            self.dependencies = []


class WCSPriorityScheduler:
    """WCS 四級優先度任務調度器"""
    
    def __init__(self, logger=None):
        self.logger = logger
        self._log_info('WCS 四級優先度調度器初始化完成')
        
        # 優先度範圍定義
        self.priority_ranges = {
            WCSPriorityLevel.ROTATION: (95, 105),   # 95-105
            WCSPriorityLevel.OUTLET: (75, 85),      # 75-85
            WCSPriorityLevel.INLET: (55, 65),       # 55-65
            WCSPriorityLevel.MANUAL: (35, 45)       # 35-45
        }
        
        # 優先度加權因子
        self.priority_weights = {
            'time_urgency': 0.3,      # 時間緊急度
            'resource_availability': 0.2,  # 資源可用性
            'system_load': 0.2,       # 系統負載
            'task_importance': 0.3    # 任務重要性
        }
        
        # 調度統計
        self.schedule_stats = {
            'total_scheduled': 0,
            'rotation_tasks': 0,
            'outlet_tasks': 0,
            'inlet_tasks': 0,
            'manual_tasks': 0,
            'priority_adjustments': 0,
            'last_schedule_time': None
        }
    
    def _log_info(self, message: str):
        """記錄資訊日誌"""
        if self.logger:
            self.logger.info(f"[WCSPriorityScheduler] {message}")
    
    def _log_warning(self, message: str):
        """記錄警告日誌"""
        if self.logger:
            self.logger.warning(f"[WCSPriorityScheduler] {message}")
    
    def _log_error(self, message: str):
        """記錄錯誤日誌"""
        if self.logger:
            self.logger.error(f"[WCSPriorityScheduler] {message}")
    
    def classify_task_priority_level(self, task) -> WCSPriorityLevel:
        """
        分類任務的優先度等級
        
        Args:
            task: 任務物件
            
        Returns:
            WCSPriorityLevel: 任務優先度等級
        """
        try:
            params = task.parameters or {}
            task_priority = getattr(task, 'priority', 40)
            
            # 基於任務參數判斷優先度等級
            wcs_task_type = params.get('wcs_task_type', '').lower()
            task_subtype = params.get('task_subtype', '').lower()
            function = params.get('function', '').lower()
            
            # 🔴 第1級：轉架高優先
            if (wcs_task_type == 'rotation' or 
                task_subtype == 'rotation' or 
                'rotation' in str(params.get('description', '')).lower()):
                return WCSPriorityLevel.ROTATION
            
            # 基於數值優先度判斷
            if task_priority >= 95:
                return WCSPriorityLevel.ROTATION
            elif task_priority >= 75:
                return WCSPriorityLevel.OUTLET
            elif task_priority >= 55:
                return WCSPriorityLevel.INLET
            else:
                return WCSPriorityLevel.MANUAL
                
        except Exception as e:
            self._log_error(f"分類任務優先度等級時發生錯誤: {e}")
            return WCSPriorityLevel.MANUAL
    
    def calculate_dynamic_priority(self, task, system_context: Dict[str, Any] = None) -> TaskScheduleInfo:
        """
        計算動態優先度
        
        Args:
            task: 任務物件
            system_context: 系統上下文資訊
            
        Returns:
            TaskScheduleInfo: 任務調度資訊
        """
        try:
            if system_context is None:
                system_context = {}
            
            original_priority = getattr(task, 'priority', 40)
            priority_level = self.classify_task_priority_level(task)
            
            # 計算各種調整因子
            time_factor = self._calculate_time_urgency_factor(task, system_context)
            resource_factor = self._calculate_resource_availability_factor(task, system_context)
            load_factor = self._calculate_system_load_factor(system_context)
            importance_factor = self._calculate_task_importance_factor(task)
            
            # 加權計算優先度加成
            priority_boost = (
                time_factor * self.priority_weights['time_urgency'] +
                resource_factor * self.priority_weights['resource_availability'] +
                load_factor * self.priority_weights['system_load'] +
                importance_factor * self.priority_weights['task_importance']
            )
            
            # 計算最終優先度
            calculated_priority = original_priority + (priority_boost * 10)  # 最多調整10分
            
            # 確保優先度在合理範圍內
            min_priority, max_priority = self.priority_ranges[priority_level]
            calculated_priority = max(min_priority, min(max_priority, calculated_priority))
            
            # 生成調度理由
            schedule_reason = self._generate_schedule_reason(
                priority_level, time_factor, resource_factor, load_factor, importance_factor
            )
            
            # 預估執行時間
            estimated_duration = self._estimate_task_duration(task)
            
            schedule_info = TaskScheduleInfo(
                task_id=task.id,
                original_priority=original_priority,
                calculated_priority=int(calculated_priority),
                priority_level=priority_level,
                priority_boost=priority_boost,
                schedule_reason=schedule_reason,
                estimated_duration=estimated_duration
            )
            
            return schedule_info
            
        except Exception as e:
            self._log_error(f"計算動態優先度時發生錯誤: {e}")
            # 返回預設調度資訊
            return TaskScheduleInfo(
                task_id=getattr(task, 'id', 0),
                original_priority=getattr(task, 'priority', 40),
                calculated_priority=getattr(task, 'priority', 40),
                priority_level=WCSPriorityLevel.MANUAL,
                schedule_reason="計算失敗，使用原始優先度"
            )
    
    def _calculate_time_urgency_factor(self, task, system_context: Dict[str, Any]) -> float:
        """計算時間緊急度因子 (0.0-1.0)"""
        try:
            # 檢查任務創建時間
            created_at = getattr(task, 'created_at', None)
            if created_at:
                time_diff = (datetime.now(timezone.utc) - created_at).total_seconds()
                # 超過5分鐘的任務提高緊急度
                if time_diff > 300:  # 5分鐘
                    return min(1.0, time_diff / 1800)  # 最多30分鐘達到最高緊急度
            
            # 檢查是否為緊急任務類型
            params = task.parameters or {}
            if params.get('task_subtype') == 'rotation':
                return 1.0  # 旋轉任務最高緊急度
                
            return 0.5  # 預設中等緊急度
            
        except Exception as e:
            self._log_error(f"計算時間緊急度因子時發生錯誤: {e}")
            return 0.5
    
    def _calculate_resource_availability_factor(self, task, system_context: Dict[str, Any]) -> float:
        """計算資源可用性因子 (0.0-1.0)"""
        try:
            # 檢查可用 AGV 數量
            available_agvs = system_context.get('available_agvs', 1)
            total_agvs = system_context.get('total_agvs', 1)
            
            if total_agvs > 0:
                availability_ratio = available_agvs / total_agvs
                return availability_ratio
            
            return 0.5  # 預設中等可用性
            
        except Exception as e:
            self._log_error(f"計算資源可用性因子時發生錯誤: {e}")
            return 0.5
    
    def _calculate_system_load_factor(self, system_context: Dict[str, Any]) -> float:
        """計算系統負載因子 (0.0-1.0，越高表示負載越輕)"""
        try:
            # 檢查待處理任務數量
            pending_tasks = system_context.get('pending_tasks_count', 0)
            
            # 負載越重，因子越低
            if pending_tasks <= 5:
                return 1.0  # 輕負載
            elif pending_tasks <= 15:
                return 0.7  # 中等負載
            elif pending_tasks <= 30:
                return 0.4  # 重負載
            else:
                return 0.2  # 極重負載
                
        except Exception as e:
            self._log_error(f"計算系統負載因子時發生錯誤: {e}")
            return 0.5
    
    def _calculate_task_importance_factor(self, task) -> float:
        """計算任務重要性因子 (0.0-1.0)"""
        try:
            params = task.parameters or {}
            
            # 基於任務類型判斷重要性
            wcs_task_type = params.get('wcs_task_type', '').lower()
            task_subtype = params.get('task_subtype', '').lower()
            
            if wcs_task_type == 'rotation' or task_subtype == 'rotation':
                return 1.0  # 旋轉任務最重要
            elif wcs_task_type == 'rack_move':
                return 0.8  # 搬運任務重要
            elif wcs_task_type in ['empty_delivery', 'full_collection']:
                return 0.6  # 派送收集任務中等重要
            else:
                return 0.4  # 其他任務較低重要性
                
        except Exception as e:
            self._log_error(f"計算任務重要性因子時發生錯誤: {e}")
            return 0.5
    
    def _estimate_task_duration(self, task) -> int:
        """預估任務執行時間 (秒)"""
        try:
            params = task.parameters or {}
            
            # 基於任務類型預估時間
            wcs_task_type = params.get('wcs_task_type', '').lower()
            task_subtype = params.get('task_subtype', '').lower()
            nodes = params.get('nodes', [])
            
            base_duration = 60  # 基礎時間60秒
            
            if wcs_task_type == 'rotation' or task_subtype == 'rotation':
                return 90  # 旋轉任務約90秒
            elif len(nodes) >= 3:
                return base_duration + (len(nodes) - 2) * 20  # 每多一個節點增加20秒
            else:
                return base_duration
                
        except Exception as e:
            self._log_error(f"預估任務執行時間時發生錯誤: {e}")
            return 60
    
    def _generate_schedule_reason(self, priority_level: WCSPriorityLevel, 
                                 time_factor: float, resource_factor: float,
                                 load_factor: float, importance_factor: float) -> str:
        """生成調度理由"""
        try:
            level_names = {
                WCSPriorityLevel.ROTATION: "轉架高優先",
                WCSPriorityLevel.OUTLET: "房間出口中優先",
                WCSPriorityLevel.INLET: "房間入口低優先",
                WCSPriorityLevel.MANUAL: "人員低優先"
            }
            
            reasons = [f"分類為{level_names.get(priority_level, '未知')}任務"]
            
            if time_factor > 0.7:
                reasons.append("時間緊急")
            if resource_factor > 0.7:
                reasons.append("資源充足")
            if load_factor < 0.3:
                reasons.append("系統負載重")
            if importance_factor > 0.8:
                reasons.append("任務重要性高")
            
            return "、".join(reasons)
            
        except Exception as e:
            self._log_error(f"生成調度理由時發生錯誤: {e}")
            return "無法生成理由"
    
    def schedule_tasks(self, tasks: List, system_context: Dict[str, Any] = None) -> List[TaskScheduleInfo]:
        """
        調度任務列表
        
        Args:
            tasks: 任務列表
            system_context: 系統上下文
            
        Returns:
            List[TaskScheduleInfo]: 按優先度排序的調度資訊
        """
        try:
            if not tasks:
                return []
            
            if system_context is None:
                system_context = {}
            
            # 計算每個任務的調度資訊
            schedule_infos = []
            for task in tasks:
                schedule_info = self.calculate_dynamic_priority(task, system_context)
                schedule_infos.append(schedule_info)
            
            # 按計算後的優先度排序 (高優先度在前)
            sorted_schedule_infos = sorted(
                schedule_infos, 
                key=lambda x: (x.calculated_priority, x.priority_level),
                reverse=True
            )
            
            # 更新統計
            self._update_schedule_statistics(sorted_schedule_infos)
            
            self._log_info(
                f"任務調度完成：總計 {len(sorted_schedule_infos)} 個任務，"
                f"🔴轉架:{self.schedule_stats['rotation_tasks']} "
                f"🟡出口:{self.schedule_stats['outlet_tasks']} "
                f"🟢入口:{self.schedule_stats['inlet_tasks']} "
                f"🔵手動:{self.schedule_stats['manual_tasks']}"
            )
            
            return sorted_schedule_infos
            
        except Exception as e:
            self._log_error(f"調度任務時發生錯誤: {e}")
            return []
    
    def _update_schedule_statistics(self, schedule_infos: List[TaskScheduleInfo]):
        """更新調度統計"""
        try:
            # 重置統計
            self.schedule_stats.update({
                'total_scheduled': len(schedule_infos),
                'rotation_tasks': 0,
                'outlet_tasks': 0,
                'inlet_tasks': 0,
                'manual_tasks': 0,
                'priority_adjustments': 0,
                'last_schedule_time': datetime.now(timezone.utc)
            })
            
            # 統計各優先度等級任務數量
            for info in schedule_infos:
                if info.priority_level == WCSPriorityLevel.ROTATION:
                    self.schedule_stats['rotation_tasks'] += 1
                elif info.priority_level == WCSPriorityLevel.OUTLET:
                    self.schedule_stats['outlet_tasks'] += 1
                elif info.priority_level == WCSPriorityLevel.INLET:
                    self.schedule_stats['inlet_tasks'] += 1
                else:
                    self.schedule_stats['manual_tasks'] += 1
                
                # 統計優先度調整次數
                if abs(info.priority_boost) > 0.1:  # 調整超過0.1被視為有意義的調整
                    self.schedule_stats['priority_adjustments'] += 1
                    
        except Exception as e:
            self._log_error(f"更新調度統計時發生錯誤: {e}")
    
    def get_schedule_statistics(self) -> Dict[str, Any]:
        """獲取調度統計資訊"""
        stats = self.schedule_stats.copy()
        
        # 計算額外統計資料
        if stats['total_scheduled'] > 0:
            stats['priority_adjustment_rate'] = (
                stats['priority_adjustments'] / stats['total_scheduled'] * 100
            )
        else:
            stats['priority_adjustment_rate'] = 0
        
        return stats
    
    def reset_statistics(self):
        """重置調度統計"""
        self.schedule_stats = {
            'total_scheduled': 0,
            'rotation_tasks': 0,
            'outlet_tasks': 0,
            'inlet_tasks': 0,
            'manual_tasks': 0,
            'priority_adjustments': 0,
            'last_schedule_time': None
        }
        self._log_info("調度統計資訊已重置")
    
    def explain_task_priority(self, task, system_context: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        解釋任務優先度計算過程
        
        Args:
            task: 任務物件
            system_context: 系統上下文
            
        Returns:
            Dict: 優先度解釋資訊
        """
        try:
            schedule_info = self.calculate_dynamic_priority(task, system_context or {})
            
            explanation = {
                'task_id': schedule_info.task_id,
                'priority_analysis': {
                    'original_priority': schedule_info.original_priority,
                    'calculated_priority': schedule_info.calculated_priority,
                    'priority_level': schedule_info.priority_level.name,
                    'priority_level_description': self._get_priority_level_description(schedule_info.priority_level),
                    'priority_boost': round(schedule_info.priority_boost, 2),
                    'schedule_reason': schedule_info.schedule_reason
                },
                'time_analysis': {
                    'estimated_duration': schedule_info.estimated_duration,
                    'estimated_duration_text': f"{schedule_info.estimated_duration}秒"
                },
                'recommendations': self._generate_priority_recommendations(schedule_info)
            }
            
            return explanation
            
        except Exception as e:
            self._log_error(f"解釋任務優先度時發生錯誤: {e}")
            return {'error': str(e)}
    
    def _get_priority_level_description(self, priority_level: WCSPriorityLevel) -> str:
        """獲取優先度等級描述"""
        descriptions = {
            WCSPriorityLevel.ROTATION: "🔴 轉架高優先 - Rack旋轉任務，系統最高優先度",
            WCSPriorityLevel.OUTLET: "🟡 房間出口中優先 - 出口Rack調度，中高優先度",
            WCSPriorityLevel.INLET: "🟢 房間入口低優先 - 入口Rack調度，中等優先度",
            WCSPriorityLevel.MANUAL: "🔵 人員低優先 - 手動請求與維護，低優先度"
        }
        return descriptions.get(priority_level, "未知優先度等級")
    
    def _generate_priority_recommendations(self, schedule_info: TaskScheduleInfo) -> List[str]:
        """生成優先度建議"""
        recommendations = []
        
        try:
            if schedule_info.priority_level == WCSPriorityLevel.ROTATION:
                recommendations.append("建議立即執行，避免影響後續Rack作業流程")
            
            if schedule_info.priority_boost > 0.5:
                recommendations.append("任務優先度已顯著提升，建議優先安排執行")
            elif schedule_info.priority_boost < -0.5:
                recommendations.append("任務優先度已降低，可延後執行")
            
            if schedule_info.estimated_duration > 120:
                recommendations.append("預估執行時間較長，建議安排在系統負載較輕時執行")
            
            return recommendations
            
        except Exception as e:
            self._log_error(f"生成優先度建議時發生錯誤: {e}")
            return ["無法生成建議"]