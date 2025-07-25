"""
Rack 狀態管理器
負責接收和處理來自 WCS 的 Rack 分析結果，為 KUKA 任務執行提供 Rack 狀態資訊
整合 Rack A/B 面狀態、製程驗證、位置追蹤等功能
"""

from typing import Dict, List, Optional, Any, Tuple
from enum import Enum, IntEnum
from dataclasses import dataclass, field
import json
from datetime import datetime, timezone


class RackSide(Enum):
    """Rack 面向定義"""
    A_SIDE = "A"    # A面 (0-90度)
    B_SIDE = "B"    # B面 (180度)


class RackDirection(IntEnum):
    """Rack 方向定義"""
    A_FACING = 0     # A面朝前 (90度)
    B_FACING = 180   # B面朝前 (-90度)


class CarrierStatus(IntEnum):
    """Carrier 狀態定義"""
    EMPTY = 0        # 空的
    OCCUPIED = 1     # 有貨物
    NG = 2          # 不良品
    PROCESSING = 3   # 處理中


class RackLocationStatus(IntEnum):
    """Rack 位置狀態"""
    UNKNOWN = 1      # 未知狀態
    UNOCCUPIED = 2   # 未佔用
    OCCUPIED = 3     # 已佔用


@dataclass
class CarrierInfo:
    """Carrier 資訊"""
    carrier_id: int
    rack_id: int
    rack_index: int  # 1-16=A面, 17-32=B面
    status: CarrierStatus
    room_id: Optional[int] = None
    product_id: Optional[int] = None
    created_at: datetime = field(default_factory=lambda: datetime.now(timezone.utc))
    updated_at: Optional[datetime] = None

    @property
    def side(self) -> RackSide:
        """獲取所屬面"""
        return RackSide.A_SIDE if 1 <= self.rack_index <= 16 else RackSide.B_SIDE

    @property
    def is_ng(self) -> bool:
        """是否為不良品"""
        return self.status == CarrierStatus.NG


@dataclass
class RackAnalysisResult:
    """Rack 分析結果 (來自 WCS)"""
    rack_id: int
    room_id: Optional[int]
    location_id: Optional[int]
    agv_id: Optional[int]
    current_direction: RackDirection
    product_id: Optional[int]
    is_carry: bool = False
    is_in_map: bool = True
    is_docked: bool = False
    
    # Carrier 狀態分析
    total_carriers: int = 0
    max_capacity: int = 32
    a_side_carriers: List[CarrierInfo] = field(default_factory=list)
    b_side_carriers: List[CarrierInfo] = field(default_factory=list)
    ng_carriers: List[CarrierInfo] = field(default_factory=list)
    
    # 狀態判斷結果
    is_empty: bool = True
    is_full: bool = False
    a_side_empty: bool = True
    a_side_full: bool = False
    b_side_empty: bool = True
    b_side_full: bool = False
    has_ng: bool = False
    needs_rotation: bool = False
    
    # 製程驗證結果
    process_compatible: bool = False
    process_error_message: str = ""
    
    # 分析時間戳記
    analysis_timestamp: datetime = field(default_factory=lambda: datetime.now(timezone.utc))

    @property
    def current_side(self) -> RackSide:
        """獲取當前朝向面"""
        return RackSide.A_SIDE if self.current_direction == RackDirection.A_FACING else RackSide.B_SIDE

    @property
    def a_side_count(self) -> int:
        """A面 Carrier 數量"""
        return len(self.a_side_carriers)

    @property
    def b_side_count(self) -> int:
        """B面 Carrier 數量"""
        return len(self.b_side_carriers)

    @property
    def ng_count(self) -> int:
        """NG Carrier 數量"""
        return len(self.ng_carriers)


class RackStateManager:
    """Rack 狀態管理器"""
    
    def __init__(self, logger=None):
        self.logger = logger
        self._log_info('Rack 狀態管理器初始化完成')
        
        # Rack 狀態快取
        self.rack_states: Dict[int, RackAnalysisResult] = {}
        
        # 製程設定快取
        self.process_settings_cache: Dict[int, Dict[str, Any]] = {}
        
        # 產品資訊快取
        self.product_cache: Dict[int, Dict[str, Any]] = {}
        
        # 狀態統計
        self.state_stats = {
            'total_racks_analyzed': 0,
            'empty_racks': 0,
            'full_racks': 0,
            'ng_racks': 0,
            'rotation_needed_racks': 0,
            'process_compatible_racks': 0,
            'last_update_time': None
        }
    
    def _log_info(self, message: str):
        """記錄資訊日誌"""
        if self.logger:
            self.logger.info(f"[RackStateManager] {message}")
    
    def _log_warning(self, message: str):
        """記錄警告日誌"""
        if self.logger:
            self.logger.warning(f"[RackStateManager] {message}")
    
    def _log_error(self, message: str):
        """記錄錯誤日誌"""
        if self.logger:
            self.logger.error(f"[RackStateManager] {message}")
    
    def update_rack_state(self, rack_analysis: RackAnalysisResult) -> bool:
        """
        更新 Rack 狀態資訊
        
        Args:
            rack_analysis: WCS 分析結果
            
        Returns:
            bool: 是否更新成功
        """
        try:
            rack_id = rack_analysis.rack_id
            
            # 驗證分析結果
            if not self._validate_analysis_result(rack_analysis):
                self._log_error(f"Rack {rack_id} 分析結果驗證失敗")
                return False
            
            # 補充分析資訊
            self._enhance_analysis_result(rack_analysis)
            
            # 更新狀態快取
            old_state = self.rack_states.get(rack_id)
            self.rack_states[rack_id] = rack_analysis
            
            # 記錄狀態變更
            if old_state:
                self._log_state_changes(old_state, rack_analysis)
            
            # 更新統計
            self._update_statistics()
            
            self._log_info(
                f"Rack {rack_id} 狀態已更新: "
                f"位置={rack_analysis.location_id}, "
                f"載貨={rack_analysis.total_carriers}, "
                f"需旋轉={rack_analysis.needs_rotation}, "
                f"有NG={rack_analysis.has_ng}"
            )
            
            return True
            
        except Exception as e:
            self._log_error(f"更新 Rack 狀態時發生錯誤: {e}")
            return False
    
    def _validate_analysis_result(self, rack_analysis: RackAnalysisResult) -> bool:
        """驗證分析結果有效性"""
        try:
            # 基本欄位檢查
            if rack_analysis.rack_id <= 0:
                return False
            
            if rack_analysis.max_capacity <= 0:
                return False
            
            if rack_analysis.total_carriers < 0:
                return False
            
            # Carrier 數量一致性檢查
            total_by_sides = len(rack_analysis.a_side_carriers) + len(rack_analysis.b_side_carriers)
            if total_by_sides != rack_analysis.total_carriers:
                self._log_warning(
                    f"Rack {rack_analysis.rack_id} Carrier 數量不一致: "
                    f"總計={rack_analysis.total_carriers}, A+B={total_by_sides}"
                )
            
            # 方向檢查
            if rack_analysis.current_direction not in [RackDirection.A_FACING, RackDirection.B_FACING]:
                self._log_warning(f"Rack {rack_analysis.rack_id} 方向值異常: {rack_analysis.current_direction}")
            
            return True
            
        except Exception as e:
            self._log_error(f"驗證分析結果時發生錯誤: {e}")
            return False
    
    def _enhance_analysis_result(self, rack_analysis: RackAnalysisResult):
        """補充分析結果資訊"""
        try:
            # 統計各面狀態
            rack_analysis.a_side_empty = len(rack_analysis.a_side_carriers) == 0
            rack_analysis.a_side_full = len(rack_analysis.a_side_carriers) >= 16
            rack_analysis.b_side_empty = len(rack_analysis.b_side_carriers) == 0
            rack_analysis.b_side_full = len(rack_analysis.b_side_carriers) >= 16
            
            # 整體狀態判斷
            rack_analysis.is_empty = rack_analysis.total_carriers == 0
            rack_analysis.is_full = rack_analysis.total_carriers >= rack_analysis.max_capacity
            rack_analysis.has_ng = len(rack_analysis.ng_carriers) > 0
            
            # 旋轉需求判斷
            rack_analysis.needs_rotation = self._check_rotation_needed(rack_analysis)
            
            # 製程相符性檢查
            rack_analysis.process_compatible, rack_analysis.process_error_message = \
                self._validate_process_compatibility(rack_analysis)
            
        except Exception as e:
            self._log_error(f"補充分析結果時發生錯誤: {e}")
    
    def _check_rotation_needed(self, rack_analysis: RackAnalysisResult) -> bool:
        """檢查是否需要旋轉"""
        try:
            # 基於位置和載貨狀態判斷
            if not rack_analysis.location_id:
                return False
            
            # 這裡需要根據實際位置邏輯判斷
            # 簡化版本：基於面向狀態判斷
            current_side = rack_analysis.current_side
            
            # 如果當前面空了但另一面有貨物，可能需要旋轉
            if current_side == RackSide.A_SIDE:
                return rack_analysis.a_side_empty and not rack_analysis.b_side_empty
            else:
                return rack_analysis.b_side_empty and not rack_analysis.a_side_empty
                
        except Exception as e:
            self._log_error(f"檢查旋轉需求時發生錯誤: {e}")
            return False
    
    def _validate_process_compatibility(self, rack_analysis: RackAnalysisResult) -> Tuple[bool, str]:
        """驗證製程相符性"""
        try:
            # 如果沒有產品或房間資訊，跳過檢查
            if not rack_analysis.product_id or not rack_analysis.room_id:
                return True, "無需製程驗證"
            
            # 獲取產品製程設定
            product_info = self._get_product_info(rack_analysis.product_id)
            if not product_info:
                return False, f"找不到產品 {rack_analysis.product_id} 資訊"
            
            # 獲取房間製程設定
            room_process_id = self._get_room_process_setting_id(rack_analysis.room_id)
            if not room_process_id:
                return False, f"找不到房間 {rack_analysis.room_id} 製程設定"
            
            # 比較製程設定
            product_process_id = product_info.get('process_settings_id')
            if product_process_id != room_process_id:
                return False, f"產品製程 {product_process_id} 與房間製程 {room_process_id} 不符"
            
            return True, "製程相符"
            
        except Exception as e:
            self._log_error(f"驗證製程相符性時發生錯誤: {e}")
            return False, f"驗證過程發生錯誤: {str(e)}"
    
    def _get_product_info(self, product_id: int) -> Optional[Dict[str, Any]]:
        """獲取產品資訊 (含快取)"""
        if product_id in self.product_cache:
            return self.product_cache[product_id]
        
        # 這裡應該從資料庫查詢產品資訊
        # 為了示例簡化，返回假資料
        product_info = {
            'id': product_id,
            'name': f'Product_{product_id}',
            'size': 'S',
            'process_settings_id': 1
        }
        
        self.product_cache[product_id] = product_info
        return product_info
    
    def _get_room_process_setting_id(self, room_id: int) -> Optional[int]:
        """獲取房間製程設定 ID"""
        # 這裡應該從資料庫查詢房間製程設定
        # 為了示例簡化，返回假資料
        return 1  # 假設所有房間都使用製程設定 ID 1
    
    def _log_state_changes(self, old_state: RackAnalysisResult, new_state: RackAnalysisResult):
        """記錄狀態變更"""
        try:
            changes = []
            
            if old_state.location_id != new_state.location_id:
                changes.append(f"位置: {old_state.location_id} → {new_state.location_id}")
            
            if old_state.total_carriers != new_state.total_carriers:
                changes.append(f"載貨: {old_state.total_carriers} → {new_state.total_carriers}")
            
            if old_state.current_direction != new_state.current_direction:
                changes.append(f"方向: {old_state.current_direction} → {new_state.current_direction}")
            
            if old_state.needs_rotation != new_state.needs_rotation:
                changes.append(f"需旋轉: {old_state.needs_rotation} → {new_state.needs_rotation}")
            
            if old_state.has_ng != new_state.has_ng:
                changes.append(f"有NG: {old_state.has_ng} → {new_state.has_ng}")
            
            if changes:
                self._log_info(f"Rack {new_state.rack_id} 狀態變更: {', '.join(changes)}")
                
        except Exception as e:
            self._log_error(f"記錄狀態變更時發生錯誤: {e}")
    
    def get_rack_state(self, rack_id: int) -> Optional[RackAnalysisResult]:
        """
        獲取 Rack 狀態
        
        Args:
            rack_id: Rack ID
            
        Returns:
            RackAnalysisResult: Rack 狀態，不存在時返回 None
        """
        return self.rack_states.get(rack_id)
    
    def get_racks_needing_rotation(self) -> List[RackAnalysisResult]:
        """獲取需要旋轉的 Rack 列表"""
        return [
            rack_state for rack_state in self.rack_states.values()
            if rack_state.needs_rotation
        ]
    
    def get_racks_with_ng(self) -> List[RackAnalysisResult]:
        """獲取有 NG 的 Rack 列表"""
        return [
            rack_state for rack_state in self.rack_states.values()
            if rack_state.has_ng
        ]
    
    def get_empty_racks(self) -> List[RackAnalysisResult]:
        """獲取空 Rack 列表"""
        return [
            rack_state for rack_state in self.rack_states.values()
            if rack_state.is_empty
        ]
    
    def get_full_racks(self) -> List[RackAnalysisResult]:
        """獲取滿載 Rack 列表"""
        return [
            rack_state for rack_state in self.rack_states.values()
            if rack_state.is_full
        ]
    
    def get_racks_at_location(self, location_id: int) -> List[RackAnalysisResult]:
        """獲取特定位置的 Rack 列表"""
        return [
            rack_state for rack_state in self.rack_states.values()
            if rack_state.location_id == location_id
        ]
    
    def get_racks_in_room(self, room_id: int) -> List[RackAnalysisResult]:
        """獲取特定房間的 Rack 列表"""
        return [
            rack_state for rack_state in self.rack_states.values()
            if rack_state.room_id == room_id
        ]
    
    def find_suitable_rack_for_task(self, task_requirements: Dict[str, Any]) -> Optional[RackAnalysisResult]:
        """
        根據任務需求尋找合適的 Rack
        
        Args:
            task_requirements: 任務需求字典
            
        Returns:
            RackAnalysisResult: 合適的 Rack，沒有找到時返回 None
        """
        try:
            required_state = task_requirements.get('state', 'any')  # empty, full, any
            required_room = task_requirements.get('room_id')
            required_location = task_requirements.get('location_id')
            exclude_ng = task_requirements.get('exclude_ng', True)
            process_id = task_requirements.get('process_id')
            
            candidates = list(self.rack_states.values())
            
            # 篩選條件
            if required_state == 'empty':
                candidates = [r for r in candidates if r.is_empty]
            elif required_state == 'full':
                candidates = [r for r in candidates if r.is_full]
            
            if required_room:
                candidates = [r for r in candidates if r.room_id == required_room]
            
            if required_location:
                candidates = [r for r in candidates if r.location_id == required_location]
            
            if exclude_ng:
                candidates = [r for r in candidates if not r.has_ng]
            
            if process_id:
                candidates = [r for r in candidates if r.process_compatible]
            
            # 排序：優先選擇狀態穩定的 Rack
            candidates.sort(
                key=lambda r: (
                    r.process_compatible,  # 製程相符的優先
                    not r.needs_rotation,   # 不需旋轉的優先
                    r.analysis_timestamp    # 分析時間越新越好
                ),
                reverse=True
            )
            
            return candidates[0] if candidates else None
            
        except Exception as e:
            self._log_error(f"尋找合適 Rack 時發生錯誤: {e}")
            return None
    
    def _update_statistics(self):
        """更新統計資訊"""
        try:
            self.state_stats.update({
                'total_racks_analyzed': len(self.rack_states),
                'empty_racks': len(self.get_empty_racks()),
                'full_racks': len(self.get_full_racks()),
                'ng_racks': len(self.get_racks_with_ng()),
                'rotation_needed_racks': len(self.get_racks_needing_rotation()),
                'process_compatible_racks': len([r for r in self.rack_states.values() if r.process_compatible]),
                'last_update_time': datetime.now(timezone.utc)
            })
            
        except Exception as e:
            self._log_error(f"更新統計資訊時發生錯誤: {e}")
    
    def get_state_statistics(self) -> Dict[str, Any]:
        """獲取狀態統計資訊"""
        stats = self.state_stats.copy()
        
        # 計算額外統計
        if stats['total_racks_analyzed'] > 0:
            stats['ng_rate'] = stats['ng_racks'] / stats['total_racks_analyzed'] * 100
            stats['rotation_needed_rate'] = stats['rotation_needed_racks'] / stats['total_racks_analyzed'] * 100
            stats['process_compatible_rate'] = stats['process_compatible_racks'] / stats['total_racks_analyzed'] * 100
        else:
            stats.update({
                'ng_rate': 0,
                'rotation_needed_rate': 0,
                'process_compatible_rate': 0
            })
        
        return stats
    
    def clear_rack_state(self, rack_id: int) -> bool:
        """
        清除 Rack 狀態
        
        Args:
            rack_id: Rack ID
            
        Returns:
            bool: 是否成功清除
        """
        try:
            if rack_id in self.rack_states:
                del self.rack_states[rack_id]
                self._update_statistics()
                self._log_info(f"已清除 Rack {rack_id} 狀態")
                return True
            else:
                self._log_warning(f"Rack {rack_id} 狀態不存在，無需清除")
                return False
                
        except Exception as e:
            self._log_error(f"清除 Rack 狀態時發生錯誤: {e}")
            return False
    
    def clear_all_states(self):
        """清除所有 Rack 狀態"""
        try:
            cleared_count = len(self.rack_states)
            self.rack_states.clear()
            self.product_cache.clear()
            self.process_settings_cache.clear()
            self._update_statistics()
            
            self._log_info(f"已清除所有 Rack 狀態，共 {cleared_count} 個")
            
        except Exception as e:
            self._log_error(f"清除所有狀態時發生錯誤: {e}")
    
    def export_rack_states(self) -> Dict[str, Any]:
        """
        匯出所有 Rack 狀態 (用於備份或分析)
        
        Returns:
            Dict: 包含所有 Rack 狀態的字典
        """
        try:
            export_data = {
                'export_timestamp': datetime.now(timezone.utc).isoformat(),
                'total_racks': len(self.rack_states),
                'statistics': self.get_state_statistics(),
                'rack_states': {}
            }
            
            for rack_id, rack_state in self.rack_states.items():
                export_data['rack_states'][str(rack_id)] = {
                    'rack_id': rack_state.rack_id,
                    'room_id': rack_state.room_id,
                    'location_id': rack_state.location_id,
                    'current_direction': int(rack_state.current_direction),
                    'total_carriers': rack_state.total_carriers,
                    'a_side_count': rack_state.a_side_count,
                    'b_side_count': rack_state.b_side_count,
                    'ng_count': rack_state.ng_count,
                    'is_empty': rack_state.is_empty,
                    'is_full': rack_state.is_full,
                    'has_ng': rack_state.has_ng,
                    'needs_rotation': rack_state.needs_rotation,
                    'process_compatible': rack_state.process_compatible,
                    'analysis_timestamp': rack_state.analysis_timestamp.isoformat()
                }
            
            return export_data
            
        except Exception as e:
            self._log_error(f"匯出 Rack 狀態時發生錯誤: {e}")
            return {'error': str(e)}
    
    def generate_task_recommendations(self) -> List[Dict[str, Any]]:
        """
        基於當前 Rack 狀態生成任務建議
        
        Returns:
            List[Dict]: 任務建議列表
        """
        try:
            recommendations = []
            
            # 旋轉任務建議
            rotation_racks = self.get_racks_needing_rotation()
            for rack in rotation_racks:
                recommendations.append({
                    'task_type': 'rotation',
                    'priority': 100,  # 最高優先度
                    'rack_id': rack.rack_id,
                    'reason': f'Rack {rack.rack_id} 需要旋轉 ({rack.current_side.value}面朝前)',
                    'estimated_urgency': 'high'
                })
            
            # NG 處理建議
            ng_racks = self.get_racks_with_ng()
            for rack in ng_racks:
                recommendations.append({
                    'task_type': 'ng_handling',
                    'priority': 90,
                    'rack_id': rack.rack_id,
                    'reason': f'Rack {rack.rack_id} 含有 {rack.ng_count} 個 NG Carrier',
                    'estimated_urgency': 'high'
                })
            
            # 空 Rack 調度建議
            empty_racks = self.get_empty_racks()
            if len(empty_racks) > 10:  # 空 Rack 過多
                recommendations.append({
                    'task_type': 'empty_rack_management',
                    'priority': 60,
                    'rack_count': len(empty_racks),
                    'reason': f'空 Rack 數量過多 ({len(empty_racks)} 個)，建議回收至停車區',
                    'estimated_urgency': 'medium'
                })
            
            # 滿載 Rack 處理建議
            full_racks = self.get_full_racks()
            for rack in full_racks:
                if not rack.process_compatible:
                    recommendations.append({
                        'task_type': 'process_mismatch_handling',
                        'priority': 80,
                        'rack_id': rack.rack_id,
                        'reason': f'Rack {rack.rack_id} 製程不相符: {rack.process_error_message}',
                        'estimated_urgency': 'high'
                    })
            
            # 依優先度排序
            recommendations.sort(key=lambda x: x['priority'], reverse=True)
            
            return recommendations
            
        except Exception as e:
            self._log_error(f"生成任務建議時發生錯誤: {e}")
            return []