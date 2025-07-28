"""
Rack 狀態分析模組
負責分析 Rack 的A/B面狀態、NG檢測、容量計算等核心邏輯
"""

import rclpy
from rclpy.node import Node
from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass
from enum import Enum
import json


class RackDirection(Enum):
    """Rack 朝向"""
    A_SIDE = 90    # A面朝前
    B_SIDE = -90   # B面朝前 (180度旋轉)


class RackSide(Enum):
    """Rack 面向"""
    A = "A"
    B = "B"


@dataclass
class CarrierInfo:
    """Carrier 資訊"""
    id: int
    rack_id: Optional[int]
    room_id: Optional[int]
    port_id: Optional[int]
    rack_index: Optional[int]  # 1-16=A面, 17-32=B面
    status_id: Optional[int]
    is_ng: bool = False


@dataclass
class RackStatus:
    """Rack 完整狀態資訊"""
    rack_id: int
    total_carriers: int
    max_capacity: int
    a_side_count: int
    b_side_count: int
    current_side: RackSide
    has_ng: bool
    is_empty: bool
    is_full: bool
    is_half_full: bool
    needs_rotation: bool
    product_size: str  # S或L
    location_id: Optional[int]
    room_id: Optional[int]


class RackAnalyzer:
    """Rack 狀態分析器"""
    
    def __init__(self, logger=None):
        self.logger = logger
        if self.logger:
            self.logger.info('Rack 狀態分析器啟動')
        
        # 容量配置
        self.CAPACITY_CONFIG = {
            'S': 32,  # S尺寸：2面 × 16個
            'L': 16   # L尺寸：2面 × 8個
        }
        
        # A/B面索引範圍
        self.A_SIDE_RANGE = range(1, 17)   # 1-16
        self.B_SIDE_RANGE = range(17, 33)  # 17-32
        
    def analyze_rack_status(self, rack_data: Dict, carriers: List[CarrierInfo], 
                          product_data: Dict) -> RackStatus:
        """
        分析 Rack 完整狀態
        
        Args:
            rack_data: Rack基本資料 (id, direction, location_id, room_id等)
            carriers: 該Rack上的所有carriers
            product_data: 產品資料 (size等)
            
        Returns:
            RackStatus: 完整的Rack狀態資訊
        """
        rack_id = rack_data['id']
        direction = rack_data.get('direction', 90)
        product_size = product_data.get('size', 'S')
        
        # 計算最大容量
        max_capacity = self.CAPACITY_CONFIG.get(product_size, 32)
        
        # 分析A/B面carrier分布
        a_side_carriers = [c for c in carriers 
                          if c.rack_index and c.rack_index in self.A_SIDE_RANGE]
        b_side_carriers = [c for c in carriers 
                          if c.rack_index and c.rack_index in self.B_SIDE_RANGE]
        
        # 檢查NG狀態
        ng_carriers = [c for c in carriers if c.is_ng]
        has_ng = len(ng_carriers) > 0
        
        # 判斷當前朝向
        current_side = RackSide.A if direction == RackDirection.A_SIDE.value else RackSide.B
        
        # 計算狀態
        total_carriers = len(carriers)
        a_side_count = len(a_side_carriers)
        b_side_count = len(b_side_carriers)
        
        is_empty = total_carriers == 0
        is_full = total_carriers == max_capacity
        is_half_full = self._check_half_full_status(a_side_count, b_side_count, product_size)
        
        # 檢查是否需要旋轉
        needs_rotation = self._check_rotation_needed(
            rack_data, a_side_count, b_side_count, current_side, product_size
        )
        
        return RackStatus(
            rack_id=rack_id,
            total_carriers=total_carriers,
            max_capacity=max_capacity,
            a_side_count=a_side_count,
            b_side_count=b_side_count,
            current_side=current_side,
            has_ng=has_ng,
            is_empty=is_empty,
            is_full=is_full,
            is_half_full=is_half_full,
            needs_rotation=needs_rotation,
            product_size=product_size,
            location_id=rack_data.get('location_id'),
            room_id=rack_data.get('room_id')
        )
    
    def _check_half_full_status(self, a_side_count: int, b_side_count: int, 
                               product_size: str) -> bool:
        """檢查是否為半滿狀態（一面滿，一面空或未滿）"""
        max_per_side = self.CAPACITY_CONFIG.get(product_size, 32) // 2
        
        # A面滿B面未滿，或B面滿A面未滿
        return ((a_side_count == max_per_side and b_side_count < max_per_side) or
                (b_side_count == max_per_side and a_side_count < max_per_side))
    
    def _check_rotation_needed(self, rack_data: Dict, a_side_count: int, 
                              b_side_count: int, current_side: RackSide, 
                              product_size: str) -> bool:
        """
        檢查是否需要旋轉Rack
        
        旋轉條件：
        1. 入口位置：當前A面且A面空了，B面有貨物
        2. 出口位置：當前A面且A面滿了，B面還能放
        """
        location_id = rack_data.get('location_id')
        if not location_id:
            return False
            
        # 這裡需要根據location_id判斷是入口還是出口
        # 暫時用簡化邏輯，實際需要查詢location表
        is_inlet = self._is_room_inlet(location_id)
        is_outlet = self._is_room_outlet(location_id)
        
        max_per_side = self.CAPACITY_CONFIG.get(product_size, 32) // 2
        
        if is_inlet and current_side == RackSide.A:
            # 入口：A面空了且B面有貨物
            return a_side_count == 0 and b_side_count > 0
            
        elif is_outlet and current_side == RackSide.A:
            # 出口：A面滿了且B面還能放
            return a_side_count == max_per_side and b_side_count < max_per_side
            
        return False
    
    def _is_room_inlet(self, location_id: int) -> bool:
        """判斷是否為房間入口（需要實際查詢資料庫）"""
        # TODO: 實作資料庫查詢邏輯
        return False
    
    def _is_room_outlet(self, location_id: int) -> bool:
        """判斷是否為房間出口（需要實際查詢資料庫）"""
        # TODO: 實作資料庫查詢邏輯
        return False
    
    def get_rotation_nodes(self, location_node_id: int) -> List[int]:
        """
        取得旋轉任務的節點路徑
        
        旋轉任務路徑：起點 -> 移出點 -> 回到起點
        """
        intermediate_node = self._get_rotation_intermediate_node(location_node_id)
        return [location_node_id, intermediate_node, location_node_id]
    
    def _get_rotation_intermediate_node(self, location_node_id: int) -> int:
        """取得旋轉作業的中間節點（暫時使用固定偏移邏輯）"""
        # TODO: 實作實際的中間節點查詢邏輯
        return location_node_id + 1000  # 暫時的偏移值
    
    def validate_rack_carriers_consistency(self, rack_status: RackStatus, 
                                         carriers: List[CarrierInfo]) -> List[str]:
        """
        驗證Rack與Carriers的一致性
        
        Returns:
            List[str]: 發現的問題列表
        """
        issues = []
        
        # 檢查carrier數量是否一致
        actual_carriers = len([c for c in carriers if c.rack_id == rack_status.rack_id])
        if actual_carriers != rack_status.total_carriers:
            issues.append(f"Carrier數量不一致: 預期{rack_status.total_carriers}, 實際{actual_carriers}")
        
        # 檢查rack_index範圍
        for carrier in carriers:
            if carrier.rack_index and carrier.rack_index not in range(1, 33):
                issues.append(f"Carrier {carrier.id} rack_index {carrier.rack_index} 超出範圍(1-32)")
        
        # 檢查A/B面分布
        a_side_actual = len([c for c in carriers if c.rack_index in self.A_SIDE_RANGE])
        b_side_actual = len([c for c in carriers if c.rack_index in self.B_SIDE_RANGE])
        
        if a_side_actual != rack_status.a_side_count:
            issues.append(f"A面carrier數量不一致: 預期{rack_status.a_side_count}, 實際{a_side_actual}")
            
        if b_side_actual != rack_status.b_side_count:
            issues.append(f"B面carrier數量不一致: 預期{rack_status.b_side_count}, 實際{b_side_actual}")
        
        return issues
    
    def destroy_node(self):
        """修正：添加destroy_node方法以符合ai_wcs_node調用"""
        if self.logger:
            self.logger.info('🔚 Rack 狀態分析器正在關閉...')
        # 清理資源
        pass


def main(args=None):
    """主函數 - 獨立運行時使用"""
    import rclpy
    from rclpy.node import Node
    
    class StandaloneRackAnalyzer(Node):
        def __init__(self):
            super().__init__('rack_analyzer')
            self.analyzer = RackAnalyzer(self.get_logger())
            
    rclpy.init(args=args)
    node = StandaloneRackAnalyzer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()