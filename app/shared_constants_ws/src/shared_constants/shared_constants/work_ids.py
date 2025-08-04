"""
共享的工作 ID 常數定義

這個模組定義了 RosAGV 系統中使用的所有工作 ID 常數，
包括 KUKA 支援的 work_id 和其他系統使用的 work_id。
"""

from typing import ClassVar, List, Dict


class WorkIds:
    """工作 ID 常數定義類別
    
    定義了 RosAGV 系統中所有可能的工作 ID 常數，
    以及相關的分類和實用方法。
    """
    
    # KUKA 支援的工作 ID
    KUKA_MOVE: ClassVar[int] = 210001          # KUKA 移動
    KUKA_RACK_MOVE: ClassVar[int] = 220001     # KUKA 移動貨架
    KUKA_WORKFLOW: ClassVar[int] = 230001      # KUKA template 流程任務
    
    # OPUI 相關工作 ID
    OPUI_CALL_EMPTY: ClassVar[int] = 100001    # OPUI 叫空車
    
    # CT AGV 相關工作 ID
    CT_AGV_WORK: ClassVar[int] = 2000102       # CT AGV 工作
    
    # KUKA 支援的工作 ID 列表
    KUKA_SUPPORTED_WORK_IDS: ClassVar[List[int]] = [
        KUKA_MOVE,
        KUKA_RACK_MOVE,
        KUKA_WORKFLOW
    ]
    
    # 工作 ID 對應的描述
    WORK_ID_DESCRIPTIONS: ClassVar[Dict[int, str]] = {
        210001: "KUKA 移動",
        220001: "KUKA 移動貨架", 
        230001: "KUKA template 流程任務",
        100001: "OPUI 叫空車",
        2000102: "CT AGV 工作"
    }
    
    # 工作 ID 對應的英文名稱
    WORK_ID_NAMES: ClassVar[Dict[int, str]] = {
        210001: "KUKA_MOVE",
        220001: "KUKA_RACK_MOVE",
        230001: "KUKA_WORKFLOW", 
        100001: "OPUI_CALL_EMPTY",
        2000102: "CT_AGV_WORK"
    }
    
    # 工作 ID 對應的 KUKA API 類型
    KUKA_API_MAPPING: ClassVar[Dict[int, str]] = {
        KUKA_MOVE: "move",           # 對應 kuka_fleet.move()
        KUKA_RACK_MOVE: "rack_move", # 對應 kuka_fleet.rack_move()
        KUKA_WORKFLOW: "workflow"    # 對應 kuka_fleet.workflow()
    }

    @classmethod
    def get_description(cls, work_id: int) -> str:
        """取得工作 ID 對應的中文描述
        
        Args:
            work_id: 工作 ID
            
        Returns:
            工作 ID 的中文描述，如果不存在則返回 "未知工作 ID"
        """
        return cls.WORK_ID_DESCRIPTIONS.get(work_id, "未知工作 ID")
    
    @classmethod
    def get_name(cls, work_id: int) -> str:
        """取得工作 ID 對應的英文名稱
        
        Args:
            work_id: 工作 ID
            
        Returns:
            工作 ID 的英文名稱，如果不存在則返回 "UNKNOWN"
        """
        return cls.WORK_ID_NAMES.get(work_id, "UNKNOWN")
    
    @classmethod
    def is_kuka_supported(cls, work_id: int) -> bool:
        """檢查工作 ID 是否為 KUKA 支援的工作 ID
        
        Args:
            work_id: 要檢查的工作 ID
            
        Returns:
            如果是 KUKA 支援的工作 ID 則返回 True，否則返回 False
        """
        return work_id in cls.KUKA_SUPPORTED_WORK_IDS
    
    @classmethod
    def get_kuka_api_type(cls, work_id: int) -> str:
        """取得 KUKA 工作 ID 對應的 API 類型
        
        Args:
            work_id: KUKA 工作 ID
            
        Returns:
            對應的 KUKA API 類型名稱，如果不是 KUKA 工作 ID 則返回空字串
        """
        return cls.KUKA_API_MAPPING.get(work_id, "")
    
    @classmethod
    def get_all_work_ids(cls) -> Dict[int, str]:
        """取得所有工作 ID 和對應的描述
        
        Returns:
            包含所有工作 ID 和描述的字典
        """
        return cls.WORK_ID_DESCRIPTIONS.copy()
    
    @classmethod
    def get_kuka_work_ids(cls) -> List[int]:
        """取得所有 KUKA 支援的工作 ID
        
        Returns:
            KUKA 支援的工作 ID 列表
        """
        return cls.KUKA_SUPPORTED_WORK_IDS.copy()