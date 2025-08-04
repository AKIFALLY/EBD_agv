"""
共享的任務狀態常數定義

這個模組定義了 RosAGV 系統中使用的所有任務狀態常數，
可以被 AGV 和 AGVC 兩個環境同時引用，解決跨容器依賴問題。

基於原始的 db_proxy.models.agvc_task.TaskStatus 實作。
"""

from typing import ClassVar, Dict


class TaskStatus:
    """任務狀態常數定義類別
    
    定義了 RosAGV 系統中所有可能的任務狀態常數，
    以及相關的實用方法。
    """
    
    # 主要狀態
    REQUESTING: ClassVar[int] = 0          # 請求中 (UI-請求執行任務)
    PENDING: ClassVar[int] = 1             # 待處理 (WCS-任務已接受，待處理)
    READY_TO_EXECUTE: ClassVar[int] = 2    # 待執行 (RCS-任務已派發，待執行)
    EXECUTING: ClassVar[int] = 3           # 執行中 (AGV-任務正在執行)
    COMPLETED: ClassVar[int] = 4           # 已完成 (AGV-任務已完成)
    CANCELLING: ClassVar[int] = 5          # 取消中 (任務取消)
    ERROR: ClassVar[int] = 6               # 錯誤 (錯誤)

    # 取消相關狀態
    WCS_CANCELLING: ClassVar[int] = 51     # WCS-取消中 (WCS-任務取消中，待處理)
    RCS_CANCELLING: ClassVar[int] = 52     # RCS-取消中 (RCS-任務取消中，取消中)
    AGV_CANCELLING: ClassVar[int] = 53     # AGV-取消中 (AGV-取消完成)
    CANCELLED: ClassVar[int] = 54          # 已取消 (任務已取消)

    # 狀態碼對應的中文描述
    STATUS_DESCRIPTIONS: ClassVar[Dict[int, str]] = {
        0: "請求中",
        1: "待處理",
        2: "待執行",
        3: "執行中",
        4: "已完成",
        5: "取消中",
        6: "錯誤",
        51: "WCS-取消中",
        52: "RCS-取消中",
        53: "AGV-取消中",
        54: "已取消"
    }

    # 狀態碼對應的英文名稱
    STATUS_NAMES: ClassVar[Dict[int, str]] = {
        0: "REQUESTING",
        1: "PENDING",
        2: "READY_TO_EXECUTE",
        3: "EXECUTING",
        4: "COMPLETED",
        5: "CANCELLING",
        6: "ERROR",
        51: "WCS_CANCELLING",
        52: "RCS_CANCELLING",
        53: "AGV_CANCELLING",
        54: "CANCELLED"
    }

    @classmethod
    def get_description(cls, status_code: int) -> str:
        """取得狀態碼對應的中文描述
        
        Args:
            status_code: 狀態碼
            
        Returns:
            狀態的中文描述，如果狀態碼不存在則返回 "未知狀態"
        """
        return cls.STATUS_DESCRIPTIONS.get(status_code, "未知狀態")

    @classmethod
    def get_name(cls, status_code: int) -> str:
        """取得狀態碼對應的英文名稱
        
        Args:
            status_code: 狀態碼
            
        Returns:
            狀態的英文名稱，如果狀態碼不存在則返回 "UNKNOWN"
        """
        return cls.STATUS_NAMES.get(status_code, "UNKNOWN")

    @classmethod
    def is_valid_status(cls, status_code: int) -> bool:
        """檢查狀態碼是否有效
        
        Args:
            status_code: 要檢查的狀態碼
            
        Returns:
            如果狀態碼有效則返回 True，否則返回 False
        """
        return status_code in cls.STATUS_DESCRIPTIONS
    
    @classmethod
    def get_all_statuses(cls) -> Dict[int, str]:
        """取得所有狀態碼和對應的描述
        
        Returns:
            包含所有狀態碼和描述的字典
        """
        return cls.STATUS_DESCRIPTIONS.copy()
    
    @classmethod
    def get_main_statuses(cls) -> Dict[int, str]:
        """取得主要狀態碼（0-6）和對應的描述
        
        Returns:
            包含主要狀態碼和描述的字典
        """
        return {k: v for k, v in cls.STATUS_DESCRIPTIONS.items() if k <= 6}
    
    @classmethod
    def get_cancel_statuses(cls) -> Dict[int, str]:
        """取得取消相關狀態碼（50+）和對應的描述
        
        Returns:
            包含取消相關狀態碼和描述的字典
        """
        return {k: v for k, v in cls.STATUS_DESCRIPTIONS.items() if k >= 50}