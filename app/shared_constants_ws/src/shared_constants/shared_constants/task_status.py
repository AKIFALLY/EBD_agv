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
    
    # ========== FROM->TO 完整任務 (1-5) ==========
    FROM_TO_START: ClassVar[int] = 1       # From->To 任務開始
    FROM_EXECUTING: ClassVar[int] = 2      # From 流程執行中
    FROM_COMPLETE: ClassVar[int] = 3       # From 流程完成，準備 To 流程
    TO_EXECUTING: ClassVar[int] = 4        # To 流程執行中
    FROM_TO_COMPLETE: ClassVar[int] = 5    # 任務完成

    # ========== 僅 FROM 任務 (11-15) ==========
    FROM_ONLY_START: ClassVar[int] = 11    # From 任務開始
    FROM_ONLY_EXECUTING: ClassVar[int] = 12  # From 流程執行中
    FROM_ONLY_COMPLETE: ClassVar[int] = 15   # 任務完成

    # ========== 僅 TO 任務 (13-15) ==========
    TO_ONLY_START: ClassVar[int] = 13      # To 任務開始
    TO_ONLY_EXECUTING: ClassVar[int] = 14  # To 流程執行中
    TO_ONLY_COMPLETE: ClassVar[int] = 15   # 任務完成 (與 FROM_ONLY_COMPLETE 共用)

    # ========== PATH 任務 (21-25) ==========
    PATH_START: ClassVar[int] = 21         # Path 任務開始
    PATH_EXECUTING: ClassVar[int] = 22     # Path 流程執行中
    PATH_COMPLETE: ClassVar[int] = 25      # 任務完成

    # ========== 系統狀態 ==========
    REQUESTING: ClassVar[int] = 0          # 請求中 (UI-請求執行任務)
    ERROR: ClassVar[int] = 6               # 錯誤

    # ========== 向後相容別名 (舊定義) ==========
    PENDING: ClassVar[int] = 1             # 別名 -> FROM_TO_START
    READY_TO_EXECUTE: ClassVar[int] = 2    # 別名 -> FROM_EXECUTING
    EXECUTING: ClassVar[int] = 3           # 別名 -> FROM_COMPLETE
    COMPLETED: ClassVar[int] = 4           # 別名 -> TO_EXECUTING (注意語義變化)
    CANCELLING: ClassVar[int] = 5          # 別名 -> FROM_TO_COMPLETE (注意語義變化)

    # 取消相關狀態
    WCS_CANCELLING: ClassVar[int] = 51     # WCS-取消中 (WCS-任務取消中，待處理)
    RCS_CANCELLING: ClassVar[int] = 52     # RCS-取消中 (RCS-任務取消中，取消中)
    AGV_CANCELLING: ClassVar[int] = 53     # AGV-取消中 (AGV-取消完成)
    CANCELLED: ClassVar[int] = 54          # 已取消 (任務已取消)

    # 狀態碼對應的中文描述
    STATUS_DESCRIPTIONS: ClassVar[Dict[int, str]] = {
        # 系統狀態
        0: "請求中",
        6: "錯誤",
        # FROM->TO 完整任務
        1: "From->To任務開始",
        2: "From流程執行中",
        3: "From流程完成",
        4: "To流程執行中",
        5: "任務完成",
        # 僅 FROM 任務
        11: "From任務開始",
        12: "From流程執行中",
        # 僅 TO 任務
        13: "To任務開始",
        14: "To流程執行中",
        15: "任務完成",
        # PATH 任務
        21: "Path任務開始",
        22: "Path流程執行中",
        25: "任務完成",
        # 取消狀態
        51: "WCS-取消中",
        52: "RCS-取消中",
        53: "AGV-取消中",
        54: "已取消"
    }

    # 狀態碼對應的英文名稱
    STATUS_NAMES: ClassVar[Dict[int, str]] = {
        # 系統狀態
        0: "REQUESTING",
        6: "ERROR",
        # FROM->TO 完整任務
        1: "FROM_TO_START",
        2: "FROM_EXECUTING",
        3: "FROM_COMPLETE",
        4: "TO_EXECUTING",
        5: "FROM_TO_COMPLETE",
        # 僅 FROM 任務
        11: "FROM_ONLY_START",
        12: "FROM_ONLY_EXECUTING",
        # 僅 TO 任務
        13: "TO_ONLY_START",
        14: "TO_ONLY_EXECUTING",
        15: "TASK_COMPLETE",
        # PATH 任務
        21: "PATH_START",
        22: "PATH_EXECUTING",
        25: "PATH_COMPLETE",
        # 取消狀態
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

    # ========== 任務開始狀態集合 ==========
    # 用於 AGV 狀態機判斷是否需要寫入路徑
    TASK_START_STATUSES: ClassVar[tuple] = (1, 11, 13, 21)

    @classmethod
    def is_task_start_status(cls, status_code: int) -> bool:
        """檢查是否為「任務開始」狀態（需要寫入路徑）

        任務開始狀態包含:
        - 1: FROM_TO_START (From->To 任務開始)
        - 11: FROM_ONLY_START (僅 From 任務開始)
        - 13: TO_ONLY_START (僅 To 任務開始)
        - 21: PATH_START (Path 任務開始)

        Args:
            status_code: 狀態碼

        Returns:
            如果是任務開始狀態則返回 True
        """
        return status_code in cls.TASK_START_STATUSES

    @classmethod
    def is_task_executing_status(cls, status_code: int) -> bool:
        """檢查是否為「執行中」狀態

        執行中狀態包含:
        - 2, 3, 4: FROM_EXECUTING, FROM_COMPLETE, TO_EXECUTING
        - 12: FROM_ONLY_EXECUTING
        - 14: TO_ONLY_EXECUTING
        - 22: PATH_EXECUTING

        Args:
            status_code: 狀態碼

        Returns:
            如果是執行中狀態則返回 True
        """
        return status_code in (2, 3, 4, 12, 14, 22)

    @classmethod
    def is_task_complete_status(cls, status_code: int) -> bool:
        """檢查是否為「任務完成」狀態

        任務完成狀態包含:
        - 5: FROM_TO_COMPLETE
        - 15: FROM_ONLY_COMPLETE / TO_ONLY_COMPLETE
        - 25: PATH_COMPLETE

        Args:
            status_code: 狀態碼

        Returns:
            如果是任務完成狀態則返回 True
        """
        return status_code in (5, 15, 25)