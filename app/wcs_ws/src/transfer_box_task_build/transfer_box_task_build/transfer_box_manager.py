"""
傳送箱管理類
負責傳送箱配置的查詢和管理
"""

from typing import Optional, List, Dict
from transfer_box_task_build import config


class TransferBoxManager:
    """傳送箱管理器"""

    def __init__(self):
        """初始化管理器"""
        self.transfer_boxs = config.TRANSFER_BOXES

    def get_transfer_box_by_work_id(self, work_id: int) -> Optional[Dict]:
        """
        根據 work_id 查找對應的傳送箱配置

        Args:
            work_id: Work ID

        Returns:
            傳送箱配置字典，若找不到則返回 None

        範例:
            work_id=2000102 → 入口傳送箱配置
            work_id=2000201 → 出口傳送箱配置
        """
        for transfer_box in self.transfer_boxs:
            if work_id in transfer_box["work_ids"]:
                return transfer_box
        return None

    def get_transfer_box_by_location(self, location_id: int) -> Optional[Dict]:
        """
        根據 location_id 查找對應的傳送箱配置

        Args:
            location_id: Location ID

        Returns:
            傳送箱配置字典，若找不到則返回 None

        範例:
            location_id=20001 → 入口傳送箱配置
            location_id=20002 → 出口傳送箱配置
        """
        for transfer_box in self.transfer_boxs:
            if transfer_box["location_id"] == location_id:
                return transfer_box
        return None

    def get_all_transfer_boxs(self) -> List[Dict]:
        """
        獲取所有傳送箱配置列表

        Returns:
            傳送箱配置列表
        """
        return self.transfer_boxs

    def get_all_work_ids(self) -> List[int]:
        """
        獲取所有傳送箱的 work_id 列表（扁平化）

        Returns:
            所有 work_id 的列表

        範例:
            返回 [2000102, 2002102, 2000201, 2001201]
        """
        work_ids = []
        for transfer_box in self.transfer_boxs:
            work_ids.extend(transfer_box["work_ids"])
        return work_ids

    def get_transfer_box_count(self) -> int:
        """
        獲取傳送箱數量

        Returns:
            傳送箱數量
        """
        return len(self.transfer_boxs)

    def get_transfer_box_name(self, work_id: int) -> str:
        """
        根據 work_id 獲取傳送箱名稱

        Args:
            work_id: Work ID

        Returns:
            傳送箱名稱，若找不到則返回 "未知傳送箱"
        """
        transfer_box = self.get_transfer_box_by_work_id(work_id)
        if transfer_box:
            return transfer_box["name"]
        return "未知傳送箱"
