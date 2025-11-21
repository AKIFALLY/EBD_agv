"""
日誌清理配置常數

定義日誌自動清理系統的配置參數，適用於 db_proxy 資料庫代理服務。
"""


class LogCleanupConfig:
    """日誌清理配置常數"""

    # 清理功能開關
    ENABLED = True  # 啟用日誌自動清理功能

    # 保留政策
    RETENTION_DAYS = 90  # 日誌保留天數（統一保留 90 天）

    # 執行時間
    CLEANUP_HOUR = 2  # 每天執行清理的小時（0-23，預設凌晨 2:00）
    CLEANUP_CHECK_INTERVAL = 3600.0  # 檢查間隔（秒，預設每小時檢查一次）

    # 批次刪除配置
    BATCH_SIZE = 1000  # 每次批次刪除的筆數（避免長時間鎖表）

    # 日誌等級
    LOG_CLEANUP_INFO = True  # 記錄清理資訊日誌
    LOG_CLEANUP_DEBUG = False  # 記錄清理除錯日誌

    @staticmethod
    def get_config_dict():
        """
        獲取配置字典

        Returns:
            包含所有配置參數的字典
        """
        return {
            'enabled': LogCleanupConfig.ENABLED,
            'retention_days': LogCleanupConfig.RETENTION_DAYS,
            'cleanup_hour': LogCleanupConfig.CLEANUP_HOUR,
            'cleanup_check_interval': LogCleanupConfig.CLEANUP_CHECK_INTERVAL,
            'batch_size': LogCleanupConfig.BATCH_SIZE,
            'log_info': LogCleanupConfig.LOG_CLEANUP_INFO,
            'log_debug': LogCleanupConfig.LOG_CLEANUP_DEBUG
        }

    @staticmethod
    def validate_config():
        """
        驗證配置參數有效性

        Returns:
            (bool, str): (是否有效, 錯誤訊息)
        """
        if not isinstance(LogCleanupConfig.RETENTION_DAYS, int) or LogCleanupConfig.RETENTION_DAYS <= 0:
            return False, "RETENTION_DAYS 必須是正整數"

        if not isinstance(LogCleanupConfig.CLEANUP_HOUR, int) or not (0 <= LogCleanupConfig.CLEANUP_HOUR <= 23):
            return False, "CLEANUP_HOUR 必須在 0-23 之間"

        if not isinstance(LogCleanupConfig.BATCH_SIZE, int) or LogCleanupConfig.BATCH_SIZE <= 0:
            return False, "BATCH_SIZE 必須是正整數"

        if not isinstance(LogCleanupConfig.CLEANUP_CHECK_INTERVAL, (int, float)) or LogCleanupConfig.CLEANUP_CHECK_INTERVAL <= 0:
            return False, "CLEANUP_CHECK_INTERVAL 必須是正數"

        return True, ""

    @staticmethod
    def get_description(key):
        """
        獲取配置參數的描述

        Args:
            key: 配置鍵名

        Returns:
            配置參數的中文描述
        """
        descriptions = {
            'enabled': '啟用日誌自動清理功能',
            'retention_days': '日誌保留天數（統一保留 90 天）',
            'cleanup_hour': '每天執行清理的小時（0-23）',
            'cleanup_check_interval': '檢查間隔（秒）',
            'batch_size': '每次批次刪除的筆數',
            'log_info': '記錄清理資訊日誌',
            'log_debug': '記錄清理除錯日誌'
        }
        return descriptions.get(key, '未知配置參數')
