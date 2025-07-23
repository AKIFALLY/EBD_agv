"""
任務狀態常數定義
與資料庫 13_works_tasks.py 中的 default_task_status 保持同步
"""


class TaskStatus:
    """任務狀態 ID 常數定義"""
    
    # 基本任務流程狀態
    REQUESTING = 0    # 請求中 - UI-請求執行任務
    PENDING = 1       # 待處理 - WCS-任務已接受，待處理
    READY = 2         # 待執行 - RCS-任務已派發，待執行
    EXECUTING = 3     # 執行中 - AGV-任務正在執行
    COMPLETED = 4     # 已完成 - AGV-任務已完成
    
    # 取消相關狀態
    CANCELLING = 5        # 取消中 - 任務取消
    WCS_CANCELLING = 51   # WCS-取消中 - WCS-任務取消中，待處理
    RCS_CANCELLING = 52   # RCS-取消中 - RCS-任務取消中，取消中
    AGV_CANCELLING = 53   # AGV-取消中 - AGV-取消完成
    CANCELLED = 54        # 已取消 - 任務已取消
    
    # 錯誤狀態
    ERROR = 6         # 錯誤


class TaskStatusInfo:
    """任務狀態資訊映射"""
    
    STATUS_MAP = {
        TaskStatus.REQUESTING: {
            'name': '請求中',
            'description': 'UI-請求執行任務',
            'color': 'is-info'
        },
        TaskStatus.PENDING: {
            'name': '待處理',
            'description': 'WCS-任務已接受，待處理',
            'color': 'is-warning'
        },
        TaskStatus.READY: {
            'name': '待執行',
            'description': 'RCS-任務已派發，待執行',
            'color': 'is-warning'
        },
        TaskStatus.EXECUTING: {
            'name': '執行中',
            'description': 'AGV-任務正在執行',
            'color': 'is-info'
        },
        TaskStatus.COMPLETED: {
            'name': '已完成',
            'description': 'AGV-任務已完成',
            'color': 'is-success'
        },
        TaskStatus.CANCELLING: {
            'name': '取消中',
            'description': '任務取消',
            'color': 'is-warning'
        },
        TaskStatus.WCS_CANCELLING: {
            'name': 'WCS-取消中',
            'description': 'WCS-任務取消中，待處理',
            'color': 'is-warning'
        },
        TaskStatus.RCS_CANCELLING: {
            'name': 'RCS-取消中',
            'description': 'RCS-任務取消中，取消中',
            'color': 'is-warning'
        },
        TaskStatus.AGV_CANCELLING: {
            'name': 'AGV-取消中',
            'description': 'AGV-取消完成',
            'color': 'is-warning'
        },
        TaskStatus.CANCELLED: {
            'name': '已取消',
            'description': '任務已取消',
            'color': 'is-danger'
        },
        TaskStatus.ERROR: {
            'name': '錯誤',
            'description': '錯誤',
            'color': 'is-danger'
        }
    }
    
    @classmethod
    def get_status_info(cls, status_id):
        """根據狀態 ID 獲取狀態資訊"""
        return cls.STATUS_MAP.get(status_id, {
            'name': f'未知({status_id})',
            'description': '未知狀態',
            'color': 'is-light'
        })
    
    @classmethod
    def get_status_name(cls, status_id):
        """根據狀態 ID 獲取狀態名稱"""
        return cls.get_status_info(status_id)['name']
    
    @classmethod
    def is_active_status(cls, status_id):
        """檢查狀態是否為活躍狀態（未完成且未取消）"""
        return status_id in [
            TaskStatus.REQUESTING,
            TaskStatus.PENDING,
            TaskStatus.READY,
            TaskStatus.EXECUTING
        ]
    
    @classmethod
    def is_cancelling_status(cls, status_id):
        """檢查狀態是否為取消相關狀態"""
        return status_id in [
            TaskStatus.CANCELLING,
            TaskStatus.WCS_CANCELLING,
            TaskStatus.RCS_CANCELLING,
            TaskStatus.AGV_CANCELLING,
            TaskStatus.CANCELLED
        ]
    
    @classmethod
    def is_completed_status(cls, status_id):
        """檢查狀態是否為已完成"""
        return status_id == TaskStatus.COMPLETED
    
    @classmethod
    def is_error_status(cls, status_id):
        """檢查狀態是否為錯誤狀態"""
        return status_id == TaskStatus.ERROR


# 為了向後兼容，提供一些常用的狀態檢查函數
def is_task_completed(status_id):
    """檢查任務是否已完成"""
    return TaskStatusInfo.is_completed_status(status_id)


def is_task_active(status_id):
    """檢查任務是否為活躍狀態"""
    return TaskStatusInfo.is_active_status(status_id)


def is_task_cancelling(status_id):
    """檢查任務是否為取消相關狀態"""
    return TaskStatusInfo.is_cancelling_status(status_id)


def get_task_status_name(status_id):
    """獲取任務狀態名稱"""
    return TaskStatusInfo.get_status_name(status_id)
