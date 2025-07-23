"""
停車格狀態常數定義
"""


class ParkingStatus:
    """停車格狀態 ID 常數定義"""
    
    AVAILABLE = 0     # 可用 - 停車格空閒，可以叫車
    TASK_ACTIVE = 1   # 任務進行中 - 已叫車，等待AGV送達
    TASK_COMPLETED = 2  # 任務完成 - 車輛已送達，等待確認取貨


class ParkingStatusInfo:
    """停車格狀態資訊映射"""
    
    STATUS_MAP = {
        ParkingStatus.AVAILABLE: {
            'name': '可用',
            'description': '停車格空閒，可以叫車',
            'color': 'is-light'
        },
        ParkingStatus.TASK_ACTIVE: {
            'name': '任務進行中',
            'description': '已叫車，等待AGV送達',
            'color': 'is-warning'
        },
        ParkingStatus.TASK_COMPLETED: {
            'name': '任務完成',
            'description': '車輛已送達，等待確認取貨',
            'color': 'is-success'
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
    def is_available(cls, status_id):
        """檢查停車格是否可用"""
        return status_id == ParkingStatus.AVAILABLE
    
    @classmethod
    def is_task_active(cls, status_id):
        """檢查停車格是否有進行中的任務"""
        return status_id == ParkingStatus.TASK_ACTIVE
    
    @classmethod
    def is_task_completed(cls, status_id):
        """檢查停車格任務是否已完成"""
        return status_id == ParkingStatus.TASK_COMPLETED


# 為了向後兼容，提供一些常用的狀態檢查函數
def is_parking_available(status_id):
    """檢查停車格是否可用"""
    return ParkingStatusInfo.is_available(status_id)


def is_parking_task_active(status_id):
    """檢查停車格是否有進行中的任務"""
    return ParkingStatusInfo.is_task_active(status_id)


def is_parking_task_completed(status_id):
    """檢查停車格任務是否已完成"""
    return ParkingStatusInfo.is_task_completed(status_id)


def get_parking_status_name(status_id):
    """獲取停車格狀態名稱"""
    return ParkingStatusInfo.get_status_name(status_id)
