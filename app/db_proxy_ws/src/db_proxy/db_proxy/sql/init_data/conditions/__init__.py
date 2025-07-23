"""
任務條件模組初始化檔案
提供統一的條件載入介面，支援模組化的條件管理

模組結構：
- full_rack_to_manual.py: 第一組 - 滿料架到人工收料區流程（起始檢查和最終確認）
- manual_area_check.py: 第二組 - 人工收料區搬運到房間入口傳送箱流程
- system_to_room_transfer.py: 第三組 - 系統準備區料架送往房間傳送箱流程
- empty_rack_transfer.py: 第四組 - 入口傳送箱空料架搬運到出口傳送箱流程
- ng_rack_recycling.py: 第五組 - 入口傳送箱NG料架搬運到NG回收區流程
- agv_rotation_check.py: 第六組 - AGV旋轉狀態檢查流程
- manual_empty_rack_recycling.py: 第七組 - 人工回收空料架搬運到系統空料架區流程

使用方式：
1. 直接導入所有條件：
   from conditions import get_all_conditions
   
2. 按類別導入條件：
   from conditions import get_full_rack_to_manual_conditions
   
3. 獲取條件統計：
   from conditions import get_condition_statistics
"""

from .full_rack_to_manual import get_full_rack_to_manual_conditions
from .manual_area_check import (
    get_manual_area_check_conditions,
    get_manual_area_extended_conditions
)
from .system_to_room_transfer import (
    get_system_to_room_transfer_conditions,
    get_system_to_room_extended_conditions
)
from .empty_rack_transfer import (
    get_empty_rack_transfer_conditions,
    get_empty_rack_extended_conditions
)
from .ng_rack_recycling import (
    get_ng_rack_recycling_conditions,
    get_ng_rack_extended_conditions
)
from .agv_rotation_check import (
    get_agv_rotation_check_conditions,
    get_agv_rotation_extended_conditions
)
from .manual_empty_rack_recycling import (
    get_manual_empty_rack_recycling_conditions,
    get_manual_empty_rack_extended_conditions
)


def get_all_conditions():
    """
    取得所有條件定義
    
    Returns:
        list: 完整的條件定義列表，按 ID 排序
    """
    all_conditions = []
    
    # 第一組：滿料架到人工收料區流程（起始檢查和最終確認）
    all_conditions.extend(get_full_rack_to_manual_conditions())

    # 第二組：人工收料區搬運到房間入口傳送箱流程
    all_conditions.extend(get_manual_area_check_conditions())
    all_conditions.extend(get_manual_area_extended_conditions())

    # 第三組：系統準備區料架送往房間傳送箱流程
    all_conditions.extend(get_system_to_room_transfer_conditions())
    all_conditions.extend(get_system_to_room_extended_conditions())

    # 第四組：入口傳送箱空料架搬運到出口傳送箱流程
    all_conditions.extend(get_empty_rack_transfer_conditions())
    all_conditions.extend(get_empty_rack_extended_conditions())

    # 第五組：入口傳送箱NG料架搬運到NG回收區流程
    all_conditions.extend(get_ng_rack_recycling_conditions())
    all_conditions.extend(get_ng_rack_extended_conditions())

    # 第六組：AGV旋轉狀態檢查流程
    all_conditions.extend(get_agv_rotation_check_conditions())
    all_conditions.extend(get_agv_rotation_extended_conditions())

    # 第七組：人工回收空料架搬運到系統空料架區流程
    all_conditions.extend(get_manual_empty_rack_recycling_conditions())
    all_conditions.extend(get_manual_empty_rack_extended_conditions())
    
    # 按 ID 排序
    all_conditions.sort(key=lambda x: x['id'])
    
    return all_conditions


def get_condition_statistics():
    """
    取得條件統計資訊
    
    Returns:
        dict: 條件統計資訊
    """
    stats = {
        'total_conditions': 0,
        'groups': {
            'full_rack_to_manual': {
                'name': '滿料架到人工收料區流程（起始檢查和最終確認）',
                'conditions': get_full_rack_to_manual_conditions(),
                'count': 0,
                'id_range': []
            },
            'manual_area_check': {
                'name': '人工收料區搬運到房間入口傳送箱流程',
                'conditions': get_manual_area_check_conditions() + get_manual_area_extended_conditions(),
                'count': 0,
                'id_range': []
            },
            'system_to_room_transfer': {
                'name': '系統準備區料架送往房間傳送箱流程',
                'conditions': get_system_to_room_transfer_conditions() + get_system_to_room_extended_conditions(),
                'count': 0,
                'id_range': []
            },
            'empty_rack_transfer': {
                'name': '入口傳送箱空料架搬運到出口傳送箱流程',
                'conditions': get_empty_rack_transfer_conditions() + get_empty_rack_extended_conditions(),
                'count': 0,
                'id_range': []
            },
            'ng_rack_recycling': {
                'name': '入口傳送箱NG料架搬運到NG回收區流程',
                'conditions': get_ng_rack_recycling_conditions() + get_ng_rack_extended_conditions(),
                'count': 0,
                'id_range': []
            },
            'agv_rotation_check': {
                'name': 'AGV旋轉狀態檢查流程',
                'conditions': get_agv_rotation_check_conditions() + get_agv_rotation_extended_conditions(),
                'count': 0,
                'id_range': []
            },
            'manual_empty_rack_recycling': {
                'name': '人工回收空料架搬運到系統空料架區流程',
                'conditions': get_manual_empty_rack_recycling_conditions() + get_manual_empty_rack_extended_conditions(),
                'count': 0,
                'id_range': []
            }
        }
    }
    
    # 計算統計資訊
    for group_name, group_info in stats['groups'].items():
        conditions = group_info['conditions']
        group_info['count'] = len(conditions)
        if conditions:
            ids = [c['id'] for c in conditions]
            group_info['id_range'] = [min(ids), max(ids)]
        stats['total_conditions'] += group_info['count']
        # 移除 conditions 欄位以減少返回資料大小
        del group_info['conditions']
    
    return stats


def validate_conditions():
    """
    驗證條件定義的完整性
    
    Returns:
        dict: 驗證結果
    """
    all_conditions = get_all_conditions()
    
    # 檢查 ID 重複
    ids = [c['id'] for c in all_conditions]
    duplicate_ids = [id for id in set(ids) if ids.count(id) > 1]
    
    # 檢查必要欄位
    missing_fields = []
    for condition in all_conditions:
        required_fields = ['id', 'conditions', 'results', 'description']
        for field in required_fields:
            if field not in condition:
                missing_fields.append(f"條件 {condition.get('id', 'unknown')} 缺少欄位: {field}")
    
    return {
        'is_valid': len(duplicate_ids) == 0 and len(missing_fields) == 0,
        'duplicate_ids': duplicate_ids,
        'missing_fields': missing_fields,
        'total_conditions': len(all_conditions),
        'condition_ids': sorted(ids)
    }


# 提供向後相容的別名
get_conditions_by_category = {
    'full_rack_to_manual': get_full_rack_to_manual_conditions,
    'manual_area_check': lambda: get_manual_area_check_conditions() + get_manual_area_extended_conditions(),
    'system_to_room_transfer': lambda: get_system_to_room_transfer_conditions() + get_system_to_room_extended_conditions(),
    'empty_rack_transfer': lambda: get_empty_rack_transfer_conditions() + get_empty_rack_extended_conditions(),
    'ng_rack_recycling': lambda: get_ng_rack_recycling_conditions() + get_ng_rack_extended_conditions(),
    'agv_rotation_check': lambda: get_agv_rotation_check_conditions() + get_agv_rotation_extended_conditions(),
    'manual_empty_rack_recycling': lambda: get_manual_empty_rack_recycling_conditions() + get_manual_empty_rack_extended_conditions()
}


__all__ = [
    'get_all_conditions',
    'get_condition_statistics',
    'validate_conditions',
    'get_conditions_by_category',
    'get_full_rack_to_manual_conditions',
    'get_manual_area_check_conditions',
    'get_system_to_room_transfer_conditions',
    'get_empty_rack_transfer_conditions',
    'get_ng_rack_recycling_conditions',
    'get_agv_rotation_check_conditions',
    'get_manual_empty_rack_recycling_conditions'
]
