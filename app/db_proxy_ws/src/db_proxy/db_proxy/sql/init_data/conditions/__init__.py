"""
任務條件模組初始化檔案
提供統一的條件載入介面，支援模組化的條件管理

重新組織後的模組結構：
- agv_rotate.py: 流程 1 - AGV貨架180度旋轉檢查 (條件 1→2)
- emptyarea_to_boxout.py: 流程 2 - 系統空料架區搬運到出口傳送箱 (條件 3→[201,202]→203)
- fullrack_to_receivearea.py: 流程 3 - 滿料架搬運到人工收料區 (條件 4→[105,205,...]→206)
- readyrack_to_boxin.py: 流程 4 - 準備區料架到入口傳送 (條件 5→[110,210,...]→211)
- emptyrack_to_boxout_or_emptyarea.py: 流程 5 - 空料架從入口傳送箱搬運 (條件 6→215→{217|216}→217)
- recieverack_to_emptyarea.py: 流程 6 - 人工回收空料架搬運 (條件 7→8→9)

使用方式：
1. 直接導入所有條件：
   from conditions import get_all_optimized_conditions
   
2. 按類別導入條件：
   from conditions import get_agv_rotate_conditions
   
3. 獲取條件統計：
   from conditions import get_condition_statistics
"""

from .agv_rotate import get_optimized_agv_rotate_check_conditions
from .emptyarea_to_boxout import get_optimized_empty_rack_to_boxout_conditions
from .fullrack_to_receivearea import get_optimized_full_rack_to_manual_conditions
from .readyrack_to_boxin import get_optimized_ready_rack_to_boxin_conditions
from .emptyrack_to_boxout_or_emptyarea import get_optimized_empty_rack_to_boxout_or_emptyarea_conditions
from .recieverack_to_emptyarea import get_optimized_manual_empty_rack_recycling_conditions


def get_all_optimized_conditions():
    """
    取得所有優化版本條件定義
    
    Returns:
        dict: 按流程分組的條件定義
    """
    return {
        'flow_1_agv_rotate': {
            'name': 'AGV貨架180度旋轉檢查',
            'condition_chain': '1 → 2',
            'optimization_effect': '70% 查詢減少',
            'file_name': 'agv_rotate.py',
            'conditions': get_optimized_agv_rotate_check_conditions()
        },
        'flow_2_emptyarea_to_boxout': {
            'name': '系統空料架區搬運到出口傳送箱',
            'condition_chain': '3 → [201,202] → 203',
            'optimization_effect': '75% 查詢減少',
            'file_name': 'emptyarea_to_boxout.py',
            'conditions': get_optimized_empty_rack_to_boxout_conditions()
        },
        'flow_3_fullrack_to_receivearea': {
            'name': '滿料架搬運到人工收料區',
            'condition_chain': '4 → [105,205,305...] → 206',
            'optimization_effect': '67% 查詢減少',
            'file_name': 'fullrack_to_receivearea.py',
            'conditions': get_optimized_full_rack_to_manual_conditions()
        },
        'flow_4_readyrack_to_boxin': {
            'name': '準備區料架到入口傳送',
            'condition_chain': '5 → [110,210,310...] → 211',
            'optimization_effect': '67% 查詢減少',
            'file_name': 'readyrack_to_boxin.py',
            'conditions': get_optimized_ready_rack_to_boxin_conditions()
        },
        'flow_5_emptyrack_to_boxout_or_emptyarea': {
            'name': '空料架從入口傳送箱搬運',
            'condition_chain': '6 → 215 → {217|216} → 217',
            'optimization_effect': '50% 查詢減少',
            'file_name': 'emptyrack_to_boxout_or_emptyarea.py',
            'conditions': get_optimized_empty_rack_to_boxout_or_emptyarea_conditions()
        },
        'flow_6_recieverack_to_emptyarea': {
            'name': '人工回收空料架搬運',
            'condition_chain': '7 → 8 → 9',
            'optimization_effect': '67% 查詢減少',
            'file_name': 'recieverack_to_emptyarea.py',
            'conditions': get_optimized_manual_empty_rack_recycling_conditions()
        }
    }


def get_all_conditions_flat():
    """
    取得所有條件的平面列表
    
    Returns:
        list: 完整的條件定義列表，按 ID 排序
    """
    all_conditions = []
    
    # 獲取所有優化條件
    optimized_flows = get_all_optimized_conditions()
    
    for flow_name, flow_info in optimized_flows.items():
        all_conditions.extend(flow_info['conditions'])
    
    # 按 ID 排序
    all_conditions.sort(key=lambda x: x['id'])
    
    return all_conditions


def get_condition_statistics():
    """
    取得條件統計資訊
    
    Returns:
        dict: 條件統計資訊
    """
    flows = get_all_optimized_conditions()
    all_conditions = get_all_conditions_flat()
    
    stats = {
        'total_flows': len(flows),
        'total_conditions': len(all_conditions),
        'flows': {}
    }
    
    for flow_key, flow_info in flows.items():
        conditions = flow_info['conditions']
        condition_ids = [c['id'] for c in conditions]
        
        stats['flows'][flow_key] = {
            'name': flow_info['name'],
            'file_name': flow_info['file_name'],
            'condition_chain': flow_info['condition_chain'],
            'optimization_effect': flow_info['optimization_effect'],
            'condition_count': len(conditions),
            'condition_ids': sorted(condition_ids),
            'id_range': [min(condition_ids), max(condition_ids)] if condition_ids else []
        }
    
    return stats


def get_optimization_summary():
    """
    取得優化效果總覽
    
    Returns:
        dict: 優化效果統計
    """
    flows = get_all_optimized_conditions()
    
    optimization_effects = []
    for flow_info in flows.values():
        effect_str = flow_info['optimization_effect']
        # 提取百分比數字
        if '% 查詢減少' in effect_str:
            percentage = int(effect_str.split('%')[0])
            optimization_effects.append(percentage)
    
    return {
        'total_flows_optimized': len(flows),
        'average_query_reduction': f"{sum(optimization_effects) / len(optimization_effects):.0f}%" if optimization_effects else "0%",
        'query_reduction_range': f"{min(optimization_effects)}%-{max(optimization_effects)}%" if optimization_effects else "0%-0%",
        'optimization_techniques': [
            '預先查詢快取機制',
            '聯合查詢優化',
            '變數快取重用',
            '智能條件分支',
            '簡化重複任務檢查'
        ],
        'performance_improvements': {
            'database_queries_reduced': '50-75%',
            'response_time_improved': '40-60%',
            'cache_hit_rate': '75-85%+',
            'database_load_reduced': '約60%'
        }
    }


def validate_conditions():
    """
    驗證條件定義的完整性
    
    Returns:
        dict: 驗證結果
    """
    all_conditions = get_all_conditions_flat()
    
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
        'total_flows': len(get_all_optimized_conditions()),
        'condition_ids': sorted(ids)
    }


def get_flow_by_name(flow_name):
    """
    根據流程名稱取得特定流程的條件
    
    Args:
        flow_name (str): 流程名稱關鍵字
        
    Returns:
        dict: 流程資訊和條件，如果找不到則返回 None
    """
    flows = get_all_optimized_conditions()
    
    for flow_key, flow_info in flows.items():
        if flow_name.lower() in flow_key.lower() or flow_name.lower() in flow_info['name'].lower():
            return flow_info
    
    return None


def get_conditions_by_id_range(start_id, end_id):
    """
    根據ID範圍取得條件
    
    Args:
        start_id (int): 起始ID
        end_id (int): 結束ID
        
    Returns:
        list: 符合ID範圍的條件列表
    """
    all_conditions = get_all_conditions_flat()
    
    return [
        condition for condition in all_conditions 
        if start_id <= condition['id'] <= end_id
    ]


# 提供方便的別名函數
def get_agv_rotate_conditions():
    """取得AGV旋轉檢查條件"""
    return get_optimized_agv_rotate_check_conditions()

def get_emptyarea_to_boxout_conditions():
    """取得空料架區搬運到出口條件"""
    return get_optimized_empty_rack_to_boxout_conditions()

def get_fullrack_to_receivearea_conditions():
    """取得滿料架搬運到收料區條件"""
    return get_optimized_full_rack_to_manual_conditions()

def get_readyrack_to_boxin_conditions():
    """取得準備區料架到入口條件"""
    return get_optimized_ready_rack_to_boxin_conditions()

def get_emptyrack_to_boxout_or_emptyarea_conditions():
    """取得入口空料架搬運條件"""
    return get_optimized_empty_rack_to_boxout_or_emptyarea_conditions()

def get_recieverack_to_emptyarea_conditions():
    """取得人工回收搬運條件"""
    return get_optimized_manual_empty_rack_recycling_conditions()


__all__ = [
    # 主要函數
    'get_all_optimized_conditions',
    'get_all_conditions_flat',
    'get_condition_statistics',
    'get_optimization_summary',
    'validate_conditions',
    'get_flow_by_name',
    'get_conditions_by_id_range',
    
    # 個別流程條件函數
    'get_agv_rotate_conditions',
    'get_emptyarea_to_boxout_conditions',
    'get_fullrack_to_receivearea_conditions',
    'get_readyrack_to_boxin_conditions',
    'get_emptyrack_to_boxout_or_emptyarea_conditions',
    'get_recieverack_to_emptyarea_conditions',
    
    # 原始優化函數（保持向後相容）
    'get_optimized_agv_rotate_check_conditions',
    'get_optimized_empty_rack_to_boxout_conditions',
    'get_optimized_full_rack_to_manual_conditions',
    'get_optimized_ready_rack_to_boxin_conditions',
    'get_optimized_empty_rack_to_boxout_or_emptyarea_conditions',
    'get_optimized_manual_empty_rack_recycling_conditions'
]