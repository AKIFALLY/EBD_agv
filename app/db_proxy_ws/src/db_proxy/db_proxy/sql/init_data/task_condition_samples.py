"""
任務條件範例資料
此檔案包含用於初始化 task_condition 表的範例資料

檔案已重構為模組化架構，但保持向後相容性。
條件定義已拆分到 conditions/ 目錄下的各個模組中：

模組結構：
- conditions/full_rack_to_manual.py: 第一組 - 滿料架到人工收料區流程
- conditions/system_to_room_transfer.py: 第二組 - 系統準備區料架送往房間傳送箱流程
- conditions/empty_rack_transfer.py: 第三組 - 入口傳送箱空料架搬運到出口傳送箱流程
- conditions/ng_rack_recycling.py: 第四組 - 入口傳送箱NG料架搬運到NG回收區流程
- conditions/agv_rotation_check.py: 第五組 - AGV旋轉狀態檢查流程

優勢：
1. 模組化管理：每個業務流程獨立檔案，便於維護
2. 快速定位：根據業務需求快速找到對應檔案
3. 擴展性強：新增條件時只需修改對應模組
4. 向後相容：現有程式碼無需修改

使用方式：
- 直接使用：from task_condition_samples import sample_conditions
- 模組化使用：from conditions import get_all_conditions
"""

# 導入模組化條件定義
try:
    from .conditions import get_all_conditions
    
    # 使用模組化載入所有條件
    sample_conditions = get_all_conditions()

except ImportError as e:
    # 如果模組載入失敗，使用原始的條件定義（向後相容）
    print(f"警告：無法載入模組化條件定義: {e}")
    print("請檢查 conditions/ 目錄是否存在且模組正確")
    sample_conditions = []


# 提供向後相容的工具函數
def get_conditions_by_group(group_name):
    """根據群組名稱取得條件"""
    try:
        from .conditions import get_conditions_by_category
        if group_name in get_conditions_by_category:
            return get_conditions_by_category[group_name]()
        else:
            raise ValueError(f"未知的群組名稱: {group_name}")
    except ImportError:
        return []


def get_condition_by_id(condition_id):
    """根據 ID 取得特定條件"""
    for condition in sample_conditions:
        if condition['id'] == condition_id:
            return condition
    return None


def get_conditions_by_id_range(start_id, end_id):
    """根據 ID 範圍取得條件"""
    return [
        condition for condition in sample_conditions
        if start_id <= condition['id'] <= end_id
    ]


def get_condition_statistics():
    """取得條件統計資訊"""
    try:
        from .conditions import get_condition_statistics as _get_stats
        return _get_stats()
    except ImportError:
        return {
            'total_conditions': len(sample_conditions),
            'groups': {},
            'error': '模組化條件載入失敗'
        }


def validate_conditions():
    """驗證條件完整性"""
    try:
        from .conditions import validate_conditions as _validate
        return _validate()
    except ImportError:
        return {
            'is_valid': False,
            'error': '模組化條件載入失敗',
            'total_conditions': len(sample_conditions)
        }
