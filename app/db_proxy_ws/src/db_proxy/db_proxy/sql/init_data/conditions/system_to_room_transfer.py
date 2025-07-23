"""
第二組：系統準備區料架送往房間傳送箱流程
包含從系統準備區將料架送往各房間入口傳送箱的完整業務邏輯

業務流程說明：
1. 檢查系統準備區是否有料架（條件 4）- 起始條件，確保有料架可以送往房間
2. 檢查各房間入口傳送箱是否無料架佔用（條件 210, 310, 410...）- 確保目標位置空閒
3. 檢查是否有重複的執行中任務（條件 211, 311, 411...）- 防止重複任務產生

條件關聯性：
- 條件 4 → 檢查系統準備區料架可用性 → next_id 指向房間檢查列表 [110,210,310...]
- 條件 210-1010 → 檢查各房間入口傳送箱的佔用狀態
- 條件 211-1011 → 防止各房間重複任務的產生

技術細節：
- 使用 location 表檢查系統準備區的料架狀態（location_status_id = 3）
- 使用 rack 表檢查房間入口傳送箱是否有料架佔用
- 使用 task 表檢查是否有執行中的重複任務
- 支援房間1-10的擴展（ID範圍：110-1010, 111-1011）

房間編號對應：
- 房間1：條件 110, 111（預留）
- 房間2：條件 210, 211
- 房間3：條件 310, 311（預留）
- ...以此類推到房間10
"""

def get_system_to_room_transfer_conditions():
    """
    取得系統準備區料架送往房間傳送箱流程的所有條件
    
    Returns:
        list: 包含該流程所有條件的列表
    """
    return [
        # 條件 4：系統準備區料架送往房間傳送箱入口 - 檢查系統送料區是否有料架
        # 功能：檢查系統準備區是否有料架可以送往房間傳送箱
        # 邏輯：查詢 location 表中 ID 為 11-18 且狀態為 3（有料架）的位置
        # 返回：result（True/False）、next_id（房間檢查列表）、location（最小位置ID）
        {
            "id": 4,
            "conditions": "SELECT \
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN 'True'\
                                        ELSE 'False'\
                                    END as result,\
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN '[110,210,310,410,510,610,710,810,910,1010]'\
                                        ELSE NULL\
                                    END as next_id,\
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN MIN(id)\
                                        ELSE NULL\
                                    END as location\
                                    FROM location \
                                    WHERE id = ANY (ARRAY[11,12,13,14,15,16,17,18]) AND location_status_id = 3",
            "results": {},
            "description": "系統準備區料架送往房間傳送箱入口 - 檢查系統送料區是否有料架"
        },

        # 條件 210：房間2入口傳送箱料架檢查
        # 功能：檢查房間2入口傳送箱是否無料架佔用
        # 邏輯：查詢 rack 表中 location_id = 20011 的記錄，確保位置空閒
        # 返回：result（True/False）、next_id（下一個檢查條件）、room_id（房間號）
        {
            "id": 210,
            "conditions": "SELECT \
                                    CASE \
                                        WHEN COUNT(*) = 0 THEN 'True'\
                                        ELSE 'False'\
                                    END as result,\
                                    CASE \
                                        WHEN COUNT(*) = 0 THEN 211\
                                        ELSE NULL\
                                    END as next_id,\
                                    CASE \
                                        WHEN COUNT(*) = 0 THEN 2\
                                        ELSE NULL\
                                    END as room_id,\
                                    20002 as location_id\
                                    FROM rack \
                                    WHERE location_id = 20011",
            "results": {},
            "description": "房間2入口傳送箱料架檢查 - 檢查是否已有料架佔用"
        },

        # 條件 211：檢查是否有相同執行的任務
        # 功能：檢查是否有相同的執行中任務，防止重複任務產生
        # 邏輯：查詢 task 表中是否有相同 work_id 和 node_id 的執行中任務
        # 返回：result（True/False）、next_id（無）、end（結束標記）
        {
            "id": 211,
            "conditions": "SELECT \
                                    CASE \
                                        WHEN COUNT(*) = 0 THEN 'True'\
                                        ELSE 'False'\
                                    END as result,\
                                    NULL as next_id,\
                                    CASE \
                                        WHEN COUNT(*) = 0 THEN 'True'\
                                        ELSE 'False'\
                                    END as end\
                                    FROM task \
                                    WHERE work_id = '220001' \
                                    AND node_id = 20001",
            "results": {},
            "description": "檢查是否有相同執行的任務 - 防止重複任務"
        }
    ]


def get_system_to_room_extended_conditions():
    """
    取得其他房間的系統傳送條件（預留擴展用）
    
    此函數可用於未來擴展房間3-10的條件
    目前返回空列表，可根據需要添加對應條件
    
    Returns:
        list: 其他房間的條件定義列表（目前為空）
    """
    # 預留給房間3-10的條件擴展
    # 格式：條件 310, 311（房間3）、410, 411（房間4）...以此類推
    return []
