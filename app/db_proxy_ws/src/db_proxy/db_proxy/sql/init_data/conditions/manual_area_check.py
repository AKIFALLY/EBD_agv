"""
人工收料區搬運到房間入口傳送箱流程
包含從人工收料區將滿料架搬運到各房間入口傳送箱的完整業務邏輯

業務流程說明：
1. 檢查人工收料區是否有空位（條件 2）- 目標位置檢查，確保有地方放置滿料架
2. 檢查房間出口傳送箱是否有滿料架（條件 205）- 確認有滿料架需要搬運
3. 檢查是否有已完成的cargo任務（條件 206）- 確認任務完成狀態
4. 檢查是否有重複的執行中任務（條件 207）- 防止重複任務產生

條件關聯性：
- 條件 2 → 檢查人工收料區空位可用性 → next_id 指向房間檢查列表 [105,205,305,405...]
- 條件 205 → 檢查房間2出口傳送箱滿料架狀態 → 有料架跳207，無料架跳206
- 條件 206 → 檢查已完成的cargo任務 → next_id 指向條件 207
- 條件 207 → 最終的重複任務檢查 → end='True'（流程結束）

技術細節：
- 使用 location 表檢查人工收料區的空位狀態（location_status_id = 2）
- 使用 rack 表檢查出口傳送箱的滿料架狀態（status_id IN (2, 3, 6)）
- 使用 task 表檢查已完成的cargo任務（status_id = 4）

房間位置對應：
- 人工收料區：位置 [51,52,53,54,55]
- 房間2出口傳送箱：location_id = 20002
- 房間3出口傳送箱：location_id = 30002（預留擴展）
- ...以此類推到房間10

房間編號對應：
- 房間1：條件 105, 106（預留）
- 房間2：條件 205, 206
- 房間3：條件 305, 306（預留）
- ...以此類推到房間10
"""

def get_manual_area_check_conditions():
    """
    取得人工收料區搬運到房間入口傳送箱流程的所有條件
    
    Returns:
        list: 包含該流程所有條件的列表
    """
    return [
        # 條件 2：人工收料區-有空位（目標位置檢查）
        # 功能：檢查人工收料區是否有空位可以放置滿料架
        # 邏輯：查詢 location 表中 ID 為 51-55 且狀態為 2（空位）的位置
        # 返回：result（True/False）、next_id（房間檢查列表）、location（最小位置ID）
        {
            "id": 2,
            "conditions": "SELECT \
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN 'True'\
                                        ELSE 'False'\
                                    END as result,\
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN '[105,205,305,405,505,605,705,805,905,1005]'\
                                        ELSE NULL\
                                    END as next_id,\
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN MIN(id)\
                                        ELSE NULL\
                                    END as location\
                                    FROM location \
                                    WHERE id = ANY (ARRAY[51,52,53,54,55]) AND location_status_id = 2",
            "results": {},
            "description": "人工收料區-有空位"
        },

        # 條件 205：房間2-出口傳送箱有滿料架檢查
        # 功能：檢查房間2出口傳送箱是否有滿料架需要搬運到人工收料區
        # 邏輯：查詢 rack 表中 location_id = 20002 且狀態為滿料架的記錄
        # 返回：result（True/False）、next_id（條件分支）、rack_id（料架ID）、room_id（房間號）
        # 入口：來自條件 2 的 next_id 列表
        {
            "id": 205,
            "conditions": "SELECT \
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN 'True'\
                                        ELSE 'False'\
                                    END as result,\
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN 207\
                                        ELSE 206\
                                    END as next_id,\
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN MIN(r.id)\
                                        ELSE NULL\
                                    END as rack_id,\
                                    2 as room_id\
                                    FROM rack r \
                                    WHERE r.location_id = 20002 AND r.status_id IN (2, 3, 6)",
            "results": {},
            "description": "房間2-出口傳送箱有滿料架"
        },

        # 條件 206：房間2-出口傳送箱有已完成的cargo任務檢查
        # 功能：檢查房間2出口傳送箱是否有已完成的cargo任務
        # 邏輯：查詢 task 表中 work_id = 2000201 且狀態為已完成的任務
        # 返回：result（True/False）、next_id（下一個條件）、room_id（房間號）、cargo_work_id（任務ID）
        # 入口：來自條件 205（當沒有滿料架時）
        {
            "id": 206,
            "conditions": "SELECT \
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN 'True'\
                                        ELSE 'False'\
                                    END as result,\
                                    207 as next_id,\
                                    2 as room_id,\
                                    2000201 as cargo_work_id\
                                    FROM task \
                                    WHERE work_id = 2000201 AND status_id = 4",
            "results": {},
            "description": "房間2-出口傳送箱有已完成的cargo任務"
        },

        # 條件 207：房間2-檢查重複任務（人工收料區搬運流程最終確認）
        # 功能：檢查是否已存在相同的執行中任務，防止重複任務產生
        # 邏輯：查詢 task 表中是否有相同 work_id 和 node_id 的執行中任務
        # 返回：result（True/False）、next_id（無）、room_id（房間號）、end（結束標記）
        # 入口：來自條件 205（有滿料架時）或條件 206（cargo任務檢查後）
        {
            "id": 207,
            "conditions": "SELECT \
                                    CASE \
                                        WHEN COUNT(*) = 0 THEN 'True'\
                                        ELSE 'False'\
                                    END as result,\
                                    NULL as next_id,\
                                    2 as room_id,\
                                    CASE \
                                        WHEN COUNT(*) = 0 THEN 'True'\
                                        ELSE 'False'\
                                    END as end\
                                    FROM task \
                                    WHERE work_id = '220001' AND node_id = 20002 AND status_id IN (0, 1, 2)",
            "results": {},
            "description": "房間2-檢查重複任務（人工收料區搬運流程最終確認）"
        }
    ]


def get_manual_area_extended_conditions():
    """
    取得其他房間的人工收料區搬運條件（預留擴展用）
    
    此函數可用於未來擴展房間3-10的條件
    目前返回空列表，可根據需要添加對應條件
    
    Returns:
        list: 其他房間的條件定義列表（目前為空）
    """
    # 預留給房間3-10的條件擴展
    # 格式：條件 305, 306（房間3）、405, 406（房間4）...以此類推
    return []
