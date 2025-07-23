"""
第三組：入口傳送箱空料架搬運到出口傳送箱流程
包含將空料架從房間入口傳送箱搬運到出口傳送箱的完整業務邏輯

業務流程說明：
1. 檢查入口傳送箱是否有空料架（條件 5）- 起始條件，確保有空料架需要搬運
2. 檢查各房間出口傳送箱是否無料架佔用（條件 215, 315, 415...）- 確保目標位置空閒
3. 檢查是否有重複的執行中任務（條件 216, 316, 416...）- 防止重複任務產生

條件關聯性：
- 條件 5 → 檢查各房間入口傳送箱的空料架狀態 → next_id 指向房間檢查列表 [115,215,315...]
- 條件 215-1015 → 檢查各房間出口傳送箱的佔用狀態
- 條件 216-1016 → 防止各房間重複任務的產生

技術細節：
- 使用 rack 表檢查入口傳送箱的空料架狀態（status_id = 1）
- 使用 rack 表檢查出口傳送箱是否有料架佔用
- 使用 task 表檢查是否有執行中的重複任務
- 支援房間1-10的擴展（ID範圍：115-1015, 116-1016）

房間位置對應：
- 房間1：入口 10001，出口 10002
- 房間2：入口 20001，出口 20002
- 房間3：入口 30001，出口 30002
- ...以此類推到房間10

房間編號對應：
- 房間1：條件 115, 116（預留）
- 房間2：條件 215, 216
- 房間3：條件 315, 316（預留）
- ...以此類推到房間10
"""

def get_empty_rack_transfer_conditions():
    """
    取得入口傳送箱空料架搬運到出口傳送箱流程的所有條件
    
    Returns:
        list: 包含該流程所有條件的列表
    """
    return [
        # 條件 5：入口傳送箱空料架搬運到出口傳送箱 - 檢查入口傳送箱是否有空料架
        # 功能：檢查各房間入口傳送箱是否有空料架需要搬運到出口傳送箱
        # 邏輯：查詢 rack 表中各房間入口位置且狀態為 1（空料架）的記錄
        # 返回：result（True/False）、next_id（房間檢查列表）、location（最小位置ID）
        {
            "id": 5,
            "conditions": "SELECT \
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN 'True'\
                                        ELSE 'False'\
                                    END as result,\
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN '[115,215,315,415,515,615,715,815,915,1015]'\
                                        ELSE NULL\
                                    END as next_id,\
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN MIN(location_id)\
                                        ELSE NULL\
                                    END as location\
                                    FROM rack \
                                    WHERE location_id = ANY (ARRAY[10001,20001,30001,40001,50001,60001,70001,80001,90001,100001]) \
                                    AND status_id = 1",
            "results": {},
            "description": "入口傳送箱空料架搬運到出口傳送箱 - 檢查入口傳送箱是否有空料架"
        },

        # 條件 215：房間2出口傳送箱位置檢查
        # 功能：檢查房間2出口傳送箱是否無料架佔用
        # 邏輯：查詢 rack 表中 location_id = 20002 的記錄，確保位置空閒
        # 返回：result（True/False）、next_id（下一個檢查條件）、location_id（位置ID）、room_id（房間號）
        {
            "id": 215,
            "conditions": "SELECT \
                                    CASE \
                                        WHEN COUNT(*) = 0 THEN 'True'\
                                        ELSE 'False'\
                                    END as result,\
                                    CASE \
                                        WHEN COUNT(*) = 0 THEN 216\
                                        ELSE NULL\
                                    END as next_id,\
                                    CASE \
                                        WHEN COUNT(*) = 0 THEN 2\
                                        ELSE NULL\
                                    END as room_id,\
                                    20002 as location_id\
                                    FROM rack \
                                    WHERE location_id = 20002",
            "results": {},
            "description": "房間2出口傳送箱位置檢查 - 檢查是否無料架佔用"
        },

        # 條件 216：檢查是否有相同執行的任務（空料架搬運）
        # 功能：檢查是否有相同的執行中任務，防止重複任務產生
        # 邏輯：查詢 task 表中是否有相同 work_id 和 node_id 的執行中任務
        # 返回：result（True/False）、next_id（無）、end（結束標記）
        {
            "id": 216,
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
                                    AND node_id = 20002",
            "results": {},
            "description": "檢查是否有相同執行的任務（空料架搬運） - 防止重複任務"
        }
    ]


def get_empty_rack_extended_conditions():
    """
    取得其他房間的空料架搬運條件（預留擴展用）
    
    此函數可用於未來擴展房間3-10的條件
    目前返回空列表，可根據需要添加對應條件
    
    Returns:
        list: 其他房間的條件定義列表（目前為空）
    """
    # 預留給房間3-10的條件擴展
    # 格式：條件 315, 316（房間3）、415, 416（房間4）...以此類推
    return []
