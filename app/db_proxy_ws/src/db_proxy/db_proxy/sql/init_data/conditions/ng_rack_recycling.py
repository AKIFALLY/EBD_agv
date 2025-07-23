"""
第四組：入口傳送箱NG料架搬運到NG回收區流程
包含將NG料架從房間入口傳送箱搬運到NG回收區的完整業務邏輯

業務流程說明：
1. 檢查NG回收區是否有空位（條件 6）- 起始條件，確保NG回收區有空位
2. 檢查各房間入口傳送箱是否有NG料架（條件 220, 320, 420...）- 確認有NG料架需要回收
3. 檢查是否有重複的執行中任務（條件 221, 321, 421...）- 防止重複任務產生

條件關聯性：
- 條件 6 → 檢查NG回收區空位可用性 → next_id 指向房間檢查列表 [120,220,320...]
- 條件 220-1020 → 檢查各房間入口傳送箱的NG料架狀態
- 條件 221-1021 → 防止各房間重複任務的產生

技術細節：
- 使用 location 表檢查NG回收區的空位狀態（location_status_id = 2）
- 使用 rack 表檢查入口傳送箱的NG料架狀態（status_id = 7）
- 使用 task 表檢查是否有執行中的重複任務
- 支援房間1-10的擴展（ID範圍：120-1020, 121-1021）

NG回收區位置：
- NG回收箱位置：[71, 72]
- 狀態要求：location_status_id = 2（空位）

房間位置對應：
- 房間1：入口 10001
- 房間2：入口 20001
- 房間3：入口 30001
- ...以此類推到房間10

房間編號對應：
- 房間1：條件 120, 121（預留）
- 房間2：條件 220, 221
- 房間3：條件 320, 321（預留）
- ...以此類推到房間10
"""

def get_ng_rack_recycling_conditions():
    """
    取得入口傳送箱NG料架搬運到NG回收區流程的所有條件
    
    Returns:
        list: 包含該流程所有條件的列表
    """
    return [
        # 條件 6：入口傳送箱NG料架搬運到NG回收區 - 檢查NG回收區是否有空位
        # 功能：檢查NG回收區是否有空位可以放置NG料架
        # 邏輯：查詢 location 表中 ID 為 71-72 且狀態為 2（空位）的位置
        # 返回：result（True/False）、next_id（房間檢查列表）、location（最小位置ID）
        {
            "id": 6,
            "conditions": "SELECT \
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN 'True'\
                                        ELSE 'False'\
                                    END as result,\
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN '[120,220,320,420,520,620,720,820,920,1020]'\
                                        ELSE NULL\
                                    END as next_id,\
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN MIN(id)\
                                        ELSE NULL\
                                    END as location\
                                    FROM location \
                                    WHERE id = ANY (ARRAY[71,72]) AND location_status_id = 2",
            "results": {},
            "description": "入口傳送箱NG料架搬運到NG回收區 - 檢查NG回收區是否有空位"
        },

        # 條件 220：房間2入口傳送箱NG料架檢查
        # 功能：檢查房間2入口傳送箱是否有NG料架需要搬運
        # 邏輯：查詢 rack 表中 location_id = 20001 且狀態為 7（NG料架）的記錄
        # 返回：result（True/False）、next_id（下一個檢查條件）、location（位置ID）
        {
            "id": 220,
            "conditions": "SELECT \
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN 'True'\
                                        ELSE 'False'\
                                    END as result,\
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN 221\
                                        ELSE NULL\
                                    END as next_id,\
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN MIN(location_id)\
                                        ELSE NULL\
                                    END as location\
                                    FROM rack \
                                    WHERE location_id = 20001 AND status_id = 7",
            "results": {},
            "description": "房間2入口傳送箱NG料架檢查 - 檢查是否有NG料架"
        },

        # 條件 221：檢查是否有相同執行的任務（NG料架搬運）
        # 功能：檢查是否有相同的執行中任務，防止重複任務產生
        # 邏輯：查詢 task 表中是否有相同 work_id 和 node_id 的執行中任務
        # 返回：result（True/False）、next_id（無）、end（結束標記）
        {
            "id": 221,
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
            "description": "檢查是否有相同執行的任務（NG料架搬運） - 防止重複任務"
        }
    ]


def get_ng_rack_extended_conditions():
    """
    取得其他房間的NG料架回收條件（預留擴展用）
    
    此函數可用於未來擴展房間3-10的條件
    目前返回空列表，可根據需要添加對應條件
    
    Returns:
        list: 其他房間的條件定義列表（目前為空）
    """
    # 預留給房間3-10的條件擴展
    # 格式：條件 320, 321（房間3）、420, 421（房間4）...以此類推
    return []
