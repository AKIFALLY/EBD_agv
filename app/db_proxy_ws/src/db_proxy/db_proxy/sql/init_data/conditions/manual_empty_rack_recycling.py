"""
人工回收空料架搬運到系統空料架區流程
包含從人工回收空料架區將空料架搬運到系統空料架區的完整業務邏輯

業務流程說明：
1. 檢查人工回收空料架區是否有料架（條件 7）- 確認有空料架需要搬運
2. 檢查空料架回收區是否有空位（條件 8）- 確保目標位置有空間
3. 檢查是否有相同的執行中任務（條件 9）- 防止重複任務產生

條件關聯性：
- 條件 7 → 檢查人工回收空料架區料架可用性 → next_id 指向條件 8
- 條件 8 → 檢查空料架回收區空位可用性 → next_id 指向條件 9
- 條件 9 → 最終的重複任務檢查 → end='True'（流程結束）

技術細節：
- 使用 location 表檢查人工回收空料架區的料架狀態（location_status_id = 3）
- 使用 location 表檢查空料架回收區的空位狀態（location_status_id = 2）
- 使用 task 表檢查是否有執行中的重複任務（status_id IN (0, 1, 2)）

位置對應：
- 人工回收空料架區：位置 [91, 92]
- 空料架回收區：位置 [51, 52, 53, 54]

工作流程：
人工回收空料架區 → 空料架回收區 → 系統空料架區
"""

def get_manual_empty_rack_recycling_conditions():
    """
    取得人工回收空料架搬運到系統空料架區流程的所有條件
    
    Returns:
        list: 包含該流程所有條件的列表
    """
    return [
        # 條件 7：人工回收空料架區-有料架檢查
        # 功能：檢查人工回收空料架區是否有空料架需要搬運到系統空料架區
        # 邏輯：查詢 location 表中 ID 為 91-92 且狀態為 3（有空料架）的位置
        # 返回：result（True/False）、next_id（下一個條件）、location（最小位置ID）
        {
            "id": 7,
            "conditions": "SELECT \
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN 'True'\
                                        ELSE 'False'\
                                    END as result,\
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN 8\
                                        ELSE NULL\
                                    END as next_id,\
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN MIN(id)\
                                        ELSE NULL\
                                    END as location\
                                    FROM location \
                                    WHERE id = ANY (ARRAY[91,92]) AND location_status_id = 3",
            "results": {},
            "description": "人工回收空料架區-有料架"
        },

        # 條件 8：空料架回收區-有空位檢查
        # 功能：檢查空料架回收區是否有空位可以放置從人工回收區搬運來的空料架
        # 邏輯：查詢 location 表中 ID 為 51-54 且狀態為 2（空位）的位置
        # 返回：result（True/False）、next_id（下一個條件）、location（最小位置ID）
        # 入口：來自條件 7（當人工回收區有料架時）
        {
            "id": 8,
            "conditions": "SELECT \
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN 'True'\
                                        ELSE 'False'\
                                    END as result,\
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN 9\
                                        ELSE NULL\
                                    END as next_id,\
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN MIN(id)\
                                        ELSE NULL\
                                    END as location\
                                    FROM location \
                                    WHERE id = ANY (ARRAY[51,52,53,54]) AND location_status_id = 2",
            "results": {},
            "description": "空料架回收區-有空位"
        },

        # 條件 9：檢查重複任務（人工回收空料架搬運流程最終確認）
        # 功能：檢查是否已存在相同的執行中任務，防止重複任務產生
        # 邏輯：查詢 task 表中是否有相同 work_id 的執行中任務
        # 返回：result（True/False）、next_id（無）、end（結束標記）
        # 入口：來自條件 8（當空料架回收區有空位時）
        {
            "id": 9,
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
                                    WHERE work_id = '230001' AND status_id IN (0, 1, 2)",
            "results": {},
            "description": "檢查重複任務（人工回收空料架搬運流程最終確認）"
        }
    ]


def get_manual_empty_rack_extended_conditions():
    """
    取得其他相關的人工回收空料架搬運條件（預留擴展用）
    
    此函數可用於未來擴展其他人工回收相關的條件
    目前返回空列表，可根據需要添加對應條件
    
    Returns:
        list: 其他相關條件定義列表（目前為空）
    """
    # 預留給其他人工回收相關條件的擴展
    # 例如：不同類型的空料架、不同的回收區域等
    return []
