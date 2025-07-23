"""
第一組：滿料架到人工收料區流程
包含滿料架從各房間搬運到人工收料區的完整業務邏輯

業務流程說明：
1. 檢查系統空架區是否有空料架（條件 1）- 起始條件，確保有空料架可以替換滿料架
2. 檢查各房間是否有滿料架需要搬運（條件 201, 202）- 確認房間內有carrier
3. 檢查是否有重複的執行中任務（條件 203）- 防止重複任務產生

條件關聯性：
- 條件 1 → 檢查空料架可用性 → next_id 指向房間檢查列表 [101,201,301...]
- 條件 201-202 → 檢查房間1-2的carrier狀態
- 條件 203 → 防止房間2重複任務

技術細節：
- 使用 carrier 表檢查房間內是否有滿料架
- 使用 location 表檢查空架區的狀態
- 使用 task 表檢查是否有執行中的重複任務

注意：條件 2、205、206、207 已移至 manual_area_check.py（人工收料區搬運流程）
"""

def get_full_rack_to_manual_conditions():
    """
    取得滿料架到人工收料區流程的所有條件
    
    Returns:
        list: 包含該流程所有條件的列表
    """
    return [
        # 條件 1：系統空架區-有空料架（起始條件）
        # 功能：檢查系統空架區是否有可用的空料架位置
        # 邏輯：查詢 location 表中 ID 為 31-34 且狀態為 3（有空料架）的位置
        # 返回：result（True/False）、next_id（房間檢查列表）、location（最小位置ID）
        {
            "id": 1,
            "conditions": "SELECT \
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN 'True'\
                                        ELSE 'False'\
                                    END as result,\
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN '[101,201,301,401,501,601,701,801,901,1001]'\
                                        ELSE NULL\
                                    END as next_id,\
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN MIN(id)\
                                        ELSE NULL\
                                    END as location\
                                    FROM location \
                                    WHERE id = ANY (ARRAY[31,32,33,34]) AND location_status_id = 3",
            "results": {},
            "description": "系統空架區-有空料架"
        },

        # 條件 201：房間1滿料架到人工收料區檢查
        # 功能：檢查房間1是否有滿料架需要搬運到人工收料區
        # 邏輯：查詢 carrier 表中 room_id = 1 的記錄
        # 返回：result（True/False）、next_id（下一個檢查條件）、room_id（房間號）
        {
            "id": 201,
            "conditions": "SELECT \
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN 'True'\
                                        ELSE 'False'\
                                    END as result,\
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN 202\
                                        ELSE NULL\
                                    END as next_id,\
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN 1\
                                        ELSE NULL\
                                    END as room_id\
                                    FROM carrier \
                                    WHERE room_id = 1",
            "results": {},
            "description": "房間1-有carrier在裡面"
        },

        # 條件 202：房間2滿料架到人工收料區檢查
        # 功能：檢查房間2是否有滿料架需要搬運到人工收料區
        # 邏輯：查詢 carrier 表中 room_id = 2 的記錄
        # 返回：result（True/False）、next_id（下一個檢查條件）、room_id（房間號）
        {
            "id": 202,
            "conditions": "SELECT \
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN 'True'\
                                        ELSE 'False'\
                                    END as result,\
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN 203\
                                        ELSE NULL\
                                    END as next_id,\
                                    CASE \
                                        WHEN COUNT(*) > 0 THEN 2\
                                        ELSE NULL\
                                    END as room_id\
                                    FROM carrier \
                                    WHERE room_id = 2",
            "results": {},
            "description": "房間2-有carrier在裡面"
        },

        # 條件 203：房間2-檢查重複任務（滿料架到人工收料區）
        # 功能：檢查是否有相同的執行中任務，防止重複任務產生
        # 邏輯：查詢 task 表中是否有相同 work_id 和 node_id 的執行中任務
        # 返回：result（True/False）、next_id（無）、room_id（房間號）、end（結束標記）
        {
            "id": 203,
            "conditions": "SELECT \
                                    CASE \
                                        WHEN COUNT(*) = 0 THEN 'True'\
                                        ELSE 'False'\
                                    END as result,\
                                    NULL as next_id,\
                                    CASE \
                                        WHEN COUNT(*) = 0 THEN 2\
                                        ELSE NULL\
                                    END as room_id,\
                                    CASE \
                                        WHEN COUNT(*) = 0 THEN 'True'\
                                        ELSE 'False'\
                                    END as end\
                                    FROM task \
                                    WHERE work_id = '220001' AND node_id = 20001",
            "results": {},
            "description": "房間2-檢查重複任務（滿料架到人工收料區）"
        }
    ]
