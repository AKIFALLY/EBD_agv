"""
第五組：AGV旋轉狀態檢查流程
包含檢查AGV旋轉狀態和任務發送條件的完整業務邏輯

業務流程說明：
1. 檢查等待旋轉狀態的 AGV 是否可以發送任務（條件 3）
2. 防止重複發送旋轉任務給同一個 AGV
3. 確保任務發送的安全性和一致性

條件關聯性：
- 條件 3 → 檢查 AGV 旋轉狀態和任務發送條件 → 直接返回結果，無 next_id

技術細節：
- 使用 agv_context 表檢查 AGV 的當前狀態
- 使用 task 表檢查是否已有子任務存在
- 使用 UNION ALL 處理有/無等待旋轉 AGV 的情況
- 返回 agv_id、task_id、result、end 等多個欄位

AGV狀態說明：
- current_state = 'wait_rotation_state'：AGV 處於等待旋轉狀態
- 子任務檢查：確保沒有重複的旋轉任務被發送
- 安全機制：當沒有等待旋轉的 AGV 時，返回 False 結果

查詢邏輯：
1. 主查詢：查找處於等待旋轉狀態的 AGV 及其任務
2. 子任務檢查：確認該任務沒有已存在的子任務
3. 備用查詢：當沒有等待旋轉的 AGV 時，返回預設的 False 結果
4. 排序：按 agv_id 排序確保結果的一致性
"""

def get_agv_rotation_check_conditions():
    """
    取得AGV旋轉狀態檢查流程的所有條件
    
    Returns:
        list: 包含該流程所有條件的列表
    """
    return [
        # 條件 3：檢查等待旋轉狀態的 AGV 是否可以發送任務
        # 功能：檢查處於等待旋轉狀態的 AGV 是否可以發送新的旋轉任務
        # 邏輯：查詢 agv_context 和 task 表，檢查是否已有子任務存在
        # 返回：agv_id（AGV ID）、task_id（任務ID）、result（True/False）、end（結束標記）
        {
            "id": 3,
            "conditions": "SELECT * FROM (\
                                    SELECT \
                                        ac.agv_id,\
                                        t.id as task_id,\
                                        CASE \
                                            WHEN NOT EXISTS (\
                                                SELECT 1 \
                                                FROM task t2 \
                                                WHERE t2.parent_task_id = t.id\
                                            ) THEN 'True'\
                                            ELSE 'False'\
                                        END as result,\
                                        NULL as next_id,\
                                        CASE \
                                            WHEN NOT EXISTS (\
                                                SELECT 1 \
                                                FROM task t2 \
                                                WHERE t2.parent_task_id = t.id\
                                            ) THEN 'True'\
                                            ELSE 'False'\
                                        END as end\
                                    FROM agv_context ac\
                                    INNER JOIN task t ON ac.agv_id = t.agv_id\
                                    WHERE ac.current_state = 'wait_rotation_state'\
                                    UNION ALL\
                                    SELECT \
                                        -1 as agv_id,\
                                        0 as task_id,\
                                        'False' as result,\
                                        NULL as next_id,\
                                        'False' as end\
                                    WHERE NOT EXISTS (\
                                        SELECT 1 \
                                        FROM agv_context ac2 \
                                        INNER JOIN task t2 ON ac2.agv_id = t2.agv_id \
                                        WHERE ac2.current_state = 'wait_rotation_state'\
                                    )\
                                ) subquery \
                                ORDER BY agv_id",
            "results": {},
            "description": "檢查等待旋轉狀態的 AGV 是否可以發送任務（防止重複發送）"
        }
    ]


def get_agv_rotation_extended_conditions():
    """
    取得AGV旋轉相關的擴展條件（預留擴展用）
    
    此函數可用於未來擴展其他AGV狀態檢查條件
    目前返回空列表，可根據需要添加對應條件
    
    Returns:
        list: 其他AGV相關條件定義列表（目前為空）
    """
    # 預留給其他AGV狀態檢查的條件擴展
    # 例如：AGV充電狀態檢查、AGV故障狀態檢查等
    return []
