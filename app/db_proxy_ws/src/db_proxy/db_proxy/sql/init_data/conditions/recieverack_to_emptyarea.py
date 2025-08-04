"""
優化版：人工回收空料架搬運流程
使用簡化的SELECT查詢，避免SQL驗證失敗

優化重點：
1. 條件 7：人工收料區空料架檢查
2. 條件 8：系統空料架區空位檢查
3. 條件 9：重複任務檢查
"""

def get_optimized_manual_empty_rack_recycling_conditions():
    """
    取得優化版人工回收空料架搬運流程的所有條件
    
    Returns:
        list: 包含優化條件定義的列表
    """
    return [
        # 條件 7：人工收料區空料架檢查（簡化版）
        {
            "id": 7,
            "conditions": """
                SELECT 
                    CASE 
                        WHEN EXISTS (
                            SELECT 1 FROM rack 
                            WHERE location_id IN (51,52,53,54,55) AND status_id = 1
                        ) THEN 'True'
                        ELSE 'False'
                    END as result,
                    CASE 
                        WHEN EXISTS (
                            SELECT 1 FROM rack 
                            WHERE location_id IN (51,52,53,54,55) AND status_id = 1
                        ) THEN 8
                        ELSE NULL
                    END as next_id,
                    (
                        SELECT MIN(location_id) FROM rack 
                        WHERE location_id IN (51,52,53,54,55) AND status_id = 1
                    ) as source_location,
                    (
                        SELECT MIN(id) FROM rack 
                        WHERE location_id IN (51,52,53,54,55) AND status_id = 1
                    ) as rack_id
            """,
            "results": {},
            "description": "人工收料區空料架檢查（簡化版）",
            "optimization_notes": "簡化人工收料區狀態檢查"
        },

        # 條件 8：系統空料架區空位檢查（簡化版）
        {
            "id": 8,
            "conditions": """
                SELECT 
                    CASE 
                        WHEN EXISTS (
                            SELECT 1 FROM location 
                            WHERE id IN (31,32,33,34) 
                            AND id NOT IN (
                                SELECT location_id FROM rack 
                                WHERE location_id IN (31,32,33,34)
                            )
                        ) THEN 'True'
                        ELSE 'False'
                    END as result,
                    CASE 
                        WHEN EXISTS (
                            SELECT 1 FROM location 
                            WHERE id IN (31,32,33,34) 
                            AND id NOT IN (
                                SELECT location_id FROM rack 
                                WHERE location_id IN (31,32,33,34)
                            )
                        ) THEN 9
                        ELSE NULL
                    END as next_id,
                    (
                        SELECT MIN(id) FROM location 
                        WHERE id IN (31,32,33,34) 
                        AND id NOT IN (
                            SELECT location_id FROM rack 
                            WHERE location_id IN (31,32,33,34)
                        )
                    ) as target_location
            """,
            "results": {},
            "description": "系統空料架區空位檢查（簡化版）",
            "optimization_notes": "簡化空料架區狀態檢查"
        },

        # 條件 9：重複任務檢查（簡化版）
        {
            "id": 9,
            "conditions": """
                SELECT 
                    CASE 
                        WHEN NOT EXISTS (
                            SELECT 1 FROM task t
                            WHERE t.status_id IN (0, 1, 2)
                            AND t.work_id = 220005
                        ) THEN 'True'
                        ELSE 'False'
                    END as result,
                    NULL as next_id,
                    CASE 
                        WHEN NOT EXISTS (
                            SELECT 1 FROM task t
                            WHERE t.status_id IN (0, 1, 2)
                            AND t.work_id = 220005
                        ) THEN 'True'
                        ELSE 'False'
                    END as end
            """,
            "results": {},
            "description": "重複任務檢查（簡化版）",
            "optimization_notes": "簡化重複任務檢查邏輯"
        }
    ]