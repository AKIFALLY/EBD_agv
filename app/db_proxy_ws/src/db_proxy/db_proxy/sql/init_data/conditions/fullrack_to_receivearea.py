"""
優化版：滿料架搬運到人工收料區流程
使用簡化的SELECT查詢，避免SQL驗證失敗

優化重點：
1. 條件 4：人工收料區空位檢查
2. 條件 105/205：房間滿料架檢查
3. 條件 206：重複任務檢查
"""

def get_optimized_full_rack_to_manual_conditions():
    """
    取得優化版滿料架搬運到人工收料區流程的所有條件
    
    Returns:
        list: 包含優化條件定義的列表
    """
    return [
        # 條件 4：人工收料區空位檢查（簡化版）
        {
            "id": 4,
            "conditions": """
                SELECT 
                    CASE 
                        WHEN EXISTS (
                            SELECT 1 FROM location 
                            WHERE id IN (51,52,53,54,55) 
                            AND id NOT IN (
                                SELECT location_id FROM rack 
                                WHERE location_id IN (51,52,53,54,55)
                            )
                        ) THEN 'True'
                        ELSE 'False'
                    END as result,
                    CASE 
                        WHEN EXISTS (
                            SELECT 1 FROM location 
                            WHERE id IN (51,52,53,54,55) 
                            AND id NOT IN (
                                SELECT location_id FROM rack 
                                WHERE location_id IN (51,52,53,54,55)
                            )
                        ) THEN (
                            CASE 
                                WHEN EXISTS (SELECT 1 FROM rack WHERE location_id / 1000 = 1 AND status_id = 2) THEN 105
                                WHEN EXISTS (SELECT 1 FROM rack WHERE location_id / 1000 = 2 AND status_id = 2) THEN 205
                                ELSE NULL
                            END
                        )
                        ELSE NULL
                    END as next_id,
                    (
                        SELECT MIN(id) FROM location 
                        WHERE id IN (51,52,53,54,55) 
                        AND id NOT IN (
                            SELECT location_id FROM rack 
                            WHERE location_id IN (51,52,53,54,55)
                        )
                    ) as target_location
            """,
            "results": {},
            "description": "人工收料區空位檢查（簡化版）",
            "optimization_notes": "簡化收料區狀態檢查"
        },

        # 條件 105：房間1滿料架檢查（簡化版）
        {
            "id": 105,
            "conditions": """
                SELECT 
                    CASE 
                        WHEN EXISTS (
                            SELECT 1 FROM rack 
                            WHERE location_id / 1000 = 1 AND status_id = 2
                        ) THEN 'True'
                        ELSE 'False'
                    END as result,
                    CASE 
                        WHEN EXISTS (
                            SELECT 1 FROM rack 
                            WHERE location_id / 1000 = 1 AND status_id = 2
                        ) THEN 206
                        ELSE NULL
                    END as next_id,
                    CASE 
                        WHEN EXISTS (
                            SELECT 1 FROM rack 
                            WHERE location_id / 1000 = 1 AND status_id = 2
                        ) THEN 1
                        ELSE NULL
                    END as room_id,
                    (
                        SELECT MIN(location_id) FROM rack 
                        WHERE location_id / 1000 = 1 AND status_id = 2
                    ) as source_location,
                    (
                        SELECT MIN(id) FROM rack 
                        WHERE location_id / 1000 = 1 AND status_id = 2
                    ) as rack_id
            """,
            "results": {},
            "description": "房間1滿料架檢查（簡化版）",
            "optimization_notes": "簡化房間狀態檢查"
        },

        # 條件 205：房間2滿料架檢查（簡化版）
        {
            "id": 205,
            "conditions": """
                SELECT 
                    CASE 
                        WHEN EXISTS (
                            SELECT 1 FROM rack 
                            WHERE location_id / 1000 = 2 AND status_id = 2
                        ) THEN 'True'
                        ELSE 'False'
                    END as result,
                    CASE 
                        WHEN EXISTS (
                            SELECT 1 FROM rack 
                            WHERE location_id / 1000 = 2 AND status_id = 2
                        ) THEN 206
                        ELSE NULL
                    END as next_id,
                    CASE 
                        WHEN EXISTS (
                            SELECT 1 FROM rack 
                            WHERE location_id / 1000 = 2 AND status_id = 2
                        ) THEN 2
                        ELSE NULL
                    END as room_id,
                    (
                        SELECT MIN(location_id) FROM rack 
                        WHERE location_id / 1000 = 2 AND status_id = 2
                    ) as source_location,
                    (
                        SELECT MIN(id) FROM rack 
                        WHERE location_id / 1000 = 2 AND status_id = 2
                    ) as rack_id
            """,
            "results": {},
            "description": "房間2滿料架檢查（簡化版）",
            "optimization_notes": "簡化房間狀態檢查"
        },

        # 條件 206：重複任務檢查（簡化版）
        {
            "id": 206,
            "conditions": """
                SELECT 
                    CASE 
                        WHEN NOT EXISTS (
                            SELECT 1 FROM task t
                            WHERE t.status_id IN (0, 1, 2)
                            AND t.work_id = 220002
                        ) THEN 'True'
                        ELSE 'False'
                    END as result,
                    NULL as next_id,
                    CASE 
                        WHEN NOT EXISTS (
                            SELECT 1 FROM task t
                            WHERE t.status_id IN (0, 1, 2)
                            AND t.work_id = 220002
                        ) THEN 'True'
                        ELSE 'False'
                    END as end
            """,
            "results": {},
            "description": "重複任務檢查（簡化版）",
            "optimization_notes": "簡化重複任務檢查邏輯"
        }
    ]