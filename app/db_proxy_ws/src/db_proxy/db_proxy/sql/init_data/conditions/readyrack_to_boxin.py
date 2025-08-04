"""
優化版：準備區料架到入口傳送流程
使用簡化的SELECT查詢，避免SQL驗證失敗

優化重點：
1. 條件 5：準備區料架檢查
2. 條件 110/210：房間入口檢查
3. 條件 211：重複任務檢查
"""

def get_optimized_ready_rack_to_boxin_conditions():
    """
    取得優化版準備區料架到入口傳送流程的所有條件
    
    Returns:
        list: 包含優化條件定義的列表
    """
    return [
        # 條件 5：準備區料架檢查（簡化版）
        {
            "id": 5,
            "conditions": """
                SELECT 
                    CASE 
                        WHEN EXISTS (
                            SELECT 1 FROM rack 
                            WHERE location_id IN (41,42,43,44,45) AND status_id = 2
                        ) THEN 'True'
                        ELSE 'False'
                    END as result,
                    CASE 
                        WHEN EXISTS (
                            SELECT 1 FROM rack 
                            WHERE location_id IN (41,42,43,44,45) AND status_id = 2
                        ) THEN (
                            CASE 
                                WHEN NOT EXISTS (SELECT 1 FROM rack WHERE location_id = 10001) THEN 110
                                WHEN NOT EXISTS (SELECT 1 FROM rack WHERE location_id = 20001) THEN 210
                                ELSE NULL
                            END
                        )
                        ELSE NULL
                    END as next_id,
                    (
                        SELECT MIN(location_id) FROM rack 
                        WHERE location_id IN (41,42,43,44,45) AND status_id = 2
                    ) as source_location,
                    (
                        SELECT MIN(id) FROM rack 
                        WHERE location_id IN (41,42,43,44,45) AND status_id = 2
                    ) as rack_id
            """,
            "results": {},
            "description": "準備區料架檢查（簡化版）",
            "optimization_notes": "簡化準備區狀態檢查"
        },

        # 條件 110：房間1入口傳送箱檢查（簡化版）
        {
            "id": 110,
            "conditions": """
                SELECT 
                    CASE 
                        WHEN NOT EXISTS (SELECT 1 FROM rack WHERE location_id = 10001) THEN 'True'
                        ELSE 'False'
                    END as result,
                    CASE 
                        WHEN NOT EXISTS (SELECT 1 FROM rack WHERE location_id = 10001) THEN 211
                        ELSE NULL
                    END as next_id,
                    CASE 
                        WHEN NOT EXISTS (SELECT 1 FROM rack WHERE location_id = 10001) THEN 1
                        ELSE NULL
                    END as room_id,
                    10001 as target_location
            """,
            "results": {},
            "description": "房間1入口傳送箱檢查（簡化版）",
            "optimization_notes": "簡化房間狀態檢查"
        },

        # 條件 210：房間2入口傳送箱檢查（簡化版）
        {
            "id": 210,
            "conditions": """
                SELECT 
                    CASE 
                        WHEN NOT EXISTS (SELECT 1 FROM rack WHERE location_id = 20001) THEN 'True'
                        ELSE 'False'
                    END as result,
                    CASE 
                        WHEN NOT EXISTS (SELECT 1 FROM rack WHERE location_id = 20001) THEN 211
                        ELSE NULL
                    END as next_id,
                    CASE 
                        WHEN NOT EXISTS (SELECT 1 FROM rack WHERE location_id = 20001) THEN 2
                        ELSE NULL
                    END as room_id,
                    20001 as target_location
            """,
            "results": {},
            "description": "房間2入口傳送箱檢查（簡化版）",
            "optimization_notes": "簡化房間狀態檢查"
        },

        # 條件 211：重複任務檢查（簡化版）
        {
            "id": 211,
            "conditions": """
                SELECT 
                    CASE 
                        WHEN NOT EXISTS (
                            SELECT 1 FROM task t
                            WHERE t.status_id IN (0, 1, 2)
                            AND t.work_id = 220003
                        ) THEN 'True'
                        ELSE 'False'
                    END as result,
                    NULL as next_id,
                    CASE 
                        WHEN NOT EXISTS (
                            SELECT 1 FROM task t
                            WHERE t.status_id IN (0, 1, 2)
                            AND t.work_id = 220003
                        ) THEN 'True'
                        ELSE 'False'
                    END as end
            """,
            "results": {},
            "description": "重複任務檢查（簡化版）",
            "optimization_notes": "簡化重複任務檢查邏輯"
        }
    ]