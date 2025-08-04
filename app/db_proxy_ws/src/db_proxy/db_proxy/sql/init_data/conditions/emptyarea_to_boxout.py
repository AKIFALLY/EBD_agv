"""
優化版：系統空料架區搬運到出口傳送箱流程
使用簡化的SELECT查詢，避免SQL驗證失敗

優化重點：
1. 條件 3：系統空料架區檢查
2. 條件 201/202：房間出口檢查  
3. 條件 203：重複任務檢查
"""

def get_optimized_empty_rack_to_boxout_conditions():
    """
    取得優化版系統空料架區搬運到出口傳送箱流程的所有條件
    
    Returns:
        list: 包含優化條件定義的列表
    """
    return [
        # 條件 3：系統空料架區檢查（簡化版）
        {
            "id": 3,
            "conditions": """
                SELECT 
                    CASE 
                        WHEN EXISTS (
                            SELECT 1 FROM rack 
                            WHERE location_id IN (31,32,33,34) AND status_id = 1
                        ) THEN 'True'
                        ELSE 'False'
                    END as result,
                    CASE 
                        WHEN EXISTS (
                            SELECT 1 FROM rack 
                            WHERE location_id IN (31,32,33,34) AND status_id = 1
                        ) THEN (
                            CASE 
                                WHEN NOT EXISTS (SELECT 1 FROM rack WHERE location_id = 20002) THEN 201
                                WHEN NOT EXISTS (SELECT 1 FROM rack WHERE location_id = 10002) THEN 202
                                ELSE NULL
                            END
                        )
                        ELSE NULL
                    END as next_id,
                    (
                        SELECT MIN(location_id) FROM rack 
                        WHERE location_id IN (31,32,33,34) AND status_id = 1
                    ) as source_location,
                    (
                        SELECT MIN(id) FROM rack 
                        WHERE location_id IN (31,32,33,34) AND status_id = 1
                    ) as rack_id
            """,
            "results": {},
            "description": "系統空料架區檢查（簡化版）",
            "optimization_notes": "簡化為基本SELECT查詢"
        },

        # 條件 201：房間2出口傳送箱檢查（簡化版）
        {
            "id": 201,
            "conditions": """
                SELECT 
                    CASE 
                        WHEN NOT EXISTS (SELECT 1 FROM rack WHERE location_id = 20002) THEN 'True'
                        ELSE 'False'
                    END as result,
                    CASE 
                        WHEN NOT EXISTS (SELECT 1 FROM rack WHERE location_id = 20002) THEN 203
                        ELSE NULL
                    END as next_id,
                    CASE 
                        WHEN NOT EXISTS (SELECT 1 FROM rack WHERE location_id = 20002) THEN 2
                        ELSE NULL
                    END as room_id,
                    20002 as target_location
            """,
            "results": {},
            "description": "房間2出口傳送箱檢查（簡化版）",
            "optimization_notes": "簡化房間狀態檢查"
        },

        # 條件 202：房間1出口傳送箱檢查（簡化版）
        {
            "id": 202,
            "conditions": """
                SELECT 
                    CASE 
                        WHEN NOT EXISTS (SELECT 1 FROM rack WHERE location_id = 10002) THEN 'True'
                        ELSE 'False'
                    END as result,
                    CASE 
                        WHEN NOT EXISTS (SELECT 1 FROM rack WHERE location_id = 10002) THEN 203
                        ELSE NULL
                    END as next_id,
                    CASE 
                        WHEN NOT EXISTS (SELECT 1 FROM rack WHERE location_id = 10002) THEN 1
                        ELSE NULL
                    END as room_id,
                    10002 as target_location
            """,
            "results": {},
            "description": "房間1出口傳送箱檢查（簡化版）",
            "optimization_notes": "簡化房間狀態檢查"
        },

        # 條件 203：重複任務檢查（簡化版）
        {
            "id": 203,
            "conditions": """
                SELECT 
                    CASE 
                        WHEN NOT EXISTS (
                            SELECT 1 FROM task t
                            WHERE t.status_id IN (0, 1, 2)
                            AND t.work_id = 220001
                        ) THEN 'True'
                        ELSE 'False'
                    END as result,
                    NULL as next_id,
                    CASE 
                        WHEN NOT EXISTS (
                            SELECT 1 FROM task t
                            WHERE t.status_id IN (0, 1, 2)
                            AND t.work_id = 220001
                        ) THEN 'True'
                        ELSE 'False'
                    END as end
            """,
            "results": {},
            "description": "重複任務檢查（簡化版）",
            "optimization_notes": "簡化重複任務檢查邏輯"
        }
    ]