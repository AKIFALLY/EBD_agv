"""
優化版：空料架從入口傳送箱搬運流程
使用簡化的SELECT查詢，避免SQL驗證失敗

優化重點：
1. 條件 6：入口傳送箱空料架檢查
2. 條件 215：目標位置選擇
3. 條件 216/217：路徑和重複任務檢查
"""

def get_optimized_empty_rack_to_boxout_or_emptyarea_conditions():
    """
    取得優化版空料架從入口傳送箱搬運流程的所有條件
    
    Returns:
        list: 包含優化條件定義的列表
    """
    return [
        # 條件 6：入口傳送箱空料架檢查（簡化版）
        {
            "id": 6,
            "conditions": """
                SELECT 
                    CASE 
                        WHEN EXISTS (
                            SELECT 1 FROM rack 
                            WHERE location_id IN (10001,20001) AND status_id = 1
                        ) THEN 'True'
                        ELSE 'False'
                    END as result,
                    CASE 
                        WHEN EXISTS (
                            SELECT 1 FROM rack 
                            WHERE location_id IN (10001,20001) AND status_id = 1
                        ) THEN 215
                        ELSE NULL
                    END as next_id,
                    (
                        SELECT MIN(location_id) FROM rack 
                        WHERE location_id IN (10001,20001) AND status_id = 1
                    ) as source_location,
                    (
                        SELECT MIN(id) FROM rack 
                        WHERE location_id IN (10001,20001) AND status_id = 1
                    ) as rack_id
            """,
            "results": {},
            "description": "入口傳送箱空料架檢查（簡化版）",
            "optimization_notes": "簡化入口傳送箱狀態檢查"
        },

        # 條件 215：目標位置選擇（簡化版）
        {
            "id": 215,
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
                        WHEN NOT EXISTS (SELECT 1 FROM rack WHERE location_id IN (10002,20002)) THEN 'True'
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
                        ) THEN 217
                        WHEN NOT EXISTS (SELECT 1 FROM rack WHERE location_id IN (10002,20002)) THEN 216
                        ELSE NULL
                    END as next_id,
                    CASE 
                        WHEN EXISTS (
                            SELECT 1 FROM location 
                            WHERE id IN (31,32,33,34) 
                            AND id NOT IN (
                                SELECT location_id FROM rack 
                                WHERE location_id IN (31,32,33,34)
                            )
                        ) THEN (
                            SELECT MIN(id) FROM location 
                            WHERE id IN (31,32,33,34) 
                            AND id NOT IN (
                                SELECT location_id FROM rack 
                                WHERE location_id IN (31,32,33,34)
                            )
                        )
                        WHEN NOT EXISTS (SELECT 1 FROM rack WHERE location_id = 10002) THEN 10002
                        WHEN NOT EXISTS (SELECT 1 FROM rack WHERE location_id = 20002) THEN 20002
                        ELSE NULL
                    END as target_location
            """,
            "results": {},
            "description": "目標位置選擇（簡化版）",
            "optimization_notes": "簡化目標位置選擇邏輯"
        },

        # 條件 216：出口傳送箱路徑（簡化版）
        {
            "id": 216,
            "conditions": """
                SELECT 
                    'True' as result,
                    217 as next_id,
                    'outlet_selected' as path_type
            """,
            "results": {},
            "description": "出口傳送箱路徑（簡化版）",
            "optimization_notes": "直接轉向最終檢查"
        },

        # 條件 217：重複任務檢查（簡化版）
        {
            "id": 217,
            "conditions": """
                SELECT 
                    CASE 
                        WHEN NOT EXISTS (
                            SELECT 1 FROM task t
                            WHERE t.status_id IN (0, 1, 2)
                            AND t.work_id = 220004
                        ) THEN 'True'
                        ELSE 'False'
                    END as result,
                    NULL as next_id,
                    CASE 
                        WHEN NOT EXISTS (
                            SELECT 1 FROM task t
                            WHERE t.status_id IN (0, 1, 2)
                            AND t.work_id = 220004
                        ) THEN 'True'
                        ELSE 'False'
                    END as end
            """,
            "results": {},
            "description": "重複任務檢查（簡化版）",
            "optimization_notes": "簡化重複任務檢查邏輯"
        }
    ]