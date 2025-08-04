"""
優化版：AGV 貨架180度旋轉條件檢查
使用簡化的SELECT查詢，避免SQL驗證失敗

優化重點：
1. 條件 1：AGV 狀態與任務簡化查詢
2. 條件 2：重複任務檢查
3. 避免使用CTE和複雜語法
"""

def get_optimized_agv_rotate_check_conditions():
    """
    取得優化版 AGV 貨架180度旋轉條件檢查的所有條件
    
    Returns:
        list: 包含優化條件定義的列表
    """
    return [
        # 條件 1：AGV 旋轉需求檢查（簡化版）
        {
            "id": 1,
            "conditions": """
                SELECT 
                    CASE 
                        WHEN EXISTS (
                            SELECT 1 FROM agv_context ac
                            INNER JOIN task t ON ac.agv_id = t.agv_id
                            WHERE ac.current_state = 'wait_rotation_state'
                            AND t.status_id IN (0, 1, 2)
                            AND NOT EXISTS (
                                SELECT 1 FROM task child_task 
                                WHERE child_task.parent_task_id = t.id
                            )
                        ) THEN 'True'
                        ELSE 'False'
                    END as result,
                    CASE 
                        WHEN EXISTS (
                            SELECT 1 FROM agv_context ac
                            INNER JOIN task t ON ac.agv_id = t.agv_id
                            WHERE ac.current_state = 'wait_rotation_state'
                            AND t.status_id IN (0, 1, 2)
                            AND NOT EXISTS (
                                SELECT 1 FROM task child_task 
                                WHERE child_task.parent_task_id = t.id
                            )
                        ) THEN 2
                        ELSE NULL
                    END as next_id,
                    (
                        SELECT ac.agv_id FROM agv_context ac
                        INNER JOIN task t ON ac.agv_id = t.agv_id
                        WHERE ac.current_state = 'wait_rotation_state'
                        AND t.status_id IN (0, 1, 2)
                        AND NOT EXISTS (
                            SELECT 1 FROM task child_task 
                            WHERE child_task.parent_task_id = t.id
                        )
                        ORDER BY t.priority DESC, t.id ASC
                        LIMIT 1
                    ) as selected_agv_id,
                    (
                        SELECT t.id FROM agv_context ac
                        INNER JOIN task t ON ac.agv_id = t.agv_id
                        WHERE ac.current_state = 'wait_rotation_state'
                        AND t.status_id IN (0, 1, 2)
                        AND NOT EXISTS (
                            SELECT 1 FROM task child_task 
                            WHERE child_task.parent_task_id = t.id
                        )
                        ORDER BY t.priority DESC, t.id ASC
                        LIMIT 1
                    ) as selected_task_id
            """,
            "results": {},
            "description": "AGV 旋轉需求檢查（簡化版）",
            "optimization_notes": "簡化為基本SELECT查詢，避免CTE語法"
        },

        # 條件 2：旋轉任務重複檢查（簡化版）
        {
            "id": 2,
            "conditions": """
                SELECT 
                    CASE 
                        WHEN NOT EXISTS (
                            SELECT 1 FROM task t
                            WHERE t.status_id IN (0, 1, 2)
                            AND t.work_id = 20003
                        ) THEN 'True'
                        ELSE 'False'
                    END as result,
                    NULL as next_id,
                    CASE 
                        WHEN NOT EXISTS (
                            SELECT 1 FROM task t
                            WHERE t.status_id IN (0, 1, 2)
                            AND t.work_id = 20003
                        ) THEN 'True'
                        ELSE 'False'
                    END as end
            """,
            "results": {},
            "description": "旋轉任務重複檢查（簡化版）",
            "optimization_notes": "簡化重複任務檢查邏輯"
        }
    ]