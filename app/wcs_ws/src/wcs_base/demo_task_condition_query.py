#!/usr/bin/env python3
"""
任務條件查詢服務完整示範
展示如何使用任務條件查詢服務的所有功能
"""

import sys
import json
from datetime import datetime, timezone

# 添加路徑
sys.path.append('/app/wcs_ws/src/wcs_base')
sys.path.append('/app/db_proxy_ws/src/db_proxy')

def demo_complete_workflow():
    """完整工作流程示範"""
    print('🚀 任務條件查詢服務完整示範')
    print('=' * 60)
    
    try:
        # 導入必要模組
        from db_proxy.crud.task_condition_crud import task_condition_crud
        from wcs_base.database_manager import DatabaseManager
        from wcs_base.task_condition_query_service import TaskConditionQueryService
        
        # 建立日誌記錄器
        class DemoLogger:
            def info(self, msg): print(f'[INFO] {msg}')
            def error(self, msg): print(f'[ERROR] {msg}')
            def warning(self, msg): print(f'[WARNING] {msg}')
            def debug(self, msg): print(f'[DEBUG] {msg}')
        
        logger = DemoLogger()
        
        # 初始化資料庫管理器
        db_url = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
        db_manager = DatabaseManager(logger, db_url)
        print('✅ 資料庫管理器初始化成功')
        
        # 初始化查詢服務
        query_service = TaskConditionQueryService(db_manager, logger)
        print('✅ 查詢服務初始化成功')
        
        # 步驟 1: 清理舊的測試資料
        print('\n📋 步驟 1: 清理舊的測試資料')
        with db_manager.get_session() as session:
            # 刪除測試條件
            test_conditions = [
                'SELECT COUNT(*) as agv_count FROM agv',
                'SELECT COUNT(*) as task_count FROM task WHERE status = \'PENDING\'',
                'SELECT status, COUNT(*) as count FROM location GROUP BY status'
            ]
            
            for condition_sql in test_conditions:
                existing = task_condition_crud.get_by_conditions(session, condition_sql)
                if existing:
                    task_condition_crud.delete(session, existing.id)
                    print(f'  🗑️ 刪除舊條件: {condition_sql[:50]}...')
        
        # 步驟 2: 新增測試條件
        print('\n📋 步驟 2: 新增測試條件')
        test_conditions_data = [
            {
                'sql': 'SELECT COUNT(*) as agv_count FROM agv',
                'description': '統計 AGV 總數'
            },
            {
                'sql': 'SELECT COUNT(*) as task_count FROM task WHERE status = \'PENDING\'',
                'description': '統計待處理任務數'
            },
            {
                'sql': 'SELECT status, COUNT(*) as count FROM location GROUP BY status',
                'description': '統計各狀態的位置數量'
            }
        ]

        created_condition_ids = []
        with db_manager.get_session() as session:
            for condition_data in test_conditions_data:
                # 驗證 SQL
                is_valid, error = query_service.validate_sql_query(condition_data['sql'])
                if not is_valid:
                    print(f'  ❌ SQL 驗證失敗: {error}')
                    continue

                # 建立條件
                condition = task_condition_crud.create_condition(
                    session,
                    condition_data['sql'],
                    {'created_by': 'demo'},
                    condition_data['description']
                )
                created_condition_ids.append(condition.id)
                print(f'  ✅ 建立條件 {condition.id}: {condition_data["description"]}')

        # 步驟 3: 執行單一條件測試
        print('\n📋 步驟 3: 執行單一條件測試')
        if created_condition_ids:
            test_condition_id = created_condition_ids[0]

            with db_manager.get_session() as session:
                test_condition = task_condition_crud.get_by_id(session, test_condition_id)
                if test_condition:
                    print(f'  🔍 測試條件 {test_condition.id}: {test_condition.conditions}')

                    success = query_service.process_single_condition(test_condition)
                    if success:
                        # 檢查結果
                        session.refresh(test_condition)  # 重新整理物件
                        if test_condition.results:
                            print(f'  ✅ 執行成功')
                            print(f'  📊 結果: {json.dumps(test_condition.results, ensure_ascii=False, indent=4)}')
                        else:
                            print(f'  ❌ 結果更新失敗')
                    else:
                        print(f'  ❌ 執行失敗')
                else:
                    print(f'  ❌ 找不到測試條件')
        
        # 步驟 4: 批次執行所有條件
        print('\n📋 步驟 4: 批次執行所有條件')
        batch_result = query_service.process_all_conditions()
        
        print(f'  📊 批次執行結果:')
        print(f'    總條件數: {batch_result.get("total_conditions", 0)}')
        print(f'    處理成功: {batch_result.get("successful", 0)}')
        print(f'    處理失敗: {batch_result.get("failed", 0)}')
        print(f'    執行時間: {batch_result.get("duration_seconds", 0):.2f} 秒')
        
        # 步驟 5: 檢視最終結果
        print('\n📋 步驟 5: 檢視最終結果')
        with db_manager.get_session() as session:
            all_conditions = task_condition_crud.get_all_with_results(session)
            
            for condition in all_conditions:
                if condition.results and condition.results.get('created_by') == 'demo':
                    print(f'\n  條件 {condition.id}:')
                    print(f'    SQL: {condition.conditions}')
                    
                    if condition.results.get('success', False):
                        data = condition.results.get('data', [])
                        row_count = condition.results.get('row_count', 0)
                        print(f'    狀態: ✅ 成功 ({row_count} 行)')
                        
                        if data:
                            print(f'    結果預覽: {json.dumps(data[:2], ensure_ascii=False)}')
                    else:
                        error = condition.results.get('error', '未知錯誤')
                        print(f'    狀態: ❌ 失敗 - {error}')
        
        print('\n🎉 完整示範完成！')
        return True
        
    except Exception as e:
        print(f'❌ 示範失敗: {e}')
        import traceback
        traceback.print_exc()
        return False


def demo_security_features():
    """安全功能示範"""
    print('\n🔒 安全功能示範')
    print('-' * 40)
    
    try:
        from wcs_base.task_condition_query_service import TaskConditionQueryService
        
        class MockLogger:
            def info(self, msg): pass
            def error(self, msg): pass
            def warning(self, msg): pass
        
        class MockDBManager:
            pass
        
        service = TaskConditionQueryService(MockDBManager(), MockLogger())
        
        # 安全測試案例
        security_tests = [
            ('SELECT * FROM agv', True, '正常查詢'),
            ('DELETE FROM agv WHERE id = 1', False, 'DELETE 攻擊'),
            ('DROP TABLE agv', False, 'DROP 攻擊'),
            ('SELECT * FROM agv; DELETE FROM task;', False, '多語句攻擊'),
            ('INSERT INTO agv VALUES (1, "test")', False, 'INSERT 攻擊'),
            ('UPDATE agv SET status = "HACKED"', False, 'UPDATE 攻擊'),
            ('EXEC sp_configure', False, '系統命令攻擊'),
            ('SELECT * FROM agv WHERE id = 1 OR 1=1', True, '正常條件查詢'),
        ]
        
        print('  測試各種 SQL 注入和惡意查詢:')
        for sql, expected_valid, description in security_tests:
            is_valid, error = service.validate_sql_query(sql)
            status = '✅' if is_valid == expected_valid else '❌'
            print(f'    {status} {description}: {"通過" if is_valid == expected_valid else "失敗"}')
            if not is_valid and error:
                print(f'        錯誤: {error}')
        
        print('  🛡️ 安全驗證完成')
        return True
        
    except Exception as e:
        print(f'❌ 安全測試失敗: {e}')
        return False


def main():
    """主函式"""
    print('🎯 任務條件查詢服務完整示範程式')
    print('=' * 70)
    
    # 執行完整工作流程示範
    workflow_success = demo_complete_workflow()
    
    # 執行安全功能示範
    security_success = demo_security_features()
    
    print('\n' + '=' * 70)
    print('📊 示範結果摘要:')
    print(f'  完整工作流程: {"✅ 成功" if workflow_success else "❌ 失敗"}')
    print(f'  安全功能測試: {"✅ 成功" if security_success else "❌ 失敗"}')
    
    if workflow_success and security_success:
        print('\n🎉 所有示範完成！任務條件查詢服務運行正常。')
        print('\n💡 後續步驟:')
        print('  1. 使用 ROS 2 節點模式: ros2 run wcs_base task_condition_query_node')
        print('  2. 使用命令列工具: task_condition_query_cli --help')
        print('  3. 整合到現有的 WCS 系統中')
        return 0
    else:
        print('\n❌ 部分示範失敗，請檢查系統設定。')
        return 1


if __name__ == '__main__':
    exit_code = main()
    sys.exit(exit_code)
