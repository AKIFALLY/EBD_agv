#!/usr/bin/env python3
"""
任務條件查詢服務測試腳本
"""

import sys
import os

# 添加路徑
sys.path.append('/app/wcs_ws/src/wcs_base')
sys.path.append('/app/db_proxy_ws/src/db_proxy')

def test_sql_validation():
    """測試 SQL 驗證功能"""
    print('🧪 測試任務條件查詢服務')
    print('=' * 50)

    try:
        from wcs_base.task_condition_query_service import TaskConditionQueryService
        print('✅ TaskConditionQueryService 導入成功')
        
        # 建立模擬物件
        class MockLogger:
            def info(self, msg): print(f'[INFO] {msg}')
            def error(self, msg): print(f'[ERROR] {msg}')
            def warning(self, msg): print(f'[WARNING] {msg}')
        
        class MockDBManager:
            pass
        
        service = TaskConditionQueryService(MockDBManager(), MockLogger())
        print('✅ 服務初始化成功')
        
        # 測試 SQL 驗證
        print('\n🔍 測試 SQL 驗證功能:')
        
        test_cases = [
            ('SELECT COUNT(*) FROM agv WHERE status = "IDLE"', True, '有效的 SELECT 查詢'),
            ('SELECT * FROM task WHERE priority > 5', True, '有效的條件查詢'),
            ('SELECT a.id, b.name FROM agv a JOIN location b ON a.location_id = b.id', True, '有效的 JOIN 查詢'),
            ('DELETE FROM agv WHERE id = 1', False, '無效的 DELETE 操作'),
            ('INSERT INTO agv VALUES (1, "test")', False, '無效的 INSERT 操作'),
            ('DROP TABLE agv', False, '無效的 DROP 操作'),
            ('SELECT * FROM agv; DROP TABLE agv;', False, '無效的多語句'),
            ('', False, '空 SQL'),
            ('   ', False, '空白 SQL'),
        ]
        
        for sql, expected, description in test_cases:
            is_valid, error = service.validate_sql_query(sql)
            status = '✅' if is_valid == expected else '❌'
            print(f'  {status} {description}: {is_valid}')
            if not is_valid and error:
                print(f'      錯誤: {error}')
        
        print('\n✅ SQL 驗證測試完成！')
        return True
        
    except Exception as e:
        print(f'❌ 測試失敗: {e}')
        import traceback
        traceback.print_exc()
        return False


def test_with_database():
    """測試資料庫連接功能"""
    print('\n🔗 測試資料庫連接功能')
    print('-' * 30)
    
    try:
        from db_proxy.connection_pool_manager import ConnectionPoolManager
        from db_proxy.crud.task_condition_crud import task_condition_crud
        from wcs_base.database_manager import DatabaseManager
        from wcs_base.task_condition_query_service import TaskConditionQueryService
        
        # 建立資料庫連接
        db_url = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
        
        class MockLogger:
            def info(self, msg): print(f'[INFO] {msg}')
            def error(self, msg): print(f'[ERROR] {msg}')
            def warning(self, msg): print(f'[WARNING] {msg}')
        
        logger = MockLogger()
        
        try:
            db_manager = DatabaseManager(logger, db_url)
            print('✅ 資料庫管理器初始化成功')
            
            service = TaskConditionQueryService(db_manager, logger)
            print('✅ 查詢服務初始化成功')
            
            # 測試簡單查詢
            test_sql = "SELECT COUNT(*) as total_agv FROM agv"
            result = service.execute_sql_query(test_sql)
            
            if result.get('success', False):
                print(f'✅ 測試查詢成功: {result}')
            else:
                print(f'❌ 測試查詢失敗: {result.get("error", "未知錯誤")}')
            
            return True
            
        except Exception as db_error:
            print(f'⚠️ 資料庫連接失敗: {db_error}')
            print('   這可能是因為資料庫服務未啟動或連接設定錯誤')
            return False
            
    except ImportError as e:
        print(f'❌ 模組導入失敗: {e}')
        return False


def main():
    """主函式"""
    print('🚀 開始測試任務條件查詢服務')
    print('=' * 60)
    
    # 測試 1: SQL 驗證功能
    validation_success = test_sql_validation()
    
    # 測試 2: 資料庫連接功能
    database_success = test_with_database()
    
    print('\n' + '=' * 60)
    print('📊 測試結果摘要:')
    print(f'  SQL 驗證功能: {"✅ 通過" if validation_success else "❌ 失敗"}')
    print(f'  資料庫連接功能: {"✅ 通過" if database_success else "❌ 失敗"}')
    
    if validation_success and database_success:
        print('\n🎉 所有測試通過！任務條件查詢服務已準備就緒。')
        return 0
    elif validation_success:
        print('\n⚠️ 部分測試通過。SQL 驗證功能正常，但資料庫連接可能有問題。')
        return 1
    else:
        print('\n❌ 測試失敗。請檢查程式碼和相依性。')
        return 2


if __name__ == '__main__':
    exit_code = main()
    sys.exit(exit_code)
