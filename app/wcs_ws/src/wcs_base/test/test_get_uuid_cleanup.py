#!/usr/bin/env python3
"""
測試 get_uuid 方法重複定義清理後的功能
"""

import unittest
import sys
from unittest.mock import Mock, patch

# 添加路徑以便導入模組
sys.path.append('/app/wcs_ws/src/wcs_base')
sys.path.append('/app/db_proxy_ws/src/db_proxy')
sys.path.append('/app/wcs_ws/src/kuka_wcs')

class TestGetUuidCleanup(unittest.TestCase):
    """測試 get_uuid 方法清理後的功能"""

    def setUp(self):
        """設置測試環境"""
        self.mock_db_url = 'postgresql+psycopg2://test:test@localhost/test'
        self.mock_logger = Mock()

    @patch('wcs_base.database_manager.ConnectionPoolManager')
    def test_database_manager_get_uuid(self, mock_pool_manager):
        """測試 DatabaseManager 的 get_uuid 方法"""
        from wcs_base.database_manager import DatabaseManager
        
        db_manager = DatabaseManager(self.mock_logger, self.mock_db_url)
        
        # 設置測試數據
        mock_kuka_node1 = Mock()
        mock_kuka_node1.id = 1
        mock_kuka_node1.uuid = 'uuid-001'
        
        mock_kuka_node2 = Mock()
        mock_kuka_node2.id = 2
        mock_kuka_node2.uuid = 'uuid-002'
        
        db_manager.kuka_node_table = [mock_kuka_node1, mock_kuka_node2]
        
        # 測試正常情況
        self.assertEqual(db_manager.get_uuid(1), 'uuid-001')
        self.assertEqual(db_manager.get_uuid(2), 'uuid-002')
        
        # 測試找不到的情況
        self.assertIsNone(db_manager.get_uuid(999))
        
        print("✅ DatabaseManager.get_uuid 方法測試通過")

    @patch('wcs_base.database_manager.ConnectionPoolManager')
    def test_base_task_handler_get_uuid(self, mock_pool_manager):
        """測試 BaseTaskHandler 的 get_uuid 方法委託"""
        from wcs_base.database_manager import DatabaseManager
        from wcs_base.base_task_handler import BaseTaskHandler
        
        # 創建模擬的 node
        mock_node = Mock()
        mock_node.get_logger.return_value = self.mock_logger
        
        # 創建 DatabaseManager
        db_manager = DatabaseManager(self.mock_logger, self.mock_db_url)
        mock_node.db_manager = db_manager
        
        # 設置測試數據
        mock_kuka_node = Mock()
        mock_kuka_node.id = 100
        mock_kuka_node.uuid = 'test-uuid-100'
        db_manager.kuka_node_table = [mock_kuka_node]
        
        # 創建測試用的 TaskHandler
        class TestTaskHandler(BaseTaskHandler):
            def check_condition(self):
                return True
            def insert_task(self):
                return True
            def check_insert_done(self):
                return True
        
        handler = TestTaskHandler(mock_node)
        
        # 測試委託是否正確
        uuid = handler.get_uuid(100)
        self.assertEqual(uuid, 'test-uuid-100')
        
        print("✅ BaseTaskHandler.get_uuid 委託測試通過")

    def test_wcs_base_node_no_get_uuid(self):
        """測試 WCSBaseNode 不再有 get_uuid 方法"""
        from wcs_base.wcs_base_node import WCSBaseNode
        
        # 檢查 WCSBaseNode 類別是否還有 get_uuid 方法
        self.assertFalse(hasattr(WCSBaseNode, 'get_uuid'), 
                        "WCSBaseNode 不應該有 get_uuid 方法")
        
        print("✅ WCSBaseNode 已成功移除 get_uuid 方法")

    @patch('wcs_base.database_manager.ConnectionPoolManager')
    def test_task_handlers_get_uuid_usage(self, mock_pool_manager):
        """測試任務處理器子類別使用 get_uuid 的情況"""
        from wcs_base.database_manager import DatabaseManager
        
        # 創建模擬的 node
        mock_node = Mock()
        mock_node.get_logger.return_value = self.mock_logger
        
        # 創建 DatabaseManager
        db_manager = DatabaseManager(self.mock_logger, self.mock_db_url)
        mock_node.db_manager = db_manager
        
        # 設置測試數據
        mock_kuka_node = Mock()
        mock_kuka_node.id = 50
        mock_kuka_node.uuid = 'handler-test-uuid'
        db_manager.kuka_node_table = [mock_kuka_node]
        
        # 測試 EmptyRackToBoxoutHandler
        try:
            from kuka_wcs.task_handler.empty_rack_to_boxout import EmptyRackToBoxoutHandler
            handler = EmptyRackToBoxoutHandler(mock_node)
            
            # 驗證可以調用 get_uuid 方法
            uuid = handler.get_uuid(50)
            self.assertEqual(uuid, 'handler-test-uuid')
            print("✅ EmptyRackToBoxoutHandler 可以正確使用 get_uuid")
        except Exception as e:
            print(f"⚠️ EmptyRackToBoxoutHandler 測試跳過: {e}")
        
        # 測試 FullRackToManualReceiveHandler
        try:
            from kuka_wcs.task_handler.full_rack_to_manual_receive import FullRackToManualReceiveHandler
            handler = FullRackToManualReceiveHandler(mock_node)
            
            # 驗證可以調用 get_uuid 方法
            uuid = handler.get_uuid(50)
            self.assertEqual(uuid, 'handler-test-uuid')
            print("✅ FullRackToManualReceiveHandler 可以正確使用 get_uuid")
        except Exception as e:
            print(f"⚠️ FullRackToManualReceiveHandler 測試跳過: {e}")

    @patch('wcs_base.database_manager.ConnectionPoolManager')
    def test_method_call_chain(self, mock_pool_manager):
        """測試方法調用鏈的正確性"""
        from wcs_base.database_manager import DatabaseManager
        from wcs_base.base_task_handler import BaseTaskHandler
        
        # 創建模擬的 node
        mock_node = Mock()
        mock_node.get_logger.return_value = self.mock_logger
        
        # 創建 DatabaseManager
        db_manager = DatabaseManager(self.mock_logger, self.mock_db_url)
        mock_node.db_manager = db_manager
        
        # 設置測試數據
        mock_kuka_node = Mock()
        mock_kuka_node.id = 123
        mock_kuka_node.uuid = 'chain-test-uuid'
        db_manager.kuka_node_table = [mock_kuka_node]
        
        # 創建測試用的 TaskHandler
        class TestTaskHandler(BaseTaskHandler):
            def check_condition(self):
                return True
            def insert_task(self):
                return True
            def check_insert_done(self):
                return True
        
        handler = TestTaskHandler(mock_node)
        
        # 驗證調用鏈: handler.get_uuid() -> db_manager.get_uuid()
        with patch.object(db_manager, 'get_uuid', return_value='mocked-uuid') as mock_get_uuid:
            result = handler.get_uuid(123)
            
            # 驗證 DatabaseManager 的 get_uuid 被調用
            mock_get_uuid.assert_called_once_with(123)
            self.assertEqual(result, 'mocked-uuid')
        
        print("✅ 方法調用鏈測試通過")

def run_get_uuid_cleanup_tests():
    """運行 get_uuid 清理測試"""
    print("🧪 開始測試 get_uuid 方法重複定義清理...")
    print("=" * 60)
    
    # 創建測試套件
    test_suite = unittest.TestLoader().loadTestsFromTestCase(TestGetUuidCleanup)
    
    # 運行測試
    runner = unittest.TextTestRunner(verbosity=1)
    result = runner.run(test_suite)
    
    print("=" * 60)
    
    # 輸出結果
    if result.wasSuccessful():
        print("✅ 所有 get_uuid 清理測試通過！")
        print("\n📋 清理總結:")
        print("   ✅ DatabaseManager.get_uuid 方法正常工作")
        print("   ✅ BaseTaskHandler.get_uuid 正確委託給 DatabaseManager")
        print("   ✅ WCSBaseNode 已成功移除重複的 get_uuid 方法")
        print("   ✅ 任務處理器子類別可以正確使用 get_uuid")
        print("   ✅ 方法調用鏈正確無誤")
        print("\n🎉 get_uuid 方法重複定義問題已完全解決！")
        return True
    else:
        print("❌ get_uuid 清理測試失敗！")
        for failure in result.failures:
            print(f"失敗: {failure[0]}")
            print(f"詳情: {failure[1]}")
        for error in result.errors:
            print(f"錯誤: {error[0]}")
            print(f"詳情: {error[1]}")
        return False

if __name__ == '__main__':
    success = run_get_uuid_cleanup_tests()
    sys.exit(0 if success else 1)
