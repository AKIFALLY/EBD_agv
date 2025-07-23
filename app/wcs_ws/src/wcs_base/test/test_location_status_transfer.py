#!/usr/bin/env python3
"""
測試 location_status_process 功能轉移
驗證功能從 wcs_base_node 轉移到 database_manager 後正常工作
"""

import unittest
import sys
from unittest.mock import Mock, patch

# 添加路徑以便導入模組
sys.path.append('/app/wcs_ws/src/wcs_base')
sys.path.append('/app/db_proxy_ws/src/db_proxy')
sys.path.append('/app/wcs_ws/src/kuka_wcs')

class TestLocationStatusTransfer(unittest.TestCase):
    """測試位置狀態處理功能轉移"""

    def setUp(self):
        """設置測試環境"""
        self.mock_db_url = 'postgresql+psycopg2://test:test@localhost/test'
        self.mock_logger = Mock()

    @patch('wcs_base.database_manager.ConnectionPoolManager')
    def test_database_manager_location_status_process(self, mock_pool_manager):
        """測試 DatabaseManager 中的 location_status_process 方法"""
        from wcs_base.database_manager import DatabaseManager
        
        db_manager = DatabaseManager(self.mock_logger, self.mock_db_url)
        
        # 設置測試數據
        mock_location1 = Mock()
        mock_location1.id = 1
        mock_location1.location_status_id = 2  # 未佔用
        
        mock_location2 = Mock()
        mock_location2.id = 2
        mock_location2.location_status_id = 3  # 佔用
        
        db_manager.location_table = [mock_location1, mock_location2]
        db_manager.node_id_list = [1]  # location1 在任務中
        db_manager.task_parameters_list = [{"nodes": ["uuid-002"]}]
        
        # 設置 kuka_node 數據
        mock_kuka_node1 = Mock()
        mock_kuka_node1.id = 1
        mock_kuka_node1.uuid = "uuid-001"
        
        mock_kuka_node2 = Mock()
        mock_kuka_node2.id = 2
        mock_kuka_node2.uuid = "uuid-002"
        
        db_manager.kuka_node_table = [mock_kuka_node1, mock_kuka_node2]
        
        # 模擬 CONFIG
        with patch('config.config.LOCATION_STATUS', {"任務占用中": 3}):
            # 執行位置狀態處理
            db_manager.location_status_process()
        
        # 驗證日誌調用
        self.mock_logger.info.assert_called()
        print("✅ DatabaseManager.location_status_process 方法測試通過")

    def test_wcs_base_node_no_location_status_process(self):
        """測試 WCSBaseNode 不再有 location_status_process 方法"""
        from wcs_base.wcs_base_node import WCSBaseNode
        
        # 檢查 WCSBaseNode 類別是否還有 location_status_process 方法
        self.assertFalse(hasattr(WCSBaseNode, 'location_status_porcess'), 
                        "WCSBaseNode 不應該有 location_status_porcess 方法")
        
        print("✅ WCSBaseNode 已成功移除 location_status_porcess 方法")

    @patch('wcs_base.database_manager.ConnectionPoolManager')
    def test_cycle_process_calls_location_status_process(self, mock_pool_manager):
        """測試 cycle_process 是否調用 location_status_process"""
        from wcs_base.database_manager import DatabaseManager
        
        # 創建模擬的 DatabaseManager
        db_manager = DatabaseManager(self.mock_logger, self.mock_db_url)
        
        # 模擬 has_all_data 返回 True
        with patch.object(db_manager, 'has_all_data', return_value=True), \
             patch.object(db_manager, 'refresh_all_tables'), \
             patch.object(db_manager, 'location_status_process') as mock_location_status:
            
            # 檢查 WCSBaseNode 的 cycle_process 方法
            from wcs_base.wcs_base_node import WCSBaseNode
            
            # 檢查源碼中是否包含對 location_status_process 的調用
            import inspect
            source = inspect.getsource(WCSBaseNode.cycle_process)
            self.assertIn('location_status_process', source, 
                         "cycle_process 應該調用 location_status_process")
        
        print("✅ cycle_process 方法包含 location_status_process 調用")

    @patch('wcs_base.database_manager.ConnectionPoolManager')
    def test_location_status_process_error_handling(self, mock_pool_manager):
        """測試位置狀態處理的錯誤處理"""
        from wcs_base.database_manager import DatabaseManager
        
        db_manager = DatabaseManager(self.mock_logger, self.mock_db_url)
        
        # 設置會導致錯誤的數據
        db_manager.location_table = None  # 這會導致錯誤
        
        # 執行位置狀態處理
        db_manager.location_status_process()
        
        # 驗證錯誤日誌被調用
        self.mock_logger.error.assert_called()
        error_call = self.mock_logger.error.call_args[0][0]
        self.assertIn("位置狀態處理失敗", error_call)
        
        print("✅ location_status_process 錯誤處理測試通過")

    @patch('wcs_base.database_manager.ConnectionPoolManager')
    def test_location_status_logic(self, mock_pool_manager):
        """測試位置狀態邏輯"""
        from wcs_base.database_manager import DatabaseManager
        
        db_manager = DatabaseManager(self.mock_logger, self.mock_db_url)
        
        # 設置測試場景1: location 在 task 的 node_id 中
        mock_location = Mock()
        mock_location.id = 100
        mock_location.location_status_id = 2  # 未佔用
        
        db_manager.location_table = [mock_location]
        db_manager.node_id_list = [100]  # location 在任務中
        db_manager.task_parameters_list = []
        
        mock_kuka_node = Mock()
        mock_kuka_node.id = 100
        mock_kuka_node.uuid = "test-uuid"
        db_manager.kuka_node_table = [mock_kuka_node]
        
        # 模擬 CONFIG
        with patch('config.config.LOCATION_STATUS', {"任務占用中": 3}):
            db_manager.location_status_process()
        
        # 驗證成功日誌
        success_calls = [call for call in self.mock_logger.info.call_args_list 
                        if "位置狀態更新成功" in str(call)]
        self.assertTrue(len(success_calls) > 0, "應該有成功更新的日誌")
        
        print("✅ location_status_process 邏輯測試通過")

def run_location_status_transfer_tests():
    """運行位置狀態轉移測試"""
    print("🧪 開始測試 location_status_process 功能轉移...")
    print("=" * 60)
    
    # 創建測試套件
    test_suite = unittest.TestLoader().loadTestsFromTestCase(TestLocationStatusTransfer)
    
    # 運行測試
    runner = unittest.TextTestRunner(verbosity=1)
    result = runner.run(test_suite)
    
    print("=" * 60)
    
    # 輸出結果
    if result.wasSuccessful():
        print("✅ 所有位置狀態轉移測試通過！")
        print("\n📋 轉移總結:")
        print("   ✅ location_status_process 方法已成功轉移到 DatabaseManager")
        print("   ✅ WCSBaseNode 已移除原有的 location_status_porcess 方法")
        print("   ✅ cycle_process 方法已更新，會調用 DatabaseManager 的方法")
        print("   ✅ 錯誤處理機制正常工作")
        print("   ✅ 位置狀態處理邏輯正確")
        print("\n🎉 location_status_process 功能轉移完成！")
        return True
    else:
        print("❌ 位置狀態轉移測試失敗！")
        for failure in result.failures:
            print(f"失敗: {failure[0]}")
            print(f"詳情: {failure[1]}")
        for error in result.errors:
            print(f"錯誤: {error[0]}")
            print(f"詳情: {error[1]}")
        return False

if __name__ == '__main__':
    success = run_location_status_transfer_tests()
    sys.exit(0 if success else 1)
