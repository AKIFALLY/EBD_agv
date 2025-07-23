#!/usr/bin/env python3
"""
測試 DatabaseManager 重構後的功能
驗證所有資料表操作、任務處理邏輯都能正確執行
"""

import unittest
import sys
import os
from unittest.mock import Mock, MagicMock, patch
import rclpy
from rclpy.node import Node

# 添加路徑以便導入模組
sys.path.append('/app/wcs_ws/src/wcs_base')
sys.path.append('/app/db_proxy_ws/src/db_proxy')
sys.path.append('/app/wcs_ws/src/kuka_wcs')

class TestDatabaseManagerIntegration(unittest.TestCase):
    """測試 DatabaseManager 整合功能"""

    def setUp(self):
        """設置測試環境"""
        # 初始化 ROS2
        if not rclpy.ok():
            rclpy.init()
        
        # 模擬資料庫連接
        self.mock_db_url = 'postgresql+psycopg2://test:test@localhost/test'
        
        # 創建模擬的 logger
        self.mock_logger = Mock()
        self.mock_logger.info = Mock()
        self.mock_logger.error = Mock()
        self.mock_logger.warn = Mock()

    def tearDown(self):
        """清理測試環境"""
        pass

    @patch('wcs_base.database_manager.ConnectionPoolManager')
    def test_database_manager_initialization(self, mock_pool_manager):
        """測試 DatabaseManager 初始化"""
        from wcs_base.database_manager import DatabaseManager
        
        # 創建 DatabaseManager 實例
        db_manager = DatabaseManager(self.mock_logger, self.mock_db_url)
        
        # 驗證初始化
        self.assertIsNotNone(db_manager)
        self.assertEqual(db_manager.logger, self.mock_logger)
        mock_pool_manager.assert_called_once_with(self.mock_db_url)
        
        # 驗證資料表屬性初始化
        self.assertIsNone(db_manager.node_table)
        self.assertIsNone(db_manager.work_table)
        self.assertIsNone(db_manager.location_table)
        self.assertIsNone(db_manager.kuka_node_table)
        self.assertEqual(db_manager.task_table, [])
        self.assertEqual(db_manager.agv_context_table, [])
        self.assertEqual(db_manager.carrier_table, [])
        self.assertEqual(db_manager.rack_table, [])

    @patch('wcs_base.database_manager.ConnectionPoolManager')
    def test_database_manager_properties(self, mock_pool_manager):
        """測試 DatabaseManager 屬性訪問"""
        from wcs_base.database_manager import DatabaseManager
        
        db_manager = DatabaseManager(self.mock_logger, self.mock_db_url)
        
        # 測試屬性訪問
        self.assertEqual(db_manager.nodes, [])
        self.assertEqual(db_manager.works, [])
        self.assertEqual(db_manager.locations, [])
        self.assertEqual(db_manager.kuka_nodes, [])
        self.assertEqual(db_manager.tasks, [])
        self.assertEqual(db_manager.agv_contexts, [])
        self.assertEqual(db_manager.carriers, [])
        self.assertEqual(db_manager.racks, [])
        
        # 測試列表屬性
        self.assertEqual(db_manager.task_ids, [])
        self.assertEqual(db_manager.parent_task_ids, [])
        self.assertEqual(db_manager.node_ids, [])
        self.assertEqual(db_manager.rack_location_ids, [])
        self.assertEqual(db_manager.task_parameters, [])

    @patch('wcs_base.database_manager.ConnectionPoolManager')
    def test_database_manager_has_all_data(self, mock_pool_manager):
        """測試 DatabaseManager has_all_data 方法"""
        from wcs_base.database_manager import DatabaseManager
        
        db_manager = DatabaseManager(self.mock_logger, self.mock_db_url)
        
        # 初始狀態應該返回 False
        self.assertFalse(db_manager.has_all_data())
        
        # 設置一些測試數據
        db_manager.task_table = [Mock()]
        db_manager.task_id_list = [1]
        db_manager.agv_context_table = [Mock()]
        db_manager.work_table = [Mock()]
        db_manager.location_table = [Mock()]
        db_manager.kuka_node_table = [Mock()]
        db_manager.carrier_table = [Mock()]
        db_manager.rack_table = [Mock()]
        
        # 現在應該返回 True
        self.assertTrue(db_manager.has_all_data())

    @patch('wcs_base.wcs_base_node.DatabaseManager')
    @patch('rclpy.node.Node.declare_parameter')
    def test_wcs_base_node_initialization(self, mock_declare_param, mock_db_manager_class):
        """測試 WCSBaseNode 使用 DatabaseManager 的初始化"""
        # 設置模擬參數
        mock_param = Mock()
        mock_param.value = self.mock_db_url
        mock_declare_param.return_value = mock_param
        
        # 模擬 DatabaseManager 實例
        mock_db_manager = Mock()
        mock_db_manager_class.return_value = mock_db_manager
        
        # 導入並創建 WCSBaseNode
        from wcs_base.wcs_base_node import WCSBaseNode
        
        with patch('wcs_base.wcs_base_node.RackRotate180Handler'), \
             patch('wcs_base.wcs_base_node.EmptyRackToBoxoutHandler'), \
             patch('wcs_base.wcs_base_node.FullRackToManualReceiveHandler'), \
             patch('wcs_base.wcs_base_node.BaseTaskHandler'):
            
            node = WCSBaseNode()
            
            # 驗證 DatabaseManager 被正確初始化
            mock_db_manager_class.assert_called_once()
            self.assertEqual(node.db_manager, mock_db_manager)

    @patch('wcs_base.database_manager.ConnectionPoolManager')
    def test_base_task_handler_initialization(self, mock_pool_manager):
        """測試 BaseTaskHandler 使用 DatabaseManager"""
        from wcs_base.database_manager import DatabaseManager
        from wcs_base.base_task_handler import BaseTaskHandler
        
        # 創建模擬的 node
        mock_node = Mock()
        mock_node.get_logger.return_value = self.mock_logger
        
        # 創建 DatabaseManager
        db_manager = DatabaseManager(self.mock_logger, self.mock_db_url)
        mock_node.db_manager = db_manager
        
        # 創建一個具體的 BaseTaskHandler 子類用於測試
        class TestTaskHandler(BaseTaskHandler):
            def check_condition(self):
                return True
            
            def insert_task(self):
                return True
            
            def check_insert_done(self):
                return True
        
        # 創建 TaskHandler 實例
        handler = TestTaskHandler(mock_node)
        
        # 驗證 DatabaseManager 引用被正確設置
        self.assertEqual(handler.db_manager, db_manager)
        self.assertEqual(handler.node, mock_node)

    @patch('wcs_base.database_manager.ConnectionPoolManager')
    def test_task_handler_database_access(self, mock_pool_manager):
        """測試任務處理器通過 DatabaseManager 訪問資料"""
        from wcs_base.database_manager import DatabaseManager
        from wcs_base.base_task_handler import BaseTaskHandler
        
        # 創建模擬的 node
        mock_node = Mock()
        mock_node.get_logger.return_value = self.mock_logger
        
        # 創建 DatabaseManager 並設置測試數據
        db_manager = DatabaseManager(self.mock_logger, self.mock_db_url)
        
        # 模擬一些資料表數據
        mock_carrier = Mock()
        mock_carrier.room_id = 1
        db_manager.carrier_table = [mock_carrier]
        
        mock_rack = Mock()
        mock_rack.id = 1
        mock_rack.location_id = 100
        db_manager.rack_table = [mock_rack]
        
        mock_location = Mock()
        mock_location.id = 100
        mock_location.location_status_id = 2
        db_manager.location_table = [mock_location]
        
        mock_node.db_manager = db_manager
        
        # 創建測試用的 TaskHandler
        class TestTaskHandler(BaseTaskHandler):
            def check_condition(self):
                return True
            
            def insert_task(self):
                return True
            
            def check_insert_done(self):
                return True
        
        handler = TestTaskHandler(mock_node)
        
        # 測試通過 DatabaseManager 訪問資料
        carriers_in_room = handler.check_room_have_carrier()
        self.assertEqual(carriers_in_room, [1])
        
        rack_location = handler.get_rack_location(1)
        self.assertEqual(rack_location, 100)

    def test_integration_workflow(self):
        """測試整體工作流程整合"""
        with patch('wcs_base.database_manager.ConnectionPoolManager'), \
             patch('wcs_base.wcs_base_node.RackRotate180Handler'), \
             patch('wcs_base.wcs_base_node.EmptyRackToBoxoutHandler'), \
             patch('wcs_base.wcs_base_node.FullRackToManualReceiveHandler'), \
             patch('wcs_base.wcs_base_node.BaseTaskHandler'), \
             patch('rclpy.node.Node.declare_parameter') as mock_declare_param:
            
            # 設置模擬參數
            mock_param = Mock()
            mock_param.value = self.mock_db_url
            mock_declare_param.return_value = mock_param
            
            from wcs_base.wcs_base_node import WCSBaseNode
            
            # 創建 WCSBaseNode
            node = WCSBaseNode()
            
            # 驗證 DatabaseManager 存在
            self.assertIsNotNone(node.db_manager)
            
            # 模擬 cycle_process 調用
            with patch.object(node.db_manager, 'refresh_all_tables') as mock_refresh, \
                 patch.object(node.db_manager, 'has_all_data', return_value=True) as mock_has_data:
                
                # 模擬任務處理器
                mock_handler = Mock()
                node.task_handler_list = [mock_handler]
                
                # 執行 cycle_process
                node.cycle_process()
                
                # 驗證調用
                mock_refresh.assert_called_once()
                mock_has_data.assert_called_once()
                mock_handler.execute.assert_called_once()

    def test_error_handling(self):
        """測試錯誤處理"""
        with patch('wcs_base.database_manager.ConnectionPoolManager') as mock_pool_manager:
            from wcs_base.database_manager import DatabaseManager
            
            # 測試資料庫連接錯誤
            mock_pool_manager.side_effect = Exception("Database connection failed")
            
            with self.assertRaises(Exception):
                DatabaseManager(self.mock_logger, "invalid_url")

def run_tests():
    """運行所有測試"""
    print("🧪 開始測試 DatabaseManager 重構功能...")
    
    # 創建測試套件
    test_suite = unittest.TestLoader().loadTestsFromTestCase(TestDatabaseManagerIntegration)
    
    # 運行測試
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    # 輸出結果
    if result.wasSuccessful():
        print("✅ 所有測試通過！DatabaseManager 重構功能正常工作。")
        return True
    else:
        print("❌ 測試失敗！")
        for failure in result.failures:
            print(f"失敗: {failure[0]}")
            print(f"詳情: {failure[1]}")
        for error in result.errors:
            print(f"錯誤: {error[0]}")
            print(f"詳情: {error[1]}")
        return False

if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)
