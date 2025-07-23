#!/usr/bin/env python3
"""
測試 ReadyRackToBoxinHandler 任務處理器
驗證準備區料架送往入口傳送箱的功能實作
"""

import unittest
import sys
from unittest.mock import Mock, patch

# 添加路徑以便導入模組
sys.path.append('/app/wcs_ws/src/wcs_base')
sys.path.append('/app/db_proxy_ws/src/db_proxy')
sys.path.append('/app/wcs_ws/src/kuka_wcs')

class TestReadyRackToBoxinHandler(unittest.TestCase):
    """測試 ReadyRackToBoxinHandler 功能"""

    def setUp(self):
        """設置測試環境"""
        self.mock_db_url = 'postgresql+psycopg2://test:test@localhost/test'
        self.mock_logger = Mock()

    @patch('wcs_base.database_manager.ConnectionPoolManager')
    def test_ready_rack_to_boxin_handler_initialization(self, mock_pool_manager):
        """測試 ReadyRackToBoxinHandler 初始化"""
        from wcs_base.database_manager import DatabaseManager
        
        # 創建模擬的 node
        mock_node = Mock()
        mock_node.get_logger.return_value = self.mock_logger
        
        # 創建 DatabaseManager
        db_manager = DatabaseManager(self.mock_logger, self.mock_db_url)
        mock_node.db_manager = db_manager
        
        # 測試 ReadyRackToBoxinHandler 初始化
        try:
            from kuka_wcs.task_handler.ready_rack_to_boxin import ReadyRackToBoxinHandler
            handler = ReadyRackToBoxinHandler(mock_node)
            
            # 驗證初始化
            self.assertEqual(handler.node, mock_node)
            self.assertEqual(handler.db_manager, db_manager)
            self.assertFalse(handler.find_task)
            self.assertFalse(handler.task_inserted)
            self.assertIsNone(handler.task_node_id)
            self.assertIsNone(handler.task_room_id)
            
            print("✅ ReadyRackToBoxinHandler 初始化測試通過")
            
        except Exception as e:
            self.fail(f"ReadyRackToBoxinHandler 初始化失敗: {e}")

    @patch('wcs_base.database_manager.ConnectionPoolManager')
    def test_check_condition_no_ready_rack(self, mock_pool_manager):
        """測試檢查條件 - 系統準備區沒有料架"""
        from wcs_base.database_manager import DatabaseManager
        from kuka_wcs.task_handler.ready_rack_to_boxin import ReadyRackToBoxinHandler
        
        # 創建模擬的 node
        mock_node = Mock()
        mock_node.get_logger.return_value = self.mock_logger
        
        # 創建 DatabaseManager
        db_manager = DatabaseManager(self.mock_logger, self.mock_db_url)
        mock_node.db_manager = db_manager
        
        handler = ReadyRackToBoxinHandler(mock_node)
        
        # 模擬系統準備區沒有料架
        with patch.object(handler, 'system_ready_area_status', return_value=(8, [11, 12, 13, 14, 15, 16, 17, 18], 0, [])):
            result = handler.check_condition()
            self.assertFalse(result)
            # 注意：find_task 的狀態由 BaseTaskHandler.execute() 管理，不在 check_condition 中設置
            
        print("✅ 系統準備區沒有料架條件測試通過")

    @patch('wcs_base.database_manager.ConnectionPoolManager')
    def test_check_condition_success(self, mock_pool_manager):
        """測試檢查條件 - 條件成立（新邏輯）"""
        from wcs_base.database_manager import DatabaseManager
        from kuka_wcs.task_handler.ready_rack_to_boxin import ReadyRackToBoxinHandler

        # 創建模擬的 node
        mock_node = Mock()
        mock_node.get_logger.return_value = self.mock_logger

        # 創建 DatabaseManager
        db_manager = DatabaseManager(self.mock_logger, self.mock_db_url)
        mock_node.db_manager = db_manager

        # 設置測試數據 - 模擬 rack 資料表
        mock_rack = Mock()
        mock_rack.id = 1001
        mock_rack.location_id = 13  # 在準備區的位置
        mock_rack.room_id = 1  # 有 room_id

        db_manager.rack_table = [mock_rack]
        db_manager.rack_location_id_list = [100, 200]  # 現有料架位置（不包含目標入口位置）

        handler = ReadyRackToBoxinHandler(mock_node)

        # 模擬條件成立的情況
        with patch.object(handler, 'system_ready_area_status', return_value=(6, [11, 12], 2, [13, 14])), \
             patch.object(handler, 'convert_to_node_id', return_value=10001), \
             patch.object(handler, 'check_kuka_task_doing', return_value=False):

            result = handler.check_condition()
            self.assertTrue(result)
            self.assertTrue(handler.find_task)
            self.assertEqual(handler.task_node_id, 10001)
            self.assertEqual(handler.task_room_id, 1)

        print("✅ 條件成立測試通過（新邏輯）")

    @patch('wcs_base.database_manager.ConnectionPoolManager')
    def test_check_condition_no_room_id(self, mock_pool_manager):
        """測試檢查條件 - rack 沒有 room_id"""
        from wcs_base.database_manager import DatabaseManager
        from kuka_wcs.task_handler.ready_rack_to_boxin import ReadyRackToBoxinHandler

        # 創建模擬的 node
        mock_node = Mock()
        mock_node.get_logger.return_value = self.mock_logger

        # 創建 DatabaseManager
        db_manager = DatabaseManager(self.mock_logger, self.mock_db_url)
        mock_node.db_manager = db_manager

        # 設置測試數據 - 模擬 rack 沒有 room_id
        mock_rack = Mock()
        mock_rack.id = 1001
        mock_rack.location_id = 13  # 在準備區的位置
        mock_rack.room_id = None  # 沒有 room_id

        db_manager.rack_table = [mock_rack]

        handler = ReadyRackToBoxinHandler(mock_node)

        # 模擬有料架但沒有 room_id 的情況
        with patch.object(handler, 'system_ready_area_status', return_value=(6, [11, 12], 2, [13, 14])):
            result = handler.check_condition()
            self.assertFalse(result)
            self.assertFalse(handler.find_task)

        print("✅ rack 沒有 room_id 測試通過")

    @patch('wcs_base.database_manager.ConnectionPoolManager')
    def test_check_condition_boxin_occupied(self, mock_pool_manager):
        """測試檢查條件 - boxin 位置已被佔用"""
        from wcs_base.database_manager import DatabaseManager
        from kuka_wcs.task_handler.ready_rack_to_boxin import ReadyRackToBoxinHandler

        # 創建模擬的 node
        mock_node = Mock()
        mock_node.get_logger.return_value = self.mock_logger

        # 創建 DatabaseManager
        db_manager = DatabaseManager(self.mock_logger, self.mock_db_url)
        mock_node.db_manager = db_manager

        # 設置測試數據
        mock_rack = Mock()
        mock_rack.id = 1001
        mock_rack.location_id = 13
        mock_rack.room_id = 1

        db_manager.rack_table = [mock_rack]
        db_manager.rack_location_id_list = [10001]  # 目標入口位置已被佔用

        handler = ReadyRackToBoxinHandler(mock_node)

        # 模擬 boxin 位置被佔用的情況
        with patch.object(handler, 'system_ready_area_status', return_value=(6, [11, 12], 2, [13, 14])), \
             patch.object(handler, 'convert_to_node_id', return_value=10001), \
             patch.object(handler, 'check_kuka_task_doing', return_value=False):

            result = handler.check_condition()
            self.assertFalse(result)
            self.assertFalse(handler.find_task)

        print("✅ boxin 位置已被佔用測試通過")

    @patch('wcs_base.database_manager.ConnectionPoolManager')
    def test_insert_task(self, mock_pool_manager):
        """測試插入任務"""
        from wcs_base.database_manager import DatabaseManager
        from kuka_wcs.task_handler.ready_rack_to_boxin import ReadyRackToBoxinHandler
        
        # 創建模擬的 node
        mock_node = Mock()
        mock_node.get_logger.return_value = self.mock_logger
        
        # 創建 DatabaseManager
        db_manager = DatabaseManager(self.mock_logger, self.mock_db_url)
        mock_node.db_manager = db_manager
        
        # 設置測試數據
        mock_work = Mock()
        mock_work.id = 220001  # CONFIG.KUKA_RACK_MOVE
        db_manager.work_table = [mock_work]
        
        handler = ReadyRackToBoxinHandler(mock_node)
        handler.task_node_id = 10001
        handler.task_room_id = 1
        
        # 模擬資料庫會話
        mock_session = Mock()
        mock_session.__enter__ = Mock(return_value=mock_session)
        mock_session.__exit__ = Mock(return_value=None)
        
        with patch.object(db_manager, 'get_session', return_value=mock_session), \
             patch.object(handler, 'create_task', return_value=Mock(id=12345)), \
             patch('config.config.KUKA_RACK_MOVE', 220001), \
             patch('config.config.PRIORITY_FOR_KUKA_FROM_EMPTY_TO_BOXOUT', 900), \
             patch('config.config.WCS_STATUS', 1):
            
            result = handler.insert_task()
            self.assertTrue(result)
            self.assertTrue(handler.task_inserted)
            
        print("✅ 插入任務測試通過")

    @patch('wcs_base.database_manager.ConnectionPoolManager')
    def test_check_insert_done(self, mock_pool_manager):
        """測試檢查任務插入完成"""
        from wcs_base.database_manager import DatabaseManager
        from kuka_wcs.task_handler.ready_rack_to_boxin import ReadyRackToBoxinHandler
        
        # 創建模擬的 node
        mock_node = Mock()
        mock_node.get_logger.return_value = self.mock_logger
        
        # 創建 DatabaseManager
        db_manager = DatabaseManager(self.mock_logger, self.mock_db_url)
        mock_node.db_manager = db_manager
        
        handler = ReadyRackToBoxinHandler(mock_node)
        handler.task_node_id = 10001
        handler.find_task = True
        handler.task_inserted = True
        
        # 測試任務存在的情況
        with patch.object(handler, 'check_kuka_task_doing', return_value=True):
            result = handler.check_insert_done()
            self.assertTrue(result)
            self.assertFalse(handler.find_task)  # 狀態應該被重置
            self.assertFalse(handler.task_inserted)
            
        print("✅ 檢查任務插入完成測試通過")

    def test_inheritance_and_methods(self):
        """測試繼承和方法實作"""
        from kuka_wcs.task_handler.ready_rack_to_boxin import ReadyRackToBoxinHandler
        from wcs_base.base_task_handler import BaseTaskHandler
        
        # 檢查繼承關係
        self.assertTrue(issubclass(ReadyRackToBoxinHandler, BaseTaskHandler))
        
        # 檢查必要方法存在
        self.assertTrue(hasattr(ReadyRackToBoxinHandler, 'check_condition'))
        self.assertTrue(hasattr(ReadyRackToBoxinHandler, 'insert_task'))
        self.assertTrue(hasattr(ReadyRackToBoxinHandler, 'check_insert_done'))
        self.assertTrue(hasattr(ReadyRackToBoxinHandler, 'execute'))
        
        print("✅ 繼承和方法實作測試通過")

    def test_wcs_base_node_integration(self):
        """測試與 WCSBaseNode 的整合"""
        from wcs_base.wcs_base_node import WCSBaseNode
        
        # 檢查 ReadyRackToBoxinHandler 是否被正確導入
        import inspect
        source = inspect.getsource(WCSBaseNode.__init__)
        self.assertIn('ReadyRackToBoxinHandler', source)
        
        print("✅ WCSBaseNode 整合測試通過")

def run_ready_rack_to_boxin_tests():
    """運行 ReadyRackToBoxinHandler 測試"""
    print("🧪 開始測試 ReadyRackToBoxinHandler 任務處理器...")
    print("=" * 60)
    
    # 創建測試套件
    test_suite = unittest.TestLoader().loadTestsFromTestCase(TestReadyRackToBoxinHandler)
    
    # 運行測試
    runner = unittest.TextTestRunner(verbosity=1)
    result = runner.run(test_suite)
    
    print("=" * 60)
    
    # 輸出結果
    if result.wasSuccessful():
        print("✅ 所有 ReadyRackToBoxinHandler 測試通過！")
        print("\n📋 實作總結:")
        print("   ✅ ReadyRackToBoxinHandler 正確繼承 BaseTaskHandler")
        print("   ✅ 實作了所有必要的抽象方法")
        print("   ✅ 使用 self.db_manager 訪問資料表數據")
        print("   ✅ 正確實作判斷條件邏輯")
        print("   ✅ 任務插入和檢查功能正常")
        print("   ✅ 已整合到 WCSBaseNode 中")
        print("\n🎉 ReadyRackToBoxinHandler 任務處理器實作完成！")
        return True
    else:
        print("❌ ReadyRackToBoxinHandler 測試失敗！")
        for failure in result.failures:
            print(f"失敗: {failure[0]}")
            print(f"詳情: {failure[1]}")
        for error in result.errors:
            print(f"錯誤: {error[0]}")
            print(f"詳情: {error[1]}")
        return False

if __name__ == '__main__':
    success = run_ready_rack_to_boxin_tests()
    sys.exit(0 if success else 1)
