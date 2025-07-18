#!/usr/bin/env python3
"""
測試四個狀態檔案中的 Hokuyo write_busy 功能
"""

import unittest
from unittest.mock import Mock, MagicMock
import sys
import importlib.util


class TestHokuyoBusyStates(unittest.TestCase):
    def setUp(self):
        """設置測試環境"""
        # 模擬依賴
        sys.modules['cargo_mover_agv.robot_context'] = Mock()
        sys.modules['db_proxy.agvc_database_client'] = Mock()
        sys.modules['db_proxy_interfaces.msg'] = Mock()
        sys.modules['agv_base.hokuyo_dms_8bit'] = Mock()
        sys.modules['agv_base.robot'] = Mock()

        # 創建模擬節點
        self.mock_node = Mock()
        self.mock_node.get_logger.return_value = Mock()
        self.mock_node.room_id = 2

        # 創建兩個模擬 Hokuyo 物件
        self.mock_hokuyo_1 = Mock()
        self.mock_hokuyo_2 = Mock()
        self.mock_node.hokuyo_dms_8bit_1 = self.mock_hokuyo_1
        self.mock_node.hokuyo_dms_8bit_2 = self.mock_hokuyo_2

        # 創建模擬 context
        self.mock_context = Mock()
        self.mock_context.robot = Mock()
        self.mock_context.robot.read_pgno_response = Mock()
        self.mock_context.robot.read_pgno_response.value = 0  # IDLE

        # 載入狀態類別
        self.load_state_classes()

    def load_state_classes(self):
        """載入四個狀態類別"""
        # 載入 TakeRackPortState (entrance)
        spec = importlib.util.spec_from_file_location(
            'take_rack_port_state',
            'src/cargo_mover_agv/cargo_mover_agv/robot_states/entrance/take_rack_port_state.py'
        )
        take_rack_port_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(take_rack_port_module)
        self.TakeRackPortState = take_rack_port_module.TakeRackPortState

        # 載入 PutTransferState (entrance)
        spec = importlib.util.spec_from_file_location(
            'put_tranfer_state',
            'src/cargo_mover_agv/cargo_mover_agv/robot_states/entrance/put_tranfer_state.py'
        )
        put_transfer_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(put_transfer_module)
        self.PutTransferState = put_transfer_module.PutTransferState

        # 載入 TakeTransferState (exit)
        spec = importlib.util.spec_from_file_location(
            'take_transfer_state',
            'src/cargo_mover_agv/cargo_mover_agv/robot_states/exit/take_transfer_state.py'
        )
        take_transfer_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(take_transfer_module)
        self.TakeTransferState = take_transfer_module.TakeTransferState

        # 載入 PutRackPortState (exit)
        spec = importlib.util.spec_from_file_location(
            'put_rack_port_state',
            'src/cargo_mover_agv/cargo_mover_agv/robot_states/exit/put_rack_port_state.py'
        )
        put_rack_port_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(put_rack_port_module)
        self.PutRackPortState = put_rack_port_module.PutRackPortState

    def test_take_rack_port_state_hokuyo_busy(self):
        """測試 TakeRackPortState 的 Hokuyo write_busy 功能"""
        state = self.TakeRackPortState(self.mock_node)

        # 測試初始狀態
        self.assertFalse(state.hokuyo_busy_write_completed)

        # 測試 _set_hokuyo_busy_entrance 方法
        state._set_hokuyo_busy_entrance()

        # 驗證使用 hokuyo_dms_8bit_1 (entrance 流程)
        self.mock_hokuyo_1.write_busy.assert_called_once_with("1")
        self.assertTrue(state.hokuyo_busy_write_completed)

    def test_put_transfer_state_hokuyo_busy(self):
        """測試 PutTransferState 的 Hokuyo write_busy 功能"""
        state = self.PutTransferState(self.mock_node)

        # 測試初始狀態
        self.assertFalse(state.hokuyo_busy_write_completed)

        # 測試 _set_hokuyo_busy_entrance 方法
        state._set_hokuyo_busy_entrance()

        # 驗證使用 hokuyo_dms_8bit_1 (entrance 流程)
        self.mock_hokuyo_1.write_busy.assert_called_with("1")
        self.assertTrue(state.hokuyo_busy_write_completed)

    def test_take_transfer_state_hokuyo_busy(self):
        """測試 TakeTransferState 的 Hokuyo write_busy 功能"""
        state = self.TakeTransferState(self.mock_node)

        # 測試初始狀態
        self.assertFalse(state.hokuyo_busy_write_completed)

        # 測試 _set_hokuyo_busy_exit 方法
        state._set_hokuyo_busy_exit()

        # 驗證使用 hokuyo_dms_8bit_2 (exit 流程)
        self.mock_hokuyo_2.write_busy.assert_called_once_with("1")
        self.assertTrue(state.hokuyo_busy_write_completed)

    def test_put_rack_port_state_hokuyo_busy(self):
        """測試 PutRackPortState 的 Hokuyo write_busy 功能"""
        state = self.PutRackPortState(self.mock_node)

        # 測試初始狀態
        self.assertFalse(state.hokuyo_busy_write_completed)

        # 測試 _set_hokuyo_busy_exit 方法
        state._set_hokuyo_busy_exit()

        # 驗證使用 hokuyo_dms_8bit_2 (exit 流程)
        self.mock_hokuyo_2.write_busy.assert_called_with("1")
        self.assertTrue(state.hokuyo_busy_write_completed)

    def test_execution_order_all_states(self):
        """測試所有狀態的執行順序"""
        states = [
            (self.TakeRackPortState, "TakeRackPortState"),
            (self.PutTransferState, "PutTransferState"),
            (self.TakeTransferState, "TakeTransferState"),
            (self.PutRackPortState, "PutRackPortState")
        ]

        for StateClass, state_name in states:
            with self.subTest(state=state_name):
                # 重置 mock
                self.mock_hokuyo_1.reset_mock()
                self.mock_hokuyo_2.reset_mock()

                state = StateClass(self.mock_node)

                # 第一次調用 handle - 應該只執行 Hokuyo write_busy
                state.handle(self.mock_context)

                # 驗證 Hokuyo write_busy 被調用
                if 'entrance' in state_name.lower() or 'take_rack_port' in state_name.lower():
                    # entrance 流程使用 hokuyo_dms_8bit_1
                    self.assertTrue(self.mock_hokuyo_1.write_busy.called,
                                    f"{state_name} 應該調用 hokuyo_1.write_busy")
                else:
                    # exit 流程使用 hokuyo_dms_8bit_2
                    self.assertTrue(self.mock_hokuyo_2.write_busy.called,
                                    f"{state_name} 應該調用 hokuyo_2.write_busy")

                # 驗證完成標誌
                self.assertTrue(state.hokuyo_busy_write_completed,
                                f"{state_name} 的 hokuyo_busy_write_completed 應該為 True")

    def tearDown(self):
        """清理測試環境"""
        pass


if __name__ == '__main__':
    unittest.main()
