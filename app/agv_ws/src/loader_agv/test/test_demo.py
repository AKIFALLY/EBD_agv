#!/usr/bin/env python3
"""
Demo 測試文件 - 展示 loader_agv take_transfer 流程測試框架的功能
這個文件展示了如何測試 take_transfer 流程的核心邏輯
"""

import unittest
from unittest.mock import Mock, MagicMock


class MockAgvPortCheckEmptyState:
    """模擬 AgvPortCheckEmptyState 類別"""

    SELECT_PORT01, SELECT_PORT02, SELECT_PORT03, SELECT_PORT04, SELECT_NONE = 1, 2, 3, 4, 0

    def __init__(self, node):
        self.node = node
        self.port_address = node.room_id * 1000 + 100
        self.eqp_id = node.room_id * 100 + 10

        # Port 選擇表
        self.select_agv_port_table = {
            (0, 0, 0, 0): self.SELECT_PORT01,
            (0, 0, 0, 1): self.SELECT_PORT01,
            (0, 0, 1, 0): self.SELECT_PORT01,
            (0, 0, 1, 1): self.SELECT_PORT01,
            (0, 1, 0, 0): self.SELECT_PORT01,
            (0, 1, 0, 1): self.SELECT_PORT01,
            (0, 1, 1, 0): self.SELECT_PORT01,
            (0, 1, 1, 1): self.SELECT_PORT01,
            (1, 0, 0, 0): self.SELECT_PORT02,
            (1, 0, 0, 1): self.SELECT_PORT02,
            (1, 0, 1, 0): self.SELECT_PORT02,
            (1, 0, 1, 1): self.SELECT_PORT02,
            (1, 1, 0, 0): self.SELECT_PORT03,
            (1, 1, 0, 1): self.SELECT_PORT03,
            (1, 1, 1, 0): self.SELECT_PORT04,
        }

        self._reset_state()

    def _reset_state(self):
        self.check_ok = False
        self.sent = False
        self.search_eqp_signal_ok = False
        self.carrier_query_sended = False
        self.carrier_query_success = False
        self.port_carriers = [True] * 4
        self.select_agv_port = self.SELECT_NONE
        self.carrier_id = None

    def enter(self):
        self._reset_state()

    def leave(self):
        self._reset_state()

    def eqp_signal_query_callback(self, response):
        # 模擬 EQP 信號查詢回調
        for i in range(4):
            self.port_carriers[i] = response.port_states[i] if hasattr(
                response, 'port_states') else False

        self.search_eqp_signal_ok = True
        port_states = tuple(int(carrier) for carrier in self.port_carriers)
        self.select_agv_port = self.select_agv_port_table.get(port_states, self.SELECT_NONE)

    def carrier_callback(self, response):
        self.carrier_query_success = response.success
        self.carrier_id = response.carrier_id if hasattr(response, 'carrier_id') else None

    def _update_context_states(self, context):
        if not self.search_eqp_signal_ok:
            return
        context.agv_port1 = self.port_carriers[0]
        context.agv_port2 = self.port_carriers[1]
        context.agv_port3 = self.port_carriers[2]
        context.agv_port4 = self.port_carriers[3]

    def _handle_port_selection(self, context):
        if self.check_ok or not self.search_eqp_signal_ok:
            return

        port_messages = {
            self.SELECT_PORT01: ("第一格空的", "AGV_PORT1", 1),
            self.SELECT_PORT02: ("第二格空的", "AGV_PORT2", 2),
            self.SELECT_PORT03: ("第三格空的", "AGV_PORT3", 3),
            self.SELECT_PORT04: ("第四格空的", "AGV_PORT4", 4)
        }

        if self.select_agv_port in port_messages:
            desc, port, number = port_messages[self.select_agv_port]
            context.get_loader_agv_port_front = number
            self.check_ok = True
        else:
            context.get_loader_agv_port_front = None
            self._reset_state()


class MockTransferCheckHaveState:
    """模擬 TransferCheckHaveState 類別"""

    def __init__(self, node):
        self.node = node
        self.port_address = node.room_id * 1000 + 10
        self.eqp_id = node.room_id * 100 + 1
        self.select_boxin_port = 0
        self.search_eqp_signal_ok = False

    def _check_take_transfer_continue(self, context):
        """檢查 take transfer 是否可以繼續的條件"""
        if not self.search_eqp_signal_ok or self.select_boxin_port == 0:
            context.take_transfer_continue = False
            return

        # Transfer continuation logic
        if self.select_boxin_port == 1 and context.boxin_port2:
            context.take_transfer_continue = True
        elif self.select_boxin_port == 3 and context.boxin_port4:
            context.take_transfer_continue = True
        else:
            context.take_transfer_continue = False


class MockRobotContext:
    """模擬 RobotContext 類別"""

    def __init__(self):
        # AGV 狀態
        self.agv_port1 = False
        self.agv_port2 = False
        self.agv_port3 = False
        self.agv_port4 = False
        self.get_loader_agv_port_front = 1

        # BOXIN 狀態
        self.boxin_port1 = False
        self.boxin_port2 = False
        self.boxin_port3 = False
        self.boxin_port4 = False

        # Transfer 狀態
        self.take_transfer_continue = False
        self.carrier_id = 0


class TestTakeTransferDemo(unittest.TestCase):
    """Take Transfer 流程 Demo 測試"""

    def setUp(self):
        """測試前設置"""
        self.mock_node = Mock()
        self.mock_node.get_logger.return_value = Mock()
        self.mock_node.room_id = 1
        self.context = MockRobotContext()

    def test_agv_port_dynamic_calculations(self):
        """測試 AGV port 動態計算"""
        # 測試不同 room_id 的計算
        test_cases = [
            (1, 1100, 110),
            (2, 2100, 210),
            (5, 5100, 510),
        ]

        for room_id, expected_port_addr, expected_eqp_id in test_cases:
            with self.subTest(room_id=room_id):
                self.mock_node.room_id = room_id
                state = MockAgvPortCheckEmptyState(self.mock_node)
                self.assertEqual(state.port_address, expected_port_addr)
                self.assertEqual(state.eqp_id, expected_eqp_id)

    def test_agv_port_selection_logic(self):
        """測試 AGV port 選擇邏輯"""
        state = MockAgvPortCheckEmptyState(self.mock_node)

        # 測試所有 port 都空的情況 - 選擇 PORT01
        port_states = (0, 0, 0, 0)
        expected = state.SELECT_PORT01
        self.assertEqual(state.select_agv_port_table[port_states], expected)

        # 測試 PORT01 有貨，選擇 PORT02
        port_states = (1, 0, 0, 0)
        expected = state.SELECT_PORT02
        self.assertEqual(state.select_agv_port_table[port_states], expected)

        # 測試 PORT01, PORT02 有貨，選擇 PORT03
        port_states = (1, 1, 0, 0)
        expected = state.SELECT_PORT03
        self.assertEqual(state.select_agv_port_table[port_states], expected)

        # 測試 PORT01, PORT02, PORT03 有貨，選擇 PORT04
        port_states = (1, 1, 1, 0)
        expected = state.SELECT_PORT04
        self.assertEqual(state.select_agv_port_table[port_states], expected)

    def test_transfer_continuation_logic(self):
        """測試 transfer continuation 邏輯"""
        state = MockTransferCheckHaveState(self.mock_node)
        state.search_eqp_signal_ok = True

        # 測試 port 1 選擇且 port 2 有貨 - 應該繼續
        state.select_boxin_port = 1
        self.context.boxin_port2 = True
        self.context.boxin_port4 = False

        state._check_take_transfer_continue(self.context)
        self.assertTrue(self.context.take_transfer_continue)

        # 測試 port 3 選擇且 port 4 有貨 - 應該繼續
        state.select_boxin_port = 3
        self.context.boxin_port2 = False
        self.context.boxin_port4 = True

        state._check_take_transfer_continue(self.context)
        self.assertTrue(self.context.take_transfer_continue)

        # 測試 port 1 選擇但 port 2 沒貨 - 不應該繼續
        state.select_boxin_port = 1
        self.context.boxin_port2 = False
        self.context.boxin_port4 = False

        state._check_take_transfer_continue(self.context)
        self.assertFalse(self.context.take_transfer_continue)

    def test_complete_flow_simulation(self):
        """測試完整流程模擬"""
        # 1. AGV Port 檢查
        agv_state = MockAgvPortCheckEmptyState(self.mock_node)

        # 模擬 EQP 查詢 - AGV port 1 有貨，選擇 port 2
        mock_response = Mock()
        mock_response.port_states = [True, False, False, False]
        agv_state.eqp_signal_query_callback(mock_response)

        # 驗證選擇結果
        self.assertEqual(agv_state.select_agv_port, agv_state.SELECT_PORT02)

        # 模擬 port 選擇處理
        agv_state._handle_port_selection(self.context)
        self.assertTrue(agv_state.check_ok)
        self.assertEqual(self.context.get_loader_agv_port_front, 2)

        # 2. Transfer 檢查
        transfer_state = MockTransferCheckHaveState(self.mock_node)
        transfer_state.select_boxin_port = 1
        transfer_state.search_eqp_signal_ok = True

        # 設置 BOXIN port 狀態
        self.context.boxin_port1 = True
        self.context.boxin_port2 = True  # port 2 有貨，應該繼續

        # 檢查 continuation 邏輯
        transfer_state._check_take_transfer_continue(self.context)
        self.assertTrue(self.context.take_transfer_continue)

    def test_error_handling(self):
        """測試錯誤處理"""
        agv_state = MockAgvPortCheckEmptyState(self.mock_node)

        # 模擬所有 port 都滿的情況
        agv_state.search_eqp_signal_ok = True
        agv_state.select_agv_port = agv_state.SELECT_NONE

        agv_state._handle_port_selection(self.context)

        # 驗證錯誤處理
        self.assertFalse(agv_state.check_ok)
        self.assertIsNone(self.context.get_loader_agv_port_front)


if __name__ == '__main__':
    print("🚀 運行 loader_agv take_transfer 流程 Demo 測試...")
    unittest.main(verbosity=2)
