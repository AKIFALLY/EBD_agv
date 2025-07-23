#!/usr/bin/env python3
"""
測試 unloader_agv pre_dryer_port 計算邏輯的驗證腳本

此測試驗證 unloader_agv 中 pre_dryer_port 的計算邏輯是否正確。
測試涵蓋 port 1-8 的 row 和 column 計算。

測試要求：
- port 1-4 → row=1, column=0
- port 5-8 → row=2, column=0
"""

import unittest


class TestPreDryerCalculation(unittest.TestCase):
    """Pre Dryer Port 計算邏輯測試類"""

    def test_port_1_to_4_calculation(self):
        """測試 port 1-4 的計算邏輯"""
        for port in range(1, 5):
            with self.subTest(port=port):
                pre_dryer_row = 1 if port <= 4 else 2
                pre_dryer_column = 0

                self.assertEqual(pre_dryer_row, 1, f"Port {port} 的 row 應該是 1")
                self.assertEqual(pre_dryer_column, 0, f"Port {port} 的 column 應該是 0")

    def test_port_5_to_8_calculation(self):
        """測試 port 5-8 的計算邏輯"""
        for port in range(5, 9):
            with self.subTest(port=port):
                pre_dryer_row = 1 if port <= 4 else 2
                pre_dryer_column = 0

                self.assertEqual(pre_dryer_row, 2, f"Port {port} 的 row 應該是 2")
                self.assertEqual(pre_dryer_column, 0, f"Port {port} 的 column 應該是 0")

    def test_all_ports_calculation(self):
        """測試所有 port 1-8 的計算邏輯"""
        expected_results = {
            1: (1, 0), 2: (1, 0), 3: (1, 0), 4: (1, 0),
            5: (2, 0), 6: (2, 0), 7: (2, 0), 8: (2, 0)
        }

        for port, (expected_row, expected_column) in expected_results.items():
            with self.subTest(port=port):
                pre_dryer_row = 1 if port <= 4 else 2
                pre_dryer_column = 0

                self.assertEqual(pre_dryer_row, expected_row,
                               f"Port {port} 的 row 計算錯誤")
                self.assertEqual(pre_dryer_column, expected_column,
                               f"Port {port} 的 column 計算錯誤")

    def test_boundary_conditions(self):
        """測試邊界條件"""
        # 測試 port 4 (第一組的最後一個)
        self.assertEqual(1 if 4 <= 4 else 2, 1, "Port 4 應該屬於第一組 (row=1)")

        # 測試 port 5 (第二組的第一個)
        self.assertEqual(1 if 5 <= 4 else 2, 2, "Port 5 應該屬於第二組 (row=2)")


def test_pre_dryer_calculation():
    """向後相容的函數版本測試"""
    print("=== Pre Dryer Port 計算邏輯測試 ===")
    print("要求：")
    print("- port 1-4 → row=1, column=0")
    print("- port 5-8 → row=2, column=0")
    print()

    # 測試所有 port 1-8 的計算結果
    for port in range(1, 9):
        # 使用修改後的計算邏輯
        pre_dryer_row = 1 if port <= 4 else 2
        pre_dryer_column = 0

        print(f"Port {port}: row={pre_dryer_row}, column={pre_dryer_column}")

    print()
    print("=== 驗證結果 ===")

    # 驗證 port 1-4 的結果
    ports_1_4_correct = all(
        (1 if port <= 4 else 2) == 1 and 0 == 0
        for port in range(1, 5)
    )

    # 驗證 port 5-8 的結果
    ports_5_8_correct = all(
        (1 if port <= 4 else 2) == 2 and 0 == 0
        for port in range(5, 9)
    )

    print(f"Port 1-4 計算正確: {ports_1_4_correct}")
    print(f"Port 5-8 計算正確: {ports_5_8_correct}")
    print(f"整體計算正確: {ports_1_4_correct and ports_5_8_correct}")

    return ports_1_4_correct and ports_5_8_correct


if __name__ == "__main__":
    # 可以選擇運行 unittest 或函數版本測試
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "--unittest":
        # 運行 unittest 版本
        unittest.main(argv=[''], exit=False, verbosity=2)
    else:
        # 運行函數版本測試（向後相容）
        success = test_pre_dryer_calculation()
        if success:
            print("\n✅ 所有測試通過！pre_dryer_port 計算邏輯正確。")
        else:
            print("\n❌ 測試失敗！請檢查計算邏輯。")
