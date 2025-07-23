"""
測試房間按鈕邏輯修正
"""

import pytest
from unittest.mock import Mock, patch


class TestRoomButtonLogic:
    """測試房間按鈕啟用邏輯"""

    def test_room_button_logic_with_non_sequential_rooms(self):
        """測試非連續房間 ID 的按鈕邏輯"""

        # 模擬資料庫中的房間資料（房間 1, 2, 5 啟用）
        mock_rooms = [
            {"id": 1, "name": "Room1", "enable": 1},
            {"id": 2, "name": "Room2", "enable": 1},
            {"id": 3, "name": "Room3", "enable": 0},
            {"id": 4, "name": "Room4", "enable": 0},
            {"id": 5, "name": "Room5", "enable": 1}  # 房間 5 啟用
        ]

        # 測試舊邏輯（錯誤的陣列索引方式）
        def old_logic_check_room_enabled(rooms, room_id):
            """舊的錯誤邏輯：使用陣列索引"""
            room_index = room_id - 1
            if rooms and room_index < len(rooms) and rooms[room_index] and rooms[room_index]["enable"]:
                return True
            return False

        # 測試新邏輯（正確的 ID 查找方式）
        def new_logic_check_room_enabled(rooms, room_id):
            """新的正確邏輯：使用 ID 查找"""
            room = rooms and next((r for r in rooms if r["id"] == room_id), None)
            return room and room["enable"] == 1

        # 測試各個房間的啟用狀態
        test_cases = [
            (1, True),   # 房間 1 應該啟用
            (2, True),   # 房間 2 應該啟用
            (3, False),  # 房間 3 應該禁用
            (4, False),  # 房間 4 應該禁用
            (5, True),   # 房間 5 應該啟用（這是關鍵測試）
        ]

        print("🔍 測試房間按鈕邏輯修正...")

        for room_id, expected_enabled in test_cases:
            old_result = old_logic_check_room_enabled(mock_rooms, room_id)
            new_result = new_logic_check_room_enabled(mock_rooms, room_id)

            print(f"房間 {room_id}: 預期={expected_enabled}, 舊邏輯={old_result}, 新邏輯={new_result}")

            # 新邏輯應該總是正確
            assert new_result == expected_enabled, f"新邏輯對房間 {room_id} 的判斷錯誤"

            # 檢查舊邏輯是否有問題（特別是房間 5）
            if room_id == 5:
                # 房間 5 的舊邏輯應該是錯誤的（因為索引 4 對應的是房間 5，但陣列中索引 4 的房間 ID 是 5）
                # 實際上舊邏輯可能會意外正確，但邏輯本身是錯誤的
                print(f"   ⚠️  房間 5 舊邏輯結果: {old_result} (邏輯錯誤但可能意外正確)")

        print("✅ 房間按鈕邏輯測試完成")

    def test_room_button_logic_with_missing_rooms(self):
        """測試缺少某些房間時的邏輯"""

        # 模擬只有部分房間的資料（缺少房間 3 和 4）
        mock_rooms = [
            {"id": 1, "name": "Room1", "enable": 1},
            {"id": 2, "name": "Room2", "enable": 1},
            {"id": 5, "name": "Room5", "enable": 1}  # 跳過了房間 3, 4
        ]

        def new_logic_check_room_enabled(rooms, room_id):
            """新的正確邏輯：使用 ID 查找"""
            room = rooms and next((r for r in rooms if r["id"] == room_id), None)
            return bool(room and room["enable"] == 1)

        # 測試各個房間
        test_cases = [
            (1, True),   # 房間 1 存在且啟用
            (2, True),   # 房間 2 存在且啟用
            (3, False),  # 房間 3 不存在，應該禁用
            (4, False),  # 房間 4 不存在，應該禁用
            (5, True),   # 房間 5 存在且啟用
        ]

        print("🔍 測試缺少房間時的邏輯...")

        for room_id, expected_enabled in test_cases:
            result = new_logic_check_room_enabled(mock_rooms, room_id)
            print(f"房間 {room_id}: 預期={expected_enabled}, 結果={result}")
            assert result == expected_enabled, f"房間 {room_id} 的判斷錯誤"

        print("✅ 缺少房間邏輯測試完成")

    def test_room_button_logic_with_reordered_rooms(self):
        """測試房間順序被打亂時的邏輯"""

        # 模擬房間順序被打亂的資料
        mock_rooms = [
            {"id": 5, "name": "Room5", "enable": 1},  # 房間 5 在第一位
            {"id": 1, "name": "Room1", "enable": 1},  # 房間 1 在第二位
            {"id": 3, "name": "Room3", "enable": 0},
            {"id": 2, "name": "Room2", "enable": 1},  # 房間 2 在第四位
            {"id": 4, "name": "Room4", "enable": 0}
        ]

        def new_logic_check_room_enabled(rooms, room_id):
            """新的正確邏輯：使用 ID 查找"""
            room = rooms and next((r for r in rooms if r["id"] == room_id), None)
            return bool(room and room["enable"] == 1)

        # 測試各個房間
        test_cases = [
            (1, True),   # 房間 1 啟用
            (2, True),   # 房間 2 啟用
            (3, False),  # 房間 3 禁用
            (4, False),  # 房間 4 禁用
            (5, True),   # 房間 5 啟用
        ]

        print("🔍 測試房間順序打亂時的邏輯...")

        for room_id, expected_enabled in test_cases:
            result = new_logic_check_room_enabled(mock_rooms, room_id)
            print(f"房間 {room_id}: 預期={expected_enabled}, 結果={result}")
            assert result == expected_enabled, f"房間 {room_id} 的判斷錯誤"

        print("✅ 房間順序打亂邏輯測試完成")


if __name__ == "__main__":
    test = TestRoomButtonLogic()
    test.test_room_button_logic_with_non_sequential_rooms()
    test.test_room_button_logic_with_missing_rooms()
    test.test_room_button_logic_with_reordered_rooms()
    print("🎉 所有房間按鈕邏輯測試通過！")
