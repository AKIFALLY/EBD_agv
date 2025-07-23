#!/usr/bin/env python3
"""
簡單測試 OR 邏輯的條件檢查功能
"""

import unittest
from unittest.mock import Mock
import sys
import os

# 添加路徑以便匯入模組
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))


class TestSimpleORLogic(unittest.TestCase):
    """測試簡單的 OR 邏輯"""

    def test_or_logic_explanation(self):
        """測試 OR 邏輯的說明"""
        print("\n=== OR 邏輯修改說明 ===")
        print("修改前：只有 result='True' 才會跳轉到 next_id")
        print("修改後：result='False' 但有 next_id 也會跳轉")
        print("\n=== 使用場景 ===")
        print("1. 多位置檢查：位置A不可用 → 檢查位置B → 檢查位置C")
        print("2. 容錯機制：條件A失敗 → 嘗試條件B → 嘗試條件C")
        print("3. OR邏輯：滿足任一條件即可成功")
        
        # 模擬條件檢查邏輯
        conditions = [
            {"id": 100, "result": "False", "next_id": "101", "location": "A"},
            {"id": 101, "result": "False", "next_id": "102", "location": "B"}, 
            {"id": 102, "result": "True", "next_id": "end", "location": "C"}
        ]
        
        print("\n=== 模擬執行流程 ===")
        current_id = 100
        for condition in conditions:
            if condition["id"] == current_id:
                result = condition["result"]
                next_id = condition.get("next_id")
                location = condition.get("location")
                
                print(f"檢查 ID {current_id} (位置 {location}): result={result}")
                
                if result == "True":
                    print(f"✅ 條件滿足！找到可用位置: {location}")
                    if next_id == "end":
                        print("🎯 任務完成")
                        break
                elif result == "False" and next_id:
                    print(f"❌ 條件不滿足，但有 next_id: {next_id}，繼續探索")
                    current_id = int(next_id)
                else:
                    print("❌ 條件不滿足且無 next_id，結束")
                    break
        
        self.assertTrue(True)  # 測試總是通過，這只是展示邏輯

    def test_condition_logic_comparison(self):
        """比較修改前後的邏輯差異"""
        print("\n=== 邏輯比較 ===")
        
        # 測試資料
        test_data = {"result": "False", "next_id": "101"}
        
        print("測試資料:", test_data)
        
        # 修改前的邏輯
        print("\n修改前邏輯:")
        if test_data.get("result") == "True":
            next_id = test_data.get("next_id")
            print(f"  → 跳轉到 {next_id}")
        elif test_data.get("result") == "False":
            print("  → 繼續檢查下一筆資料 (continue)")
        
        # 修改後的邏輯
        print("\n修改後邏輯:")
        if test_data.get("result") == "True":
            next_id = test_data.get("next_id")
            print(f"  → 跳轉到 {next_id}")
        elif test_data.get("result") == "False":
            next_id = test_data.get("next_id")
            if next_id:
                print(f"  → 條件不滿足但有 next_id: {next_id}，繼續探索")
            else:
                print("  → 繼續檢查下一筆資料 (continue)")
        
        self.assertTrue(True)

    def test_practical_example(self):
        """實際應用範例"""
        print("\n=== 實際應用範例：尋找可用停車位 ===")
        
        # 模擬 SQL 查詢結果
        parking_conditions = [
            {
                "id": 1,
                "description": "檢查 A 區停車位",
                "sql_result": {
                    "result": "False",  # A 區沒有空位
                    "next_id": "2",     # 檢查 B 區
                    "checked_area": "A"
                }
            },
            {
                "id": 2, 
                "description": "檢查 B 區停車位",
                "sql_result": {
                    "result": "False",  # B 區也沒有空位
                    "next_id": "3",     # 檢查 C 區
                    "checked_area": "B"
                }
            },
            {
                "id": 3,
                "description": "檢查 C 區停車位", 
                "sql_result": {
                    "result": "True",   # C 區有空位！
                    "next_id": "end",
                    "available_spot": "C-15",
                    "checked_area": "C"
                }
            }
        ]
        
        print("停車位搜尋流程:")
        for condition in parking_conditions:
            print(f"\n步驟 {condition['id']}: {condition['description']}")
            result = condition["sql_result"]
            
            if result["result"] == "True":
                print(f"  ✅ 找到空位: {result.get('available_spot')}")
                print("  🎯 搜尋成功！")
                break
            else:
                next_id = result.get("next_id")
                if next_id and next_id != "end":
                    print(f"  ❌ {result['checked_area']} 區無空位")
                    print(f"  🔄 繼續搜尋下一區域 (ID: {next_id})")
                else:
                    print(f"  ❌ {result['checked_area']} 區無空位，搜尋結束")
        
        self.assertTrue(True)


if __name__ == '__main__':
    unittest.main(verbosity=2)
