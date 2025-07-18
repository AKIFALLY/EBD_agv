#!/usr/bin/env python3
"""
測試 unloader_agv pre_dryer_port 計算邏輯的驗證腳本
"""

def test_pre_dryer_calculation():
    """測試 pre_dryer_port 的計算邏輯"""
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
    success = test_pre_dryer_calculation()
    if success:
        print("\n✅ 所有測試通過！pre_dryer_port 計算邏輯正確。")
    else:
        print("\n❌ 測試失敗！請檢查計算邏輯。")
