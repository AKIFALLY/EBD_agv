#!/usr/bin/env python3
"""
測試 empty_rack_to_boxout.py 中新的條件檢查邏輯
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from db_proxy.crud.task_condition_crud import task_condition_crud
from db_proxy.connection import connection_pool
import json

def test_task_condition_data():
    """
    測試 task_condition 表格中的資料結構
    """
    print("🔍 測試 task_condition 表格資料...")
    
    try:
        with connection_pool.get_session() as session:
            # 取得所有條件記錄
            all_conditions = task_condition_crud.get_all(session)
            
            print(f"📊 找到 {len(all_conditions)} 筆條件記錄")
            
            for condition in all_conditions:
                print(f"\n📋 ID: {condition.id}")
                print(f"   描述: {condition.description}")
                print(f"   條件: {condition.conditions}")
                
                if condition.results:
                    print(f"   結果結構:")
                    print(f"     - success: {condition.results.get('success')}")
                    print(f"     - row_count: {condition.results.get('row_count')}")
                    print(f"     - columns: {condition.results.get('columns')}")
                    
                    data = condition.results.get('data', [])
                    if data:
                        print(f"     - data ({len(data)} 筆):")
                        for i, item in enumerate(data):
                            print(f"       [{i}] {item}")
                    else:
                        print(f"     - data: 無資料")
                else:
                    print(f"   結果: 無")
                    
    except Exception as e:
        print(f"❌ 測試失敗: {e}")

def simulate_condition_check():
    """
    模擬條件檢查流程
    """
    print("\n🔄 模擬條件檢查流程...")
    
    try:
        with connection_pool.get_session() as session:
            # 從 ID 1 開始
            current_id = 1
            max_iterations = 10
            iteration = 0
            collected_data = {}
            
            while iteration < max_iterations:
                iteration += 1
                print(f"\n🔍 第 {iteration} 次迭代，檢查 ID: {current_id}")
                
                # 取得條件記錄
                condition = task_condition_crud.get_by_id(session, current_id)
                if not condition or not condition.results:
                    print(f"⚠️ ID {current_id} 無條件記錄或結果，回到起始點")
                    current_id = 1
                    continue
                
                results = condition.results
                success = results.get("success", False)
                data_list = results.get("data", [])
                
                print(f"   Success: {success}")
                print(f"   Data 筆數: {len(data_list)}")
                
                if not success:
                    print(f"   查詢未成功，回到起始點")
                    current_id = 1
                    continue
                
                # 處理每筆資料
                next_id = None
                for i, data_item in enumerate(data_list):
                    print(f"   處理資料 [{i}]: {data_item}")
                    
                    # 檢查結束條件
                    if data_item.get("end") is True:
                        print(f"   ✅ 遇到結束條件！")
                        print(f"   📊 收集到的資料: {collected_data}")
                        return True
                    
                    # 檢查 result 欄位
                    result_value = data_item.get("result")
                    if result_value == "True":
                        # 收集資料
                        for key, value in data_item.items():
                            if key not in ["result", "next_id", "end"]:
                                collected_data[key] = value
                        
                        # 取得 next_id
                        next_id = data_item.get("next_id")
                        print(f"   ✅ 條件滿足，next_id: {next_id}")
                        break
                    elif result_value == "False":
                        print(f"   ❌ 條件不滿足")
                        continue
                
                # 處理 next_id
                if next_id:
                    # 簡化處理：假設是單一數字
                    try:
                        if next_id.isdigit():
                            current_id = int(next_id)
                        else:
                            print(f"   ⚠️ next_id 格式複雜: {next_id}，回到起始點")
                            current_id = 1
                    except:
                        print(f"   ⚠️ next_id 處理失敗，回到起始點")
                        current_id = 1
                else:
                    print(f"   📋 無 next_id，回到起始點")
                    current_id = 1
            
            print(f"⚠️ 達到最大迭代次數 {max_iterations}")
            return False
            
    except Exception as e:
        print(f"❌ 模擬失敗: {e}")
        return False

if __name__ == "__main__":
    print("🚀 開始測試 task_condition 條件檢查邏輯")
    
    # 測試資料結構
    test_task_condition_data()
    
    # 模擬條件檢查
    simulate_condition_check()
    
    print("\n🎉 測試完成")
