#!/usr/bin/env python3
"""
測試裝飾器系統和函數自動註冊
"""

import sys
from pathlib import Path

# Add flow_wcs to path
flow_wcs_path = Path(__file__).parent
if str(flow_wcs_path) not in sys.path:
    sys.path.insert(0, str(flow_wcs_path))

def test_decorator_registration():
    """測試裝飾器註冊系統"""
    from flow_wcs.decorators import get_function_library, list_registered_functions
    from flow_wcs.flow_executor import FlowExecutor
    
    print("=== 測試裝飾器註冊系統 ===\n")
    
    # 1. 測試註冊表
    print("1. 檢查註冊的函數列表:")
    functions = list_registered_functions()
    print(f"   已註冊函數數量: {len(functions)}")
    for func in functions[:5]:  # 顯示前5個
        print(f"   - {func}")
    print("   ...\n")
    
    # 2. 測試函數庫
    print("2. 檢查函數庫結構:")
    library = get_function_library()
    for category, funcs in library.items():
        print(f"   {category}: {len(funcs)} 個函數")
        if funcs:
            print(f"      範例: {funcs[0]['name']} - {funcs[0]['description']}")
    print()
    
    # 3. 測試 FlowExecutor.get_function_library()
    print("3. 測試 FlowExecutor.get_function_library():")
    executor_library = FlowExecutor.get_function_library()
    print(f"   返回類別數: {len(executor_library)}")
    for category in executor_library:
        print(f"   - {category}: {len(executor_library[category])} 個函數")
    print()
    
    # 4. 測試函數執行
    print("4. 測試函數執行:")
    test_cases = [
        ("query.locations", {"type": "room_inlet", "has_rack": True}),
        ("check.empty", {"data": []}),
        ("check.empty", {"data": [1, 2, 3]}),
        ("control.count", {"variable": [1, 2, 3, 4, 5]})
    ]
    
    for func_name, params in test_cases:
        try:
            result = FlowExecutor.execute_function_test(func_name, params)
            print(f"   ✅ {func_name}({params}) = {result}")
        except Exception as e:
            print(f"   ❌ {func_name}({params}) 錯誤: {e}")
    
    print("\n=== 測試完成 ===")
    return True

def test_api_endpoint():
    """測試 API 端點"""
    import httpx
    import asyncio
    
    async def test():
        print("\n=== 測試 API 端點 ===\n")
        
        async with httpx.AsyncClient() as client:
            # 1. 測試獲取函數庫
            print("1. 測試 GET /api/flow/functions:")
            try:
                response = await client.get("http://localhost:8000/api/flow/functions")
                if response.status_code == 200:
                    data = response.json()
                    print(f"   ✅ 成功獲取函數庫")
                    print(f"   - 來源: {data.get('source')}")
                    print(f"   - 版本: {data.get('version')}")
                    print(f"   - 類別數: {len(data.get('functions', {}))}")
                else:
                    print(f"   ❌ 狀態碼: {response.status_code}")
            except Exception as e:
                print(f"   ❌ 錯誤: {e}")
            
            # 2. 測試執行函數
            print("\n2. 測試 POST /api/flow/execute:")
            test_requests = [
                {
                    "function_name": "check.empty",
                    "params": {"data": []},
                    "variables": {}
                },
                {
                    "function_name": "control.count",
                    "params": {"variable": [1, 2, 3, 4, 5]},
                    "variables": {}
                }
            ]
            
            for req in test_requests:
                try:
                    response = await client.post(
                        "http://localhost:8000/api/flow/execute",
                        json=req
                    )
                    if response.status_code == 200:
                        data = response.json()
                        print(f"   ✅ {req['function_name']} = {data.get('result')}")
                    else:
                        print(f"   ❌ {req['function_name']}: 狀態碼 {response.status_code}")
                except Exception as e:
                    print(f"   ❌ {req['function_name']}: {e}")
        
        print("\n=== API 測試完成 ===")
    
    # 執行異步測試
    try:
        asyncio.run(test())
    except Exception as e:
        print(f"API 測試失敗: {e}")
        print("請確認 web_api 服務正在運行 (port 8000)")

if __name__ == "__main__":
    print("開始測試裝飾器系統和函數自動註冊...\n")
    
    # 測試裝飾器註冊
    test_decorator_registration()
    
    # 詢問是否測試 API
    print("\n是否要測試 API 端點? (需要 web_api 服務運行在 port 8000)")
    response = input("輸入 y 測試, 其他跳過: ")
    if response.lower() == 'y':
        test_api_endpoint()
    
    print("\n所有測試完成!")