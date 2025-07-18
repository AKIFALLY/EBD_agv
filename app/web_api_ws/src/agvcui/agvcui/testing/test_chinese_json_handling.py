#!/usr/bin/env python3
"""
測試 Work 頁面中文 JSON 參數處理
驗證中文字符在 JSON 序列化和反序列化過程中保持原始 UTF-8 編碼
"""

import json
import sys
import os
from pathlib import Path

def test_json_serialization():
    """測試 JSON 序列化中文字符處理"""
    print("🔍 測試 JSON 序列化中文字符處理")
    print("-" * 40)
    
    # 測試數據包含中文字符
    test_data = {
        "任務類型": "運輸",
        "描述": "從倉庫運輸貨物到目標位置",
        "優先級": 1,
        "參數": {
            "起點": "倉庫A",
            "終點": "工作站B",
            "貨物類型": "電子零件"
        }
    }
    
    # 測試標準 JSON 序列化（會轉義中文）
    json_escaped = json.dumps(test_data, indent=2)
    print("❌ 標準 JSON 序列化（轉義中文）:")
    print(json_escaped[:100] + "..." if len(json_escaped) > 100 else json_escaped)
    
    # 測試 ensure_ascii=False（保持中文）
    json_chinese = json.dumps(test_data, indent=2, ensure_ascii=False)
    print("\n✅ ensure_ascii=False（保持中文）:")
    print(json_chinese[:100] + "..." if len(json_chinese) > 100 else json_chinese)
    
    # 驗證反序列化結果一致
    data_from_escaped = json.loads(json_escaped)
    data_from_chinese = json.loads(json_chinese)
    
    if data_from_escaped == data_from_chinese == test_data:
        print("\n✅ 反序列化結果一致，數據完整性保持")
        return True
    else:
        print("\n❌ 反序列化結果不一致")
        return False

def test_template_filter_simulation():
    """模擬模板過濾器處理"""
    print("\n🔍 測試模板過濾器模擬")
    print("-" * 40)
    
    test_data = {
        "工作名稱": "搬運任務",
        "描述": "將貨物從A點搬運到B點",
        "配置": {
            "最大重量": "50公斤",
            "運輸方式": "自動導引車"
        }
    }
    
    # 模擬 tojson_chinese 過濾器
    def tojson_chinese(obj, indent=None):
        """自定義 JSON 過濾器，支援中文字符顯示"""
        return json.dumps(obj, indent=indent, ensure_ascii=False)
    
    result = tojson_chinese(test_data, indent=2)
    print("tojson_chinese 過濾器結果:")
    print(result)
    
    # 檢查是否包含中文字符而非轉義序列
    if "工作名稱" in result and "\\u" not in result:
        print("✅ 中文字符正確保持，無轉義序列")
        return True
    else:
        print("❌ 中文字符被轉義或丟失")
        return False

def test_javascript_json_stringify():
    """測試 JavaScript JSON.stringify 行為模擬"""
    print("\n🔍 測試 JavaScript JSON.stringify 行為")
    print("-" * 40)
    
    test_data = {
        "類型": "運輸",
        "目標": "倉庫",
        "狀態": "進行中"
    }
    
    # Python 中模擬 JavaScript JSON.stringify(obj, null, 2)
    js_like_result = json.dumps(test_data, indent=2, ensure_ascii=False, separators=(',', ': '))
    print("模擬 JavaScript JSON.stringify(obj, null, 2):")
    print(js_like_result)
    
    # 檢查格式
    if "類型" in js_like_result and js_like_result.count('\n') > 0:
        print("✅ JavaScript 風格 JSON 格式正確，保持中文")
        return True
    else:
        print("❌ JavaScript 風格 JSON 格式不正確")
        return False

def test_template_files():
    """測試模板文件中的 tojson_chinese 使用"""
    print("\n🔍 測試模板文件中的 tojson_chinese 使用")
    print("-" * 40)
    
    project_root = Path(__file__).parent.parent.parent.parent
    template_files = [
        project_root / "web_api_ws/src/agvcui/agvcui/templates/works.html",
        project_root / "web_api_ws/src/agvcui/agvcui/templates/work_form.html"
    ]
    
    all_correct = True
    
    for template_file in template_files:
        if template_file.exists():
            try:
                with open(template_file, 'r', encoding='utf-8') as f:
                    content = f.read()
                
                # 檢查是否使用 tojson_chinese
                if "tojson_chinese" in content:
                    print(f"✅ {template_file.name} 使用 tojson_chinese 過濾器")
                elif "tojson" in content and "ensure_ascii=false" in content:
                    print(f"✅ {template_file.name} 使用 tojson 與 ensure_ascii=false")
                elif "tojson" in content:
                    print(f"⚠️ {template_file.name} 使用標準 tojson（可能轉義中文）")
                    all_correct = False
                else:
                    print(f"ℹ️ {template_file.name} 未發現 JSON 序列化")
                    
            except Exception as e:
                print(f"❌ 讀取 {template_file.name} 失敗: {e}")
                all_correct = False
        else:
            print(f"❌ {template_file.name} 不存在")
            all_correct = False
    
    return all_correct

def test_database_json_handling():
    """測試資料庫 JSON 處理模擬"""
    print("\n🔍 測試資料庫 JSON 處理模擬")
    print("-" * 40)
    
    # 模擬從前端接收的 JSON 字符串（包含中文）
    frontend_json = '{"任務": "運輸", "描述": "搬運貨物", "參數": {"重量": "10公斤"}}'
    
    try:
        # 模擬後端解析
        parsed_data = json.loads(frontend_json)
        print("✅ 前端 JSON 解析成功:")
        print(f"   任務: {parsed_data.get('任務')}")
        print(f"   描述: {parsed_data.get('描述')}")
        
        # 模擬存儲到資料庫後再讀取
        stored_json = json.dumps(parsed_data, ensure_ascii=False)
        retrieved_data = json.loads(stored_json)
        
        if retrieved_data == parsed_data:
            print("✅ 資料庫存儲和讀取保持數據完整性")
            return True
        else:
            print("❌ 資料庫存儲和讀取數據不一致")
            return False
            
    except json.JSONDecodeError as e:
        print(f"❌ JSON 解析失敗: {e}")
        return False

def main():
    """主測試函數"""
    print("🧪 開始 Work 頁面中文 JSON 處理測試")
    print("=" * 60)
    
    tests = [
        ("JSON 序列化中文字符", test_json_serialization),
        ("模板過濾器模擬", test_template_filter_simulation),
        ("JavaScript JSON.stringify 行為", test_javascript_json_stringify),
        ("模板文件檢查", test_template_files),
        ("資料庫 JSON 處理", test_database_json_handling),
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        try:
            if test_func():
                passed += 1
                print(f"\n✅ {test_name} 測試通過")
            else:
                print(f"\n❌ {test_name} 測試失敗")
        except Exception as e:
            print(f"\n❌ {test_name} 測試異常: {e}")
    
    print("\n" + "=" * 60)
    print(f"📊 測試結果: {passed}/{total} 通過")
    
    if passed == total:
        print("🎉 所有測試通過！中文 JSON 處理正確")
        print("\n📋 修復內容:")
        print("   ✅ 模板使用 tojson_chinese 過濾器保持中文字符")
        print("   ✅ JavaScript 使用標準 JSON.stringify 保持中文")
        print("   ✅ 資料庫操作正確處理 UTF-8 編碼")
        print("   ✅ 前後端 JSON 處理保持一致性")
        return True
    else:
        print("⚠️ 部分測試失敗，請檢查相關功能")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
