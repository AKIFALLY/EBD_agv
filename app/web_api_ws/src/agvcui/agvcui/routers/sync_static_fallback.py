#!/usr/bin/env python3
"""
Sync Static Fallback - 自動更新靜態備援函數列表
用於定期從 flow_wcs API 或快取檔案同步函數到靜態備援
"""

import json
import yaml
import asyncio
import urllib.request
import urllib.error
from pathlib import Path
from datetime import datetime
from typing import Dict, Any, Optional

# 配置
CACHE_FILE = Path("/app/config/wcs/flow_functions_cache.yaml")
STATIC_FALLBACK_FILE = Path("/app/config/wcs/static_fallback_functions.json")
LINEAR_FLOW_DESIGNER_PY = Path("/app/web_api_ws/src/agvcui/agvcui/routers/linear_flow_designer.py")

def fetch_functions_from_api() -> Optional[Dict[str, Any]]:
    """從 flow_wcs API 獲取函數列表"""
    # 嘗試多個可能的 API 端點
    urls = [
        "http://localhost:8000/api/flow/functions",
        "http://agvc.webapi/api/flow/functions",
        "http://192.168.100.100:8000/api/flow/functions"
    ]
    
    for url in urls:
        try:
            print(f"嘗試從 {url} 獲取函數...")
            with urllib.request.urlopen(url, timeout=5) as response:
                if response.status == 200:
                    data = json.loads(response.read().decode('utf-8'))
                    if data.get("success") and data.get("functions"):
                        print(f"✅ 成功從 API 獲取函數")
                        return data["functions"]
        except (urllib.error.URLError, urllib.error.HTTPError) as e:
            print(f"❌ {url} 失敗: {e}")
            continue
        except Exception as e:
            print(f"❌ {url} 失敗: {e}")
            continue
    
    print(f"❌ 所有 API 端點都無法訪問")
    return None

def load_functions_from_cache() -> Optional[Dict[str, Any]]:
    """從快取檔案載入函數"""
    if CACHE_FILE.exists():
        try:
            with open(CACHE_FILE, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                print(f"✅ 從快取載入函數 (更新時間: {data.get('meta', {}).get('updated_at', 'unknown')})")
                return data.get("functions")
        except Exception as e:
            print(f"❌ 快取載入失敗: {e}")
    return None

def generate_python_code(functions: Dict[str, Any]) -> str:
    """生成 Python 程式碼格式的函數定義"""
    code_lines = []
    code_lines.append("    # Static fallback with defaults (Layer 3)")
    code_lines.append("    # Auto-generated at: " + datetime.now().isoformat())
    code_lines.append("    functions = {")
    
    for category, funcs in functions.items():
        code_lines.append(f'        "{category}": [')
        for func in funcs:
            code_lines.append('            {')
            code_lines.append(f'                "name": "{func.get("name", "")}",')
            code_lines.append(f'                "description": "{func.get("description", "")}",')
            
            # 處理 params
            params = func.get("params", [])
            params_str = json.dumps(params)
            code_lines.append(f'                "params": {params_str},')
            
            # 處理 returns
            returns = func.get("returns", "any")
            code_lines.append(f'                "returns": "{returns}"' + (',' if func.get("defaults") else ''))
            
            # 處理 defaults
            if func.get("defaults"):
                # 將 JSON boolean 轉換為 Python boolean 字串表示
                defaults_str = json.dumps(func.get("defaults"), ensure_ascii=False)
                # 替換 JSON 的 true/false/null 為 Python 的 True/False/None
                defaults_str = defaults_str.replace(': true', ': True').replace(': false', ': False').replace(': null', ': None')
                code_lines.append(f'                "defaults": {defaults_str}')
            
            code_lines.append('            },')
        code_lines.append('        ],')
    
    code_lines.append('    }')
    
    return '\n'.join(code_lines)

def update_static_fallback_in_file(functions: Dict[str, Any]) -> bool:
    """更新 linear_flow_designer.py 中的靜態備援"""
    try:
        if not LINEAR_FLOW_DESIGNER_PY.exists():
            print(f"❌ 找不到檔案: {LINEAR_FLOW_DESIGNER_PY}")
            return False
            
        with open(LINEAR_FLOW_DESIGNER_PY, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 找到靜態備援的開始和結束位置
        start_marker = "    # Static fallback with defaults (Layer 3)"
        end_marker = '    return {\n        "success": True,'
        
        start_idx = content.find(start_marker)
        end_idx = content.find(end_marker)
        
        if start_idx == -1 or end_idx == -1:
            print("❌ 無法找到靜態備援區段")
            return False
        
        # 生成新的程式碼
        new_code = generate_python_code(functions)
        
        # 替換內容
        new_content = content[:start_idx] + new_code + '\n\n    # Always return categorized format\n' + content[end_idx:]
        
        # 寫回檔案
        with open(LINEAR_FLOW_DESIGNER_PY, 'w', encoding='utf-8') as f:
            f.write(new_content)
        
        print(f"✅ 成功更新靜態備援 ({len(functions)} 個類別)")
        return True
        
    except Exception as e:
        print(f"❌ 更新檔案失敗: {e}")
        return False

def save_static_fallback_json(functions: Dict[str, Any]):
    """保存靜態備援為 JSON 格式（便於查看）"""
    try:
        STATIC_FALLBACK_FILE.parent.mkdir(parents=True, exist_ok=True)
        
        data = {
            "meta": {
                "updated_at": datetime.now().isoformat(),
                "version": "2.0.0",
                "source": "sync_script"
            },
            "functions": functions
        }
        
        with open(STATIC_FALLBACK_FILE, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        
        print(f"✅ 保存 JSON 備份至: {STATIC_FALLBACK_FILE}")
        
    except Exception as e:
        print(f"❌ 保存 JSON 失敗: {e}")

def sync_static_fallback():
    """主同步函數"""
    print("=" * 50)
    print("🔄 開始同步靜態備援函數")
    print("=" * 50)
    
    # 1. 嘗試從 API 獲取
    functions = fetch_functions_from_api()
    
    # 2. 如果 API 失敗，從快取載入
    if not functions:
        print("⚠️ API 無法存取，嘗試從快取載入...")
        functions = load_functions_from_cache()
    
    if not functions:
        print("❌ 無法獲取函數列表")
        return False
    
    # 3. 統計資訊
    total_functions = sum(len(funcs) for funcs in functions.values())
    print(f"📊 獲取到 {len(functions)} 個類別，共 {total_functions} 個函數")
    
    for category, funcs in functions.items():
        print(f"  - {category}: {len(funcs)} 個函數")
    
    # 4. 更新靜態備援
    print("\n📝 更新靜態備援...")
    success = update_static_fallback_in_file(functions)
    
    # 5. 保存 JSON 備份
    if success:
        save_static_fallback_json(functions)
    
    print("\n" + "=" * 50)
    if success:
        print("✅ 同步完成！靜態備援已更新")
    else:
        print("❌ 同步失敗")
    print("=" * 50)
    
    return success

def main():
    """主程式"""
    sync_static_fallback()

if __name__ == "__main__":
    main()