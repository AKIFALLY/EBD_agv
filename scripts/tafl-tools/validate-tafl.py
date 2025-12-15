#!/usr/bin/env python3
"""
TAFL 檔案驗證工具
用法: python3 validate_tafl.py <tafl_file.yaml>
"""

import sys
import os

# 添加 tafl 模組路徑
sys.path.insert(0, '/home/ct/EBD_agv/app/tafl_ws/src/tafl')

from tafl.parser import TAFLParser
from tafl.validator import TAFLValidator

def find_tafl_file(filepath):
    """尋找 TAFL 檔案，從 config/tafl 尋找（包含子目錄）"""
    # 如果是絕對路徑或相對路徑，直接返回
    if filepath.startswith('/') or filepath.startswith('./'):
        return filepath if os.path.exists(filepath) else None

    # 在 config/tafl 目錄（包含子目錄）尋找
    config_dir = "/home/ct/EBD_agv/app/config/tafl"
    if os.path.exists(config_dir):
        for root, dirs, files in os.walk(config_dir):
            if filepath in files:
                found_path = os.path.join(root, filepath)
                print("✅ 在配置目錄找到檔案")
                return found_path

    # 當前目錄
    if os.path.exists(f"./{filepath}"):
        return f"./{filepath}"

    return None

def validate_tafl_file(filepath):
    """驗證單個 TAFL 檔案"""
    
    # 尋找檔案
    actual_path = find_tafl_file(filepath)
    if not actual_path:
        print(f"❌ 檔案不存在: {filepath}")
        print("   嘗試的位置:")
        print("   - /home/ct/EBD_agv/app/config/tafl/")
        print("   - ./當前目錄")
        return False
    
    filepath = actual_path
    
    print(f"驗證 TAFL 檔案: {filepath}")
    print("-" * 50)
    
    # 初始化解析器和驗證器
    parser = TAFLParser()
    validator = TAFLValidator()
    
    try:
        # 讀取檔案
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 解析 TAFL
        ast = parser.parse_string(content)
        print("✅ 語法解析成功")
        print(f"   - 流程步驟數: {len(ast.flow)}")
        
        # 統計動詞使用
        verbs = {}
        for node in ast.flow:
            if hasattr(node, '__class__'):
                verb = node.__class__.__name__.replace('Node', '').lower()
                verbs[verb] = verbs.get(verb, 0) + 1
        
        if verbs:
            verb_list = sorted(verbs.items(), key=lambda x: x[1], reverse=True)
            print(f"   - 使用的動詞: {', '.join([f'{v}({c})' for v, c in verb_list[:5]])}")
        
        # 驗證 TAFL
        validator.errors = []  # 重置錯誤列表
        if validator.validate(ast):
            print("✅ 格式驗證通過")
            print("\n✨ TAFL 檔案格式正確，可以使用！")
            return True
        else:
            errors = validator.get_errors()
            print("❌ 驗證失敗")
            print(f"   錯誤: {errors}")
            return False
            
    except Exception as e:
        print(f"❌ 解析錯誤: {e}")
        return False

def main():
    if len(sys.argv) != 2:
        print("用法: python3 validate_tafl.py <tafl_file.yaml>")
        print("\n範例:")
        print("  python3 validate_tafl.py my_flow.tafl.yaml")
        print("  python3 validate_tafl.py flows/rack_rotation_outlet.yaml")
        print("\n檔案搜尋順序:")
        print("  1. /home/ct/EBD_agv/app/config/tafl/")
        print("  2. ./當前目錄")
        sys.exit(1)
    
    filepath = sys.argv[1]
    success = validate_tafl_file(filepath)
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()