#!/bin/bash
# Flow Functions 開發輔助工具
# 簡化開發流程：編輯 → 測試 → 同步

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROSAGV_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
FLOW_EXECUTOR="/home/ct/RosAGV/app/flow_wcs_ws/src/flow_wcs/flow_wcs/flow_executor.py"

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 顯示標題
show_header() {
    echo -e "${CYAN}=====================================${NC}"
    echo -e "${CYAN}   Flow Functions 開發工具${NC}"
    echo -e "${CYAN}=====================================${NC}"
    echo ""
}

# 編輯 flow_executor.py
edit_flow() {
    echo -e "${BLUE}📝 開啟 flow_executor.py 編輯器${NC}"
    echo -e "   檔案: $FLOW_EXECUTOR"
    echo ""
    
    # 檢查編輯器
    if command -v code > /dev/null; then
        code "$FLOW_EXECUTOR"
        echo -e "${GREEN}✅ 已用 VS Code 開啟${NC}"
    elif command -v vim > /dev/null; then
        vim "$FLOW_EXECUTOR"
    elif command -v nano > /dev/null; then
        nano "$FLOW_EXECUTOR"
    else
        echo -e "${RED}❌ 找不到編輯器${NC}"
        exit 1
    fi
}

# 驗證語法
validate_syntax() {
    echo -e "${BLUE}🔍 驗證 Python 語法${NC}"
    
    if python3 -m py_compile "$FLOW_EXECUTOR" 2>/dev/null; then
        echo -e "${GREEN}✅ 語法檢查通過${NC}"
        
        # 檢查裝飾器使用
        echo ""
        echo -e "${BLUE}📊 @flow_function 統計:${NC}"
        DECORATOR_COUNT=$(grep -c "@flow_function" "$FLOW_EXECUTOR" || echo "0")
        echo -e "   發現 $DECORATOR_COUNT 個 @flow_function 裝飾器"
        
        # 顯示最新添加的函數
        echo ""
        echo -e "${BLUE}🆕 最近的 @flow_function:${NC}"
        grep -B1 "@flow_function" "$FLOW_EXECUTOR" | tail -6
        
        return 0
    else
        echo -e "${RED}❌ 語法錯誤，請修正後再試${NC}"
        python3 -m py_compile "$FLOW_EXECUTOR"
        return 1
    fi
}

# 測試函數載入
test_loading() {
    echo -e "${BLUE}🧪 測試函數載入${NC}"
    
    # 創建測試腳本（只測試裝飾器，不實際載入模組）
    cat > /tmp/test_flow_functions.py << 'EOF'
#!/usr/bin/env python3
import sys
import re

try:
    # 直接解析檔案中的 @flow_function 裝飾器
    flow_executor_path = '/home/ct/RosAGV/app/flow_wcs_ws/src/flow_wcs/flow_wcs/flow_executor.py'
    
    with open(flow_executor_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 找出所有 @flow_function 裝飾器
    pattern = r'@flow_function\((.*?)\)\s*\ndef\s+(\w+)'
    matches = re.findall(pattern, content, re.DOTALL)
    
    # 解析函數
    categories = {}
    functions = []
    
    for decorator_args, func_name in matches:
        # 簡單解析第一個參數（category）
        try:
            # 提取第一個引號內的內容作為 category
            category_match = re.search(r'"([^"]+)"', decorator_args)
            if category_match:
                category = category_match.group(1)
                if category not in categories:
                    categories[category] = []
                categories[category].append(func_name)
                functions.append(func_name)
        except:
            pass
    
    print(f"✅ 發現 {len(functions)} 個 @flow_function 裝飾器")
    print("\n📂 函數分類:")
    for cat, funcs in sorted(categories.items()):
        print(f"  • {cat}: {len(funcs)} 個函數")
        # 顯示前3個函數
        for func in funcs[:3]:
            print(f"    - {func}")
        if len(funcs) > 3:
            print(f"    ... 還有 {len(funcs)-3} 個")
    
    # 檢查是否有重複的函數名
    from collections import Counter
    duplicates = [name for name, count in Counter(functions).items() if count > 1]
    if duplicates:
        print(f"\n⚠️  發現重複的函數名: {', '.join(duplicates)}")
    
    sys.exit(0)
except Exception as e:
    print(f"❌ 測試失敗: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
EOF
    
    python3 /tmp/test_flow_functions.py
    return $?
}

# 自動同步所有檔案
auto_sync() {
    echo -e "${BLUE}🔄 自動同步所有相關檔案${NC}"
    
    # 執行自動同步腳本
    if [ -f "$SCRIPT_DIR/auto-sync-functions.sh" ]; then
        "$SCRIPT_DIR/auto-sync-functions.sh"
    else
        echo -e "${RED}❌ auto-sync-functions.sh 不存在${NC}"
        return 1
    fi
}

# 快速預覽
quick_preview() {
    echo -e "${BLUE}👁️  快速預覽函數變更${NC}"
    echo ""
    
    # 比較快取和手動檔案
    if [ -f "$ROSAGV_DIR/app/config/wcs/flow_functions_cache.yaml" ] && [ -f "$ROSAGV_DIR/app/config/wcs/flow_functions.yaml" ]; then
        python3 << EOF
import yaml

def load_functions(file_path):
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
            return data.get('functions', {})
    except:
        return {}

cache = load_functions('$ROSAGV_DIR/app/config/wcs/flow_functions_cache.yaml')
manual = load_functions('$ROSAGV_DIR/app/config/wcs/flow_functions.yaml')

# 找出差異
cache_funcs = set()
manual_funcs = set()

for cat, funcs in cache.items():
    for func in funcs:
        cache_funcs.add(func.get('name', ''))

for cat, funcs in manual.items():
    for func in funcs:
        manual_funcs.add(func.get('name', ''))

new = cache_funcs - manual_funcs
removed = manual_funcs - cache_funcs

if new:
    print("✨ 新增函數:")
    for func in sorted(new)[:5]:
        print(f"  + {func}")
    if len(new) > 5:
        print(f"  ... 還有 {len(new)-5} 個")

if removed:
    print("⚠️  移除函數:")
    for func in sorted(removed)[:5]:
        print(f"  - {func}")
    if len(removed) > 5:
        print(f"  ... 還有 {len(removed)-5} 個")

if not new and not removed:
    print("✅ 所有檔案已同步")
EOF
    fi
}

# 顯示幫助
show_help() {
    show_header
    echo "用法: $0 [命令]"
    echo ""
    echo "命令:"
    echo "  edit       編輯 flow_executor.py"
    echo "  validate   驗證 Python 語法"
    echo "  test       測試函數載入"
    echo "  sync       自動同步所有檔案"
    echo "  preview    預覽函數變更"
    echo "  workflow   完整工作流程 (驗證→測試→同步)"
    echo "  help       顯示此幫助信息"
    echo ""
    echo "快速開發流程:"
    echo "  1. $0 edit      # 編輯並添加 @flow_function"
    echo "  2. $0 workflow  # 驗證、測試並同步"
    echo ""
    echo "範例: 添加新函數"
    echo '  @flow_function("action", "我的新函數", ["param1"], "boolean")'
    echo '  def my_new_function(self, params):'
    echo '      return True'
}

# 完整工作流程
run_workflow() {
    show_header
    echo -e "${YELLOW}執行完整工作流程...${NC}"
    echo ""
    
    # 1. 驗證語法
    if ! validate_syntax; then
        echo -e "${RED}工作流程中止：語法錯誤${NC}"
        exit 1
    fi
    echo ""
    
    # 2. 測試載入
    if ! test_loading; then
        echo -e "${RED}工作流程中止：函數載入失敗${NC}"
        exit 1
    fi
    echo ""
    
    # 3. 自動同步
    auto_sync
    echo ""
    
    # 4. 預覽結果
    quick_preview
    echo ""
    
    echo -e "${GREEN}✨ 工作流程完成！${NC}"
}

# 主程式
case "$1" in
    edit)
        show_header
        edit_flow
        ;;
    validate)
        show_header
        validate_syntax
        ;;
    test)
        show_header
        test_loading
        ;;
    sync)
        show_header
        auto_sync
        ;;
    preview)
        show_header
        quick_preview
        ;;
    workflow)
        run_workflow
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        show_header
        if [ -z "$1" ]; then
            echo -e "${YELLOW}提示: 使用 '$0 help' 查看可用命令${NC}"
            echo ""
            echo "快速開始:"
            echo "  $0 edit      # 編輯 flow_executor.py"
            echo "  $0 workflow  # 執行完整更新流程"
        else
            echo -e "${RED}未知命令: $1${NC}"
            echo ""
            echo "使用 '$0 help' 查看可用命令"
            exit 1
        fi
        ;;
esac