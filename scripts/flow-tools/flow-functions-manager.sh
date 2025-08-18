#!/bin/bash
# Flow Functions 管理工具
# 統一管理 Flow WCS 函數系統

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROSAGV_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

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
    echo -e "${CYAN}   Flow Functions 管理工具${NC}"
    echo -e "${CYAN}=====================================${NC}"
    echo ""
}

# 顯示狀態
show_status() {
    echo -e "${BLUE}📊 Flow Functions 系統狀態${NC}"
    echo -e "${BLUE}────────────────────────────${NC}"
    
    # 檢查各個檔案
    echo -e "\n${YELLOW}📁 檔案狀態：${NC}"
    
    # 1. 動態快取
    CACHE_FILE="$ROSAGV_DIR/app/config/wcs/flow_functions_cache.yaml"
    if [ -f "$CACHE_FILE" ]; then
        CACHE_TIME=$(stat -c %y "$CACHE_FILE" | cut -d' ' -f1,2 | cut -d'.' -f1)
        CACHE_SIZE=$(du -h "$CACHE_FILE" | cut -f1)
        echo -e "  動態快取: ${GREEN}✓${NC} ($CACHE_SIZE, 更新: $CACHE_TIME)"
    else
        echo -e "  動態快取: ${RED}✗${NC} (不存在)"
    fi
    
    # 2. 手動維護檔案
    MANUAL_FILE="/home/ct/RosAGV/app/config/wcs/flow_functions.yaml"
    if [ -f "$MANUAL_FILE" ]; then
        MANUAL_TIME=$(stat -c %y "$MANUAL_FILE" | cut -d' ' -f1,2 | cut -d'.' -f1)
        MANUAL_SIZE=$(du -h "$MANUAL_FILE" | cut -f1)
        echo -e "  手動維護: ${GREEN}✓${NC} ($MANUAL_SIZE, 更新: $MANUAL_TIME)"
    else
        echo -e "  手動維護: ${YELLOW}✗${NC} (不存在)"
    fi
    
    # 3. 靜態備援 (Linear Flow Designer)
    STATIC_FILE="/home/ct/RosAGV/app/config/wcs/static_fallback_functions.json"
    if [ -f "$STATIC_FILE" ]; then
        STATIC_TIME=$(stat -c %y "$STATIC_FILE" | cut -d' ' -f1,2 | cut -d'.' -f1)
        STATIC_SIZE=$(du -h "$STATIC_FILE" | cut -f1)
        echo -e "  靜態備援: ${GREEN}✓${NC} ($STATIC_SIZE, 更新: $STATIC_TIME)"
    else
        echo -e "  靜態備援: ${YELLOW}✗${NC} (不存在)"
    fi
    
    # 檢查 Flow WCS 服務
    echo -e "\n${YELLOW}🔌 服務狀態：${NC}"
    if curl -s -o /dev/null -w "%{http_code}" http://localhost:8000/api/flow/functions | grep -q "200"; then
        echo -e "  Flow WCS API: ${GREEN}✓ 運行中${NC}"
    else
        echo -e "  Flow WCS API: ${RED}✗ 未運行${NC}"
    fi
    
    # 檢查 Linear Flow Designer
    if curl -s -o /dev/null -w "%{http_code}" http://localhost:8001/linear-flow | grep -q "200"; then
        echo -e "  Linear Flow Designer: ${GREEN}✓ 運行中${NC}"
    else
        echo -e "  Linear Flow Designer: ${YELLOW}✗ 未運行${NC}"
    fi
    
    # 統計函數數量
    if [ -f "$CACHE_FILE" ]; then
        echo -e "\n${YELLOW}📈 函數統計：${NC}"
        python3 -c "
import yaml
try:
    with open('$CACHE_FILE', 'r') as f:
        data = yaml.safe_load(f)
        functions = data.get('functions', {})
        total = 0
        for category, funcs in functions.items():
            count = len(funcs)
            total += count
            print(f'  {category}: {count} 個函數')
        print(f'  ${GREEN}總計: {total} 個函數${NC}')
except:
    print('  ${RED}無法讀取快取${NC}')
"
    fi
}

# 更新手動維護檔案
update_manual() {
    echo -e "${BLUE}🔄 更新手動維護的 flow_functions.yaml${NC}"
    "$SCRIPT_DIR/update-flow-functions.sh"
}

# 更新靜態備援
update_static() {
    echo -e "${BLUE}🔄 更新 Linear Flow Designer 靜態備援${NC}"
    "$ROSAGV_DIR/scripts/sync-static-fallback.sh" sync
}

# 從 API 重新生成快取
refresh_cache() {
    echo -e "${BLUE}🔄 從 Flow WCS API 重新生成快取${NC}"
    
    # 不需要刪除舊快取，直接覆蓋即可
    # rm -f $ROSAGV_DIR/app/config/wcs/flow_functions_cache.yaml
    
    # 觸發 Linear Flow Designer 重新載入
    echo -e "  正在觸發 Linear Flow Designer 重新載入..."
    curl -s http://localhost:8001/linear-flow/api/functions > /dev/null 2>&1
    
    if [ -f "$ROSAGV_DIR/app/config/wcs/flow_functions_cache.yaml" ]; then
        echo -e "${GREEN}✅ 快取已重新生成${NC}"
    else
        echo -e "${YELLOW}⚠️  快取生成失敗，嘗試直接從 API 生成...${NC}"
        
        # 直接從 API 生成
        if curl -s http://localhost:8000/api/flow/functions | python3 -c "
import sys
import json
import yaml
from datetime import datetime

try:
    data = json.load(sys.stdin)
    if data.get('success'):
        output = {
            'meta': {
                'version': '2.0.0',
                'system': 'flow_wcs',
                'updated_at': datetime.now().isoformat()
            },
            'functions': data.get('functions', {})
        }
        with open('$ROSAGV_DIR/app/config/wcs/flow_functions_cache.yaml', 'w') as f:
            yaml.dump(output, f, allow_unicode=True, sort_keys=False)
        print('✅ 快取已生成')
        sys.exit(0)
except:
    pass
sys.exit(1)
" ; then
            echo -e "${GREEN}✅ 快取已從 API 生成${NC}"
        else
            echo -e "${RED}❌ 無法生成快取${NC}"
            return 1
        fi
    fi
}

# 比較差異
compare_files() {
    echo -e "${BLUE}🔍 比較函數檔案差異${NC}"
    echo -e "${BLUE}────────────────────────────${NC}"
    
    CACHE_FILE="$ROSAGV_DIR/app/config/wcs/flow_functions_cache.yaml"
    MANUAL_FILE="/home/ct/RosAGV/app/config/wcs/flow_functions.yaml"
    
    if [ ! -f "$CACHE_FILE" ]; then
        echo -e "${RED}❌ 快取檔案不存在${NC}"
        return 1
    fi
    
    if [ ! -f "$MANUAL_FILE" ]; then
        echo -e "${YELLOW}⚠️  手動維護檔案不存在${NC}"
        return 1
    fi
    
    python3 << EOF
import yaml
import sys

def load_functions(file_path):
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
            return data.get('functions', {})
    except:
        return {}

cache_funcs = load_functions('$CACHE_FILE')
manual_funcs = load_functions('$MANUAL_FILE')

# 統計差異
cache_set = set()
manual_set = set()

for cat, funcs in cache_funcs.items():
    for func in funcs:
        cache_set.add(func.get('name', ''))

for cat, funcs in manual_funcs.items():
    for func in funcs:
        manual_set.add(func.get('name', ''))

new_funcs = cache_set - manual_set
removed_funcs = manual_set - cache_set
common_funcs = cache_set & manual_set

print("\n📊 差異分析：")
print(f"  快取檔案: {len(cache_set)} 個函數")
print(f"  手動維護: {len(manual_set)} 個函數")
print("")

if new_funcs:
    print(f"  ✨ 新增函數: {len(new_funcs)} 個")
    for func in sorted(new_funcs)[:3]:
        print(f"     + {func}")
    if len(new_funcs) > 3:
        print(f"     ... 還有 {len(new_funcs)-3} 個")

if removed_funcs:
    print(f"  ⚠️  移除函數: {len(removed_funcs)} 個")
    for func in sorted(removed_funcs)[:3]:
        print(f"     - {func}")
    if len(removed_funcs) > 3:
        print(f"     ... 還有 {len(removed_funcs)-3} 個")

if common_funcs:
    print(f"  ✅ 相同函數: {len(common_funcs)} 個")

if not new_funcs and not removed_funcs:
    print("  ✅ 沒有差異，檔案已同步")
EOF
}

# 完整更新流程
full_update() {
    show_header
    echo -e "${YELLOW}執行完整更新流程...${NC}\n"
    
    # 1. 重新生成快取
    refresh_cache
    echo ""
    
    # 2. 更新手動維護檔案
    update_manual
    echo ""
    
    # 3. 更新靜態備援
    update_static
    echo ""
    
    echo -e "${GREEN}✨ 完整更新完成！${NC}"
}

# 顯示幫助
show_help() {
    show_header
    echo "用法: $0 [命令]"
    echo ""
    echo "命令:"
    echo "  status      顯示 Flow Functions 系統狀態"
    echo "  refresh     從 API 重新生成快取"
    echo "  update      更新手動維護的 flow_functions.yaml"
    echo "  static      更新 Linear Flow Designer 靜態備援"
    echo "  compare     比較快取和手動維護檔案的差異"
    echo "  full        執行完整更新流程"
    echo "  help        顯示此幫助信息"
    echo ""
    echo "檔案說明:"
    echo "  動態快取: $ROSAGV_DIR/app/config/wcs/flow_functions_cache.yaml"
    echo "  手動維護: /app/config/wcs/flow_functions.yaml"
    echo "  靜態備援: Linear Flow Designer 內嵌函數"
    echo ""
    echo "範例:"
    echo "  $0 status   # 查看系統狀態"
    echo "  $0 compare  # 比較差異"
    echo "  $0 full     # 完整更新所有檔案"
}

# 主程式
case "$1" in
    status)
        show_header
        show_status
        ;;
    refresh)
        show_header
        refresh_cache
        ;;
    update)
        show_header
        update_manual
        ;;
    static)
        show_header
        update_static
        ;;
    compare)
        show_header
        compare_files
        ;;
    full)
        full_update
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        if [ -z "$1" ]; then
            show_header
            show_status
            echo ""
            echo "使用 '$0 help' 查看更多選項"
        else
            show_header
            echo -e "${RED}未知命令: $1${NC}"
            echo ""
            echo "使用 '$0 help' 查看可用命令"
            exit 1
        fi
        ;;
esac