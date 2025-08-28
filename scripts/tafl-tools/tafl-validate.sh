#!/bin/bash
# TAFL (Task Automation Flow Language) 驗證工具
# 用於驗證 TAFL 格式檔案的語法和結構正確性

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# TAFL 工作目錄
TAFL_DIR="/home/ct/RosAGV/app/tafl_ws"
TAFL_TOOLS_DIR="/home/ct/RosAGV/scripts/tafl-tools"
TAFL_VALIDATOR="$TAFL_TOOLS_DIR/validate-tafl.py"
TAFL_BATCH_VALIDATOR="$TAFL_TOOLS_DIR/validate-all-tafl.py"

# TAFL 檔案位置（優先順序）
CONFIG_TAFL_DIR="/home/ct/RosAGV/app/config/tafl"  # 優先：正式配置
MIGRATED_TAFL_DIR="$TAFL_DIR/migrated_flows"       # 次要：開發/測試

# 顯示使用說明
show_usage() {
    echo -e "${CYAN}📋 TAFL 驗證工具${NC}"
    echo -e "${CYAN}===================${NC}"
    echo ""
    echo -e "${YELLOW}用法:${NC}"
    echo -e "  r tafl-validate <file>     # 驗證單個 TAFL 檔案"
    echo -e "  r tafl-validate all        # 驗證所有 TAFL 檔案"
    echo -e "  r tafl-validate list       # 列出所有 TAFL 檔案"
    echo -e "  r tafl-validate help       # 顯示此說明"
    echo ""
    echo -e "${YELLOW}範例:${NC}"
    echo -e "  r tafl-validate my_flow.yaml"
    echo -e "  r tafl-validate /path/to/flow.tafl.yaml"
    echo -e "  r tafl-validate migrated_flows/rack_rotation_room_outlet_tafl.yaml"
    echo ""
    echo -e "${YELLOW}TAFL 檔案位置:${NC}"
    echo -e "  優先目錄: $CONFIG_TAFL_DIR/  (正式配置)"
    echo -e "  次要目錄: $MIGRATED_TAFL_DIR/  (開發/測試)"
    echo ""
    echo -e "${YELLOW}驗證內容:${NC}"
    echo -e "  ✅ YAML 語法正確性"
    echo -e "  ✅ TAFL 語法解析 (動詞識別)"
    echo -e "  ✅ 核心必要參數檢查:"
    echo -e "      - query: 必須有 target"
    echo -e "      - check: 必須有 condition"
    echo -e "      - create: 必須有 target"
    echo -e "      - update: 必須有 target 和 id"
    echo -e "      - notify: 必須有 message"
    echo -e "  ✅ Metadata 完整性 (id, name)"
    echo -e "  ✅ Settings 檢查 (timeout, max_retries)"
    echo -e "  ⚠️  變數使用警告 (未定義/未使用)"
}

# 列出所有 TAFL 檔案
list_tafl_files() {
    echo -e "${CYAN}📁 TAFL 檔案列表${NC}"
    echo -e "${CYAN}===================${NC}"
    
    local config_count=0
    local migrated_count=0
    
    # 優先列出 config/tafl 目錄的檔案（包含子目錄）
    if [ -d "$CONFIG_TAFL_DIR" ]; then
        echo -e "\n${GREEN}⭐ 正式配置 TAFL 檔案 (config/tafl):${NC}"
        # 搜尋所有子目錄
        while IFS= read -r file; do
            # 顯示相對路徑
            relative_path="${file#$CONFIG_TAFL_DIR/}"
            echo "  $relative_path"
            ((config_count++))
        done < <(find "$CONFIG_TAFL_DIR" -type f \( -name "*.yaml" -o -name "*.tafl" \) 2>/dev/null | sort)
        
        if [ $config_count -eq 0 ]; then
            echo "  (空目錄)"
        fi
    fi
    
    # 列出 migrated_flows 目錄的檔案
    if [ -d "$MIGRATED_TAFL_DIR" ]; then
        echo -e "\n${YELLOW}🔧 開發/測試 TAFL 檔案 (migrated_flows):${NC}"
        for file in "$MIGRATED_TAFL_DIR"/*tafl*.yaml "$MIGRATED_TAFL_DIR"/*.tafl; do
            if [ -f "$file" ]; then
                echo "  $(basename "$file")"
                ((migrated_count++))
            fi
        done 2>/dev/null
        if [ $migrated_count -eq 0 ]; then
            echo "  (空目錄)"
        fi
    fi
    
    # 統計
    local total_count=$((config_count + migrated_count))
    echo -e "\n${CYAN}📊 統計:${NC}"
    echo -e "  正式配置: $config_count 個"
    echo -e "  開發測試: $migrated_count 個"
    echo -e "  ${GREEN}總計: $total_count 個 TAFL 檔案${NC}"
}

# 驗證單個 TAFL 檔案
validate_single_file() {
    local file="$1"
    
    # 如果沒有路徑，嘗試在預設目錄尋找
    if [[ ! "$file" == /* ]] && [[ ! "$file" == ./* ]]; then
        # 優先在 config/tafl 目錄尋找（包含子目錄）
        found_file=$(find "$CONFIG_TAFL_DIR" -type f -name "$file" 2>/dev/null | head -1)
        if [ -n "$found_file" ]; then
            file="$found_file"
            echo -e "${GREEN}✔ 在正式配置目錄找到檔案${NC}"
        # 次要在 migrated_flows 目錄
        elif [ -f "$MIGRATED_TAFL_DIR/$file" ]; then
            file="$MIGRATED_TAFL_DIR/$file"
            echo -e "${YELLOW}✔ 在開發/測試目錄找到檔案${NC}"
        # 嘗試在當前目錄
        elif [ -f "./$file" ]; then
            file="./$file"
            echo -e "${BLUE}✔ 在當前目錄找到檔案${NC}"
        fi
    fi
    
    # 檢查檔案是否存在
    if [ ! -f "$file" ]; then
        echo -e "${RED}❌ 檔案不存在: $file${NC}"
        echo -e "${YELLOW}提示: 使用 'r tafl-validate list' 查看可用的 TAFL 檔案${NC}"
        return 1
    fi
    
    # 切換到 TAFL 目錄執行驗證
    cd "$TAFL_DIR" || exit 1
    
    echo -e "${CYAN}🔍 驗證 TAFL 檔案${NC}"
    echo -e "${CYAN}===================${NC}"
    
    # 執行驗證
    if python3 "$TAFL_VALIDATOR" "$file"; then
        echo -e "\n${GREEN}✅ 驗證成功！檔案格式正確。${NC}"
        return 0
    else
        echo -e "\n${RED}❌ 驗證失敗！請檢查錯誤訊息。${NC}"
        return 1
    fi
}

# 驗證所有 TAFL 檔案
validate_all_files() {
    echo -e "${CYAN}🔍 批量驗證所有 TAFL 檔案${NC}"
    echo -e "${CYAN}=============================${NC}"
    
    # 切換到 TAFL 目錄
    cd "$TAFL_DIR" || exit 1
    
    # 執行批量驗證
    if python3 "$TAFL_BATCH_VALIDATOR"; then
        echo -e "\n${GREEN}✅ 批量驗證完成${NC}"
        return 0
    else
        echo -e "\n${YELLOW}⚠️ 部分檔案驗證失敗，請查看上方詳細資訊${NC}"
        return 1
    fi
}

# 主程式
main() {
    # 檢查 Python 環境
    if ! command -v python3 &> /dev/null; then
        echo -e "${RED}❌ 錯誤: 需要 Python 3${NC}"
        exit 1
    fi
    
    # 檢查驗證腳本是否存在
    if [ ! -f "$TAFL_VALIDATOR" ]; then
        echo -e "${YELLOW}⚠️ 驗證腳本不存在，正在創建...${NC}"
        # 這裡可以自動創建驗證腳本
        echo -e "${RED}❌ 無法找到驗證腳本: $TAFL_VALIDATOR${NC}"
        exit 1
    fi
    
    # 處理參數
    case "${1:-help}" in
        "help"|"-h"|"--help")
            show_usage
            ;;
        "list"|"ls")
            list_tafl_files
            ;;
        "all"|"--all")
            validate_all_files
            ;;
        *)
            validate_single_file "$1"
            ;;
    esac
}

# 執行主程式
main "$@"