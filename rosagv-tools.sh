#!/bin/bash
# RosAGV 便捷工具選單
# 使用方法: bash rosagv-tools.sh [command]
# 或直接執行: ./rosagv-tools.sh [command]

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# 工具路徑
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

show_main_menu() {
    echo -e "${CYAN}🛠️ RosAGV 宿主機便捷工具選單${NC}"
    echo -e "${CYAN}=============================${NC}"
    echo ""
    echo -e "${YELLOW}🔍 系統診斷工具:${NC}"
    echo -e "  ${GREEN}agvc-check${NC}         # AGVC 管理主機快速健康檢查"
    echo -e "  ${GREEN}agv-check${NC}          # AGV 車載系統快速健康檢查"
    echo -e "  ${GREEN}system-health${NC}      # 完整系統健康檢查"
    echo -e "  ${GREEN}quick-diag${NC}         # 快速綜合診斷"
    echo ""
    echo -e "${YELLOW}🐳 容器管理工具:${NC}"
    echo -e "  ${GREEN}containers-status${NC}  # 檢查所有容器狀態"
    echo -e "  ${GREEN}agv-start${NC}          # 啟動 AGV 容器"
    echo -e "  ${GREEN}agv-stop${NC}           # 停止 AGV 容器"
    echo -e "  ${GREEN}agvc-start${NC}         # 啟動 AGVC 系統"
    echo -e "  ${GREEN}agvc-stop${NC}          # 停止 AGVC 系統"
    echo ""
    echo -e "${YELLOW}🎮 節點管理工具:${NC}"
    echo -e "  ${GREEN}node-status${NC}        # 所有節點狀態"
    echo -e "  ${GREEN}node-start [name]${NC}  # 啟動特定節點"
    echo -e "  ${GREEN}node-stop [name]${NC}   # 停止特定節點"
    echo -e "  ${GREEN}node-restart [name]${NC} # 重啟節點"
    echo -e "  ${GREEN}agv-nodes [name]${NC}   # 管理遠端 AGV 節點"
    echo ""
    echo -e "${YELLOW}🌐 網路診斷工具:${NC}"
    echo -e "  ${GREEN}network-check${NC}      # 系統端口檢查"
    echo -e "  ${GREEN}zenoh-check${NC}        # Zenoh 連接檢查"
    echo ""
    echo -e "${YELLOW}⚙️ 配置管理工具:${NC}"
    echo -e "  ${GREEN}zenoh-config${NC}       # Zenoh Router 配置管理"
    echo -e "  ${GREEN}hardware-config${NC}    # 硬體映射配置管理"
    echo ""
    echo -e "${YELLOW}📋 日誌分析工具:${NC}"
    echo -e "  ${GREEN}log-scan${NC}           # 日誌錯誤掃描"
    echo -e "  ${GREEN}log-errors${NC}         # 高級錯誤掃描"
    echo ""
    echo -e "${YELLOW}🛠️ 開發工具:${NC}"
    echo -e "  ${GREEN}dev-status${NC}         # 開發環境狀態"
    echo -e "  ${GREEN}dev-build${NC}          # 快速建置"
    echo -e "  ${GREEN}dev-test${NC}           # 快速測試"
    echo -e "  ${GREEN}dev-check${NC}          # 代碼檢查"
    echo ""
    echo -e "${YELLOW}📝 TAFL 語言工具:${NC}"
    echo -e "  ${GREEN}tafl-validate [file]${NC} # 驗證 TAFL 檔案格式"
    echo -e "  ${GREEN}tafl-validate all${NC}    # 驗證所有 TAFL 檔案"
    echo -e "  ${GREEN}tafl-validate list${NC}   # 列出所有 TAFL 檔案"
    echo ""
    echo -e "${YELLOW}❓ 幫助說明:${NC}"
    echo -e "  ${GREEN}tools-help${NC}         # 工具詳細說明"
    echo -e "  ${GREEN}menu${NC}               # 顯示此選單"
    echo ""
    echo -e "${BLUE}⚠️ 使用前提條件:${NC}"
    echo -e "  ${YELLOW}必須將 RosAGV 目錄加入 PATH 環境變數${NC}"
    echo -e "  在 ~/.bashrc 中添加: export PATH=\"/home/ct/EBD_agv:\$PATH\""
    echo -e "  然後執行: source ~/.bashrc"
    echo ""
    echo -e "${BLUE}使用方法:${NC}"
    echo -e "  r agvc-check        # 執行 AGVC 健康檢查 (推薦短命令)"
    echo -e "  r containers-status # 檢查容器狀態"
    echo -e "  r menu              # 顯示選單"
    echo -e "  r quick-diag        # 快速診斷"
    echo ""
    echo -e "${BLUE}完整命令:${NC}"
    echo -e "  bash rosagv-tools.sh agvc-check       # 完整命令形式"
    echo ""
}

show_tools_help() {
    echo -e "${CYAN}🛠️ RosAGV 工具詳細說明${NC}"
    echo -e "${CYAN}=====================${NC}"
    echo ""
    echo -e "${RED}⚠️ 重要前提條件:${NC}"
    echo -e "  使用 r 工具集之前，必須將 RosAGV 目錄加入 PATH 環境變數"
    echo -e "  在 ~/.bashrc 中添加: ${YELLOW}export PATH=\"/home/ct/EBD_agv:\$PATH\"${NC}"
    echo -e "  設定完成後執行: ${YELLOW}source ~/.bashrc${NC}"
    echo -e "  驗證設定: ${YELLOW}which r${NC} (應顯示 /home/ct/EBD_agv/r)"
    echo ""
    echo -e "${YELLOW}系統診斷工具:${NC}"
    echo -e "  agvc-check       - 檢查 AGVC 管理系統健康狀況"
    echo -e "  agv-check        - 檢查 AGV 車載系統健康狀況"
    echo -e "  system-health    - 執行完整系統健康檢查"
    echo -e "  quick-diag       - 快速診斷系統狀態和容器"
    echo ""
    echo -e "${YELLOW}容器管理工具:${NC}"
    echo -e "  containers-status - 顯示所有 Docker 容器狀態"
    echo -e "  agv-start        - 啟動 AGV 車載系統容器"
    echo -e "  agv-stop         - 停止 AGV 車載系統容器"
    echo -e "  agvc-start       - 啟動 AGVC 管理系統 (包含資料庫、Nginx)"
    echo -e "  agvc-stop        - 停止 AGVC 管理系統"
    echo ""
    echo -e "${YELLOW}網路診斷工具:${NC}"
    echo -e "  network-check    - 檢查系統關鍵端口連接狀況"
    echo -e "  zenoh-check      - 檢查 Zenoh Router 連接狀況"
    echo ""
    echo -e "${YELLOW}配置管理工具:${NC}"
    echo -e "  zenoh-config     - 管理 Zenoh Router 配置 (查看/編輯/驗證)"
    echo -e "  hardware-config  - 管理硬體映射配置 (設備/MAC地址)"
    echo ""
    echo -e "${YELLOW}日誌分析工具:${NC}"
    echo -e "  log-scan         - 掃描所有容器日誌中的警告和錯誤"
    echo -e "  log-errors       - 深度掃描嚴重錯誤和異常"
    echo ""
    echo -e "${YELLOW}開發工具:${NC}"
    echo -e "  dev-status       - 顯示開發環境狀態"
    echo -e "  dev-build        - 執行快速建置"
    echo -e "  dev-test         - 執行單元測試"
    echo -e "  dev-check        - 執行代碼風格檢查"
    echo ""
}

# 工具執行函數
run_agvc_check() {
    echo -e "${BLUE}🔍 執行 AGVC 管理主機健康檢查...${NC}"
    bash -c "cd '$SCRIPT_DIR' && scripts/system-tools/health-check.sh --agvc --quick"
}

run_agv_check() {
    echo -e "${BLUE}🔍 執行 AGV 車載系統健康檢查...${NC}"
    bash -c "cd '$SCRIPT_DIR' && scripts/system-tools/health-check.sh --agv --quick"
}

run_system_health() {
    echo -e "${BLUE}🔍 執行完整系統健康檢查...${NC}"
    bash -c "cd '$SCRIPT_DIR' && scripts/system-tools/health-check.sh --agvc --full"
}

run_quick_diag() {
    echo -e "${BLUE}🔍 執行快速綜合診斷...${NC}"
    bash -c "cd '$SCRIPT_DIR' && echo '🔍 系統快速診斷...' && scripts/system-tools/health-check.sh --agvc --quick && echo -e '\n📊 容器狀態...' && scripts/docker-tools/container-status.sh all"
}

run_containers_status() {
    echo -e "${BLUE}📊 檢查容器狀態...${NC}"
    bash -c "cd '$SCRIPT_DIR' && scripts/docker-tools/container-status.sh all"
}

run_agv_start() {
    echo -e "${BLUE}🚀 啟動 AGV 容器...${NC}"
    bash -c "cd '$SCRIPT_DIR' && scripts/docker-tools/agv-container.sh start"
}

run_agv_stop() {
    echo -e "${BLUE}⏹️ 停止 AGV 容器...${NC}"
    bash -c "cd '$SCRIPT_DIR' && scripts/docker-tools/agv-container.sh stop"
}

run_agvc_start() {
    echo -e "${BLUE}🚀 啟動 AGVC 系統...${NC}"
    bash -c "cd '$SCRIPT_DIR' && scripts/docker-tools/agvc-container.sh start"
}

run_agvc_stop() {
    echo -e "${BLUE}⏹️ 停止 AGVC 系統...${NC}"
    bash -c "cd '$SCRIPT_DIR' && scripts/docker-tools/agvc-container.sh stop"
}

run_network_check() {
    echo -e "${BLUE}🌐 執行網路檢查...${NC}"
    bash -c "cd '$SCRIPT_DIR' && scripts/network-tools/port-check.sh system"
}

run_zenoh_check() {
    echo -e "${BLUE}🔗 檢查 Zenoh 連接...${NC}"
    bash -c "cd '$SCRIPT_DIR' && scripts/network-tools/zenoh-network.sh connectivity"
}

run_log_scan() {
    echo -e "${BLUE}📋 掃描日誌錯誤...${NC}"
    bash -c "cd '$SCRIPT_DIR' && scripts/log-tools/log-analyzer.sh all --severity 3"
}

run_log_errors() {
    echo -e "${BLUE}📋 掃描嚴重錯誤...${NC}"
    bash -c "cd '$SCRIPT_DIR' && scripts/log-tools/log-analyzer.sh all --severity 4"
}

run_dev_status() {
    echo -e "${BLUE}🛠️ 檢查開發環境狀態...${NC}"
    bash -c "cd '$SCRIPT_DIR' && scripts/dev-tools/dev-tools.sh status"
}

run_dev_build() {
    echo -e "${BLUE}🔨 執行快速建置...${NC}"
    bash -c "cd '$SCRIPT_DIR' && scripts/dev-tools/build-helper.sh fast"
}

run_dev_test() {
    echo -e "${BLUE}🧪 執行單元測試...${NC}"
    bash -c "cd '$SCRIPT_DIR' && scripts/dev-tools/test-runner.sh unit"
}

run_dev_check() {
    echo -e "${BLUE}✅ 執行代碼檢查...${NC}"
    bash -c "cd '$SCRIPT_DIR' && scripts/dev-tools/code-analyzer.sh style"
}

run_zenoh_config() {
    echo -e "${BLUE}⚙️ Zenoh Router 配置管理...${NC}"
    bash -c "cd '$SCRIPT_DIR' && scripts/config-tools/zenoh-config.sh"
}

run_hardware_config() {
    echo -e "${BLUE}⚙️ 硬體映射配置管理...${NC}"
    bash -c "cd '$SCRIPT_DIR' && scripts/config-tools/hardware-mapping.sh"
}

# TAFL 驗證工具
run_tafl_validate() {
    local script="$SCRIPT_DIR/scripts/tafl-tools/tafl-validate.sh"
    if [ -f "$script" ]; then
        bash "$script" "$@"
    else
        echo -e "${RED}❌ 找不到 TAFL 驗證工具腳本${NC}"
        echo -e "${YELLOW}檔案應該在: $script${NC}"
        exit 1
    fi
}


# 節點管理工具
run_node_status() {
    echo -e "${BLUE}📊 檢查所有節點狀態...${NC}"
    bash -c "cd '$SCRIPT_DIR' && docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c 'source /app/setup.bash && agvc_source && manage_all_nodes status'"
}

run_node_start() {
    local node_name="${2:-}"
    if [ -z "$node_name" ]; then
        echo -e "${BLUE}🚀 啟動所有節點...${NC}"
        bash -c "cd '$SCRIPT_DIR' && docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c 'source /app/setup.bash && agvc_source && manage_all_nodes start'"
    else
        echo -e "${BLUE}🚀 啟動節點: $node_name...${NC}"
        bash -c "cd '$SCRIPT_DIR' && docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c 'source /app/setup.bash && agvc_source && manage_${node_name} start'"
    fi
}

run_node_stop() {
    local node_name="${2:-}"
    if [ -z "$node_name" ]; then
        echo -e "${BLUE}🛑 停止所有節點...${NC}"
        bash -c "cd '$SCRIPT_DIR' && docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c 'source /app/setup.bash && agvc_source && manage_all_nodes stop'"
    else
        echo -e "${BLUE}🛑 停止節點: $node_name...${NC}"
        bash -c "cd '$SCRIPT_DIR' && docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c 'source /app/setup.bash && agvc_source && manage_${node_name} stop'"
    fi
}

run_node_restart() {
    local node_name="${2:-}"
    if [ -z "$node_name" ]; then
        echo -e "${BLUE}🔄 重啟所有節點...${NC}"
        bash -c "cd '$SCRIPT_DIR' && docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c 'source /app/setup.bash && agvc_source && manage_all_nodes restart'"
    else
        echo -e "${BLUE}🔄 重啟節點: $node_name...${NC}"
        bash -c "cd '$SCRIPT_DIR' && docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c 'source /app/setup.bash && agvc_source && manage_${node_name} restart'"
    fi
}

run_agv_nodes() {
    local agv_name="${2:-}"
    local action="${3:-status}"
    
    if [ -z "$agv_name" ]; then
        echo -e "${RED}❌ 請指定 AGV 名稱${NC}"
        echo "用法: r agv-nodes <agv_name> [action]"
        echo "可用的 AGV: cargo02, loader02, unloader02"
        echo "可用的動作: status, start, stop, restart"
        return 1
    fi
    
    echo -e "${BLUE}🚗 管理 AGV 節點: $agv_name...${NC}"
    bash -c "cd '$SCRIPT_DIR' && docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c 'source /app/setup.bash && agvc_source && manage_agv_launch $agv_name $action'"
}

# 主程式邏輯
case "${1:-menu}" in
    # 系統診斷
    "agvc-check")
        run_agvc_check
        ;;
    "agv-check")
        run_agv_check
        ;;
    "system-health")
        run_system_health
        ;;
    "quick-diag")
        run_quick_diag
        ;;
        
    # 容器管理
    "containers-status")
        run_containers_status
        ;;
    "agv-start")
        run_agv_start
        ;;
    "agv-stop")
        run_agv_stop
        ;;
    "agvc-start")
        run_agvc_start
        ;;
    "agvc-stop")
        run_agvc_stop
        ;;
        
    # 網路診斷
    "network-check")
        run_network_check
        ;;
    "zenoh-check")
        run_zenoh_check
        ;;
        
    # 配置管理
    "zenoh-config")
        run_zenoh_config
        ;;
    "hardware-config")
        run_hardware_config
        ;;
        
    # 日誌分析
    "log-scan")
        run_log_scan
        ;;
    "log-errors")
        run_log_errors
        ;;
        
    # 開發工具
    "dev-status")
        run_dev_status
        ;;
    "dev-build")
        run_dev_build
        ;;
    "dev-test")
        run_dev_test
        ;;
    "dev-check")
        run_dev_check
        ;;
        
    # 配置管理
    "zenoh-config")
        run_zenoh_config
        ;;
    "hardware-config")
        run_hardware_config
        ;;
        
    # TAFL 工具
    "tafl-validate")
        shift  # 移除 'tafl-validate' 參數
        run_tafl_validate "$@"
        ;;
        
    # 節點管理
    "node-status")
        run_node_status
        ;;
    "node-start")
        run_node_start "$@"
        ;;
    "node-stop")
        run_node_stop "$@"
        ;;
    "node-restart")
        run_node_restart "$@"
        ;;
    "agv-nodes")
        run_agv_nodes "$@"
        ;;
        
    # 幫助選單
    "tools-help")
        show_tools_help
        ;;
    "menu"|"help"|"-h"|"--help"|"")
        show_main_menu
        ;;
        
    *)
        echo -e "${RED}❌ 未知命令: $1${NC}"
        echo ""
        show_main_menu
        exit 1
        ;;
esac