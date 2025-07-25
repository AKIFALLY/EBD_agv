#!/bin/bash
# RosAGV Docker 工具集
# 版本: 1.0
# 說明: 統一的 Docker 管理工具函數集，可以一次載入所有 Docker 工具功能

# ============================================================================
# 初始化和路徑設定
# ============================================================================

# 獲取腳本目錄
DOCKER_TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 檢查並載入所有 Docker 工具腳本
if [ -f "$DOCKER_TOOLS_DIR/agv-container.sh" ]; then
    source "$DOCKER_TOOLS_DIR/agv-container.sh"
fi

if [ -f "$DOCKER_TOOLS_DIR/agvc-container.sh" ]; then
    source "$DOCKER_TOOLS_DIR/agvc-container.sh"
fi

if [ -f "$DOCKER_TOOLS_DIR/container-status.sh" ]; then
    source "$DOCKER_TOOLS_DIR/container-status.sh"
fi

if [ -f "$DOCKER_TOOLS_DIR/quick-exec.sh" ]; then
    source "$DOCKER_TOOLS_DIR/quick-exec.sh"
fi

# ============================================================================
# 統一的 Docker 工具界面
# ============================================================================

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

show_docker_tools_header() {
    echo -e "${CYAN}🐳 RosAGV Docker 管理工具套件${NC}"
    echo -e "${CYAN}==============================${NC}"
    echo ""
}

show_docker_tools_help() {
    show_docker_tools_header
    
    echo -e "${YELLOW}📦 已載入的工具:${NC}"
    echo -e "  • agv-container.sh   - AGV 容器管理"
    echo -e "  • agvc-container.sh  - AGVC 容器管理"
    echo -e "  • container-status.sh - 容器狀態檢查"
    echo -e "  • quick-exec.sh      - 快速執行命令"
    echo ""
    
    echo -e "${YELLOW}🚀 快速操作命令:${NC}"
    echo -e "  ${GREEN}agv_start${NC}      - 啟動 AGV 系統"
    echo -e "  ${GREEN}agv_stop${NC}       - 停止 AGV 系統"
    echo -e "  ${GREEN}agv_enter${NC}      - 進入 AGV 容器"
    echo -e "  ${GREEN}agv_logs${NC}       - 查看 AGV 日誌"
    echo ""
    echo -e "  ${GREEN}agvc_start${NC}     - 啟動 AGVC 系統"
    echo -e "  ${GREEN}agvc_stop${NC}      - 停止 AGVC 系統"
    echo -e "  ${GREEN}agvc_enter${NC}     - 進入 AGVC 容器"
    echo -e "  ${GREEN}agvc_logs${NC}      - 查看 AGVC 日誌"
    echo ""
    echo -e "  ${GREEN}all_status${NC}     - 查看所有容器狀態"
    echo -e "  ${GREEN}all_health${NC}     - 系統健康檢查"
    echo -e "  ${GREEN}all_ports${NC}      - 檢查端口狀態"
    echo ""
    echo -e "  ${GREEN}quick_agv${NC}      - 在 AGV 容器執行命令"
    echo -e "  ${GREEN}quick_agvc${NC}     - 在 AGVC 容器執行命令"
    echo ""
    
    echo -e "${YELLOW}💡 使用範例:${NC}"
    echo -e "  agv_start                    # 啟動 AGV 系統"
    echo -e "  agvc_enter                   # 進入 AGVC 容器"
    echo -e "  all_status                   # 查看所有容器狀態"
    echo -e "  quick_agv node-list          # 快速查看 AGV 節點"
    echo -e "  quick_agvc check-status      # 快速檢查 AGVC 狀態"
    echo ""
    
    echo -e "${YELLOW}📝 詳細工具說明:${NC}"
    echo -e "  $DOCKER_TOOLS_DIR/agv-container.sh help"
    echo -e "  $DOCKER_TOOLS_DIR/agvc-container.sh help"
    echo -e "  $DOCKER_TOOLS_DIR/container-status.sh help"
    echo -e "  $DOCKER_TOOLS_DIR/quick-exec.sh help"
}

# ============================================================================
# 便捷別名和函數
# ============================================================================

# AGV 相關別名
agv_start() {
    "$DOCKER_TOOLS_DIR/agv-container.sh" start
}

agv_stop() {
    "$DOCKER_TOOLS_DIR/agv-container.sh" stop
}

agv_restart() {
    "$DOCKER_TOOLS_DIR/agv-container.sh" restart
}

agv_enter() {
    "$DOCKER_TOOLS_DIR/agv-container.sh" exec
}

agv_logs() {
    "$DOCKER_TOOLS_DIR/agv-container.sh" logs "$@"
}

agv_health() {
    "$DOCKER_TOOLS_DIR/agv-container.sh" health
}

agv_status() {
    "$DOCKER_TOOLS_DIR/agv-container.sh" status
}

# AGVC 相關別名
agvc_start() {
    "$DOCKER_TOOLS_DIR/agvc-container.sh" start
}

agvc_stop() {
    "$DOCKER_TOOLS_DIR/agvc-container.sh" stop
}

agvc_restart() {
    "$DOCKER_TOOLS_DIR/agvc-container.sh" restart
}

agvc_enter() {
    "$DOCKER_TOOLS_DIR/agvc-container.sh" exec
}

agvc_logs() {
    "$DOCKER_TOOLS_DIR/agvc-container.sh" logs "$@"
}

agvc_health() {
    "$DOCKER_TOOLS_DIR/agvc-container.sh" health
}

agvc_status() {
    "$DOCKER_TOOLS_DIR/agvc-container.sh" status
}

agvc_services() {
    "$DOCKER_TOOLS_DIR/agvc-container.sh" services
}

# 系統整體操作
all_status() {
    "$DOCKER_TOOLS_DIR/container-status.sh" all
}

all_health() {
    "$DOCKER_TOOLS_DIR/container-status.sh" health
}

all_ports() {
    "$DOCKER_TOOLS_DIR/container-status.sh" ports
}

all_resources() {
    "$DOCKER_TOOLS_DIR/container-status.sh" resources
}

all_network() {
    "$DOCKER_TOOLS_DIR/container-status.sh" network
}

all_summary() {
    "$DOCKER_TOOLS_DIR/container-status.sh" summary
}

# 全系統啟動/停止
all_start() {
    echo -e "${CYAN}🚀 啟動所有 RosAGV 系統${NC}"
    echo -e "${CYAN}=====================${NC}"
    
    echo -e "\n${YELLOW}1. 啟動 AGVC 系統${NC}"
    agvc_start
    
    echo -e "\n${YELLOW}2. 啟動 AGV 系統${NC}"
    agv_start
    
    echo -e "\n${YELLOW}3. 系統狀態摘要${NC}"
    sleep 3
    all_summary
}

all_stop() {
    echo -e "${CYAN}🛑 停止所有 RosAGV 系統${NC}"
    echo -e "${CYAN}=====================${NC}"
    
    echo -e "\n${YELLOW}1. 停止 AGV 系統${NC}"
    agv_stop
    
    echo -e "\n${YELLOW}2. 停止 AGVC 系統${NC}"
    agvc_stop
}

all_restart() {
    echo -e "${CYAN}🔄 重啟所有 RosAGV 系統${NC}"
    echo -e "${CYAN}=====================${NC}"
    
    all_stop
    echo -e "\n${YELLOW}等待 5 秒後重新啟動...${NC}"
    sleep 5
    all_start
}

# ============================================================================
# 進階功能函數
# ============================================================================

# 快速診斷函數
quick_diagnose() {
    echo -e "${CYAN}🔍 RosAGV 快速診斷${NC}"
    echo -e "${CYAN}==================${NC}"
    
    # 1. Docker 服務
    echo -e "\n${YELLOW}1. Docker 服務檢查${NC}"
    if docker info >/dev/null 2>&1; then
        print_success "Docker 服務正常運行"
    else
        print_error "Docker 服務未運行"
        return 1
    fi
    
    # 2. 容器狀態
    echo -e "\n${YELLOW}2. 容器快速檢查${NC}"
    local agv_running=$(docker ps -q -f name=rosagv | wc -l)
    local agvc_running=$(docker ps -q -f name=agvc_server | wc -l)
    local postgres_running=$(docker ps -q -f name=postgres | wc -l)
    local nginx_running=$(docker ps -q -f name=nginx | wc -l)
    
    echo -e "  AGV 容器: $([ $agv_running -gt 0 ] && echo -e "${GREEN}運行中${NC}" || echo -e "${RED}未運行${NC}")"
    echo -e "  AGVC 容器: $([ $agvc_running -gt 0 ] && echo -e "${GREEN}運行中${NC}" || echo -e "${RED}未運行${NC}")"
    echo -e "  PostgreSQL: $([ $postgres_running -gt 0 ] && echo -e "${GREEN}運行中${NC}" || echo -e "${RED}未運行${NC}")"
    echo -e "  Nginx: $([ $nginx_running -gt 0 ] && echo -e "${GREEN}運行中${NC}" || echo -e "${RED}未運行${NC}")"
    
    # 3. 關鍵端口
    echo -e "\n${YELLOW}3. 關鍵端口檢查${NC}"
    local ports_ok=0
    
    if timeout 1 bash -c "echo > /dev/tcp/localhost/80" 2>/dev/null; then
        echo -e "  Port 80 (Web): ${GREEN}可連接${NC}"
        ((ports_ok++))
    else
        echo -e "  Port 80 (Web): ${RED}無法連接${NC}"
    fi
    
    if timeout 1 bash -c "echo > /dev/tcp/192.168.100.100/7447" 2>/dev/null; then
        echo -e "  Port 7447 (Zenoh): ${GREEN}可連接${NC}"
        ((ports_ok++))
    else
        echo -e "  Port 7447 (Zenoh): ${RED}無法連接${NC}"
    fi
    
    # 4. 建議
    echo -e "\n${YELLOW}4. 診斷結果${NC}"
    local total_services=$((agv_running + agvc_running + postgres_running + nginx_running))
    
    if [ $total_services -eq 4 ] && [ $ports_ok -eq 2 ]; then
        print_success "系統運行正常"
    elif [ $total_services -gt 0 ]; then
        print_warning "系統部分運行，建議執行 'all_health' 進行詳細檢查"
    else
        print_error "系統未運行，請執行 'all_start' 啟動系統"
    fi
}

# 系統維護模式
maintenance_mode() {
    echo -e "${CYAN}🔧 進入維護模式${NC}"
    echo -e "${CYAN}===============${NC}"
    
    echo -e "${YELLOW}此模式將:${NC}"
    echo "  1. 停止所有容器"
    echo "  2. 清理未使用的 Docker 資源"
    echo "  3. 顯示系統資源使用情況"
    echo ""
    
    echo -ne "${YELLOW}確定要進入維護模式嗎? (y/N): ${NC}"
    read -r confirm
    
    if [[ "$confirm" =~ ^[Yy]$ ]]; then
        echo -e "\n${YELLOW}1. 停止所有容器${NC}"
        all_stop
        
        echo -e "\n${YELLOW}2. 清理 Docker 資源${NC}"
        docker system prune -f
        
        echo -e "\n${YELLOW}3. 系統資源狀況${NC}"
        df -h | grep -E "^/dev|Filesystem"
        echo ""
        docker system df
        
        echo -e "\n${GREEN}✅ 維護模式完成${NC}"
        echo -e "${YELLOW}使用 'all_start' 重新啟動系統${NC}"
    else
        echo -e "${YELLOW}取消維護模式${NC}"
    fi
}

# ============================================================================
# 輔助函數 (從其他腳本繼承的函數需要的)
# ============================================================================

print_status() {
    local icon="$1"
    local color="$2"
    local message="$3"
    echo -e "${color}${icon} ${message}${NC}"
}

print_success() {
    print_status "✅" "$GREEN" "$1"
}

print_error() {
    print_status "❌" "$RED" "$1"
}

print_warning() {
    print_status "⚠️" "$YELLOW" "$1"
}

print_info() {
    print_status "ℹ️" "$BLUE" "$1"
}

# ============================================================================
# 主程式邏輯
# ============================================================================

# 如果腳本被直接執行（而非被 source）
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    case "${1:-help}" in
        "help"|"-h"|"--help")
            show_docker_tools_help
            ;;
        "diagnose")
            quick_diagnose
            ;;
        "maintenance")
            maintenance_mode
            ;;
        *)
            echo -e "${RED}錯誤: 未知命令 '$1'${NC}"
            echo ""
            show_docker_tools_help
            exit 1
            ;;
    esac
else
    # 被 source 時顯示載入訊息
    echo -e "${GREEN}✅ RosAGV Docker 工具集已載入${NC}"
    echo -e "輸入 ${CYAN}show_docker_tools_help${NC} 查看可用命令"
fi

# ============================================================================
# 導出所有函數供外部使用
# ============================================================================

# 導出主要函數
export -f show_docker_tools_help
export -f show_docker_tools_header

# 導出 AGV 函數
export -f agv_start
export -f agv_stop
export -f agv_restart
export -f agv_enter
export -f agv_logs
export -f agv_health
export -f agv_status

# 導出 AGVC 函數
export -f agvc_start
export -f agvc_stop
export -f agvc_restart
export -f agvc_enter
export -f agvc_logs
export -f agvc_health
export -f agvc_status
export -f agvc_services

# 導出系統函數
export -f all_status
export -f all_health
export -f all_ports
export -f all_resources
export -f all_network
export -f all_summary
export -f all_start
export -f all_stop
export -f all_restart

# 導出進階函數
export -f quick_diagnose
export -f maintenance_mode

# 導出輔助函數
export -f print_status
export -f print_success
export -f print_error
export -f print_warning
export -f print_info