#!/bin/bash
# RosAGV 系統工具集
# 版本: 1.0
# 說明: 統一的系統診斷和監控工具函數集

# ============================================================================
# 初始化和路徑設定
# ============================================================================

# 獲取腳本目錄
SYSTEM_TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 檢查並載入所有系統工具腳本
if [ -f "$SYSTEM_TOOLS_DIR/health-check.sh" ]; then
    source "$SYSTEM_TOOLS_DIR/health-check.sh"
fi

if [ -f "$SYSTEM_TOOLS_DIR/service-monitor.sh" ]; then
    source "$SYSTEM_TOOLS_DIR/service-monitor.sh"
fi

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# ============================================================================
# 統一的系統工具界面
# ============================================================================

show_system_tools_header() {
    echo -e "${CYAN}🔍 RosAGV 系統診斷和監控工具套件${NC}"
    echo -e "${CYAN}===================================${NC}"
    echo ""
}

show_system_tools_help() {
    show_system_tools_header
    
    echo -e "${YELLOW}📦 已載入的工具:${NC}"
    echo -e "  • health-check.sh    - 全面系統健康檢查"
    echo -e "  • service-monitor.sh - 服務狀態監控"
    echo ""
    
    echo -e "${YELLOW}🚀 快速操作命令:${NC}"
    echo -e "  ${GREEN}system_health${NC}       - 執行完整健康檢查"
    echo -e "  ${GREEN}system_quick_check${NC}  - 快速健康檢查"
    echo -e "  ${GREEN}system_monitor${NC}      - 啟動服務監控"
    echo -e "  ${GREEN}system_watch${NC}        - 連續監控服務"
    echo -e "  ${GREEN}system_status${NC}       - 顯示所有服務狀態"
    echo -e "  ${GREEN}system_restart${NC}      - 重啟指定服務"
    echo ""
    
    echo -e "${YELLOW}💡 使用範例:${NC}"
    echo -e "  system_health                 # 完整健康檢查"
    echo -e "  system_quick_check            # 快速檢查"
    echo -e "  system_monitor --auto-restart # 自動監控與重啟"
    echo -e "  system_restart postgres       # 重啟 PostgreSQL"
    echo ""
    
    echo -e "${YELLOW}📝 詳細工具說明:${NC}"
    echo -e "  $SYSTEM_TOOLS_DIR/health-check.sh --help"
    echo -e "  $SYSTEM_TOOLS_DIR/service-monitor.sh --help"
}

# ============================================================================
# 便捷別名和函數
# ============================================================================

# 健康檢查相關
system_health() {
    "$SYSTEM_TOOLS_DIR/health-check.sh" --full "$@"
}

system_quick_check() {
    "$SYSTEM_TOOLS_DIR/health-check.sh" --quick "$@"
}

system_health_report() {
    "$SYSTEM_TOOLS_DIR/health-check.sh" --report "$@"
}

system_health_fix() {
    "$SYSTEM_TOOLS_DIR/health-check.sh" --fix "$@"
}

# 服務監控相關
system_status() {
    "$SYSTEM_TOOLS_DIR/service-monitor.sh" status "$@"
}

system_monitor() {
    "$SYSTEM_TOOLS_DIR/service-monitor.sh" monitor "$@"
}

system_watch() {
    "$SYSTEM_TOOLS_DIR/service-monitor.sh" watch "$@"
}

system_restart() {
    "$SYSTEM_TOOLS_DIR/service-monitor.sh" restart "$@"
}

system_check_service() {
    "$SYSTEM_TOOLS_DIR/service-monitor.sh" check "$@"
}

system_deps() {
    "$SYSTEM_TOOLS_DIR/service-monitor.sh" deps "$@"
}

system_alerts() {
    "$SYSTEM_TOOLS_DIR/service-monitor.sh" alerts "$@"
}

# 組合操作
system_full_check() {
    echo -e "${CYAN}🔍 執行全面系統檢查${NC}"
    echo -e "${CYAN}===================${NC}"
    
    echo -e "\n${YELLOW}1. 健康檢查${NC}"
    system_health --quick
    
    echo -e "\n${YELLOW}2. 服務狀態${NC}"
    system_status
    
    echo -e "\n${YELLOW}3. 依賴關係${NC}"
    system_deps
}

system_emergency_check() {
    echo -e "${RED}🚨 緊急診斷模式${NC}"
    echo -e "${RED}===============${NC}"
    
    # 快速檢查並嘗試修復
    system_health --fix
    
    # 顯示服務狀態
    system_status
    
    # 如果有問題，嘗試重啟服務
    echo -e "\n${YELLOW}嘗試重啟關鍵服務...${NC}"
    system_restart zenoh
    system_restart postgres
    system_restart api
}

# ============================================================================
# 主程式邏輯
# ============================================================================

# 如果腳本被直接執行（而非被 source）
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    case "${1:-help}" in
        "help"|"-h"|"--help")
            show_system_tools_help
            ;;
        "status")
            system_status
            ;;
        "health")
            system_health
            ;;
        "monitor")
            shift
            system_monitor "$@"
            ;;
        "full")
            system_full_check
            ;;
        "emergency")
            system_emergency_check
            ;;
        *)
            echo -e "${RED}錯誤: 未知命令 '$1'${NC}"
            echo ""
            show_system_tools_help
            exit 1
            ;;
    esac
else
    # 被 source 時顯示載入訊息
    echo -e "${GREEN}✅ RosAGV 系統工具集已載入${NC}"
    echo -e "輸入 ${CYAN}show_system_tools_help${NC} 查看可用命令"
fi

# ============================================================================
# 導出所有函數供外部使用
# ============================================================================

# 導出主要函數
export -f show_system_tools_help
export -f show_system_tools_header

# 導出健康檢查函數
export -f system_health
export -f system_quick_check
export -f system_health_report
export -f system_health_fix

# 導出監控函數
export -f system_status
export -f system_monitor
export -f system_watch
export -f system_restart
export -f system_check_service
export -f system_deps
export -f system_alerts

# 導出組合函數
export -f system_full_check
export -f system_emergency_check