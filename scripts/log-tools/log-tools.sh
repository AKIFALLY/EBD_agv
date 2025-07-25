#!/bin/bash
# RosAGV 日誌分析工具集
# 版本: 1.0
# 說明: 統一的日誌分析和監控工具函數集

# ============================================================================
# 初始化和路徑設定
# ============================================================================

# 獲取腳本目錄
LOG_TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 檢查並載入所有日誌工具腳本
if [ -f "$LOG_TOOLS_DIR/log-analyzer.sh" ]; then
    source "$LOG_TOOLS_DIR/log-analyzer.sh"
fi

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# ============================================================================
# 統一的日誌工具界面
# ============================================================================

show_log_tools_header() {
    echo -e "${CYAN}📊 RosAGV 日誌分析工具套件${NC}"
    echo -e "${CYAN}===========================${NC}"
    echo ""
}

show_log_tools_help() {
    show_log_tools_header
    
    echo -e "${YELLOW}📦 已載入的工具:${NC}"
    echo -e "  • log-analyzer.sh    - 智能日誌分析"
    echo ""
    
    echo -e "${YELLOW}🚀 快速操作命令:${NC}"
    echo -e "  ${GREEN}log_analyze${NC}         - 分析指定來源日誌"
    echo -e "  ${GREEN}log_quick_scan${NC}      - 快速掃描所有容器錯誤"
    echo -e "  ${GREEN}log_find_errors${NC}     - 查找特定錯誤模式"
    echo -e "  ${GREEN}log_stats${NC}           - 顯示日誌統計資訊"
    echo -e "  ${GREEN}log_timeline${NC}        - 顯示錯誤時間軸"
    echo -e "  ${GREEN}log_suggestions${NC}     - 獲取解決建議"
    echo -e "  ${GREEN}log_export_report${NC}   - 生成分析報告"
    echo ""
    
    echo -e "${YELLOW}💡 使用範例:${NC}"
    echo -e "  log_analyze agv                # 分析 AGV 容器日誌"
    echo -e "  log_quick_scan                 # 快速掃描所有錯誤"
    echo -e "  log_find_errors \"timeout\"      # 查找超時錯誤"
    echo -e "  log_stats postgres             # PostgreSQL 日誌統計"
    echo -e "  log_export_report /tmp/report  # 生成完整報告"
    echo ""
    
    echo -e "${YELLOW}📝 詳細工具說明:${NC}"
    echo -e "  $LOG_TOOLS_DIR/log-analyzer.sh --help"
}

# ============================================================================
# 便捷別名和函數
# ============================================================================

# 基本日誌分析
log_analyze() {
    "$LOG_TOOLS_DIR/log-analyzer.sh" "$@"
}

# 快速掃描所有容器的關鍵錯誤
log_quick_scan() {
    echo -e "${CYAN}🔍 快速掃描所有容器錯誤${NC}"
    echo -e "${CYAN}========================${NC}"
    
    "$LOG_TOOLS_DIR/log-analyzer.sh" all --severity 3 --lines 500
}

# 查找特定錯誤模式
log_find_errors() {
    local pattern="$1"
    if [ -z "$pattern" ]; then
        echo -e "${RED}錯誤: 請提供錯誤模式${NC}"
        return 1
    fi
    
    echo -e "${CYAN}🔎 查找錯誤模式: $pattern${NC}"
    echo -e "${CYAN}========================${NC}"
    
    "$LOG_TOOLS_DIR/log-analyzer.sh" all --pattern "$pattern" --suggestions
}

# 顯示統計資訊
log_stats() {
    local source="${1:-all}"
    
    echo -e "${CYAN}📈 日誌統計分析${NC}"
    echo -e "${CYAN}===============${NC}"
    
    "$LOG_TOOLS_DIR/log-analyzer.sh" "$source" --stats
}

# 顯示錯誤時間軸
log_timeline() {
    local source="${1:-all}"
    
    echo -e "${CYAN}⏰ 錯誤時間軸${NC}"
    echo -e "${CYAN}============${NC}"
    
    "$LOG_TOOLS_DIR/log-analyzer.sh" "$source" --timeline
}

# 獲取解決建議
log_suggestions() {
    local source="${1:-all}"
    
    echo -e "${CYAN}💡 解決建議${NC}"
    echo -e "${CYAN}============${NC}"
    
    "$LOG_TOOLS_DIR/log-analyzer.sh" "$source" --suggestions
}

# 生成完整分析報告
log_export_report() {
    local output_file="${1:-/tmp/rosagv_log_report_$(date +%Y%m%d_%H%M%S).txt}"
    
    echo -e "${CYAN}📄 生成日誌分析報告${NC}"
    echo -e "${CYAN}==================${NC}"
    
    {
        echo "======================================"
        echo "RosAGV 日誌分析報告"
        echo "======================================"
        echo "生成時間: $(date '+%Y-%m-%d %H:%M:%S')"
        echo "主機名稱: $(hostname)"
        echo ""
        
        echo "1. 所有容器錯誤掃描"
        echo "===================="
        "$LOG_TOOLS_DIR/log-analyzer.sh" all --severity 2 --lines 1000
        
        echo ""
        echo "2. 統計資訊"
        echo "==========="
        "$LOG_TOOLS_DIR/log-analyzer.sh" all --stats
        
        echo ""
        echo "3. 錯誤時間軸"
        echo "============"
        "$LOG_TOOLS_DIR/log-analyzer.sh" all --timeline
        
        echo ""
        echo "4. 解決建議"
        echo "==========="
        "$LOG_TOOLS_DIR/log-analyzer.sh" all --suggestions
        
    } > "$output_file"
    
    echo -e "${GREEN}✅ 報告已生成: $output_file${NC}"
    
    # 顯示報告摘要
    local total_errors=$(grep -c "錯誤" "$output_file" 2>/dev/null || echo 0)
    local critical_errors=$(grep -c "CRITICAL\|嚴重" "$output_file" 2>/dev/null || echo 0)
    
    echo -e "報告摘要:"
    echo -e "  總錯誤數: ${RED}$total_errors${NC}"
    echo -e "  嚴重錯誤: ${PURPLE}$critical_errors${NC}"
}

# 即時監控日誌錯誤
log_monitor() {
    local source="${1:-all}"
    local interval="${2:-5}"
    
    echo -e "${CYAN}📡 即時監控日誌錯誤${NC}"
    echo -e "${CYAN}==================${NC}"
    echo -e "監控來源: ${BLUE}$source${NC}"
    echo -e "檢查間隔: ${BLUE}${interval}秒${NC}"
    echo -e "${YELLOW}按 Ctrl+C 停止監控${NC}"
    echo ""
    
    while true; do
        local timestamp=$(date '+%H:%M:%S')
        echo -ne "\r[$timestamp] 檢查中..."
        
        # 檢查最近的錯誤
        local errors=$("$LOG_TOOLS_DIR/log-analyzer.sh" "$source" --severity 3 --lines 50 --json 2>/dev/null | grep -o '"total_errors": [0-9]*' | cut -d: -f2 | tr -d ' ')
        
        if [ -n "$errors" ] && [ "$errors" -gt 0 ]; then
            echo -e "\n${RED}🚨 發現 $errors 個錯誤 [$timestamp]${NC}"
            "$LOG_TOOLS_DIR/log-analyzer.sh" "$source" --severity 4 --lines 20
            echo ""
        fi
        
        sleep "$interval"
    done
}

# 日誌清理建議
log_cleanup_advice() {
    echo -e "${CYAN}🧹 日誌清理建議${NC}"
    echo -e "${CYAN}===============${NC}"
    
    # 檢查容器日誌大小
    echo -e "${YELLOW}容器日誌大小:${NC}"
    
    local containers=("rosagv" "agvc_server" "postgres" "nginx")
    local total_size=0
    
    for container in "${containers[@]}"; do
        if docker ps -q -f name="$container" >/dev/null 2>&1; then
            local log_file=$(docker inspect "$container" --format='{{.LogPath}}' 2>/dev/null)
            if [ -n "$log_file" ] && [ -f "$log_file" ]; then
                local size=$(du -h "$log_file" 2>/dev/null | cut -f1)
                local size_bytes=$(du -b "$log_file" 2>/dev/null | cut -f1)
                total_size=$((total_size + size_bytes))
                
                echo -e "  $container: ${BLUE}$size${NC}"
                
                # 如果日誌過大，提供建議
                if [ "$size_bytes" -gt 104857600 ]; then  # 100MB
                    echo -e "    ${YELLOW}⚠️  建議清理 (>100MB)${NC}"
                fi
            fi
        else
            echo -e "  $container: ${RED}容器未運行${NC}"
        fi
    done
    
    local total_mb=$((total_size / 1048576))
    echo -e "\n總日誌大小: ${BLUE}${total_mb}MB${NC}"
    
    if [ "$total_mb" -gt 500 ]; then
        echo -e "\n${YELLOW}💡 清理建議:${NC}"
        echo -e "  • 使用 docker logs --tail=1000 限制日誌行數"
        echo -e "  • 設定 Docker 日誌輪轉: docker run --log-opt max-size=10m"
        echo -e "  • 定期清理舊日誌檔案"
    else
        echo -e "\n${GREEN}✅ 日誌大小正常${NC}"
    fi
}

# 組合診斷功能
log_full_diagnosis() {
    echo -e "${CYAN}🔍 完整日誌診斷${NC}"
    echo -e "${CYAN}===============${NC}"
    
    echo -e "\n${YELLOW}1. 快速錯誤掃描${NC}"
    log_quick_scan
    
    echo -e "\n${YELLOW}2. 統計分析${NC}"
    log_stats all
    
    echo -e "\n${YELLOW}3. 解決建議${NC}"
    log_suggestions all
    
    echo -e "\n${YELLOW}4. 清理建議${NC}"
    log_cleanup_advice
}

# ============================================================================
# 主程式邏輯
# ============================================================================

# 如果腳本被直接執行（而非被 source）
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    case "${1:-help}" in
        "help"|"-h"|"--help")
            show_log_tools_help
            ;;
        "analyze")
            shift
            log_analyze "$@"
            ;;
        "scan")
            log_quick_scan
            ;;
        "stats")
            shift
            log_stats "$@"
            ;;
        "monitor")
            shift
            log_monitor "$@"
            ;;
        "diagnosis")
            log_full_diagnosis
            ;;
        "cleanup")
            log_cleanup_advice
            ;;
        *)
            echo -e "${RED}錯誤: 未知命令 '$1'${NC}"
            echo ""
            show_log_tools_help
            exit 1
            ;;
    esac
else
    # 被 source 時顯示載入訊息
    echo -e "${GREEN}✅ RosAGV 日誌分析工具集已載入${NC}"
    echo -e "輸入 ${CYAN}show_log_tools_help${NC} 查看可用命令"
fi

# ============================================================================
# 導出所有函數供外部使用
# ============================================================================

# 導出主要函數
export -f show_log_tools_help
export -f show_log_tools_header

# 導出分析函數
export -f log_analyze
export -f log_quick_scan
export -f log_find_errors
export -f log_stats
export -f log_timeline
export -f log_suggestions
export -f log_export_report
export -f log_monitor
export -f log_cleanup_advice
export -f log_full_diagnosis