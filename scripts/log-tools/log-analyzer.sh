#!/bin/bash
# RosAGV 智能日誌分析工具
# 版本: 1.0
# 說明: 分析容器日誌，識別錯誤模式，提供解決建議和統計分析

# ============================================================================
# 初始化和設定
# ============================================================================

# 獲取腳本目錄和專案根目錄
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

# 預定義錯誤模式
declare -A ERROR_PATTERNS=(
    ["CRITICAL"]="CRITICAL|FATAL|critical|fatal"
    ["ERROR"]="ERROR|error|Error|\[ERROR\]"
    ["WARNING"]="WARNING|WARN|warning|warn|\[WARN\]"
    ["EXCEPTION"]="Exception|exception|Traceback|traceback"
    ["TIMEOUT"]="timeout|Timeout|TIMEOUT|timed out"
    ["CONNECTION"]="connection refused|Connection refused|connection failed|Connection failed"
    ["MEMORY"]="out of memory|OutOfMemoryError|memory leak|Memory"
    ["DOCKER"]="docker: Error|Docker daemon|container.*failed"
    ["ROS"]="ros2.*error|ROS.*ERROR|rmw.*error"
    ["ZENOH"]="zenoh.*error|Zenoh.*failed"
    ["SQL"]="SQL.*error|database.*error|PostgreSQL.*error"
    ["PERMISSION"]="permission denied|Permission denied|access denied"
)

# 錯誤嚴重程度分級
declare -A ERROR_SEVERITY=(
    ["CRITICAL"]=5
    ["ERROR"]=4
    ["EXCEPTION"]=4
    ["TIMEOUT"]=3
    ["CONNECTION"]=3
    ["MEMORY"]=4
    ["DOCKER"]=3
    ["ROS"]=3
    ["ZENOH"]=3
    ["SQL"]=3
    ["PERMISSION"]=2
    ["WARNING"]=1
)

# 解決建議
declare -A ERROR_SOLUTIONS=(
    ["CRITICAL"]="立即檢查系統狀態，可能需要重啟服務"
    ["ERROR"]="檢查相關配置和服務狀態"
    ["EXCEPTION"]="檢查代碼邏輯和輸入參數"
    ["TIMEOUT"]="檢查網路連接和服務響應時間"
    ["CONNECTION"]="檢查目標服務是否運行，網路是否通暢"
    ["MEMORY"]="檢查記憶體使用情況，考慮增加記憶體或優化程式"
    ["DOCKER"]="檢查 Docker 服務狀態和容器配置"
    ["ROS"]="檢查 ROS 2 環境和節點狀態"
    ["ZENOH"]="檢查 Zenoh Router 配置和網路連接"
    ["SQL"]="檢查資料庫連接和 SQL 語法"
    ["PERMISSION"]="檢查檔案權限和用戶權限"
    ["WARNING"]="注意相關警告，可能影響系統穩定性"
)

# ============================================================================
# 輔助函數
# ============================================================================

show_header() {
    echo -e "${CYAN}📊 RosAGV 智能日誌分析工具${NC}"
    echo -e "${CYAN}=============================${NC}"
    echo ""
}

show_help() {
    show_header
    echo -e "${YELLOW}用法:${NC}"
    echo "  $(basename $0) [來源] [選項]"
    echo ""
    echo -e "${YELLOW}日誌來源:${NC}"
    echo -e "  ${GREEN}agv${NC}           - AGV 容器日誌"
    echo -e "  ${GREEN}agvc${NC}          - AGVC 容器日誌"
    echo -e "  ${GREEN}postgres${NC}      - PostgreSQL 日誌"
    echo -e "  ${GREEN}nginx${NC}         - Nginx 日誌"
    echo -e "  ${GREEN}all${NC}           - 所有容器日誌 (預設)"
    echo -e "  ${GREEN}<檔案路徑>${NC}    - 指定日誌檔案"
    echo ""
    echo -e "${YELLOW}選項:${NC}"
    echo -e "  ${GREEN}--lines <數量>${NC}   - 分析最近 N 行日誌 (預設: 1000)"
    echo -e "  ${GREEN}--severity <級別>${NC} - 只顯示指定嚴重程度以上的錯誤"
    echo -e "  ${GREEN}--pattern <模式>${NC}  - 自定義錯誤模式"
    echo -e "  ${GREEN}--output <檔案>${NC}   - 將分析結果輸出到檔案"
    echo -e "  ${GREEN}--json${NC}          - 以 JSON 格式輸出"
    echo -e "  ${GREEN}--stats${NC}         - 顯示統計資訊"
    echo -e "  ${GREEN}--timeline${NC}      - 顯示錯誤時間軸"
    echo -e "  ${GREEN}--suggestions${NC}   - 顯示解決建議"
    echo ""
    echo -e "${YELLOW}嚴重程度級別:${NC}"
    echo -e "  5 - CRITICAL (嚴重)"
    echo -e "  4 - ERROR/EXCEPTION (錯誤)"
    echo -e "  3 - TIMEOUT/CONNECTION (連接問題)"
    echo -e "  2 - PERMISSION (權限問題)"
    echo -e "  1 - WARNING (警告)"
    echo ""
    echo -e "${YELLOW}範例:${NC}"
    echo "  $(basename $0) agv --lines 500           # 分析 AGV 最近 500 行日誌"
    echo "  $(basename $0) all --severity 3          # 分析所有容器嚴重程度 3 以上錯誤"
    echo "  $(basename $0) /var/log/app.log --stats  # 分析指定檔案並顯示統計"
    echo "  $(basename $0) agvc --timeline           # 分析 AGVC 日誌並顯示時間軸"
}

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

get_severity_color() {
    local severity="$1"
    case "$severity" in
        5) echo "$RED" ;;
        4) echo "$PURPLE" ;;
        3) echo "$YELLOW" ;;
        2) echo "$BLUE" ;;
        1) echo "$CYAN" ;;
        *) echo "$NC" ;;
    esac
}

get_severity_icon() {
    local severity="$1"
    case "$severity" in
        5) echo "🔥" ;;
        4) echo "❌" ;;
        3) echo "⚠️" ;;
        2) echo "🚫" ;;
        1) echo "💡" ;;
        *) echo "📝" ;;
    esac
}

# ============================================================================
# 日誌來源處理
# ============================================================================

get_container_logs() {
    local container="$1"
    local lines="$2"
    
    case "$container" in
        "agv")
            if docker ps -q -f name=rosagv >/dev/null 2>&1; then
                docker logs --tail="$lines" rosagv 2>&1
            else
                echo "AGV 容器未運行" >&2
                return 1
            fi
            ;;
        "agvc")
            if docker ps -q -f name=agvc_server >/dev/null 2>&1; then
                docker logs --tail="$lines" agvc_server 2>&1
            else
                echo "AGVC 容器未運行" >&2
                return 1
            fi
            ;;
        "postgres")
            if docker ps -q -f name=postgres >/dev/null 2>&1; then
                docker logs --tail="$lines" postgres 2>&1
            else
                echo "PostgreSQL 容器未運行" >&2
                return 1
            fi
            ;;
        "nginx")
            if docker ps -q -f name=nginx >/dev/null 2>&1; then
                docker logs --tail="$lines" nginx 2>&1
            else
                echo "Nginx 容器未運行" >&2
                return 1
            fi
            ;;
        "all")
            {
                echo "=== AGV 容器日誌 ==="
                get_container_logs "agv" "$lines" 2>/dev/null || echo "AGV 容器日誌無法取得"
                echo ""
                echo "=== AGVC 容器日誌 ==="
                get_container_logs "agvc" "$lines" 2>/dev/null || echo "AGVC 容器日誌無法取得"
                echo ""
                echo "=== PostgreSQL 容器日誌 ==="
                get_container_logs "postgres" "$lines" 2>/dev/null || echo "PostgreSQL 容器日誌無法取得"
                echo ""
                echo "=== Nginx 容器日誌 ==="
                get_container_logs "nginx" "$lines" 2>/dev/null || echo "Nginx 容器日誌無法取得"
            }
            ;;
        *)
            echo "未知容器: $container" >&2
            return 1
            ;;
    esac
}

get_file_logs() {
    local file_path="$1"
    local lines="$2"
    
    if [ ! -f "$file_path" ]; then
        echo "檔案不存在: $file_path" >&2
        return 1
    fi
    
    if [ ! -r "$file_path" ]; then
        echo "檔案無法讀取: $file_path" >&2
        return 1
    fi
    
    tail -n "$lines" "$file_path"
}

# ============================================================================
# 錯誤分析函數
# ============================================================================

analyze_errors() {
    local log_content="$1"
    local min_severity="$2"
    local custom_pattern="$3"
    
    declare -A error_counts
    declare -A error_lines
    local total_errors=0
    local line_num=0
    
    # 如果有自定義模式，添加到檢查列表
    if [ -n "$custom_pattern" ]; then
        ERROR_PATTERNS["CUSTOM"]="$custom_pattern"
        ERROR_SEVERITY["CUSTOM"]=3
        ERROR_SOLUTIONS["CUSTOM"]="根據自定義模式檢查相關問題"
    fi
    
    while IFS= read -r line; do
        ((line_num++))
        
        for pattern_name in "${!ERROR_PATTERNS[@]}"; do
            local pattern="${ERROR_PATTERNS[$pattern_name]}"
            local severity="${ERROR_SEVERITY[$pattern_name]}"
            
            # 檢查嚴重程度過濾
            if [ "$severity" -lt "$min_severity" ]; then
                continue
            fi
            
            if echo "$line" | grep -qiE "$pattern"; then
                error_counts["$pattern_name"]=$((${error_counts["$pattern_name"]:-0} + 1))
                
                # 保存錯誤行（最多保存 5 行）
                if [ ${error_counts["$pattern_name"]} -le 5 ]; then
                    error_lines["$pattern_name"]+="[$line_num] $line"$'\n'
                fi
                
                ((total_errors++))
                break  # 一行只匹配第一個模式
            fi
        done
    done <<< "$log_content"
    
    # 顯示分析結果
    show_analysis_results error_counts error_lines "$total_errors"
}

show_analysis_results() {
    local -n counts_ref=$1
    local -n lines_ref=$2
    local total="$3"
    
    echo -e "${CYAN}📊 錯誤分析結果${NC}"
    echo -e "${CYAN}===============${NC}"
    
    if [ "$total" -eq 0 ]; then
        print_success "未發現匹配的錯誤模式"
        return
    fi
    
    echo -e "總錯誤數: ${RED}$total${NC}"
    echo ""
    
    # 按嚴重程度排序顯示
    for severity in 5 4 3 2 1; do
        local has_errors=false
        
        for pattern_name in "${!counts_ref[@]}"; do
            local pattern_severity="${ERROR_SEVERITY[$pattern_name]}"
            if [ "$pattern_severity" -eq "$severity" ]; then
                if [ ! "$has_errors" = "true" ]; then
                    echo -e "${YELLOW}嚴重程度 $severity:${NC}"
                    has_errors=true
                fi
                
                local count="${counts_ref[$pattern_name]}"
                local color=$(get_severity_color "$severity")
                local icon=$(get_severity_icon "$severity")
                
                echo -e "  $icon ${color}$pattern_name${NC}: $count 次"
                
                # 顯示部分錯誤行
                if [ -n "${lines_ref[$pattern_name]}" ]; then
                    echo -e "${BLUE}    範例錯誤:${NC}"
                    echo "${lines_ref[$pattern_name]}" | head -3 | sed 's/^/      /'
                fi
                echo ""
            fi
        done
    done
}

# ============================================================================
# 統計分析
# ============================================================================

show_statistics() {
    local log_content="$1"
    
    echo -e "${CYAN}📈 日誌統計分析${NC}"
    echo -e "${CYAN}===============${NC}"
    
    local total_lines=$(echo "$log_content" | wc -l)
    local error_lines=$(echo "$log_content" | grep -ciE "${ERROR_PATTERNS["ERROR"]}")
    local warning_lines=$(echo "$log_content" | grep -ciE "${ERROR_PATTERNS["WARNING"]}")
    local critical_lines=$(echo "$log_content" | grep -ciE "${ERROR_PATTERNS["CRITICAL"]}")
    
    echo -e "總行數: ${BLUE}$total_lines${NC}"
    echo -e "錯誤行數: ${RED}$error_lines${NC} ($(( error_lines * 100 / (total_lines + 1) ))%)"
    echo -e "警告行數: ${YELLOW}$warning_lines${NC} ($(( warning_lines * 100 / (total_lines + 1) ))%)"
    echo -e "嚴重錯誤: ${PURPLE}$critical_lines${NC}"
    echo ""
    
    # 時間分布分析（如果日誌包含時間戳）
    local time_pattern="[0-9]{4}-[0-9]{2}-[0-9]{2}|[0-9]{2}:[0-9]{2}:[0-9]{2}"
    if echo "$log_content" | grep -qE "$time_pattern"; then
        echo -e "${YELLOW}時間分布分析:${NC}"
        
        # 按小時統計錯誤
        echo "$log_content" | grep -iE "${ERROR_PATTERNS["ERROR"]}" | \
        grep -oE "[0-9]{2}:[0-9]{2}:[0-9]{2}" | \
        cut -d: -f1 | sort | uniq -c | sort -nr | head -5 | \
        while read count hour; do
            echo -e "  ${hour}:xx 時段: ${RED}$count${NC} 個錯誤"
        done
        echo ""
    fi
    
    # 最常見的錯誤關鍵字
    echo -e "${YELLOW}最常見錯誤關鍵字:${NC}"
    echo "$log_content" | grep -iE "${ERROR_PATTERNS["ERROR"]}" | \
    tr ' ' '\n' | tr '[:upper:]' '[:lower:]' | \
    grep -E '^[a-z]{3,}$' | sort | uniq -c | sort -nr | head -5 | \
    while read count word; do
        echo -e "  ${word}: ${RED}$count${NC} 次"
    done
}

# ============================================================================
# 時間軸分析
# ============================================================================

show_timeline() {
    local log_content="$1"
    
    echo -e "${CYAN}⏰ 錯誤時間軸${NC}"
    echo -e "${CYAN}============${NC}"
    
    local time_pattern="[0-9]{4}-[0-9]{2}-[0-9]{2}.*[0-9]{2}:[0-9]{2}:[0-9]{2}"
    
    echo "$log_content" | while IFS= read -r line; do
        for pattern_name in "${!ERROR_PATTERNS[@]}"; do
            local pattern="${ERROR_PATTERNS[$pattern_name]}"
            local severity="${ERROR_SEVERITY[$pattern_name]}"
            
            if echo "$line" | grep -qiE "$pattern"; then
                local timestamp=$(echo "$line" | grep -oE "$time_pattern" | head -1)
                local color=$(get_severity_color "$severity")
                local icon=$(get_severity_icon "$severity")
                
                if [ -n "$timestamp" ]; then
                    echo -e "$icon ${color}[$timestamp]${NC} $pattern_name: $(echo "$line" | cut -c1-80)..."
                else
                    echo -e "$icon ${color}[NO_TIME]${NC} $pattern_name: $(echo "$line" | cut -c1-80)..."
                fi
                break
            fi
        done
    done | tail -20
}

# ============================================================================
# 解決建議
# ============================================================================

show_suggestions() {
    local log_content="$1"
    
    echo -e "${CYAN}💡 解決建議${NC}"
    echo -e "${CYAN}============${NC}"
    
    declare -A found_patterns
    
    # 檢查哪些錯誤模式被發現
    for pattern_name in "${!ERROR_PATTERNS[@]}"; do
        local pattern="${ERROR_PATTERNS[$pattern_name]}"
        if echo "$log_content" | grep -qiE "$pattern"; then
            found_patterns["$pattern_name"]=1
        fi
    done
    
    if [ ${#found_patterns[@]} -eq 0 ]; then
        print_info "未發現需要建議的錯誤模式"
        return
    fi
    
    # 按嚴重程度顯示建議
    for severity in 5 4 3 2 1; do
        for pattern_name in "${!found_patterns[@]}"; do
            local pattern_severity="${ERROR_SEVERITY[$pattern_name]}"
            if [ "$pattern_severity" -eq "$severity" ]; then
                local icon=$(get_severity_icon "$severity")
                local color=$(get_severity_color "$severity")
                local solution="${ERROR_SOLUTIONS[$pattern_name]}"
                
                echo -e "$icon ${color}$pattern_name${NC}:"
                echo -e "   ${solution}"
                echo ""
            fi
        done
    done
}

# ============================================================================
# JSON 輸出
# ============================================================================

output_json() {
    local log_content="$1"
    local min_severity="$2"
    
    echo "{"
    echo "  \"timestamp\": \"$(date -u +%Y-%m-%dT%H:%M:%SZ)\","
    echo "  \"analysis_results\": {"
    
    # 分析錯誤
    declare -A error_counts
    local total_errors=0
    
    while IFS= read -r line; do
        for pattern_name in "${!ERROR_PATTERNS[@]}"; do
            local pattern="${ERROR_PATTERNS[$pattern_name]}"
            local severity="${ERROR_SEVERITY[$pattern_name]}"
            
            if [ "$severity" -ge "$min_severity" ]; then
                if echo "$line" | grep -qiE "$pattern"; then
                    error_counts["$pattern_name"]=$((${error_counts["$pattern_name"]:-0} + 1))
                    ((total_errors++))
                    break
                fi
            fi
        done
    done <<< "$log_content"
    
    echo "    \"total_errors\": $total_errors,"
    echo "    \"error_patterns\": {"
    
    local first=true
    for pattern_name in "${!error_counts[@]}"; do
        if [ "$first" = "true" ]; then
            first=false
        else
            echo ","
        fi
        local count="${error_counts[$pattern_name]}"
        local severity="${ERROR_SEVERITY[$pattern_name]}"
        echo -n "      \"$pattern_name\": {\"count\": $count, \"severity\": $severity}"
    done
    
    echo ""
    echo "    }"
    echo "  }"
    echo "}"
}

# ============================================================================
# 主程式邏輯
# ============================================================================

main() {
    local source="all"
    local lines=1000
    local min_severity=1
    local custom_pattern=""
    local output_file=""
    local json_output=false
    local show_stats=false
    local show_timeline=false
    local show_suggestions=false
    
    # 解析參數
    while [[ $# -gt 0 ]]; do
        case $1 in
            agv|agvc|postgres|nginx|all)
                source="$1"
                shift
                ;;
            --lines)
                lines="$2"
                shift 2
                ;;
            --severity)
                min_severity="$2"
                shift 2
                ;;
            --pattern)
                custom_pattern="$2"
                shift 2
                ;;
            --output)
                output_file="$2"
                shift 2
                ;;
            --json)
                json_output=true
                shift
                ;;
            --stats)
                show_stats=true
                shift
                ;;
            --timeline)
                show_timeline=true
                shift
                ;;
            --suggestions)
                show_suggestions=true
                shift
                ;;
            -h|--help|help)
                show_help
                exit 0
                ;;
            *)
                # 檢查是否為檔案路徑
                if [ -f "$1" ]; then
                    source="$1"
                else
                    print_error "未知選項或檔案不存在: $1"
                    show_help
                    exit 1
                fi
                shift
                ;;
        esac
    done
    
    # 獲取日誌內容
    local log_content=""
    
    if [ -f "$source" ]; then
        log_content=$(get_file_logs "$source" "$lines")
    else
        log_content=$(get_container_logs "$source" "$lines")
    fi
    
    if [ $? -ne 0 ] || [ -z "$log_content" ]; then
        print_error "無法獲取日誌內容"
        exit 1
    fi
    
    # 執行分析
    {
        if [ "$json_output" = "true" ]; then
            output_json "$log_content" "$min_severity"
        else
            show_header
            echo -e "分析來源: ${BLUE}$source${NC}"
            echo -e "分析行數: ${BLUE}$lines${NC}"
            echo -e "最低嚴重程度: ${BLUE}$min_severity${NC}"
            echo ""
            
            analyze_errors "$log_content" "$min_severity" "$custom_pattern"
            
            if [ "$show_stats" = "true" ]; then
                echo ""
                show_statistics "$log_content"
            fi
            
            if [ "$show_timeline" = "true" ]; then
                echo ""
                show_timeline "$log_content"
            fi
            
            if [ "$show_suggestions" = "true" ]; then
                echo ""
                show_suggestions "$log_content"
            fi
        fi
    } | if [ -n "$output_file" ]; then
        tee "$output_file"
    else
        cat
    fi
}

# 如果腳本被直接執行（而非被 source）
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi