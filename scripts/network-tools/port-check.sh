#!/bin/bash
# RosAGV 端口連接檢查工具
# 版本: 1.0
# 說明: 檢查系統關鍵端口連接狀況，包括 Zenoh、Web API、資料庫等服務端口

set -e  # 遇到錯誤立即退出

# ============================================================================
# 常數定義
# ============================================================================

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

# 路徑定義
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# 系統關鍵端口定義
declare -A SYSTEM_PORTS=(
    ["zenoh"]="7447"
    ["web_api"]="8000"
    ["agvcui"]="8001"
    ["opui"]="8002"
    ["postgres"]="5432"
    ["nginx"]="80"
    ["ssh"]="2200"
)

# 端口類型分類
declare -A PORT_CATEGORIES=(
    ["zenoh"]="通訊"
    ["web_api"]="Web服務"
    ["agvcui"]="Web服務"
    ["opui"]="Web服務"
    ["postgres"]="資料庫"
    ["nginx"]="代理"
    ["ssh"]="管理"
)

# 預設設定
DEFAULT_TIMEOUT=3
DEFAULT_HOSTS=("localhost" "127.0.0.1")

# ============================================================================
# 工具函數
# ============================================================================

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

show_header() {
    echo -e "${CYAN}🔌 RosAGV 端口連接檢查工具${NC}"
    echo -e "${CYAN}===============================${NC}"
    echo ""
}

show_usage() {
    echo "使用方式:"
    echo "  $0 [action] [options]"
    echo ""
    echo "可用動作:"
    echo "  all                   # 檢查所有系統端口 (預設)"
    echo "  system                # 檢查系統關鍵端口"
    echo "  web                   # 檢查 Web 服務端口"
    echo "  database              # 檢查資料庫端口"
    echo "  communication         # 檢查通訊端口"
    echo "  custom                # 檢查自定義端口"
    echo "  listening             # 檢查本機監聽端口"
    echo "  conflicts             # 檢查端口衝突"
    echo "  scan                  # 掃描端口範圍"
    echo "  -h, --help           # 顯示此幫助資訊"
    echo ""
    echo "選項:"
    echo "  --host <HOST>        # 指定要檢查的主機 (預設: localhost)"
    echo "  --port <PORT>        # 指定單一端口 (用於 custom 動作)"
    echo "  --ports <PORTS>      # 指定多個端口，逗號分隔"
    echo "  --range <START-END>  # 指定端口範圍 (用於 scan 動作)"
    echo "  --timeout <SEC>      # 設定連接超時時間 (預設: 3秒)"
    echo "  --verbose            # 顯示詳細輸出"
    echo "  --json               # 以 JSON 格式輸出結果"
    echo ""
    echo "範例:"
    echo "  $0                                    # 檢查所有系統端口"
    echo "  $0 web                               # 檢查 Web 服務端口"
    echo "  $0 custom --port 3306                # 檢查自定義端口"
    echo "  $0 custom --ports 3306,6379,9200     # 檢查多個端口"
    echo "  $0 scan --range 8000-8010            # 掃描端口範圍"
    echo "  $0 all --host 192.168.100.100        # 檢查遠端主機"
    echo "  $0 conflicts                         # 檢查端口衝突"
}

# ============================================================================
# 核心檢查函數
# ============================================================================

check_dependencies() {
    local missing_deps=()
    
    if ! command -v timeout &> /dev/null; then
        missing_deps+=("timeout")
    fi
    
    # 檢查網路工具
    local has_network_tool=false
    if command -v nc &> /dev/null || command -v ncat &> /dev/null || command -v telnet &> /dev/null; then
        has_network_tool=true
    fi
    
    if [ "$has_network_tool" = false ]; then
        missing_deps+=("netcat 或 telnet")
    fi
    
    if [ ${#missing_deps[@]} -gt 0 ]; then
        log_error "缺少必要工具: ${missing_deps[*]}"
        log_info "請安裝: sudo apt-get install netcat-openbsd coreutils"
        return 1
    fi
    
    return 0
}

test_port_connection() {
    local host="$1"
    local port="$2"
    local timeout="${3:-$DEFAULT_TIMEOUT}"
    local method="${4:-tcp}"
    
    # 使用 /dev/tcp 測試 (最快的方法)
    if timeout "$timeout" bash -c "echo > /dev/$method/$host/$port" 2>/dev/null; then
        return 0
    fi
    
    # 備用方法：使用 nc
    if command -v nc &> /dev/null; then
        if timeout "$timeout" nc -z "$host" "$port" 2>/dev/null; then
            return 0
        fi
    fi
    
    # 備用方法：使用 ncat
    if command -v ncat &> /dev/null; then
        if timeout "$timeout" ncat -z "$host" "$port" 2>/dev/null; then
            return 0
        fi
    fi
    
    return 1
}

get_port_info() {
    local port="$1"
    
    # 檢查端口是否在監聽
    local listening_info=""
    if command -v ss &> /dev/null; then
        listening_info=$(ss -tlnp | grep ":$port " || true)
    elif command -v netstat &> /dev/null; then
        listening_info=$(netstat -tlnp 2>/dev/null | grep ":$port " || true)
    fi
    
    if [ -n "$listening_info" ]; then
        echo "$listening_info"
    else
        echo "未監聽"
    fi
}

get_process_using_port() {
    local port="$1"
    
    if command -v lsof &> /dev/null; then
        lsof -ti:$port 2>/dev/null | head -1
    elif command -v ss &> /dev/null; then
        ss -tlnp | grep ":$port " | grep -o 'pid=[0-9]*' | cut -d= -f2 | head -1
    elif command -v netstat &> /dev/null; then
        netstat -tlnp 2>/dev/null | grep ":$port " | awk '{print $7}' | cut -d/ -f1
    fi
}

measure_response_time() {
    local host="$1"
    local port="$2"
    local timeout="${3:-$DEFAULT_TIMEOUT}"
    
    local start_time=$(date +%s%N)
    if test_port_connection "$host" "$port" "$timeout"; then
        local end_time=$(date +%s%N)
        local response_time=$(( (end_time - start_time) / 1000000 ))  # 轉換為毫秒
        echo "$response_time"
    else
        echo "-1"
    fi
}

# ============================================================================
# 檢查功能實現
# ============================================================================

check_single_port() {
    local host="$1"
    local port="$2"
    local service_name="$3"
    local timeout="$4"
    local verbose="$5"
    local json_output="$6"
    
    local status="closed"
    local response_time="-1"
    local port_info="未監聽"
    local process_info=""
    
    # 測試連接
    if test_port_connection "$host" "$port" "$timeout"; then
        status="open"
        if [ "$verbose" = "true" ]; then
            response_time=$(measure_response_time "$host" "$port" "$timeout")
        fi
    fi
    
    # 獲取端口詳細資訊
    if [ "$host" = "localhost" ] || [ "$host" = "127.0.0.1" ]; then
        port_info=$(get_port_info "$port")
        if [ "$status" = "open" ]; then
            local pid=$(get_process_using_port "$port")
            if [ -n "$pid" ] && [ "$pid" != "-" ]; then
                process_info=$(ps -p "$pid" -o comm= 2>/dev/null || echo "未知")
            fi
        fi
    fi
    
    # 輸出結果
    if [ "$json_output" = "true" ]; then
        echo "{\"host\":\"$host\",\"port\":$port,\"service\":\"$service_name\",\"status\":\"$status\",\"response_time\":$response_time,\"process\":\"$process_info\"}"
    else
        local status_icon="❌"
        local status_color="$RED"
        
        if [ "$status" = "open" ]; then
            status_icon="✅"
            status_color="$GREEN"
        fi
        
        echo -e "${PURPLE}$service_name${NC} (${host}:${port})"
        echo -e "  狀態: ${status_color}${status_icon} $status${NC}"
        
        if [ "$verbose" = "true" ] && [ "$response_time" != "-1" ]; then
            echo -e "  響應時間: ${BLUE}${response_time}ms${NC}"
        fi
        
        if [ -n "$process_info" ] && [ "$process_info" != "未知" ]; then
            echo -e "  進程: ${BLUE}$process_info${NC}"
        fi
        
        if [ "$port_info" != "未監聽" ] && [ "$verbose" = "true" ]; then
            echo -e "  詳細資訊: ${BLUE}$port_info${NC}"
        fi
        
        echo ""
    fi
}

check_system_ports() {
    local host="$1"
    local timeout="$2"
    local verbose="$3"
    local json_output="$4"
    
    if [ "$json_output" = "true" ]; then
        echo "["
        local first=true
    else
        log_info "檢查系統關鍵端口..."
        echo ""
    fi
    
    for service in "${!SYSTEM_PORTS[@]}"; do
        local port="${SYSTEM_PORTS[$service]}"
        local category="${PORT_CATEGORIES[$service]}"
        local display_name="$service ($category)"
        
        if [ "$json_output" = "true" ]; then
            if [ "$first" = false ]; then
                echo ","
            fi
            check_single_port "$host" "$port" "$service" "$timeout" "$verbose" "$json_output"
            first=false
        else
            check_single_port "$host" "$port" "$display_name" "$timeout" "$verbose" "$json_output"
        fi
    done
    
    if [ "$json_output" = "true" ]; then
        echo "]"
    fi
}

check_ports_by_category() {
    local category="$1"
    local host="$2"
    local timeout="$3"
    local verbose="$4"
    local json_output="$5"
    
    local found_ports=()
    
    # 根據類別篩選端口
    case $category in
        "web")
            for service in "${!PORT_CATEGORIES[@]}"; do
                if [ "${PORT_CATEGORIES[$service]}" = "Web服務" ]; then
                    found_ports+=("$service:${SYSTEM_PORTS[$service]}")
                fi
            done
            ;;
        "database")
            for service in "${!PORT_CATEGORIES[@]}"; do
                if [ "${PORT_CATEGORIES[$service]}" = "資料庫" ]; then
                    found_ports+=("$service:${SYSTEM_PORTS[$service]}")
                fi
            done
            ;;
        "communication")
            for service in "${!PORT_CATEGORIES[@]}"; do
                if [ "${PORT_CATEGORIES[$service]}" = "通訊" ]; then
                    found_ports+=("$service:${SYSTEM_PORTS[$service]}")
                fi
            done
            ;;
    esac
    
    if [ ${#found_ports[@]} -eq 0 ]; then
        log_warning "未找到類別 '$category' 的端口"
        return 1
    fi
    
    if [ "$json_output" = "true" ]; then
        echo "["
        local first=true
    else
        log_info "檢查 $category 類別端口..."
        echo ""
    fi
    
    for port_info in "${found_ports[@]}"; do
        local service="${port_info%%:*}"
        local port="${port_info##*:}"
        local display_name="$service (${PORT_CATEGORIES[$service]})"
        
        if [ "$json_output" = "true" ]; then
            if [ "$first" = false ]; then
                echo ","
            fi
            check_single_port "$host" "$port" "$service" "$timeout" "$verbose" "$json_output"
            first=false
        else
            check_single_port "$host" "$port" "$display_name" "$timeout" "$verbose" "$json_output"
        fi
    done
    
    if [ "$json_output" = "true" ]; then
        echo "]"
    fi
}

check_custom_ports() {
    local host="$1"
    local ports_list="$2"
    local timeout="$3"
    local verbose="$4"
    local json_output="$5"
    
    # 解析端口列表
    IFS=',' read -ra PORTS <<< "$ports_list"
    
    if [ "$json_output" = "true" ]; then
        echo "["
        local first=true
    else
        log_info "檢查自定義端口..."
        echo ""
    fi
    
    for port in "${PORTS[@]}"; do
        # 移除空格
        port=$(echo "$port" | tr -d ' ')
        
        # 驗證端口號
        if ! [[ "$port" =~ ^[0-9]+$ ]] || [ "$port" -lt 1 ] || [ "$port" -gt 65535 ]; then
            if [ "$json_output" != "true" ]; then
                log_error "無效的端口號: $port"
            fi
            continue
        fi
        
        if [ "$json_output" = "true" ]; then
            if [ "$first" = false ]; then
                echo ","
            fi
            check_single_port "$host" "$port" "自定義端口" "$timeout" "$verbose" "$json_output"
            first=false
        else
            check_single_port "$host" "$port" "自定義端口 $port" "$timeout" "$verbose" "$json_output"
        fi
    done
    
    if [ "$json_output" = "true" ]; then
        echo "]"
    fi
}

scan_port_range() {
    local host="$1"
    local start_port="$2"
    local end_port="$3"
    local timeout="$4"
    local verbose="$5"
    local json_output="$6"
    
    # 驗證端口範圍
    if [ "$start_port" -gt "$end_port" ]; then
        log_error "起始端口不能大於結束端口"
        return 1
    fi
    
    if [ "$start_port" -lt 1 ] || [ "$end_port" -gt 65535 ]; then
        log_error "端口範圍必須在 1-65535 之間"
        return 1
    fi
    
    local range_size=$((end_port - start_port + 1))
    if [ "$range_size" -gt 1000 ]; then
        log_warning "端口範圍較大 ($range_size 個端口)，掃描可能需要較長時間"
        echo -n "繼續嗎？(y/N): "
        read -r confirmation
        if [[ ! "$confirmation" =~ ^[Yy]$ ]]; then
            log_info "掃描已取消"
            return 0
        fi
    fi
    
    if [ "$json_output" = "true" ]; then
        echo "["
        local first=true
    else
        log_info "掃描端口範圍 $start_port-$end_port..."
        echo ""
    fi
    
    local open_count=0
    
    for ((port=start_port; port<=end_port; port++)); do
        if test_port_connection "$host" "$port" "$timeout"; then
            open_count=$((open_count + 1))
            
            if [ "$json_output" = "true" ]; then
                if [ "$first" = false ]; then
                    echo ","
                fi
                check_single_port "$host" "$port" "掃描端口" "$timeout" "$verbose" "$json_output"
                first=false
            else
                check_single_port "$host" "$port" "開放端口 $port" "$timeout" "$verbose" "$json_output"
            fi
        elif [ "$verbose" = "true" ] && [ "$json_output" != "true" ]; then
            echo -e "${BLUE}端口 $port: 關閉${NC}"
        fi
        
        # 顯示進度 (每100個端口)
        if [ "$json_output" != "true" ] && [ $((port % 100)) -eq 0 ]; then
            local progress=$(( (port - start_port + 1) * 100 / range_size ))
            echo -e "${YELLOW}掃描進度: $progress% ($port/$end_port)${NC}"
        fi
    done
    
    if [ "$json_output" = "true" ]; then
        echo "]"
    else
        echo ""
        log_success "掃描完成！找到 $open_count 個開放端口"
    fi
}

check_listening_ports() {
    local verbose="$1"
    local json_output="$2"
    
    log_info "檢查本機監聽端口..."
    echo ""
    
    local listening_ports=""
    if command -v ss &> /dev/null; then
        listening_ports=$(ss -tlnp)
    elif command -v netstat &> /dev/null; then
        listening_ports=$(netstat -tlnp 2>/dev/null)
    else
        log_error "缺少 ss 或 netstat 工具，無法檢查監聽端口"
        return 1
    fi
    
    if [ "$json_output" = "true" ]; then
        echo "["
        echo "$listening_ports" | grep -E ":([0-9]+)\s" | awk '{print $4}' | cut -d: -f2 | sort -n | uniq | while read -r port; do
            if [ -n "$port" ]; then
                local process_info=$(get_process_using_port "$port")
                echo "{\"port\":$port,\"status\":\"listening\",\"process\":\"$process_info\"}"
            fi
        done | paste -sd "," -
        echo "]"
    else
        echo -e "${CYAN}本機監聽端口列表:${NC}"
        echo "$listening_ports" | grep -v "Active\|Proto" | while read -r line; do
            if [ -n "$line" ]; then
                local port=$(echo "$line" | awk '{print $4}' | cut -d: -f2)
                local process=$(echo "$line" | awk '{print $7}' | cut -d/ -f2)
                
                if [ -n "$port" ] && [ "$port" != "Port" ]; then
                    echo -e "  ${GREEN}端口 $port${NC} - ${BLUE}$process${NC}"
                    
                    if [ "$verbose" = "true" ]; then
                        echo "    $line"
                    fi
                fi
            fi
        done
    fi
}

check_port_conflicts() {
    local verbose="$1"
    local json_output="$2"
    
    log_info "檢查端口衝突..."
    echo ""
    
    # 檢查系統端口是否被其他進程佔用
    local conflicts_found=false
    
    if [ "$json_output" = "true" ]; then
        echo "["
        local first=true
    fi
    
    for service in "${!SYSTEM_PORTS[@]}"; do
        local port="${SYSTEM_PORTS[$service]}"
        local expected_process=""
        
        # 根據服務類型設定預期進程
        case $service in
            "zenoh") expected_process="zenoh" ;;
            "web_api"|"agvcui"|"opui") expected_process="python" ;;
            "postgres") expected_process="postgres" ;;
            "nginx") expected_process="nginx" ;;
            "ssh") expected_process="sshd" ;;
        esac
        
        local current_process=$(get_process_using_port "$port")
        if [ -n "$current_process" ]; then
            current_process=$(ps -p "$current_process" -o comm= 2>/dev/null || echo "未知")
        fi
        
        # 檢查是否為預期進程
        local is_conflict=false
        if [ -n "$current_process" ] && [ "$current_process" != "未知" ]; then
            if [ -n "$expected_process" ] && [[ "$current_process" != *"$expected_process"* ]]; then
                is_conflict=true
                conflicts_found=true
            fi
        fi
        
        if [ "$json_output" = "true" ]; then
            if [ "$first" = false ]; then
                echo ","
            fi
            echo "{\"service\":\"$service\",\"port\":$port,\"expected\":\"$expected_process\",\"current\":\"$current_process\",\"conflict\":$is_conflict}"
            first=false
        else
            if [ "$is_conflict" = true ]; then
                log_warning "端口衝突detected: 端口 $port ($service)"
                echo -e "  預期進程: ${BLUE}$expected_process${NC}"
                echo -e "  實際進程: ${RED}$current_process${NC}"
                echo ""
            elif [ "$verbose" = "true" ]; then
                log_success "端口 $port ($service) - 正常"
                if [ -n "$current_process" ]; then
                    echo -e "  進程: ${BLUE}$current_process${NC}"
                fi
                echo ""
            fi
        fi
    done
    
    if [ "$json_output" = "true" ]; then
        echo "]"
    else
        if [ "$conflicts_found" = false ]; then
            log_success "未發現端口衝突"
        fi
    fi
}

# ============================================================================
# 主程式
# ============================================================================

main() {
    # 檢查依賴
    if ! check_dependencies; then
        exit 1
    fi
    
    # 解析參數
    local action="all"
    local target_host="localhost"
    local custom_port=""
    local custom_ports=""
    local port_range=""
    local timeout="$DEFAULT_TIMEOUT"
    local verbose="false"
    local json_output="false"
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            all|system|web|database|communication|custom|listening|conflicts|scan)
                action="$1"
                shift
                ;;
            --host)
                target_host="$2"
                shift 2
                ;;
            --port)
                custom_port="$2"
                shift 2
                ;;
            --ports)
                custom_ports="$2"
                shift 2
                ;;
            --range)
                port_range="$2"
                shift 2
                ;;
            --timeout)
                timeout="$2"
                shift 2
                ;;
            --verbose)
                verbose="true"
                shift
                ;;
            --json)
                json_output="true"
                shift
                ;;
            -h|--help)
                show_header
                show_usage
                exit 0
                ;;
            *)
                log_error "未知參數: $1"
                show_usage
                exit 1
                ;;
        esac
    done
    
    # 處理自定義端口參數
    if [ "$action" = "custom" ]; then
        if [ -n "$custom_port" ]; then
            custom_ports="$custom_port"
        elif [ -z "$custom_ports" ]; then
            log_error "custom 動作需要 --port 或 --ports 參數"
            exit 1
        fi
    fi
    
    # 處理端口範圍掃描參數
    if [ "$action" = "scan" ]; then
        if [ -z "$port_range" ]; then
            log_error "scan 動作需要 --range 參數"
            exit 1
        fi
        
        if [[ ! "$port_range" =~ ^([0-9]+)-([0-9]+)$ ]]; then
            log_error "端口範圍格式錯誤，應為: START-END"
            exit 1
        fi
        
        local start_port="${BASH_REMATCH[1]}"
        local end_port="${BASH_REMATCH[2]}"
    fi
    
    # 顯示標題
    if [ "$json_output" != "true" ]; then
        show_header
    fi
    
    # 執行對應動作
    case $action in
        all|system)
            check_system_ports "$target_host" "$timeout" "$verbose" "$json_output"
            ;;
        web|database|communication)
            check_ports_by_category "$action" "$target_host" "$timeout" "$verbose" "$json_output"
            ;;
        custom)
            check_custom_ports "$target_host" "$custom_ports" "$timeout" "$verbose" "$json_output"
            ;;
        listening)
            check_listening_ports "$verbose" "$json_output"
            ;;
        conflicts)
            check_port_conflicts "$verbose" "$json_output"
            ;;
        scan)
            scan_port_range "$target_host" "$start_port" "$end_port" "$timeout" "$verbose" "$json_output"
            ;;
        *)
            log_error "未知動作: $action"
            show_usage
            exit 1
            ;;
    esac
}

# 如果直接執行此腳本
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi