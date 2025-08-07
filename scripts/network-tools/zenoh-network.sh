#!/bin/bash
# RosAGV Zenoh 網路診斷工具
# 版本: 1.0
# 說明: 專門診斷 Zenoh Router 連接性、通訊品質和性能的工具

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
ZENOH_CONFIG_FILE="$PROJECT_ROOT/app/routerconfig.json5"

# Zenoh 相關常數
DEFAULT_ZENOH_PORT=7447
ZENOH_TCP_TIMEOUT=3
ZENOH_UDP_TIMEOUT=1

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
    echo -e "${CYAN}🌐 RosAGV Zenoh 網路診斷工具${NC}"
    echo -e "${CYAN}================================${NC}"
    echo ""
}

show_usage() {
    echo "使用方式:"
    echo "  $0 [action] [options]"
    echo ""
    echo "可用動作:"
    echo "  connectivity          # Zenoh 連接性檢查 (預設)"
    echo "  endpoints             # 檢查所有 Zenoh 端點"
    echo "  performance           # 通訊性能測試"
    echo "  router-status         # Zenoh Router 狀態檢查"
    echo "  config-analysis       # 配置檔案分析"
    echo "  troubleshoot          # 故障排除診斷"
    echo "  full-check            # 完整診斷檢查"
    echo "  -h, --help           # 顯示此幫助資訊"
    echo ""
    echo "選項:"
    echo "  --host <IP>          # 指定要測試的主機 IP"
    echo "  --port <PORT>        # 指定要測試的端口 (預設: 7447)"
    echo "  --timeout <SEC>      # 設定連接超時時間 (預設: 3秒)"
    echo "  --verbose            # 顯示詳細輸出"
    echo ""
    echo "範例:"
    echo "  $0                           # 基本連接性檢查"
    echo "  $0 endpoints                 # 檢查所有端點"
    echo "  $0 connectivity --host 192.168.100.100"
    echo "  $0 performance --verbose     # 詳細性能測試"
    echo "  $0 full-check               # 完整系統診斷"
}

# ============================================================================
# 核心診斷函數
# ============================================================================

check_dependencies() {
    local missing_deps=()
    
    if ! command -v nc &> /dev/null && ! command -v ncat &> /dev/null; then
        missing_deps+=("netcat")
    fi
    
    if ! command -v timeout &> /dev/null; then
        missing_deps+=("timeout")
    fi
    
    if [ ${#missing_deps[@]} -gt 0 ]; then
        log_error "缺少必要工具: ${missing_deps[*]}"
        log_info "請安裝: sudo apt-get install netcat-openbsd coreutils"
        return 1
    fi
    
    return 0
}

parse_zenoh_config() {
    if [ ! -f "$ZENOH_CONFIG_FILE" ]; then
        log_warning "Zenoh 配置檔案不存在: $ZENOH_CONFIG_FILE"
        return 1
    fi
    
    # 使用 Python 解析 JSON5 配置
    python3 -c "
import json5
import sys

try:
    with open('$ZENOH_CONFIG_FILE', 'r') as f:
        config = json5.load(f)
    
    # 提取端點資訊
    endpoints = []
    if 'connect' in config and 'endpoints' in config['connect']:
        endpoints.extend(config['connect']['endpoints'])
    if 'listen' in config and 'endpoints' in config['listen']:
        endpoints.extend(config['listen']['endpoints'])
    
    for endpoint in endpoints:
        print(endpoint)
        
except Exception as e:
    print(f'解析配置檔案時發生錯誤: {e}', file=sys.stderr)
    sys.exit(1)
" 2>/dev/null || echo "tcp/0.0.0.0:7447"
}

test_tcp_connection() {
    local host="$1"
    local port="$2"
    local timeout="${3:-$ZENOH_TCP_TIMEOUT}"
    
    if timeout "$timeout" bash -c "echo > /dev/tcp/$host/$port" 2>/dev/null; then
        return 0
    else
        return 1
    fi
}

test_udp_connection() {
    local host="$1"
    local port="$2"
    local timeout="${3:-$ZENOH_UDP_TIMEOUT}"
    
    # UDP 連接測試比較複雜，使用 nc 或 ncat
    if command -v nc &> /dev/null; then
        timeout "$timeout" nc -u -z "$host" "$port" 2>/dev/null
    elif command -v ncat &> /dev/null; then
        timeout "$timeout" ncat -u -z "$host" "$port" 2>/dev/null
    else
        log_warning "無法測試 UDP 連接，缺少 netcat 工具"
        return 1
    fi
}

measure_latency() {
    local host="$1"
    local port="$2"
    local protocol="${3:-tcp}"
    
    if [ "$protocol" = "tcp" ]; then
        # 測量 TCP 連接延遲
        local start_time=$(date +%s%N)
        if test_tcp_connection "$host" "$port" 1; then
            local end_time=$(date +%s%N)
            local latency=$(( (end_time - start_time) / 1000000 ))  # 轉換為毫秒
            echo "$latency"
        else
            echo "-1"
        fi
    else
        echo "0"  # UDP 延遲測量較複雜，暫時返回 0
    fi
}

# ============================================================================
# 主要診斷功能
# ============================================================================

diagnose_connectivity() {
    local target_host="$1"
    local target_port="$2"
    local verbose="$3"
    
    log_info "開始 Zenoh 連接性診斷..."
    echo ""
    
    # 如果沒有指定主機，從配置檔案解析
    if [ -z "$target_host" ]; then
        log_info "從配置檔案解析 Zenoh 端點..."
        local endpoints=($(parse_zenoh_config))
        
        if [ ${#endpoints[@]} -eq 0 ]; then
            log_warning "未找到配置的端點，使用預設值"
            endpoints=("tcp/0.0.0.0:7447")
        fi
        
        # 測試每個端點
        for endpoint in "${endpoints[@]}"; do
            # 解析端點格式 (protocol/host:port)
            if [[ "$endpoint" =~ ^([^/]+)/([^:]+):([0-9]+)$ ]]; then
                local protocol="${BASH_REMATCH[1]}"
                local host="${BASH_REMATCH[2]}"
                local port="${BASH_REMATCH[3]}"
                
                # 0.0.0.0 替換為 localhost
                if [ "$host" = "0.0.0.0" ]; then
                    host="localhost"
                fi
                
                test_single_endpoint "$protocol" "$host" "$port" "$verbose"
            else
                log_warning "無法解析端點格式: $endpoint"
            fi
        done
    else
        # 測試指定的主機和端口
        local port="${target_port:-$DEFAULT_ZENOH_PORT}"
        test_single_endpoint "tcp" "$target_host" "$port" "$verbose"
    fi
}

test_single_endpoint() {
    local protocol="$1"
    local host="$2"
    local port="$3"
    local verbose="$4"
    
    echo -e "${PURPLE}測試端點: ${protocol}/${host}:${port}${NC}"
    
    # 連接性測試
    if [ "$protocol" = "tcp" ]; then
        if test_tcp_connection "$host" "$port"; then
            log_success "✅ TCP 連接成功"
            
            # 延遲測試
            if [ "$verbose" = "true" ]; then
                local latency=$(measure_latency "$host" "$port" "tcp")
                if [ "$latency" != "-1" ]; then
                    echo -e "   ${BLUE}延遲: ${latency}ms${NC}"
                fi
            fi
        else
            log_error "❌ TCP 連接失敗"
            
            # 提供診斷建議
            echo -e "   ${YELLOW}建議檢查:${NC}"
            echo "   - Zenoh Router 是否運行"
            echo "   - 防火牆設定"
            echo "   - 網路連通性"
        fi
    elif [ "$protocol" = "udp" ]; then
        if test_udp_connection "$host" "$port"; then
            log_success "✅ UDP 連接成功"
        else
            log_error "❌ UDP 連接失敗"
        fi
    else
        log_warning "不支援的協議: $protocol"
    fi
    
    echo ""
}

check_router_status() {
    log_info "檢查 Zenoh Router 狀態..."
    echo ""
    
    # 檢查進程
    if pgrep -f "zenoh" > /dev/null; then
        log_success "✅ 發現 Zenoh 相關進程"
        if command -v pgrep &> /dev/null; then
            echo -e "   ${BLUE}進程資訊:${NC}"
            pgrep -af "zenoh" | while read -r pid cmd; do
                echo "   PID: $pid - $cmd"
            done
        fi
    else
        log_warning "⚠️  未發現 Zenoh 相關進程"
        echo -e "   ${YELLOW}建議:${NC} 檢查 Zenoh Router 是否啟動"
    fi
    
    echo ""
    
    # 檢查端口監聽
    log_info "檢查端口監聽狀況..."
    if command -v ss &> /dev/null; then
        local listening_ports=$(ss -tlnp | grep :$DEFAULT_ZENOH_PORT || true)
        if [ -n "$listening_ports" ]; then
            log_success "✅ 端口 $DEFAULT_ZENOH_PORT 正在監聽"
            echo "$listening_ports"
        else
            log_warning "⚠️  端口 $DEFAULT_ZENOH_PORT 未在監聽"
        fi
    elif command -v netstat &> /dev/null; then
        local listening_ports=$(netstat -tlnp 2>/dev/null | grep :$DEFAULT_ZENOH_PORT || true)
        if [ -n "$listening_ports" ]; then
            log_success "✅ 端口 $DEFAULT_ZENOH_PORT 正在監聽"
            echo "$listening_ports"
        else
            log_warning "⚠️  端口 $DEFAULT_ZENOH_PORT 未在監聽"
        fi
    else
        log_warning "無法檢查端口狀態，缺少 ss 或 netstat 工具"
    fi
    
    echo ""
}

analyze_config() {
    log_info "分析 Zenoh 配置檔案..."
    echo ""
    
    if [ ! -f "$ZENOH_CONFIG_FILE" ]; then
        log_error "配置檔案不存在: $ZENOH_CONFIG_FILE"
        return 1
    fi
    
    # 顯示配置檔案基本資訊
    echo -e "${BLUE}配置檔案路徑:${NC} $ZENOH_CONFIG_FILE"
    echo -e "${BLUE}檔案大小:${NC} $(du -h "$ZENOH_CONFIG_FILE" | cut -f1)"
    echo -e "${BLUE}最後修改:${NC} $(stat -c %y "$ZENOH_CONFIG_FILE" 2>/dev/null || stat -f %Sm "$ZENOH_CONFIG_FILE" 2>/dev/null || echo "未知")"
    echo ""
    
    # 解析並顯示端點配置
    log_info "解析端點配置..."
    local endpoints=($(parse_zenoh_config))
    
    if [ ${#endpoints[@]} -gt 0 ]; then
        echo -e "${GREEN}找到 ${#endpoints[@]} 個端點:${NC}"
        for endpoint in "${endpoints[@]}"; do
            echo "  - $endpoint"
        done
    else
        log_warning "未找到配置的端點"
    fi
    
    echo ""
    
    # 配置檔案語法檢查
    log_info "檢查配置檔案語法..."
    if python3 -c "
import json5
try:
    with open('$ZENOH_CONFIG_FILE', 'r') as f:
        json5.load(f)
    print('✅ 配置檔案語法正確')
except Exception as e:
    print(f'❌ 配置檔案語法錯誤: {e}')
    exit(1)
" 2>/dev/null; then
        log_success "配置檔案語法驗證通過"
    else
        log_error "配置檔案語法驗證失敗"
        return 1
    fi
    
    echo ""
}

performance_test() {
    local verbose="$1"
    
    log_info "開始 Zenoh 性能測試..."
    echo ""
    
    # 從配置解析端點
    local endpoints=($(parse_zenoh_config))
    
    if [ ${#endpoints[@]} -eq 0 ]; then
        log_warning "未找到配置的端點，使用預設端點"
        endpoints=("tcp/localhost:7447")
    fi
    
    for endpoint in "${endpoints[@]}"; do
        if [[ "$endpoint" =~ ^([^/]+)/([^:]+):([0-9]+)$ ]]; then
            local protocol="${BASH_REMATCH[1]}"
            local host="${BASH_REMATCH[2]}"
            local port="${BASH_REMATCH[3]}"
            
            if [ "$host" = "0.0.0.0" ]; then
                host="localhost"
            fi
            
            echo -e "${PURPLE}性能測試: ${protocol}/${host}:${port}${NC}"
            
            if [ "$protocol" = "tcp" ]; then
                # 連接時間測試 (多次測量取平均)
                local total_latency=0
                local successful_tests=0
                local test_count=5
                
                for i in $(seq 1 $test_count); do
                    local latency=$(measure_latency "$host" "$port" "tcp")
                    if [ "$latency" != "-1" ]; then
                        total_latency=$((total_latency + latency))
                        successful_tests=$((successful_tests + 1))
                    fi
                    
                    if [ "$verbose" = "true" ]; then
                        echo "  測試 $i: ${latency}ms"
                    fi
                done
                
                if [ $successful_tests -gt 0 ]; then
                    local avg_latency=$((total_latency / successful_tests))
                    log_success "平均連接延遲: ${avg_latency}ms (${successful_tests}/${test_count} 成功)"
                    
                    # 性能評估
                    if [ $avg_latency -lt 10 ]; then
                        echo -e "   ${GREEN}性能評估: 優秀${NC}"
                    elif [ $avg_latency -lt 50 ]; then
                        echo -e "   ${BLUE}性能評估: 良好${NC}"
                    elif [ $avg_latency -lt 100 ]; then
                        echo -e "   ${YELLOW}性能評估: 一般${NC}"
                    else
                        echo -e "   ${RED}性能評估: 需要優化${NC}"
                    fi
                else
                    log_error "所有連接測試都失敗"
                fi
            fi
            
            echo ""
        fi
    done
}

troubleshoot_zenoh() {
    log_info "開始 Zenoh 故障排除診斷..."
    echo ""
    
    # 1. 基本檢查
    echo -e "${CYAN}=== 基本環境檢查 ===${NC}"
    
    # 檢查 Docker 容器
    if command -v docker &> /dev/null; then
        local agv_container=$(docker ps --filter "name=rosagv" --format "table {{.Names}}\t{{.Status}}" 2>/dev/null || true)
        local agvc_container=$(docker ps --filter "name=agvc_server" --format "table {{.Names}}\t{{.Status}}" 2>/dev/null || true)
        
        if [ -n "$agv_container" ]; then
            log_success "AGV 容器運行中"
            echo "$agv_container"
        else
            log_warning "AGV 容器未運行"
        fi
        
        if [ -n "$agvc_container" ]; then
            log_success "AGVC 容器運行中"
            echo "$agvc_container"
        else
            log_warning "AGVC 容器未運行"
        fi
    fi
    
    echo ""
    
    # 2. 網路連接檢查
    echo -e "${CYAN}=== 網路連接檢查 ===${NC}"
    diagnose_connectivity "" "" "false"
    
    # 3. 配置檔案檢查
    echo -e "${CYAN}=== 配置檔案檢查 ===${NC}"
    analyze_config
    
    # 4. 進程和端口檢查
    echo -e "${CYAN}=== 服務狀態檢查 ===${NC}"
    check_router_status
    
    # 5. 常見問題檢查
    echo -e "${CYAN}=== 常見問題檢查 ===${NC}"
    
    # 檢查防火牆
    if command -v ufw &> /dev/null; then
        local ufw_status=$(ufw status 2>/dev/null | head -1 || true)
        if [[ "$ufw_status" == *"active"* ]]; then
            log_warning "UFW 防火牆處於啟用狀態"
            echo "   建議檢查是否允許端口 $DEFAULT_ZENOH_PORT"
        fi
    fi
    
    # 檢查系統資源
    if command -v free &> /dev/null; then
        local memory_usage=$(free | awk 'NR==2{printf "%.2f%%", $3*100/$2}')
        echo -e "${BLUE}記憶體使用率:${NC} $memory_usage"
    fi
    
    if command -v df &> /dev/null; then
        local disk_usage=$(df / | awk 'NR==2{print $5}')
        echo -e "${BLUE}磁碟使用率:${NC} $disk_usage"
    fi
    
    echo ""
    
    # 提供解決建議
    echo -e "${CYAN}=== 故障排除建議 ===${NC}"
    echo "1. 如果連接失敗，請檢查："
    echo "   - Docker 容器是否正常運行"
    echo "   - Zenoh Router 服務是否啟動"
    echo "   - 網路防火牆設定"
    echo ""
    echo "2. 如果性能不佳，請檢查："
    echo "   - 系統資源使用情況"
    echo "   - 網路頻寬和延遲"
    echo "   - Zenoh 配置參數"
    echo ""
    echo "3. 常用修復指令："
    echo "   - 重啟 AGV 容器: docker compose -f docker-compose.yml restart"
    echo "   - 重啟 AGVC 容器: docker compose -f docker-compose.agvc.yml restart"
    echo "   - 檢查容器日誌: docker compose logs -f <container_name>"
    
    echo ""
}

full_diagnostic() {
    show_header
    log_info "開始完整 Zenoh 網路診斷..."
    echo ""
    
    # 依序執行所有診斷
    analyze_config
    echo -e "\n${CYAN}===================================${NC}\n"
    
    check_router_status
    echo -e "\n${CYAN}===================================${NC}\n"
    
    diagnose_connectivity "" "" "true"
    echo -e "\n${CYAN}===================================${NC}\n"
    
    performance_test "false"
    echo -e "\n${CYAN}===================================${NC}\n"
    
    log_success "完整診斷完成！"
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
    local action="connectivity"
    local target_host=""
    local target_port=""
    local timeout="$ZENOH_TCP_TIMEOUT"
    local verbose="false"
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            connectivity|endpoints|performance|router-status|config-analysis|troubleshoot|full-check)
                action="$1"
                shift
                ;;
            --host)
                target_host="$2"
                shift 2
                ;;
            --port)
                target_port="$2"
                shift 2
                ;;
            --timeout)
                timeout="$2"
                ZENOH_TCP_TIMEOUT="$timeout"
                shift 2
                ;;
            --verbose)
                verbose="true"
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
    
    # 執行對應動作
    case $action in
        connectivity)
            show_header
            diagnose_connectivity "$target_host" "$target_port" "$verbose"
            ;;
        endpoints)
            show_header
            log_info "檢查所有 Zenoh 端點..."
            echo ""
            diagnose_connectivity "" "" "$verbose"
            ;;
        performance)
            show_header
            performance_test "$verbose"
            ;;
        router-status)
            show_header
            check_router_status
            ;;
        config-analysis)
            show_header
            analyze_config
            ;;
        troubleshoot)
            troubleshoot_zenoh
            ;;
        full-check)
            full_diagnostic
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
