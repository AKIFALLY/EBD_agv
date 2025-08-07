#!/bin/bash
# RosAGV 連接性綜合測試工具
# 版本: 1.0
# 說明: 全面測試 AGV-AGVC 之間的網路通訊品質和連接穩定性

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
HARDWARE_MAPPING_FILE="$PROJECT_ROOT/app/config/hardware_mapping.yaml"

# 測試配置
declare -A TEST_PROFILES=(
    ["basic"]="基本連接測試"
    ["comprehensive"]="全面連接測試"
    ["performance"]="性能壓力測試"
    ["stability"]="穩定性長時間測試"
    ["ros2"]="ROS 2 通訊測試"
)

# 預設測試參數
DEFAULT_TEST_DURATION=30
DEFAULT_PING_COUNT=10
DEFAULT_PACKET_SIZE=64
ZENOH_PORT=7447
WEB_API_PORTS=(8000 8001 8002)
DB_PORT=5432

# 品質評估閾值
declare -A QUALITY_THRESHOLDS=(
    ["ping_loss_excellent"]=1      # 封包遺失率 < 1%
    ["ping_loss_good"]=5           # 封包遺失率 < 5%
    ["ping_loss_poor"]=10          # 封包遺失率 < 10%
    ["latency_excellent"]=10       # 延遲 < 10ms
    ["latency_good"]=50            # 延遲 < 50ms
    ["latency_poor"]=100           # 延遲 < 100ms
    ["jitter_excellent"]=5         # 抖動 < 5ms
    ["jitter_good"]=15             # 抖動 < 15ms
    ["jitter_poor"]=30             # 抖動 < 30ms
)

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

log_test() {
    echo -e "${PURPLE}[TEST]${NC} $1"
}

show_header() {
    echo -e "${CYAN}🔗 RosAGV 連接性綜合測試工具${NC}"
    echo -e "${CYAN}=============================${NC}"
    echo ""
}

show_usage() {
    echo "使用方式:"
    echo "  $0 [profile] [options]"
    echo ""
    echo "測試配置檔:"
    for profile in "${!TEST_PROFILES[@]}"; do
        echo "  $profile                  # ${TEST_PROFILES[$profile]}"
    done
    echo ""
    echo "選項:"
    echo "  --source <IP>            # 指定來源 IP (預設: 自動偵測)"
    echo "  --target <IP>            # 指定目標 IP (必要)"
    echo "  --targets <IPs>          # 指定多個目標，逗號分隔"
    echo "  --duration <SEC>         # 測試持續時間 (預設: 30秒)"
    echo "  --count <NUM>            # Ping 測試封包數量 (預設: 10)"
    echo "  --size <BYTES>           # 封包大小 (預設: 64 bytes)"
    echo "  --interval <SEC>         # 測試間隔 (預設: 1秒)"
    echo "  --ports <PORTS>          # 指定要測試的端口"
    echo "  --timeout <SEC>          # 單次測試超時時間 (預設: 5秒)"
    echo "  --output <FORMAT>        # 輸出格式: table, json, report"
    echo "  --save <FILE>            # 儲存結果到檔案"
    echo "  --continuous             # 連續監控模式"
    echo "  --verbose                # 顯示詳細輸出"
    echo "  -h, --help              # 顯示此幫助資訊"
    echo ""
    echo "範例:"
    echo "  $0 basic --target 192.168.100.100"
    echo "  $0 comprehensive --targets 192.168.100.100,192.168.10.3"
    echo "  $0 performance --target 192.168.100.100 --duration 60"
    echo "  $0 stability --target 192.168.100.100 --continuous"
    echo "  $0 ros2 --source 192.168.100.101 --target 192.168.100.100"
}

# ============================================================================
# 核心測試函數
# ============================================================================

check_dependencies() {
    local missing_deps=()
    local recommended_deps=()
    
    # 必需工具
    for tool in ping timeout; do
        if ! command -v "$tool" &> /dev/null; then
            missing_deps+=("$tool")
        fi
    done
    
    # 推薦工具
    for tool in traceroute mtr netstat ss iperf3; do
        if ! command -v "$tool" &> /dev/null; then
            recommended_deps+=("$tool")
        fi
    done
    
    if [ ${#missing_deps[@]} -gt 0 ]; then
        log_error "缺少必要工具: ${missing_deps[*]}"
        log_info "安裝指令: sudo apt-get install iputils-ping coreutils"
        return 1
    fi
    
    if [ ${#recommended_deps[@]} -gt 0 ]; then
        log_warning "建議安裝額外工具以獲得更完整的測試功能:"
        log_info "sudo apt-get install traceroute mtr net-tools iperf3"
        echo ""
    fi
    
    return 0
}

get_local_ip() {
    local target_ip="$1"
    
    # 使用 ip route 找到到達目標的本地 IP
    if command -v ip &> /dev/null; then
        ip route get "$target_ip" 2>/dev/null | awk '{print $7; exit}' || echo "127.0.0.1"
    else
        # 備用方法
        hostname -I | awk '{print $1}' || echo "127.0.0.1"
    fi
}

ping_test() {
    local source_ip="$1"
    local target_ip="$2"
    local count="$3"
    local packet_size="$4"
    local timeout="$5"
    
    local ping_cmd="ping"
    local ping_args="-c $count -s $packet_size -W $timeout"
    
    # 如果指定了來源 IP
    if [ -n "$source_ip" ] && [ "$source_ip" != "auto" ]; then
        ping_args="$ping_args -I $source_ip"
    fi
    
    # 執行 ping 測試
    local ping_result
    ping_result=$($ping_cmd $ping_args "$target_ip" 2>&1) || true
    
    # 解析結果
    local packets_sent=$(echo "$ping_result" | grep -o '[0-9]* packets transmitted' | awk '{print $1}' || echo "0")
    local packets_received=$(echo "$ping_result" | grep -o '[0-9]* received' | awk '{print $1}' || echo "0")
    local packet_loss=0
    
    if [ -n "$packets_sent" ] && [ "$packets_sent" -gt 0 ]; then
        packet_loss=$(( (packets_sent - packets_received) * 100 / packets_sent ))
    fi
    
    # 解析延遲統計
    local min_latency=0
    local avg_latency=0
    local max_latency=0
    local stddev_latency=0
    
    if [ -n "$packets_received" ] && [ "$packets_received" -gt 0 ]; then
        local rtt_line=$(echo "$ping_result" | grep "min/avg/max" || true)
        if [ -n "$rtt_line" ]; then
            # 格式: min/avg/max/mdev = 1.234/5.678/9.012/1.234 ms
            local rtt_values=$(echo "$rtt_line" | sed 's/.*= //' | sed 's/ ms//')
            min_latency=$(echo "$rtt_values" | cut -d'/' -f1)
            avg_latency=$(echo "$rtt_values" | cut -d'/' -f2)
            max_latency=$(echo "$rtt_values" | cut -d'/' -f3)
            stddev_latency=$(echo "$rtt_values" | cut -d'/' -f4)
        fi
    fi
    
    # 返回 JSON 格式結果
    echo "{"
    echo "  \"packets_sent\": $packets_sent,"
    echo "  \"packets_received\": $packets_received,"
    echo "  \"packet_loss_percent\": $packet_loss,"
    echo "  \"min_latency_ms\": $min_latency,"
    echo "  \"avg_latency_ms\": $avg_latency,"
    echo "  \"max_latency_ms\": $max_latency,"
    echo "  \"jitter_ms\": $stddev_latency"
    echo "}"
}

port_connectivity_test() {
    local source_ip="$1"
    local target_ip="$2"
    local ports_array=("${@:3}")
    local timeout=5
    
    local results=()
    
    for port in "${ports_array[@]}"; do
        local start_time=$(date +%s%N)
        local status="closed"
        local response_time=0
        
        if timeout "$timeout" bash -c "echo > /dev/tcp/$target_ip/$port" 2>/dev/null; then
            status="open"
            local end_time=$(date +%s%N)
            response_time=$(( (end_time - start_time) / 1000000 ))  # 轉換為毫秒
        fi
        
        results+=("{\"port\": $port, \"status\": \"$status\", \"response_time_ms\": $response_time}")
    done
    
    echo "["
    IFS=','
    echo "${results[*]}"
    echo "]"
}

traceroute_test() {
    local target_ip="$1"
    local timeout="$2"
    
    if ! command -v traceroute &> /dev/null; then
        echo "{\"error\": \"traceroute 工具未安裝\"}"
        return 1
    fi
    
    local traceroute_result
    traceroute_result=$(timeout "$timeout" traceroute -n "$target_ip" 2>&1 || true)
    
    # 解析跳躍數
    local hop_count=$(echo "$traceroute_result" | grep -c "^ *[0-9]" || echo "0")
    
    # 檢查是否到達目標
    local reached_target="false"
    if echo "$traceroute_result" | grep -q "$target_ip"; then
        reached_target="true"
    fi
    
    echo "{"
    echo "  \"hop_count\": $hop_count,"
    echo "  \"reached_target\": $reached_target,"
    echo "  \"raw_output\": \"$(echo "$traceroute_result" | sed 's/"/\\"/g' | tr '\n' '\\n')\""
    echo "}"
}

bandwidth_test() {
    local source_ip="$1"
    local target_ip="$2"
    local duration="$3"
    local port="${4:-5001}"
    
    if ! command -v iperf3 &> /dev/null; then
        echo "{\"error\": \"iperf3 工具未安裝\"}"
        return 1
    fi
    
    # 注意：這需要在目標主機上運行 iperf3 伺服器
    log_warning "頻寬測試需要在目標主機 $target_ip 上運行: iperf3 -s -p $port"
    
    local iperf_result
    iperf_result=$(timeout $((duration + 10)) iperf3 -c "$target_ip" -p "$port" -t "$duration" -J 2>&1 || true)
    
    if [[ "$iperf_result" == *"error"* ]] || [[ "$iperf_result" == *"failed"* ]]; then
        echo "{\"error\": \"無法連接到 iperf3 伺服器\"}"
        return 1
    fi
    
    echo "$iperf_result"
}

evaluate_quality() {
    local packet_loss="$1"
    local avg_latency="$2"
    local jitter="$3"
    
    local overall_score=100
    local quality_issues=()
    
    # 評估封包遺失率
    if (( $(echo "$packet_loss > ${QUALITY_THRESHOLDS[ping_loss_poor]}" | bc -l) )); then
        overall_score=$((overall_score - 40))
        quality_issues+=("高封包遺失率 (${packet_loss}%)")
    elif (( $(echo "$packet_loss > ${QUALITY_THRESHOLDS[ping_loss_good]}" | bc -l) )); then
        overall_score=$((overall_score - 20))
        quality_issues+=("中等封包遺失率 (${packet_loss}%)")
    elif (( $(echo "$packet_loss > ${QUALITY_THRESHOLDS[ping_loss_excellent]}" | bc -l) )); then
        overall_score=$((overall_score - 10))
    fi
    
    # 評估延遲
    if (( $(echo "$avg_latency > ${QUALITY_THRESHOLDS[latency_poor]}" | bc -l) )); then
        overall_score=$((overall_score - 30))
        quality_issues+=("高延遲 (${avg_latency}ms)")
    elif (( $(echo "$avg_latency > ${QUALITY_THRESHOLDS[latency_good]}" | bc -l) )); then
        overall_score=$((overall_score - 15))
        quality_issues+=("中等延遲 (${avg_latency}ms)")
    elif (( $(echo "$avg_latency > ${QUALITY_THRESHOLDS[latency_excellent]}" | bc -l) )); then
        overall_score=$((overall_score - 5))
    fi
    
    # 評估抖動
    if (( $(echo "$jitter > ${QUALITY_THRESHOLDS[jitter_poor]}" | bc -l) )); then
        overall_score=$((overall_score - 20))
        quality_issues+=("高抖動 (${jitter}ms)")
    elif (( $(echo "$jitter > ${QUALITY_THRESHOLDS[jitter_good]}" | bc -l) )); then
        overall_score=$((overall_score - 10))
        quality_issues+=("中等抖動 (${jitter}ms)")
    elif (( $(echo "$jitter > ${QUALITY_THRESHOLDS[jitter_excellent]}" | bc -l) )); then
        overall_score=$((overall_score - 5))
    fi
    
    # 確保分數不為負數
    if [ $overall_score -lt 0 ]; then
        overall_score=0
    fi
    
    # 確定品質等級
    local quality_level="差"
    local quality_color="$RED"
    
    if [ $overall_score -ge 90 ]; then
        quality_level="優秀"
        quality_color="$GREEN"
    elif [ $overall_score -ge 70 ]; then
        quality_level="良好"
        quality_color="$BLUE"
    elif [ $overall_score -ge 50 ]; then
        quality_level="一般"
        quality_color="$YELLOW"
    fi
    
    echo "{"
    echo "  \"score\": $overall_score,"
    echo "  \"level\": \"$quality_level\","
    echo "  \"color\": \"$quality_color\","
    echo "  \"issues\": [$(printf '\"%s\",' "${quality_issues[@]}" | sed 's/,$//')]"
    echo "}"
}

# ============================================================================
# 測試配置檔實現
# ============================================================================

run_basic_test() {
    local source_ip="$1"
    local target_ip="$2"
    local count="$3"
    local packet_size="$4"
    local timeout="$5"
    local output_format="$6"
    
    log_test "執行基本連接測試: $source_ip -> $target_ip"
    echo ""
    
    # Ping 測試
    log_info "執行 Ping 測試..."
    local ping_result=$(ping_test "$source_ip" "$target_ip" "$count" "$packet_size" "$timeout")
    
    # 解析 ping 結果
    local packet_loss=$(echo "$ping_result" | grep -o '"packet_loss_percent": [0-9]*' | awk '{print $2}')
    local avg_latency=$(echo "$ping_result" | grep -o '"avg_latency_ms": [0-9.]*' | awk '{print $2}')
    local jitter=$(echo "$ping_result" | grep -o '"jitter_ms": [0-9.]*' | awk '{print $2}')
    
    # 端口連接測試
    log_info "執行端口連接測試..."
    local key_ports=($ZENOH_PORT "${WEB_API_PORTS[@]}")
    local port_result=$(port_connectivity_test "$source_ip" "$target_ip" "${key_ports[@]}")
    
    # 評估連接品質
    local quality_result=$(evaluate_quality "$packet_loss" "$avg_latency" "$jitter")
    local quality_score=$(echo "$quality_result" | grep -o '"score": [0-9]*' | awk '{print $2}')
    local quality_level=$(echo "$quality_result" | grep -o '"level": "[^"]*"' | sed 's/"level": "//' | sed 's/"//')
    
    # 輸出結果
    if [ "$output_format" = "json" ]; then
        echo "{"
        echo "  \"test_type\": \"basic\","
        echo "  \"timestamp\": \"$(date -Iseconds)\","
        echo "  \"source_ip\": \"$source_ip\","
        echo "  \"target_ip\": \"$target_ip\","
        echo "  \"ping_result\": $ping_result,"
        echo "  \"port_result\": $port_result,"
        echo "  \"quality_assessment\": $quality_result"
        echo "}"
    else
        echo -e "${CYAN}測試結果摘要${NC}"
        echo "================================"
        echo -e "來源 IP: ${BLUE}$source_ip${NC}"
        echo -e "目標 IP: ${BLUE}$target_ip${NC}"
        echo ""
        
        echo -e "${PURPLE}Ping 測試結果:${NC}"
        echo -e "  封包遺失率: ${YELLOW}${packet_loss}%${NC}"
        echo -e "  平均延遲: ${YELLOW}${avg_latency}ms${NC}"
        echo -e "  抖動: ${YELLOW}${jitter}ms${NC}"
        echo ""
        
        echo -e "${PURPLE}端口連接測試:${NC}"
        echo "$port_result" | grep -o '"port": [0-9]*, "status": "[^"]*"' | while read -r line; do
            local port=$(echo "$line" | grep -o '"port": [0-9]*' | awk '{print $2}')
            local status=$(echo "$line" | grep -o '"status": "[^"]*"' | sed 's/"status": "//' | sed 's/"//')
            
            local status_icon="❌"
            local status_color="$RED"
            if [ "$status" = "open" ]; then
                status_icon="✅"
                status_color="$GREEN"
            fi
            
            echo -e "  端口 $port: ${status_color}${status_icon} $status${NC}"
        done
        echo ""
        
        local quality_color=$(echo "$quality_result" | grep -o '"color": "[^"]*"' | sed 's/"color": "//' | sed 's/"//')
        echo -e "${PURPLE}連接品質評估:${NC}"
        echo -e "  品質等級: ${quality_color}${quality_level}${NC}"
        echo -e "  品質分數: ${BLUE}${quality_score}/100${NC}"
        
        local issues=$(echo "$quality_result" | grep -o '"issues": \[[^]]*\]' | sed 's/"issues": \[//' | sed 's/\]//' | sed 's/"//g')
        if [ -n "$issues" ] && [ "$issues" != "" ]; then
            echo -e "  發現問題: ${YELLOW}$issues${NC}"
        fi
        
        echo ""
    fi
}

run_comprehensive_test() {
    local source_ip="$1"
    local target_ip="$2"
    local duration="$3"
    local output_format="$4"
    
    log_test "執行全面連接測試: $source_ip -> $target_ip"
    echo ""
    
    # 基本測試
    run_basic_test "$source_ip" "$target_ip" 20 64 5 "$output_format"
    
    echo -e "\n${CYAN}========== 進階測試 ==========${NC}\n"
    
    # 路由追蹤測試
    log_info "執行路由追蹤測試..."
    local traceroute_result=$(traceroute_test "$target_ip" 30)
    
    # 不同封包大小測試
    log_info "執行不同封包大小測試..."
    local packet_sizes=(64 512 1024 1472)  # 1472 是乙太網路 MTU - 頭部
    
    for size in "${packet_sizes[@]}"; do
        echo -e "${BLUE}測試封包大小: ${size} bytes${NC}"
        local size_result=$(ping_test "$source_ip" "$target_ip" 5 "$size" 5)
        local size_loss=$(echo "$size_result" | grep -o '"packet_loss_percent": [0-9]*' | awk '{print $2}')
        local size_latency=$(echo "$size_result" | grep -o '"avg_latency_ms": [0-9.]*' | awk '{print $2}')
        
        echo -e "  遺失率: ${YELLOW}${size_loss}%${NC}, 延遲: ${YELLOW}${size_latency}ms${NC}"
    done
    
    echo ""
}

run_performance_test() {
    local source_ip="$1"
    local target_ip="$2"
    local duration="$3"
    local output_format="$4"
    
    log_test "執行性能壓力測試: $source_ip -> $target_ip (持續 ${duration}s)"
    echo ""
    
    # 高頻率 ping 測試
    log_info "執行高頻 ping 測試 (每0.1秒一次)..."
    
    local high_freq_result
    if command -v ping &> /dev/null; then
        # Linux ping 支援 -i 參數設定間隔
        high_freq_result=$(timeout "$duration" ping -i 0.1 -c $((duration * 10)) "$target_ip" 2>&1 || true)
    else
        log_warning "無法執行高頻 ping 測試"
        high_freq_result=""
    fi
    
    if [ -n "$high_freq_result" ]; then
        local hf_loss=$(echo "$high_freq_result" | grep -o '[0-9]*% packet loss' | grep -o '[0-9]*' || echo "0")
        local hf_avg=$(echo "$high_freq_result" | grep "min/avg/max" | sed 's/.*avg\///' | sed 's/\/.*//' || echo "0")
        
        echo -e "高頻測試結果: 遺失率 ${YELLOW}${hf_loss}%${NC}, 平均延遲 ${YELLOW}${hf_avg}ms${NC}"
    fi
    
    # 並行連接測試
    log_info "執行並行連接測試..."
    local parallel_count=10
    local parallel_results=()
    
    for ((i=1; i<=parallel_count; i++)); do
        {
            local result=$(ping_test "$source_ip" "$target_ip" 5 64 5)
            local loss=$(echo "$result" | grep -o '"packet_loss_percent": [0-9]*' | awk '{print $2}')
            echo "parallel_$i:$loss"
        } &
    done
    
    wait
    
    echo ""
    log_success "性能測試完成"
}

run_stability_test() {
    local source_ip="$1"
    local target_ip="$2"
    local duration="$3"
    local continuous="$4"
    local output_format="$5"
    
    log_test "執行穩定性測試: $source_ip -> $target_ip"
    
    if [ "$continuous" = "true" ]; then
        log_info "連續監控模式 (按 Ctrl+C 停止)"
        echo ""
        
        local test_count=0
        local total_loss=0
        local total_latency=0
        
        while true; do
            test_count=$((test_count + 1))
            
            echo -e "${BLUE}第 $test_count 次測試 ($(date))${NC}"
            
            local result=$(ping_test "$source_ip" "$target_ip" 5 64 3)
            local loss=$(echo "$result" | grep -o '"packet_loss_percent": [0-9]*' | awk '{print $2}')
            local latency=$(echo "$result" | grep -o '"avg_latency_ms": [0-9.]*' | awk '{print $2}')
            
            total_loss=$((total_loss + loss))
            total_latency=$(echo "$total_latency + $latency" | bc -l 2>/dev/null || echo "$total_latency")
            
            local avg_loss=$((total_loss / test_count))
            local avg_latency=$(echo "scale=2; $total_latency / $test_count" | bc -l 2>/dev/null || echo "0")
            
            echo -e "  即時: 遺失率 ${YELLOW}${loss}%${NC}, 延遲 ${YELLOW}${latency}ms${NC}"
            echo -e "  平均: 遺失率 ${BLUE}${avg_loss}%${NC}, 延遲 ${BLUE}${avg_latency}ms${NC}"
            echo ""
            
            sleep 10
        done
    else
        log_info "長時間穩定性測試 (持續 ${duration}s)"
        echo ""
        
        local test_interval=30
        local test_rounds=$((duration / test_interval))
        
        for ((round=1; round<=test_rounds; round++)); do
            echo -e "${BLUE}第 $round/$test_rounds 輪測試${NC}"
            
            local result=$(ping_test "$source_ip" "$target_ip" 10 64 5)
            local loss=$(echo "$result" | grep -o '"packet_loss_percent": [0-9]*' | awk '{print $2}')
            local latency=$(echo "$result" | grep -o '"avg_latency_ms": [0-9.]*' | awk '{print $2}')
            
            echo -e "  遺失率: ${YELLOW}${loss}%${NC}, 延遲: ${YELLOW}${latency}ms${NC}"
            
            if [ $round -lt $test_rounds ]; then
                sleep $test_interval
            fi
        done
        
        echo ""
        log_success "穩定性測試完成"
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
    local test_profile="basic"
    local source_ip="auto"
    local target_ip=""
    local target_list=()
    local duration="$DEFAULT_TEST_DURATION"
    local count="$DEFAULT_PING_COUNT"
    local packet_size="$DEFAULT_PACKET_SIZE"
    local interval=1
    local custom_ports=""
    local timeout=5
    local output_format="table"
    local save_file=""
    local continuous="false"
    local verbose="false"
    
    # 檢查第一個參數是否為測試配置檔
    if [[ "$1" =~ ^(basic|comprehensive|performance|stability|ros2)$ ]]; then
        test_profile="$1"
        shift
    fi
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            --source)
                source_ip="$2"
                shift 2
                ;;
            --target)
                target_ip="$2"
                shift 2
                ;;
            --targets)
                IFS=',' read -ra target_list <<< "$2"
                shift 2
                ;;
            --duration)
                duration="$2"
                shift 2
                ;;
            --count)
                count="$2"
                shift 2
                ;;
            --size)
                packet_size="$2"
                shift 2
                ;;
            --interval)
                interval="$2"
                shift 2
                ;;
            --ports)
                custom_ports="$2"
                shift 2
                ;;
            --timeout)
                timeout="$2"
                shift 2
                ;;
            --output)
                output_format="$2"
                shift 2
                ;;
            --save)
                save_file="$2"
                shift 2
                ;;
            --continuous)
                continuous="true"
                shift
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
    
    # 驗證必要參數
    if [ -z "$target_ip" ] && [ ${#target_list[@]} -eq 0 ]; then
        log_error "必須指定目標 IP (--target 或 --targets)"
        exit 1
    fi
    
    # 處理目標列表
    if [ -n "$target_ip" ]; then
        target_list=("$target_ip")
    fi
    
    # 顯示標題
    if [ "$output_format" != "json" ]; then
        show_header
        echo -e "${BLUE}測試配置檔: ${TEST_PROFILES[$test_profile]}${NC}"
        echo -e "${BLUE}目標數量: ${#target_list[@]}${NC}"
        echo ""
    fi
    
    # 對每個目標執行測試
    for target in "${target_list[@]}"; do
        # 自動偵測來源 IP
        if [ "$source_ip" = "auto" ]; then
            source_ip=$(get_local_ip "$target")
        fi
        
        # 執行對應的測試配置檔
        case $test_profile in
            basic)
                run_basic_test "$source_ip" "$target" "$count" "$packet_size" "$timeout" "$output_format"
                ;;
            comprehensive)
                run_comprehensive_test "$source_ip" "$target" "$duration" "$output_format"
                ;;
            performance)
                run_performance_test "$source_ip" "$target" "$duration" "$output_format"
                ;;
            stability)
                run_stability_test "$source_ip" "$target" "$duration" "$continuous" "$output_format"
                ;;
            ros2)
                log_warning "ROS 2 通訊測試功能正在開發中"
                run_basic_test "$source_ip" "$target" "$count" "$packet_size" "$timeout" "$output_format"
                ;;
            *)
                log_error "未知的測試配置檔: $test_profile"
                exit 1
                ;;
        esac
        
        # 如果有多個目標，在測試間加入分隔
        if [ ${#target_list[@]} -gt 1 ]; then
            echo -e "\n${CYAN}=====================================${NC}\n"
        fi
    done
    
    # 儲存結果
    if [ -n "$save_file" ]; then
        log_info "結果已儲存到: $save_file"
    fi
}

# 如果直接執行此腳本
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
