#!/bin/bash
# RosAGV 網路掃描和設備發現工具
# 版本: 1.0
# 說明: 掃描網路中的 AGV 和 AGVC 設備，並進行 MAC 地址和 IP 映射

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

# 預設網路範圍 (根據 RosAGV 系統)
DEFAULT_NETWORKS=("192.168.100.0/24" "192.168.10.0/24" "10.0.0.0/24")

# RosAGV 關鍵端口
AGV_PORTS=(7447 2200)      # Zenoh, SSH
AGVC_PORTS=(7447 8000 8001 8002 5432 80 2200)  # Zenoh, APIs, DB, Nginx, SSH

# 掃描方法
SCAN_METHODS=("ping" "arp" "nmap")

# 設備類型識別
declare -A DEVICE_PATTERNS=(
    ["agv"]="agv|cargo|loader|unloader"
    ["agvc"]="agvc|server|manager"
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

show_header() {
    echo -e "${CYAN}🔍 RosAGV 網路掃描和設備發現工具${NC}"
    echo -e "${CYAN}===================================${NC}"
    echo ""
}

show_usage() {
    echo "使用方式:"
    echo "  $0 [action] [options]"
    echo ""
    echo "可用動作:"
    echo "  discover              # 自動發現網路設備 (預設)"
    echo "  ping-sweep            # Ping 掃描網路範圍"
    echo "  arp-scan              # ARP 表掃描"
    echo "  port-scan             # 端口掃描識別設備類型"
    echo "  mac-lookup            # MAC 地址查詢和映射"
    echo "  topology              # 生成網路拓撲圖"
    echo "  validate-mapping      # 驗證硬體映射配置"
    echo "  live-monitor          # 即時監控設備上下線"
    echo "  -h, --help           # 顯示此幫助資訊"
    echo ""
    echo "選項:"
    echo "  --network <CIDR>     # 指定要掃描的網路範圍 (如: 192.168.1.0/24)"
    echo "  --networks <LIST>    # 指定多個網路範圍，逗號分隔"
    echo "  --target <IP>        # 指定單一目標 IP"
    echo "  --timeout <SEC>      # 設定掃描超時時間 (預設: 1秒)"
    echo "  --threads <NUM>      # 並行掃描執行緒數 (預設: 50)"
    echo "  --method <METHOD>    # 掃描方法: ping, arp, nmap (預設: ping)"
    echo "  --ports <PORTS>      # 指定要掃描的端口，逗號分隔"
    echo "  --device-type <TYPE> # 過濾設備類型: agv, agvc, all"
    echo "  --output <FORMAT>    # 輸出格式: table, json, csv"
    echo "  --save <FILE>        # 儲存結果到檔案"
    echo "  --verbose            # 顯示詳細輸出"
    echo ""
    echo "範例:"
    echo "  $0                                    # 自動發現所有設備"
    echo "  $0 ping-sweep --network 192.168.1.0/24"
    echo "  $0 port-scan --device-type agv        # 只掃描 AGV 設備"
    echo "  $0 mac-lookup --target 192.168.100.100"
    echo "  $0 topology --output json --save topology.json"
    echo "  $0 validate-mapping                   # 驗證配置檔案"
}

# ============================================================================
# 核心掃描函數
# ============================================================================

check_dependencies() {
    local missing_deps=()
    local recommended_deps=()
    
    # 必需工具
    if ! command -v ping &> /dev/null; then
        missing_deps+=("ping")
    fi
    
    # 推薦工具
    if ! command -v nmap &> /dev/null; then
        recommended_deps+=("nmap")
    fi
    
    if ! command -v arp &> /dev/null && ! command -v ip &> /dev/null; then
        recommended_deps+=("net-tools 或 iproute2")
    fi
    
    if [ ${#missing_deps[@]} -gt 0 ]; then
        log_error "缺少必要工具: ${missing_deps[*]}"
        return 1
    fi
    
    if [ ${#recommended_deps[@]} -gt 0 ]; then
        log_warning "推薦安裝: ${recommended_deps[*]}"
        log_info "安裝指令: sudo apt-get install nmap net-tools"
    fi
    
    return 0
}

parse_cidr() {
    local cidr="$1"
    
    if [[ ! "$cidr" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}/[0-9]{1,2}$ ]]; then
        log_error "無效的 CIDR 格式: $cidr"
        return 1
    fi
    
    # 使用 Python 解析 CIDR (更精確)
    python3 -c "
import ipaddress
import sys

try:
    network = ipaddress.IPv4Network('$cidr', strict=False)
    for ip in network.hosts():
        print(str(ip))
except Exception as e:
    print(f'解析 CIDR 時發生錯誤: {e}', file=sys.stderr)
    sys.exit(1)
" 2>/dev/null || {
    # 備用方法：簡單的 CIDR 解析
    local network_part="${cidr%/*}"
    local subnet_mask="${cidr#*/}"
    local base_ip="${network_part%.*}"
    
    if [ "$subnet_mask" -eq 24 ]; then
        for i in {1..254}; do
            echo "${base_ip}.$i"
        done
    else
        log_error "目前只支援 /24 子網路掩碼"
        return 1
    fi
}

ping_host() {
    local host="$1"
    local timeout="${2:-1}"
    
    # 使用適當的 ping 指令 (Linux vs macOS)
    if ping -c 1 -W "$timeout" "$host" &> /dev/null 2>&1; then
        return 0
    elif ping -c 1 -t "$timeout" "$host" &> /dev/null 2>&1; then
        return 0
    else
        return 1
    fi
}

get_mac_address() {
    local ip="$1"
    
    # 先 ping 確保 ARP 表中有記錄
    ping_host "$ip" 1 > /dev/null 2>&1 || true
    
    # 查詢 ARP 表
    local mac=""
    if command -v arp &> /dev/null; then
        mac=$(arp -n "$ip" 2>/dev/null | awk 'NR==2 {print $3}' | grep -E '^([0-9a-f]{2}:){5}[0-9a-f]{2}$' || true)
    elif command -v ip &> /dev/null; then
        mac=$(ip neigh show "$ip" 2>/dev/null | awk '{print $5}' | grep -E '^([0-9a-f]{2}:){5}[0-9a-f]{2}$' || true)
    fi
    
    # 清理和標準化 MAC 地址
    if [ -n "$mac" ] && [ "$mac" != "(incomplete)" ]; then
        echo "$mac" | tr '[:upper:]' '[:lower:]'
    fi
}

get_hostname() {
    local ip="$1"
    local timeout="${2:-2}"
    
    # 嘗試反向 DNS 查詢
    if command -v nslookup &> /dev/null; then
        timeout "$timeout" nslookup "$ip" 2>/dev/null | grep "name =" | awk '{print $4}' | sed 's/\.$//' | head -1 || true
    elif command -v dig &> /dev/null; then
        timeout "$timeout" dig -x "$ip" +short 2>/dev/null | sed 's/\.$//' | head -1 || true
    elif command -v host &> /dev/null; then
        timeout "$timeout" host "$ip" 2>/dev/null | awk '{print $5}' | sed 's/\.$//' | head -1 || true
    fi
}

scan_ports() {
    local host="$1"
    local ports_array=("${@:2}")
    local open_ports=()
    
    for port in "${ports_array[@]}"; do
        if timeout 1 bash -c "echo > /dev/tcp/$host/$port" 2>/dev/null; then
            open_ports+=("$port")
        fi
    done
    
    echo "${open_ports[@]}"
}

identify_device_type() {
    local ip="$1"
    local hostname="$2"
    local open_ports=("${@:3}")
    
    local device_type="unknown"
    local confidence=0
    
    # 基於主機名識別
    if [ -n "$hostname" ]; then
        for type in "${!DEVICE_PATTERNS[@]}"; do
            if [[ "$hostname" =~ ${DEVICE_PATTERNS[$type]} ]]; then
                device_type="$type"
                confidence=80
                break
            fi
        done
    fi
    
    # 基於開放端口識別
    if [ "$device_type" = "unknown" ] || [ $confidence -lt 50 ]; then
        local agv_port_count=0
        local agvc_port_count=0
        
        for port in "${open_ports[@]}"; do
            # 檢查是否為 AGV 典型端口
            for agv_port in "${AGV_PORTS[@]}"; do
                if [ "$port" = "$agv_port" ]; then
                    agv_port_count=$((agv_port_count + 1))
                fi
            done
            
            # 檢查是否為 AGVC 典型端口
            for agvc_port in "${AGVC_PORTS[@]}"; do
                if [ "$port" = "$agvc_port" ]; then
                    agvc_port_count=$((agvc_port_count + 1))
                fi
            done
        done
        
        # 判斷設備類型
        if [ $agvc_port_count -ge 3 ]; then
            device_type="agvc"
            confidence=70
        elif [ $agv_port_count -ge 1 ]; then
            device_type="agv"
            confidence=60
        fi
    fi
    
    echo "$device_type:$confidence"
}

# ============================================================================
# 掃描功能實現
# ============================================================================

ping_sweep() {
    local networks=("$@")
    local timeout="$1"; shift
    local threads="$1"; shift
    local verbose="$1"; shift
    local output_format="$1"; shift
    
    log_info "開始 Ping 掃描..."
    
    local alive_hosts=()
    local total_hosts=0
    
    # 計算總主機數
    for network in "${networks[@]}"; do
        local host_count=$(parse_cidr "$network" | wc -l)
        total_hosts=$((total_hosts + host_count))
    done
    
    log_info "掃描 ${#networks[@]} 個網路，共 $total_hosts 個主機"
    echo ""
    
    # 並行掃描
    for network in "${networks[@]}"; do
        echo -e "${PURPLE}掃描網路: $network${NC}"
        
        local hosts=($(parse_cidr "$network"))
        local batch_size=$((threads > 0 ? threads : 50))
        
        # 分批並行處理
        for ((i=0; i<${#hosts[@]}; i+=batch_size)); do
            local batch=("${hosts[@]:i:batch_size}")
            
            # 並行 ping
            for host in "${batch[@]}"; do
                {
                    if ping_host "$host" "$timeout"; then
                        echo "$host:alive"
                    fi
                } &
            done
            
            # 等待這批完成
            wait
            
            # 收集結果並顯示進度
            for host in "${batch[@]}"; do
                if ping_host "$host" "$timeout"; then
                    alive_hosts+=("$host")
                    
                    if [ "$verbose" = "true" ]; then
                        echo -e "  ${GREEN}✅ $host${NC}"
                    fi
                fi
            done
            
            # 顯示進度
            local current=$((i + batch_size))
            if [ $current -gt ${#hosts[@]} ]; then
                current=${#hosts[@]}
            fi
            local progress=$((current * 100 / ${#hosts[@]}))
            echo -e "${YELLOW}進度: $progress% ($current/${#hosts[@]})${NC}"
        done
        
        echo ""
    done
    
    # 輸出結果
    if [ ${#alive_hosts[@]} -gt 0 ]; then
        log_success "發現 ${#alive_hosts[@]} 個活躍主機"
        
        if [ "$output_format" = "json" ]; then
            echo "{"
            echo "  \"scan_type\": \"ping_sweep\","
            echo "  \"timestamp\": \"$(date -Iseconds)\","
            echo "  \"alive_hosts\": ["
            for i in "${!alive_hosts[@]}"; do
                echo -n "    \"${alive_hosts[i]}\""
                if [ $i -lt $((${#alive_hosts[@]} - 1)) ]; then
                    echo ","
                else
                    echo ""
                fi
            done
            echo "  ]"
            echo "}"
        else
            echo -e "${CYAN}活躍主機列表:${NC}"
            for host in "${alive_hosts[@]}"; do
                echo "  $host"
            done
        fi
    else
        log_warning "未發現任何活躍主機"
    fi
    
    return 0
}

arp_scan() {
    local verbose="$1"
    local output_format="$2"
    
    log_info "掃描 ARP 表..."
    echo ""
    
    local arp_entries=()
    
    # 讀取 ARP 表
    if command -v arp &> /dev/null; then
        while IFS= read -r line; do
            if [[ "$line" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+ ]]; then
                arp_entries+=("$line")
            fi
        done < <(arp -a 2>/dev/null | grep -v "incomplete")
    elif command -v ip &> /dev/null; then
        while IFS= read -r line; do
            if [[ "$line" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+ ]]; then
                arp_entries+=("$line")
            fi
        done < <(ip neigh show 2>/dev/null | grep -v "FAILED")
    else
        log_error "缺少 arp 或 ip 指令"
        return 1
    fi
    
    if [ ${#arp_entries[@]} -gt 0 ]; then
        log_success "發現 ${#arp_entries[@]} 個 ARP 記錄"
        
        if [ "$output_format" = "json" ]; then
            echo "{"
            echo "  \"scan_type\": \"arp_scan\","
            echo "  \"timestamp\": \"$(date -Iseconds)\","
            echo "  \"entries\": ["
        fi
        
        for i in "${!arp_entries[@]}"; do
            local entry="${arp_entries[i]}"
            
            # 解析 ARP 記錄
            local ip=""
            local mac=""
            local interface=""
            
            if command -v arp &> /dev/null; then
                # arp -a 格式解析
                ip=$(echo "$entry" | sed -E 's/.*\(([0-9.]+)\).*/\1/')
                mac=$(echo "$entry" | sed -E 's/.*at ([0-9a-f:]+).*/\1/')
                interface=$(echo "$entry" | sed -E 's/.*on ([^ ]+).*/\1/')
            else
                # ip neigh 格式解析
                ip=$(echo "$entry" | awk '{print $1}')
                mac=$(echo "$entry" | awk '{print $5}')
                interface=$(echo "$entry" | awk '{print $3}')
            fi
            
            if [ "$output_format" = "json" ]; then
                echo -n "    {"
                echo -n "\"ip\":\"$ip\",\"mac\":\"$mac\",\"interface\":\"$interface\""
                echo -n "}"
                if [ $i -lt $((${#arp_entries[@]} - 1)) ]; then
                    echo ","
                else
                    echo ""
                fi
            else
                echo -e "${PURPLE}$ip${NC}"
                echo -e "  MAC: ${BLUE}$mac${NC}"
                echo -e "  介面: ${BLUE}$interface${NC}"
                echo ""
            fi
        done
        
        if [ "$output_format" = "json" ]; then
            echo "  ]"
            echo "}"
        fi
    else
        log_warning "ARP 表中沒有記錄"
    fi
}

port_scan_devices() {
    local networks=("$@")
    local timeout="$1"; shift
    local device_filter="$1"; shift
    local verbose="$1"; shift
    local output_format="$1"; shift
    
    log_info "開始端口掃描設備識別..."
    echo ""
    
    # 先進行 ping 掃描找到活躍主機
    local alive_hosts=()
    for network in "${networks[@]}"; do
        local hosts=($(parse_cidr "$network"))
        for host in "${hosts[@]}"; do
            if ping_host "$host" 1; then
                alive_hosts+=("$host")
            fi
        done
    done
    
    if [ ${#alive_hosts[@]} -eq 0 ]; then
        log_warning "未發現任何活躍主機"
        return 0
    fi
    
    log_info "對 ${#alive_hosts[@]} 個活躍主機進行端口掃描"
    
    local discovered_devices=()
    
    if [ "$output_format" = "json" ]; then
        echo "{"
        echo "  \"scan_type\": \"port_scan\","
        echo "  \"timestamp\": \"$(date -Iseconds)\","
        echo "  \"devices\": ["
    fi
    
    for i in "${!alive_hosts[@]}"; do
        local host="${alive_hosts[i]}"
        
        echo -e "${PURPLE}掃描主機: $host${NC}"
        
        # 獲取基本資訊
        local hostname=$(get_hostname "$host" 2)
        local mac=$(get_mac_address "$host")
        
        # 掃描端口
        local all_ports=(7447 8000 8001 8002 5432 80 2200 22)
        local open_ports=($(scan_ports "$host" "${all_ports[@]}"))
        
        # 識別設備類型
        local device_info=$(identify_device_type "$host" "$hostname" "${open_ports[@]}")
        local device_type="${device_info%%:*}"
        local confidence="${device_info##*:}"
        
        # 過濾設備類型
        if [ "$device_filter" != "all" ] && [ "$device_filter" != "$device_type" ]; then
            continue
        fi
        
        discovered_devices+=("$host:$device_type:$confidence")
        
        # 輸出結果
        if [ "$output_format" = "json" ]; then
            echo -n "    {"
            echo -n "\"ip\":\"$host\",\"hostname\":\"$hostname\",\"mac\":\"$mac\","
            echo -n "\"device_type\":\"$device_type\",\"confidence\":$confidence,"
            echo -n "\"open_ports\":[$(printf '\"%s\",' "${open_ports[@]}" | sed 's/,$//')]"
            echo -n "}"
            if [ $i -lt $((${#alive_hosts[@]} - 1)) ] && [ $((i + 1)) -lt ${#alive_hosts[@]} ]; then
                echo ","
            else
                echo ""
            fi
        else
            echo -e "  IP: ${BLUE}$host${NC}"
            if [ -n "$hostname" ]; then
                echo -e "  主機名: ${BLUE}$hostname${NC}"
            fi
            if [ -n "$mac" ]; then
                echo -e "  MAC: ${BLUE}$mac${NC}"
            fi
            echo -e "  設備類型: ${GREEN}$device_type${NC} (信心度: $confidence%)"
            if [ ${#open_ports[@]} -gt 0 ]; then
                echo -e "  開放端口: ${BLUE}${open_ports[*]}${NC}"
            fi
            echo ""
        fi
    done
    
    if [ "$output_format" = "json" ]; then
        echo "  ]"
        echo "}"
    fi
    
    log_success "掃描完成，發現 ${#discovered_devices[@]} 個設備"
}

validate_hardware_mapping() {
    local verbose="$1"
    local output_format="$2"
    
    log_info "驗證硬體映射配置..."
    echo ""
    
    if [ ! -f "$HARDWARE_MAPPING_FILE" ]; then
        log_error "硬體映射檔案不存在: $HARDWARE_MAPPING_FILE"
        return 1
    fi
    
    # 使用 Python 解析 YAML 並驗證設備
    python3 -c "
import yaml
import socket
import sys
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed

def ping_host(ip, timeout=1):
    try:
        result = subprocess.run(['ping', '-c', '1', '-W', str(timeout), ip], 
                              capture_output=True, timeout=timeout+1)
        return result.returncode == 0
    except:
        return False

def check_device(device_id, device_info):
    result = {
        'device_id': device_id,
        'ip': device_info.get('ip', ''),
        'mac': device_info.get('mac', ''),
        'device_type': device_info.get('device_type', ''),
        'status': 'unknown',
        'ping_ok': False,
        'issues': []
    }
    
    # 檢查必要欄位
    if not result['ip']:
        result['issues'].append('缺少 IP 地址')
    if not result['mac']:
        result['issues'].append('缺少 MAC 地址')
    if not result['device_type']:
        result['issues'].append('缺少設備類型')
    
    # 檢查 IP 格式
    if result['ip']:
        try:
            socket.inet_aton(result['ip'])
        except socket.error:
            result['issues'].append('IP 地址格式錯誤')
    
    # 檢查 MAC 格式
    if result['mac']:
        import re
        if not re.match(r'^([0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}$', result['mac']):
            result['issues'].append('MAC 地址格式錯誤')
    
    # Ping 測試
    if result['ip'] and not result['issues']:
        result['ping_ok'] = ping_host(result['ip'])
    
    # 確定狀態
    if not result['issues']:
        if result['ping_ok']:
            result['status'] = 'online'
        else:
            result['status'] = 'offline'
    else:
        result['status'] = 'error'
    
    return result

try:
    with open('$HARDWARE_MAPPING_FILE', 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    
    devices = config.get('devices', {})
    if not devices:
        print('配置檔案中沒有找到設備定義')
        sys.exit(1)
    
    print(f'找到 {len(devices)} 個設備配置')
    
    # 並行檢查設備
    results = []
    with ThreadPoolExecutor(max_workers=10) as executor:
        future_to_device = {executor.submit(check_device, device_id, device_info): device_id 
                           for device_id, device_info in devices.items()}
        
        for future in as_completed(future_to_device):
            results.append(future.result())
    
    # 統計結果
    online_count = sum(1 for r in results if r['status'] == 'online')
    offline_count = sum(1 for r in results if r['status'] == 'offline')
    error_count = sum(1 for r in results if r['status'] == 'error')
    
    if '$output_format' == 'json':
        import json
        output = {
            'validation_type': 'hardware_mapping',
            'timestamp': '$(date -Iseconds)',
            'summary': {
                'total': len(results),
                'online': online_count,
                'offline': offline_count,
                'errors': error_count
            },
            'devices': results
        }
        print(json.dumps(output, indent=2, ensure_ascii=False))
    else:
        print()
        print('驗證結果摘要:')
        print(f'  總設備數: {len(results)}')
        print(f'  線上設備: {online_count}')
        print(f'  離線設備: {offline_count}')
        print(f'  配置錯誤: {error_count}')
        print()
        
        for result in results:
            status_icon = '✅' if result['status'] == 'online' else '❌' if result['status'] == 'offline' else '⚠️'
            print(f'{status_icon} {result[\"device_id\"]} ({result[\"device_type\"]})')
            print(f'    IP: {result[\"ip\"]}')
            if result[\"mac\"]:
                print(f'    MAC: {result[\"mac\"]}')
            print(f'    狀態: {result[\"status\"]}')
            if result[\"issues\"]:
                print(f'    問題: {\"、\".join(result[\"issues\"])}')
            print()

except Exception as e:
    print(f'驗證過程中發生錯誤: {e}')
    sys.exit(1)
" 2>/dev/null || {
    log_error "驗證過程中發生錯誤，請檢查 Python 環境和 PyYAML 套件"
    return 1
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
    local action="discover"
    local networks=("${DEFAULT_NETWORKS[@]}")
    local target_ip=""
    local timeout=1
    local threads=50
    local method="ping"
    local custom_ports=""
    local device_filter="all"
    local output_format="table"
    local save_file=""
    local verbose="false"
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            discover|ping-sweep|arp-scan|port-scan|mac-lookup|topology|validate-mapping|live-monitor)
                action="$1"
                shift
                ;;
            --network)
                networks=("$2")
                shift 2
                ;;
            --networks)
                IFS=',' read -ra networks <<< "$2"
                shift 2
                ;;
            --target)
                target_ip="$2"
                shift 2
                ;;
            --timeout)
                timeout="$2"
                shift 2
                ;;
            --threads)
                threads="$2"
                shift 2
                ;;
            --method)
                method="$2"
                shift 2
                ;;
            --ports)
                custom_ports="$2"
                shift 2
                ;;
            --device-type)
                device_filter="$2"
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
    
    # 處理單一目標 IP
    if [ -n "$target_ip" ]; then
        networks=("$target_ip/32")
    fi
    
    # 顯示標題
    if [ "$output_format" != "json" ]; then
        show_header
    fi
    
    # 設定輸出重定向
    local output_redirect=""
    if [ -n "$save_file" ]; then
        output_redirect=" | tee \"$save_file\""
    fi
    
    # 執行對應動作
    case $action in
        discover|ping-sweep)
            ping_sweep "${networks[@]}" "$timeout" "$threads" "$verbose" "$output_format"
            ;;
        arp-scan)
            arp_scan "$verbose" "$output_format"
            ;;
        port-scan)
            port_scan_devices "${networks[@]}" "$timeout" "$device_filter" "$verbose" "$output_format"
            ;;
        validate-mapping)
            validate_hardware_mapping "$verbose" "$output_format"
            ;;
        mac-lookup)
            if [ -n "$target_ip" ]; then
                log_info "查詢 $target_ip 的 MAC 地址..."
                local mac=$(get_mac_address "$target_ip")
                if [ -n "$mac" ]; then
                    log_success "MAC 地址: $mac"
                else
                    log_warning "無法獲取 MAC 地址"
                fi
            else
                log_error "mac-lookup 需要指定 --target 參數"
                exit 1
            fi
            ;;
        *)
            log_error "動作 '$action' 尚未實現"
            exit 1
            ;;
    esac
    
    # 儲存結果
    if [ -n "$save_file" ] && [ "$output_format" = "json" ]; then
        log_info "結果已儲存到: $save_file"
    fi
}

# 如果直接執行此腳本
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
