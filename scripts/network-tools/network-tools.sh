#!/bin/bash
# RosAGV 網路工具函數集合
# 版本: 1.0
# 說明: 統一的網路診斷工具函數集，可以一次載入所有網路工具功能

# ============================================================================
# 初始化和路徑設定
# ============================================================================

# 獲取腳本目錄
NETWORK_TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 檢查並載入所有網路工具腳本
if [ -f "$NETWORK_TOOLS_DIR/zenoh-network.sh" ]; then
    source "$NETWORK_TOOLS_DIR/zenoh-network.sh"
fi

if [ -f "$NETWORK_TOOLS_DIR/port-check.sh" ]; then
    source "$NETWORK_TOOLS_DIR/port-check.sh"
fi

if [ -f "$NETWORK_TOOLS_DIR/network-scan.sh" ]; then
    source "$NETWORK_TOOLS_DIR/network-scan.sh"
fi

if [ -f "$NETWORK_TOOLS_DIR/connectivity-test.sh" ]; then
    source "$NETWORK_TOOLS_DIR/connectivity-test.sh"
fi

# ============================================================================
# 統一的網路工具界面
# ============================================================================

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

show_network_tools_header() {
    echo -e "${CYAN}🌐 RosAGV 網路診斷工具套件${NC}"
    echo -e "${CYAN}==============================${NC}"
    echo ""
}

show_network_tools_help() {
    show_network_tools_header
    
    echo -e "${BLUE}可用的網路工具:${NC}"
    echo ""
    
    echo -e "${PURPLE}🔧 直接執行工具:${NC}"
    echo "  zenoh-network.sh          # Zenoh 網路專用診斷"
    echo "  port-check.sh             # 端口連接檢查"
    echo "  network-scan.sh           # 網路掃描和設備發現"
    echo "  connectivity-test.sh      # 連接性綜合測試"
    echo ""
    
    echo -e "${PURPLE}⚡ 便捷函數 (載入後可用):${NC}"
    echo "  network_quick_check       # 快速網路健康檢查"
    echo "  network_diagnose         # 全面網路診斷"
    echo "  network_monitor          # 即時網路監控"
    echo "  network_test_connection  # 測試特定連接"
    echo "  network_scan_devices     # 掃描網路設備"
    echo "  network_check_ports      # 檢查系統端口"
    echo "  network_troubleshoot     # 網路故障排除"
    echo "  network_performance      # 網路性能測試"
    echo ""
    
    echo -e "${PURPLE}📋 批量操作:${NC}"
    echo "  network_check_all_agv    # 檢查所有 AGV 設備"
    echo "  network_check_all_agvc   # 檢查所有 AGVC 設備"
    echo "  network_validate_mapping # 驗證硬體映射配置"
    echo "  network_generate_report  # 生成網路診斷報告"
    echo ""
    
    echo -e "${PURPLE}🎯 常用場景:${NC}"
    echo "  network_startup_check    # 系統啟動時的網路檢查"
    echo "  network_maintenance_check # 維護檢查"
    echo "  network_emergency_check  # 緊急故障檢查"
    echo "  network_health_monitor   # 健康狀態監控"
    echo ""
    
    echo -e "${BLUE}使用範例:${NC}"
    echo "  # 載入網路工具集"
    echo "  source scripts/network-tools/network-tools.sh"
    echo ""
    echo "  # 快速健康檢查"
    echo "  network_quick_check"
    echo ""
    echo "  # 全面診斷"
    echo "  network_diagnose"
    echo ""
    echo "  # 測試特定連接"
    echo "  network_test_connection 192.168.100.100"
    echo ""
    echo "  # 即時監控"
    echo "  network_monitor"
    echo ""
    
    echo -e "${YELLOW}注意事項:${NC}"
    echo "  - 部分功能需要額外工具 (nmap, iperf3, traceroute)"
    echo "  - 建議以 root 權限執行以獲得完整功能"
    echo "  - 網路掃描可能需要較長時間"
}

# ============================================================================
# 便捷函數實現
# ============================================================================

network_quick_check() {
    echo -e "${CYAN}🔍 快速網路健康檢查${NC}"
    echo "=============================="
    echo ""
    
    # 檢查 Zenoh 連接
    echo -e "${PURPLE}檢查 Zenoh 連接...${NC}"
    if command -v "$NETWORK_TOOLS_DIR/zenoh-network.sh" &> /dev/null; then
        "$NETWORK_TOOLS_DIR/zenoh-network.sh" connectivity --timeout 2 2>/dev/null || {
            echo -e "${RED}❌ Zenoh 連接檢查失敗${NC}"
        }
    fi
    echo ""
    
    # 檢查系統端口
    echo -e "${PURPLE}檢查系統關鍵端口...${NC}"
    if command -v "$NETWORK_TOOLS_DIR/port-check.sh" &> /dev/null; then
        "$NETWORK_TOOLS_DIR/port-check.sh" system --timeout 2 2>/dev/null || {
            echo -e "${RED}❌ 端口檢查失敗${NC}"
        }
    fi
    echo ""
    
    echo -e "${GREEN}✅ 快速檢查完成${NC}"
}

network_diagnose() {
    local target_ip="$1"
    local verbose="${2:-false}"
    
    echo -e "${CYAN}🔬 全面網路診斷${NC}"
    echo "=============================="
    echo ""
    
    # Zenoh 網路診斷
    echo -e "${BLUE}=== Zenoh 網路診斷 ===${NC}"
    if [ -f "$NETWORK_TOOLS_DIR/zenoh-network.sh" ]; then
        "$NETWORK_TOOLS_DIR/zenoh-network.sh" troubleshoot
    fi
    echo ""
    
    # 端口狀態檢查
    echo -e "${BLUE}=== 端口狀態檢查 ===${NC}"
    if [ -f "$NETWORK_TOOLS_DIR/port-check.sh" ]; then
        if [ "$verbose" = "true" ]; then
            "$NETWORK_TOOLS_DIR/port-check.sh" all --verbose
        else
            "$NETWORK_TOOLS_DIR/port-check.sh" all
        fi
    fi
    echo ""
    
    # 設備掃描
    echo -e "${BLUE}=== 網路設備掃描 ===${NC}"
    if [ -f "$NETWORK_TOOLS_DIR/network-scan.sh" ]; then
        "$NETWORK_TOOLS_DIR/network-scan.sh" discover
    fi
    echo ""
    
    # 連接測試 (如果指定了目標)
    if [ -n "$target_ip" ]; then
        echo -e "${BLUE}=== 連接性測試 ===${NC}"
        if [ -f "$NETWORK_TOOLS_DIR/connectivity-test.sh" ]; then
            "$NETWORK_TOOLS_DIR/connectivity-test.sh" basic --target "$target_ip"
        fi
        echo ""
    fi
    
    echo -e "${GREEN}✅ 全面診斷完成${NC}"
}

network_monitor() {
    local interval="${1:-10}"
    local target_ip="$2"
    
    echo -e "${CYAN}📊 即時網路監控 (每 ${interval}s 更新)${NC}"
    echo "按 Ctrl+C 停止監控"
    echo "======================================"
    echo ""
    
    local iteration=0
    
    while true; do
        iteration=$((iteration + 1))
        echo -e "${BLUE}=== 監控輪次 $iteration ($(date)) ===${NC}"
        
        # 快速連接檢查
        if [ -n "$target_ip" ]; then
            echo -e "${PURPLE}Ping 測試: $target_ip${NC}"
            if ping -c 1 -W 2 "$target_ip" &> /dev/null; then
                echo -e "  ${GREEN}✅ 連接正常${NC}"
            else
                echo -e "  ${RED}❌ 連接失敗${NC}"
            fi
        fi
        
        # 端口狀態
        echo -e "${PURPLE}系統端口狀態:${NC}"
        if [ -f "$NETWORK_TOOLS_DIR/port-check.sh" ]; then
            "$NETWORK_TOOLS_DIR/port-check.sh" system --timeout 1 2>/dev/null | grep -E "(✅|❌)" | head -3
        fi
        
        echo ""
        sleep "$interval"
    done
}

network_test_connection() {
    local target_ip="$1"
    local test_type="${2:-basic}"
    
    if [ -z "$target_ip" ]; then
        echo -e "${RED}錯誤: 請提供目標 IP 地址${NC}"
        echo "使用方式: network_test_connection <IP> [basic|comprehensive|performance]"
        return 1
    fi
    
    echo -e "${CYAN}🔗 測試連接: $target_ip${NC}"
    echo "=============================="
    echo ""
    
    if [ -f "$NETWORK_TOOLS_DIR/connectivity-test.sh" ]; then
        "$NETWORK_TOOLS_DIR/connectivity-test.sh" "$test_type" --target "$target_ip"
    else
        echo -e "${RED}連接測試工具未找到${NC}"
        return 1
    fi
}

network_scan_devices() {
    local network_range="${1:-auto}"
    local device_type="${2:-all}"
    
    echo -e "${CYAN}🔍 掃描網路設備${NC}"
    echo "=============================="
    echo ""
    
    if [ -f "$NETWORK_TOOLS_DIR/network-scan.sh" ]; then
        if [ "$network_range" = "auto" ]; then
            "$NETWORK_TOOLS_DIR/network-scan.sh" port-scan --device-type "$device_type"
        else
            "$NETWORK_TOOLS_DIR/network-scan.sh" port-scan --network "$network_range" --device-type "$device_type"
        fi
    else
        echo -e "${RED}網路掃描工具未找到${NC}"
        return 1
    fi
}

network_check_ports() {
    local category="${1:-all}"
    local host="${2:-localhost}"
    
    echo -e "${CYAN}🔌 檢查端口狀態${NC}"
    echo "=============================="
    echo ""
    
    if [ -f "$NETWORK_TOOLS_DIR/port-check.sh" ]; then
        "$NETWORK_TOOLS_DIR/port-check.sh" "$category" --host "$host" --verbose
    else
        echo -e "${RED}端口檢查工具未找到${NC}"
        return 1
    fi
}

network_troubleshoot() {
    local issue_type="${1:-general}"
    
    echo -e "${CYAN}🔧 網路故障排除${NC}"
    echo "=============================="
    echo ""
    
    case $issue_type in
        "zenoh"|"communication")
            echo -e "${PURPLE}Zenoh 通訊故障排除:${NC}"
            if [ -f "$NETWORK_TOOLS_DIR/zenoh-network.sh" ]; then
                "$NETWORK_TOOLS_DIR/zenoh-network.sh" troubleshoot
            fi
            ;;
        "ports"|"connection")
            echo -e "${PURPLE}端口連接故障排除:${NC}"
            if [ -f "$NETWORK_TOOLS_DIR/port-check.sh" ]; then
                "$NETWORK_TOOLS_DIR/port-check.sh" conflicts --verbose
            fi
            ;;
        "devices"|"discovery")
            echo -e "${PURPLE}設備發現故障排除:${NC}"
            if [ -f "$NETWORK_TOOLS_DIR/network-scan.sh" ]; then
                "$NETWORK_TOOLS_DIR/network-scan.sh" validate-mapping
            fi
            ;;
        *)
            echo -e "${PURPLE}通用網路故障排除:${NC}"
            echo "執行全面診斷..."
            network_diagnose "" "true"
            ;;
    esac
}

network_performance() {
    local target_ip="$1"
    local duration="${2:-30}"
    
    if [ -z "$target_ip" ]; then
        echo -e "${RED}錯誤: 請提供目標 IP 地址${NC}"
        echo "使用方式: network_performance <IP> [duration_seconds]"
        return 1
    fi
    
    echo -e "${CYAN}⚡ 網路性能測試${NC}"
    echo "=============================="
    echo ""
    
    if [ -f "$NETWORK_TOOLS_DIR/connectivity-test.sh" ]; then
        "$NETWORK_TOOLS_DIR/connectivity-test.sh" performance --target "$target_ip" --duration "$duration"
    else
        echo -e "${RED}連接測試工具未找到${NC}"
        return 1
    fi
}

# ============================================================================
# 批量操作函數
# ============================================================================

network_check_all_agv() {
    echo -e "${CYAN}🚗 檢查所有 AGV 設備${NC}"
    echo "=============================="
    echo ""
    
    # 從硬體映射檔案讀取 AGV 設備
    local project_root="$(cd "$NETWORK_TOOLS_DIR/../.." && pwd)"
    local hardware_mapping="$project_root/app/config/hardware_mapping.yaml"
    
    if [ ! -f "$hardware_mapping" ]; then
        echo -e "${RED}硬體映射檔案不存在: $hardware_mapping${NC}"
        return 1
    fi
    
    # 使用 Python 解析 YAML 並提取 AGV 設備
    python3 -c "
import yaml
import sys

try:
    with open('$hardware_mapping', 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    
    devices = config.get('devices', {})
    agv_devices = {k: v for k, v in devices.items() 
                   if v.get('device_type', '').lower() in ['agv', 'cargo', 'loader', 'unloader']}
    
    if not agv_devices:
        print('未找到 AGV 設備配置')
        sys.exit(1)
    
    for device_id, device_info in agv_devices.items():
        ip = device_info.get('ip', '')
        device_type = device_info.get('device_type', '')
        print(f'{device_id}:{ip}:{device_type}')

except Exception as e:
    print(f'解析配置檔案時發生錯誤: {e}', file=sys.stderr)
    sys.exit(1)
" | while IFS=':' read -r device_id ip device_type; do
        echo -e "${PURPLE}檢查 $device_id ($device_type): $ip${NC}"
        
        # 快速連接測試
        if ping -c 1 -W 2 "$ip" &> /dev/null; then
            echo -e "  ${GREEN}✅ Ping 成功${NC}"
            
            # 檢查 Zenoh 端口
            if timeout 2 bash -c "echo > /dev/tcp/$ip/7447" 2>/dev/null; then
                echo -e "  ${GREEN}✅ Zenoh 端口開放${NC}"
            else
                echo -e "  ${YELLOW}⚠️  Zenoh 端口未開放${NC}"
            fi
        else
            echo -e "  ${RED}❌ Ping 失敗${NC}"
        fi
        echo ""
    done
}

network_check_all_agvc() {
    echo -e "${CYAN}🖥️  檢查所有 AGVC 設備${NC}"
    echo "=============================="
    echo ""
    
    # 從硬體映射檔案讀取 AGVC 設備
    local project_root="$(cd "$NETWORK_TOOLS_DIR/../.." && pwd)"
    local hardware_mapping="$project_root/app/config/hardware_mapping.yaml"
    
    if [ ! -f "$hardware_mapping" ]; then
        echo -e "${RED}硬體映射檔案不存在: $hardware_mapping${NC}"
        return 1
    fi
    
    # 使用 Python 解析 YAML 並提取 AGVC 設備
    python3 -c "
import yaml
import sys

try:
    with open('$hardware_mapping', 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    
    devices = config.get('devices', {})
    agvc_devices = {k: v for k, v in devices.items() 
                    if v.get('device_type', '').lower() in ['agvc', 'server', 'manager']}
    
    if not agvc_devices:
        print('未找到 AGVC 設備配置')
        sys.exit(1)
    
    for device_id, device_info in agvc_devices.items():
        ip = device_info.get('ip', '')
        device_type = device_info.get('device_type', '')
        print(f'{device_id}:{ip}:{device_type}')

except Exception as e:
    print(f'解析配置檔案時發生錯誤: {e}', file=sys.stderr)
    sys.exit(1)
" | while IFS=':' read -r device_id ip device_type; do
        echo -e "${PURPLE}檢查 $device_id ($device_type): $ip${NC}"
        
        # 快速連接測試
        if ping -c 1 -W 2 "$ip" &> /dev/null; then
            echo -e "  ${GREEN}✅ Ping 成功${NC}"
            
            # 檢查關鍵端口
            local ports=(7447 8000 8001 8002 5432)
            local port_names=("Zenoh" "Web API" "AGVCUI" "OPUI" "PostgreSQL")
            
            for i in "${!ports[@]}"; do
                local port="${ports[i]}"
                local name="${port_names[i]}"
                
                if timeout 2 bash -c "echo > /dev/tcp/$ip/$port" 2>/dev/null; then
                    echo -e "  ${GREEN}✅ $name ($port)${NC}"
                else
                    echo -e "  ${YELLOW}⚠️  $name ($port) 未開放${NC}"
                fi
            done
        else
            echo -e "  ${RED}❌ Ping 失敗${NC}"
        fi
        echo ""
    done
}

network_validate_mapping() {
    echo -e "${CYAN}📋 驗證硬體映射配置${NC}"
    echo "=============================="
    echo ""
    
    if [ -f "$NETWORK_TOOLS_DIR/network-scan.sh" ]; then
        "$NETWORK_TOOLS_DIR/network-scan.sh" validate-mapping --verbose
    else
        echo -e "${RED}網路掃描工具未找到${NC}"
        return 1
    fi
}

network_generate_report() {
    local output_file="${1:-network_report_$(date +%Y%m%d_%H%M%S).txt}"
    
    echo -e "${CYAN}📊 生成網路診斷報告${NC}"
    echo "=============================="
    echo ""
    
    {
        echo "RosAGV 網路診斷報告"
        echo "生成時間: $(date)"
        echo "========================================"
        echo ""
        
        echo "=== 系統概覽 ==="
        echo "主機名: $(hostname)"
        echo "IP 地址: $(hostname -I)"
        echo "網路介面:"
        ip addr show | grep -E "^[0-9]:|inet " | sed 's/^/  /'
        echo ""
        
        echo "=== Zenoh 網路狀態 ==="
        if [ -f "$NETWORK_TOOLS_DIR/zenoh-network.sh" ]; then
            "$NETWORK_TOOLS_DIR/zenoh-network.sh" connectivity 2>/dev/null || echo "Zenoh 檢查失敗"
        fi
        echo ""
        
        echo "=== 系統端口狀態 ==="
        if [ -f "$NETWORK_TOOLS_DIR/port-check.sh" ]; then
            "$NETWORK_TOOLS_DIR/port-check.sh" system 2>/dev/null || echo "端口檢查失敗"
        fi
        echo ""
        
        echo "=== 設備掃描結果 ==="
        if [ -f "$NETWORK_TOOLS_DIR/network-scan.sh" ]; then
            "$NETWORK_TOOLS_DIR/network-scan.sh" discover 2>/dev/null || echo "設備掃描失敗"
        fi
        echo ""
        
        echo "=== 硬體映射驗證 ==="
        if [ -f "$NETWORK_TOOLS_DIR/network-scan.sh" ]; then
            "$NETWORK_TOOLS_DIR/network-scan.sh" validate-mapping 2>/dev/null || echo "映射驗證失敗"
        fi
        echo ""
        
        echo "========================================"
        echo "報告生成完成: $(date)"
        
    } | tee "$output_file"
    
    echo ""
    echo -e "${GREEN}✅ 報告已生成: $output_file${NC}"
}

# ============================================================================
# 場景化函數
# ============================================================================

network_startup_check() {
    echo -e "${CYAN}🚀 系統啟動網路檢查${NC}"
    echo "=============================="
    echo ""
    
    # 基本連接檢查
    echo -e "${PURPLE}1. 基本連接檢查${NC}"
    network_quick_check
    echo ""
    
    # 關鍵服務檢查
    echo -e "${PURPLE}2. 關鍵服務檢查${NC}"
    network_check_ports "system"
    echo ""
    
    # Zenoh 專項檢查
    echo -e "${PURPLE}3. Zenoh 通訊檢查${NC}"
    if [ -f "$NETWORK_TOOLS_DIR/zenoh-network.sh" ]; then
        "$NETWORK_TOOLS_DIR/zenoh-network.sh" router-status
    fi
    echo ""
    
    echo -e "${GREEN}✅ 啟動檢查完成${NC}"
}

network_maintenance_check() {
    echo -e "${CYAN}🔧 維護檢查${NC}"
    echo "=============================="
    echo ""
    
    # 全面系統檢查
    network_diagnose "" "true"
    
    # 設備狀態檢查
    echo -e "${BLUE}=== AGV 設備檢查 ===${NC}"
    network_check_all_agv
    
    echo -e "${BLUE}=== AGVC 設備檢查 ===${NC}"
    network_check_all_agvc
    
    # 配置驗證
    echo -e "${BLUE}=== 配置驗證 ===${NC}"
    network_validate_mapping
    
    echo -e "${GREEN}✅ 維護檢查完成${NC}"
}

network_emergency_check() {
    local issue_description="$1"
    
    echo -e "${RED}🚨 緊急故障檢查${NC}"
    echo "=============================="
    if [ -n "$issue_description" ]; then
        echo -e "${YELLOW}問題描述: $issue_description${NC}"
    fi
    echo ""
    
    # 快速診斷
    echo -e "${PURPLE}1. 快速狀態檢查${NC}"
    network_quick_check
    echo ""
    
    # 故障排除
    echo -e "${PURPLE}2. 故障排除診斷${NC}"
    network_troubleshoot
    echo ""
    
    # 生成緊急報告
    echo -e "${PURPLE}3. 生成緊急報告${NC}"
    local emergency_report="emergency_network_report_$(date +%Y%m%d_%H%M%S).txt"
    network_generate_report "$emergency_report"
    
    echo -e "${YELLOW}⚠️  緊急檢查完成，請查看報告: $emergency_report${NC}"
}

network_health_monitor() {
    local duration="${1:-continuous}"
    local interval="${2:-30}"
    
    echo -e "${CYAN}💓 網路健康監控${NC}"
    echo "=============================="
    echo ""
    
    if [ "$duration" = "continuous" ]; then
        echo "連續監控模式 (按 Ctrl+C 停止)"
        echo ""
        
        local check_count=0
        while true; do
            check_count=$((check_count + 1))
            echo -e "${BLUE}=== 健康檢查 #$check_count ($(date)) ===${NC}"
            
            # 執行快速檢查
            network_quick_check
            
            echo -e "${YELLOW}下次檢查將在 ${interval}s 後執行...${NC}"
            echo ""
            sleep "$interval"
        done
    else
        echo "定時監控模式 (持續 ${duration}s)"
        echo ""
        
        local end_time=$(($(date +%s) + duration))
        local check_count=0
        
        while [ $(date +%s) -lt $end_time ]; do
            check_count=$((check_count + 1))
            echo -e "${BLUE}=== 健康檢查 #$check_count ===${NC}"
            
            network_quick_check
            
            if [ $(date +%s) -lt $end_time ]; then
                echo -e "${YELLOW}等待 ${interval}s...${NC}"
                sleep "$interval"
            fi
        done
        
        echo -e "${GREEN}✅ 監控週期完成${NC}"
    fi
}

# ============================================================================
# 主程式 (如果直接執行)
# ============================================================================

main() {
    if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
        case "${1:-help}" in
            help|--help|-h)
                show_network_tools_help
                ;;
            quick-check)
                network_quick_check
                ;;
            diagnose)
                network_diagnose "$2" "$3"
                ;;
            monitor)
                network_monitor "$2" "$3"
                ;;
            test-connection)
                network_test_connection "$2" "$3"
                ;;
            scan-devices)
                network_scan_devices "$2" "$3"
                ;;
            check-ports)
                network_check_ports "$2" "$3"
                ;;
            troubleshoot)
                network_troubleshoot "$2"
                ;;
            performance)
                network_performance "$2" "$3"
                ;;
            check-all-agv)
                network_check_all_agv
                ;;
            check-all-agvc)
                network_check_all_agvc
                ;;
            validate-mapping)
                network_validate_mapping
                ;;
            generate-report)
                network_generate_report "$2"
                ;;
            startup-check)
                network_startup_check
                ;;
            maintenance-check)
                network_maintenance_check
                ;;
            emergency-check)
                network_emergency_check "$2"
                ;;
            health-monitor)
                network_health_monitor "$2" "$3"
                ;;
            *)
                echo -e "${RED}未知指令: $1${NC}"
                echo ""
                show_network_tools_help
                return 1 2>/dev/null || exit 1
                ;;
        esac
    fi
}

# 如果直接執行此腳本，運行主程式
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi